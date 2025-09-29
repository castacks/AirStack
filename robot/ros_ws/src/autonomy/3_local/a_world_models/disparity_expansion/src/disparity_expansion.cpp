/*
 * Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

// Applies C-Space expansion on disparity images.
#include <disparity_expansion/disparity_expansion.hpp>

DisparityExpansionNode::DisparityExpansionNode(const rclcpp::NodeOptions& options)
    : Node("DisparityExpansionNode", options) {
    this->declare_parameter("metric_depth_scale", 2.0);
    this->get_parameter("metric_depth_scale", this->metric_depth_scale);
    this->declare_parameter("expansion_radius", 2.0);
    this->get_parameter("expansion_radius", this->expansion_radius);
    this->declare_parameter("lut_max_disparity", 164);
    this->get_parameter("lut_max_disparity", this->lut_max_disparity);
    table_d.resize(this->lut_max_disparity, 0.0);
    this->declare_parameter("padding", 2.0);
    this->get_parameter("padding", this->padding);
    this->declare_parameter("baseline_fallback", 0.5);
    this->declare_parameter("bg_multiplier", 5.0);
    this->get_parameter("bg_multiplier", this->bg_multiplier);
    this->declare_parameter("sensor_pixel_error", 0.5);
    this->get_parameter("sensor_pixel_error", this->pixel_error);
    this->declare_parameter("downsample_scale", 2.0);
    this->get_parameter("downsample_scale", this->downsample_scale);
    this->declare_parameter("ignore_left_pixels", 0);
    this->get_parameter("ignore_left_pixels", this->ignore_left_pixels);

    // subscribers
    this->cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "camera_info", 1,
        std::bind(&DisparityExpansionNode::set_cam_info, this, std::placeholders::_1));

    this->disparity_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
        "disparity", 1,
        std::bind(&DisparityExpansionNode::process_disparity_image, this, std::placeholders::_1));

    this->depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "depth", 1,
        std::bind(&DisparityExpansionNode::process_depth_image, this, std::placeholders::_1));

    // publishers
    this->expanded_disparity_fg_pub =
        this->create_publisher<sensor_msgs::msg::Image>("expanded_disparity_fg", 10);
    this->expanded_disparity_bg_pub =
        this->create_publisher<sensor_msgs::msg::Image>("expanded_disparity_bg", 10);
    this->expansion_poly_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("expansion_poly", 10);
    this->expansion_cloud_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("expansion_cloud", 10);
    this->frustum_pub = this->create_publisher<visualization_msgs::msg::Marker>("frustum", 10);

    RCLCPP_INFO_STREAM(this->get_logger(), this->get_name() << " waiting");
}

void DisparityExpansionNode::set_cam_info(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr& msg_info) {
    if (this->got_cam_info) return;
    this->model_.fromCameraInfo(msg_info);
    RCLCPP_INFO_ONCE(this->get_logger(), "Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",
                     this->model_.fx(), this->model_.fy(), this->model_.cx(), this->model_.cy());
    this->cx = this->model_.cx() / this->downsample_scale;
    this->cy = this->model_.cy() / this->downsample_scale;
    this->fx = this->fy = this->model_.fx() / this->downsample_scale;
    this->width = msg_info->width / this->downsample_scale;
    this->height = msg_info->height / this->downsample_scale;
    this->baseline = -msg_info->p.at(3) / msg_info->p.at(0);
    if (this->baseline == 0.0) {
        this->get_parameter("baseline_fallback", this->baseline);
        RCLCPP_ERROR_STREAM_ONCE(
            this->get_logger(),
            "Baseline from camera_info was 0, setting to this->baseline_fallback:"
                << this->baseline);
    }
    // this->baseline *= this->downsample_scale;
    this->generate_expansion_lookup_table();
    this->got_cam_info = true;
    RCLCPP_INFO(this->get_logger(), "Baseline: %.4f meters", this->baseline);
    RCLCPP_INFO(this->get_logger(), "Focal length (fx): %.4f pixels", this->fx);
}

void DisparityExpansionNode::generate_expansion_lookup_table() {
    if (this->LUT_ready) {
        RCLCPP_ERROR(this->get_logger(), "LUT all ready");

        return;
    }
    RCLCPP_INFO(this->get_logger(), "Fx Fy Cx Cy: %f %f , %f %f \nW H this->baseline: %d %d %f",
                this->fx, this->fy, this->cx, this->cy, this->width, this->height, this->baseline);
    double r = this->expansion_radius;  // expansion radius in cm
    this->table_u = std::vector<std::vector<LUTCell>>(this->lut_max_disparity,
                                                      std::vector<LUTCell>(this->width));
    this->table_v = std::vector<std::vector<LUTCell>>(this->lut_max_disparity,
                                                      std::vector<LUTCell>(this->height));
    int u1, u2, v1, v2;
    double x, y, z;
    double disparity;

    for (unsigned int disp_idx = 1; disp_idx < lut_max_disparity; disp_idx++) {
        disparity = disp_idx / this->metric_depth_scale;  // 1 cell = 0.5m, z is in meters
        //r = this->expansion_radius;                       // * exp(DEPTH_ERROR_COEFF*z);
        z = this->baseline * this->fx / disparity;
	r = std::min(this->expansion_radius, z-0.01);

        double disp_new = this->baseline * this->fx / (z - this->expansion_radius) + 0.5;
        table_d.at(disp_idx) = disp_new;

        for (int v = (int)height - 1; v >= 0; --v) {
            y = (v - this->cy) * z / this->fy;

            double beta = atan2(z, y);
            double beta1 = asin(r / sqrt(z * z + y * y));

            double r1 = z / tan(beta + beta1);
            double r2 = z / tan(beta - beta1);
            v1 = this->fy * r1 / z + this->cy;
            v2 = this->fy * r2 / z + this->cy;

	    //*
            if ((v2 - v1) < 0)
                RCLCPP_ERROR(this->get_logger(),
                             "Something MESSED disp_idx=%d v=%d cy=%f fy=%f r=%f x=%f y=%f z=%f "
                             "beta=%f beta1=%f b+=%f b-=%f r1=%f r2=%f v1=%d v2=%d tan+=%f tan-=%f",
                             disp_idx, v, this->cy, this->fy, r, x, y, z, beta, beta1, (beta+beta1), (beta-beta1), r1, r2, v1,
                             v2, tan(beta + beta1), tan(beta - beta1));
	    else if((z < r) && std::isfinite(beta1)){
	      RCLCPP_ERROR(this->get_logger(), "Z WAS LESS THAN R BUT THERE IS NO PROBLEMdisp_idx=%d v=%d cy=%f fy=%f r=%f x=%f y=%f z=%f "
                             "beta=%f beta1=%f b+=%f b-=%f r1=%f r2=%f v1=%d v2=%d tan+=%f tan-=%f",
                             disp_idx, v, this->cy, this->fy, r, x, y, z, beta, beta1, (beta+beta1), (beta-beta1), r1, r2, v1,
                             v2, tan(beta + beta1), tan(beta - beta1));
	    }
	    //*/
            if (v1 < 0) v1 = 0;
            if (v1 > (height - 1)) v1 = height - 1;

            if (v2 < 0) v2 = 0;
            if (v2 > (height - 1)) v2 = height - 1;

            this->table_v.at(disp_idx).at(v).idx1 = v1;
            this->table_v.at(disp_idx).at(v).idx2 = v2;
        }

        for (int u = (int)this->width - 1; u >= 0; --u) {
            x = (u - this->cx) * z / this->fx;

            double alpha = atan2(z, x);
            double alpha1 = asin(r / sqrt(z * z + x * x));

            double r1 = z / tan(alpha + alpha1);
            double r2 = z / tan(alpha - alpha1);
            u1 = this->fx * r1 / z + this->cx;
            u2 = this->fx * r2 / z + this->cx;
	    //*
            if ((u2 - u1) < 0)
                RCLCPP_ERROR(this->get_logger(),
                             "Something MESSED disp_idx=%d u=%d cx=%f fx=%f r=%f x=%f y=%f z=%f "
                             "alpha=%f alpha1=%f a+=%f a-=%f r1=%f r2=%f u1=%d u2=%d",
                             disp_idx, u, this->cx, this->fx, r, x, y, z, alpha, alpha1, (alpha+alpha1), (alpha-alpha1), r1, r2, u1,
                             u2);
	    else if((z < r) && std::isfinite(alpha1)){
	      RCLCPP_ERROR(this->get_logger(), "Z WAS LESS THAN R BUT THERE IS NO PROBLEMdisp_idx=%d u=%d cx=%f fx=%f r=%f x=%f y=%f z=%f "
                             "alpha=%f alpha1=%f a+=%f a-=%f r1=%f r2=%f u1=%d u2=%d",
                             disp_idx, u, this->cx, this->fx, r, x, y, z, alpha, alpha1, (alpha+alpha1), (alpha-alpha1), r1, r2, u1,
                             u2);
	    }
	    //*/
            if (u1 < 0) u1 = 0;
            if (u1 > (this->width - 1)) u1 = this->width - 1;

            if (u2 < 0) u2 = 0;
            if (u2 > (this->width - 1)) u2 = this->width - 1;

            this->table_u.at(disp_idx).at(u).idx1 = u1;
            this->table_u.at(disp_idx).at(u).idx2 = u2;
        }
    }

    RCLCPP_WARN(this->get_logger(), "Expansion LUT created: LUT MAX: %d , ROBOT SIZE: %f",
                lut_max_disparity / 2, this->expansion_radius);
    this->LUT_ready = true;
}

void DisparityExpansionNode::process_depth_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    if (cv_ptr->image.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty depth image.");
        return;
    }
    cv::Mat disparity_image = this->convert_depth_to_disparity(cv_ptr->image);

    auto disparity_msg = std::make_shared<stereo_msgs::msg::DisparityImage>();
    disparity_msg->header = msg->header;
    disparity_msg->image =
        *cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::TYPE_32FC1, disparity_image)
             .toImageMsg();
    disparity_msg->f = this->fx;

    // Call the existing disparity processing function
    this->process_disparity_image(disparity_msg);
}

cv::Mat DisparityExpansionNode::convert_depth_to_disparity(const cv::Mat& depth_image) {
    if (depth_image.empty()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Depth image is empty in depthToDisparity. Raising an error.");
        throw std::runtime_error("Depth image is empty in depthToDisparity.");
    }

    cv::Mat disparity_image(depth_image.size(), CV_32F);

    // DISPARITY FORMULA

    disparity_image = (this->baseline * this->fx) / (depth_image);  // MULT BY 100

    RCLCPP_INFO_ONCE(this->get_logger(), "Baseline: %.4f", this->baseline);
    RCLCPP_INFO_ONCE(this->get_logger(), "Focal length (fx): %.4f", this->fx);
    cv::patchNaNs(disparity_image, 0.0f);

    disparity_image.setTo(0.0f, disparity_image == std::numeric_limits<float>::infinity());

    return disparity_image;
}

void DisparityExpansionNode::process_disparity_image(
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp) {

    //auto begin = std::chrono::high_resolution_clock::now();
  
    if (!this->LUT_ready) {
        auto& clock = *this->get_clock();
        RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 5,
                             "LUT not ready yet, probably waiting for camera info. Not processing disparity");
        return;
    }

    auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
    img_msg->header = msg_disp->header;
    img_msg->height = msg_disp->image.height;
    img_msg->width = msg_disp->image.width;
    img_msg->encoding = msg_disp->image.encoding;
    img_msg->is_bigendian = msg_disp->image.is_bigendian;
    img_msg->step = msg_disp->image.step;
    img_msg->data = msg_disp->image.data;

    cv_bridge::CvImagePtr fg_msg(new cv_bridge::CvImage());
    cv_bridge::CvImagePtr bg_msg(new cv_bridge::CvImage());

    cv_bridge::CvImageConstPtr cv_ptrdisparity;
    try {
        cv_ptrdisparity = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat disparity32F = cv_ptrdisparity->image;
    if (disparity32F.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Received empty disparity image");
        return;
    }
    cv::resize(disparity32F, disparity32F, cv::Size(), 1.0 / this->downsample_scale,
               1.0 / this->downsample_scale, cv::INTER_AREA);
    if (disparity32F.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Resized disparity image is empty");
        return;
    }
    if(ignore_left_pixels > 0)
      disparity32F(cv::Range::all(), cv::Range(0, ignore_left_pixels)) = std::numeric_limits<float>::quiet_NaN();
    
    cv::patchNaNs(disparity32F, 0.f);

    static cv::Mat disparity_fg;
    static cv::Mat disparity_bg;
    cv::Mat disparity32F_bg;

    if(disparity_fg.size() != disparity32F.size())
      disparity_fg = cv::Mat(disparity32F.size(), disparity32F.type(), -std::numeric_limits<float>::infinity());
    else
      disparity_fg.setTo(-std::numeric_limits<float>::infinity());

    if(disparity_bg.size() != disparity32F.size())
      disparity_bg = cv::Mat(disparity32F.size(), disparity32F.type(), std::numeric_limits<float>::infinity());
    else
      disparity_bg.setTo(std::numeric_limits<float>::infinity());

    fg_msg->header = msg_disp->header;
    fg_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
    bg_msg->header = msg_disp->header;
    bg_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    RCLCPP_INFO_ONCE(this->get_logger(), "IMG TYPE 32FC, GOOD");

    rclcpp::Time start = this->get_clock()->now();
    
    // In a first step, an intermediate disparity image is generated by horizontally expanding all
    // disparity values from the stereo disparity map using the u1/u2 look-up table.

    // The first step expands disparities along the image XY axis Figure 3 (right)
    expand(disparity32F, disparity32F, disparity_fg, disparity_bg, false);
    
    // In a second step, each pixel in the intermediate disparity image is expanded vertically using
    // the v1/v2 look-up table and the expansion column is stored with a new disparity value
    cv::transpose(disparity_fg, disparity32F);
    cv::transpose(disparity_bg, disparity32F_bg);
    cv::transpose(disparity_fg, disparity_fg);
    cv::transpose(disparity_bg, disparity_bg);
    expand(disparity32F, disparity32F_bg, disparity_fg, disparity_bg, true);
    cv::transpose(disparity_fg, disparity_fg);
    cv::transpose(disparity_bg, disparity_bg);
    
    disparity_fg.setTo(-1.f, disparity_fg == -std::numeric_limits<float>::infinity());
    disparity_bg.setTo(-1.f, disparity_bg == std::numeric_limits<float>::infinity());

    /*
    auto end = std::chrono::high_resolution_clock::now();
    double elapsed = ((std::chrono::duration<double>)(end - begin)).count();
    static std::vector<double> elapsed_history;
    elapsed_history.push_back(elapsed);
    if(elapsed_history.size() > 10)
      elapsed_history.erase(elapsed_history.begin());
    double average = 0.;
    for(double e : elapsed_history)
      average += e;
    average /= elapsed_history.size();
    RCLCPP_INFO_STREAM(this->get_logger(), "elapsed: " << elapsed << " s | average: " << average << " s");
    */
    
    fg_msg->image = disparity_fg;
    bg_msg->image = disparity_bg;

    expanded_disparity_fg_pub->publish(*(fg_msg->toImageMsg()));
    expanded_disparity_bg_pub->publish(*(bg_msg->toImageMsg()));

    this->publish_frustum(msg_disp);
    // only visualize expansion polygon if there's a subscriber
    if (expansion_poly_pub->get_subscription_count() > 0)
        this->publish_expansion_poly(msg_disp, fg_msg, bg_msg);
    // only visualize expansion cloud if there's a subscriber
    if (expansion_cloud_pub->get_subscription_count() > 0)
        this->publish_expansion_cloud(msg_disp, cv_ptrdisparity, fg_msg, bg_msg);

    return;
}

void DisparityExpansionNode::expand(cv::Mat& disparity_fg_in, cv::Mat& disparity_bg_in,
				    cv::Mat& disparity_fg_out, cv::Mat& disparity_bg_out,
				    bool second_pass){
  for (int v = 0; v < disparity_fg_in.rows - 1; v++) {
    for (int u = 0; u < disparity_fg_in.cols - 1; u++) {
      float disparity_value = disparity_fg_in.at<float>(v, u);

      // skip invalid disparities
      if (std::isnan(double(disparity_value * this->metric_depth_scale)) ||
	  ((int(disparity_value * this->metric_depth_scale) + 1) >=
	   this->lut_max_disparity) ||
	  ((int(disparity_value * this->metric_depth_scale) + 1) <= 0))
	continue;

      // get expansion bounds
      std::vector<std::vector<LUTCell>>* table = &table_u;
      if(second_pass)
	table = &table_v;
      unsigned int u1 =
	table->at(int(disparity_value * this->metric_depth_scale) + 1).at(u).idx1;
      unsigned int u2 =
	table->at(int(disparity_value * this->metric_depth_scale) + 1).at(u).idx2;
      cv::Rect roi = cv::Rect(u1, v, (u2 - u1), 1);
      if(roi.width <= 0)
	continue;

      double min, max;
      cv::Point p1, p2;
      cv::minMaxLoc(disparity_fg_in(roi), &min, &max, &p1, &p2);
      int max_idx = p2.x;
      float disp_new_fg = max;
      if(second_pass){
	disp_new_fg = this->baseline * this->fx / (this->baseline * this->fx / max - this->expansion_radius) + this->pixel_error;
	if(disp_new_fg < 0.f)
	  disp_new_fg = std::numeric_limits<float>::infinity();
      }

      cv::Mat submat_t = disparity_bg_in(roi);
      cv::minMaxLoc(submat_t, &min, &max, &p1, &p2, disparity_bg_in(roi) != std::numeric_limits<float>::infinity());
      float disp_to_depth = this->baseline * this->fx / max;

      // find how much this region of background disp should be expanded by looking at the connected components of the max disp
      if(second_pass && this->padding < 0.0){
	cv::Mat submat;
	cv::divide(baseline * this->fx, submat_t, submat);
	submat = (submat - disp_to_depth);
      
	float range = this->bg_multiplier * this->expansion_radius;
	float max_depth = 0.0;
	bool found = true;
	int ctr = 1;
	while(found){
	  found = false;
	  for(int j = 0; j < submat.cols; j++){
	    float val = submat.at<float>(0, j);
	    if(std::isfinite(val) && (val < ctr * range) && (val > max_depth)){
	      found = true;
	      max_depth = val;
	    }
	  }
	  ctr++;
	}
	disp_to_depth += max_depth;
      }

      if(second_pass)
	disp_to_depth += fabs(this->padding) + this->expansion_radius;
      float disp_new_bg = this->baseline * this->fx / (disp_to_depth);
      if(second_pass){
	disp_new_bg -= this->pixel_error;
	disp_new_bg = disp_new_bg < 0.0 ? 0.0001 : disp_new_bg;
      }
      cv::max(disparity_fg_out(roi), disp_new_fg, disparity_fg_out(roi));
      cv::min(disparity_bg_out(roi), disp_new_bg, disparity_bg_out(roi));

      int u_temp = u1 + max_idx;
      if(u_temp <= u)
	u = u2;
      else
	u = u_temp - 1;
    }
  }
}

void DisparityExpansionNode::publish_frustum(
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg) {
    visualization_msgs::msg::Marker marker;
    auto sensor_frame = msg->header.frame_id;
    marker.header.frame_id = sensor_frame;
    marker.ns = "frustum";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = marker.scale.y = marker.scale.z = 1.;
    marker.color.a = 0.3;
    marker.color.r = 0.5;
    marker.color.g = 1.0;
    marker.color.b = 0.9;

    double max_depth = this->lut_max_disparity * this->baseline * this->fx;
    // v, u
    std::tuple<double, double> top_left = {0.0, 0.0};
    std::tuple<double, double> top_right = {0.0, width};
    std::tuple<double, double> bottom_left = {height, 0.0};
    std::tuple<double, double> bottom_right = {height, width};
    std::vector<std::tuple<double, double>> fov = {top_left, top_right, bottom_right, bottom_left};

    std::vector<geometry_msgs::msg::Point> points_3d;
    for (auto& [v, u] : fov) {
        geometry_msgs::msg::Point point;
        point.x = (u - this->cx) * max_depth / this->fx;
        point.y = (v - this->cy) * max_depth / this->fy;
        point.z = max_depth;
        points_3d.push_back(point);
    }
    geometry_msgs::msg::Point top_left_p = points_3d.at(0);
    geometry_msgs::msg::Point top_right_p = points_3d.at(1);
    geometry_msgs::msg::Point bottom_right_p = points_3d.at(2);
    geometry_msgs::msg::Point bottom_left_p = points_3d.at(3);
    geometry_msgs::msg::Point center_p;  // 0,0,0

    // push back the triangles that make up the frustum. 4 sides, and 2 triangles to make the box in
    // left face
    marker.points.push_back(top_left_p);
    marker.points.push_back(bottom_left_p);
    marker.points.push_back(center_p);

    // top face
    marker.points.push_back(top_left_p);
    marker.points.push_back(top_right_p);
    marker.points.push_back(center_p);

    // right face
    marker.points.push_back(top_right_p);
    marker.points.push_back(bottom_right_p);
    marker.points.push_back(center_p);

    // bottom face
    marker.points.push_back(bottom_left_p);
    marker.points.push_back(bottom_right_p);
    marker.points.push_back(center_p);

    // frustum base
    // upper right triangle
    marker.points.push_back(top_left_p);
    marker.points.push_back(top_right_p);
    marker.points.push_back(bottom_right_p);
    // lower left triangle
    marker.points.push_back(top_left_p);
    marker.points.push_back(bottom_left_p);
    marker.points.push_back(bottom_right_p);

    this->frustum_pub->publish(marker);
}

void DisparityExpansionNode::publish_expansion_poly(
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp,
    const cv_bridge::CvImagePtr& fg_msg, const cv_bridge::CvImagePtr& bg_msg) {
    visualization_msgs::msg::MarkerArray marker_arr;
    visualization_msgs::msg::Marker marker;
    marker.header = msg_disp->header;
    marker.ns = "occ_space";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = 0;  // xyz_centroid[0];
    marker.pose.position.y = 0;  // xyz_centroid[1];
    marker.pose.position.z = 0;  // xyz_centroid[2];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, 0.0);
    marker.pose.orientation = tf2::toMsg(q);

    marker.scale.x = marker.scale.y = marker.scale.z = 0.5;
    marker.color.a = 0.3;  // Don't forget to set the alpha!

    geometry_msgs::msg::PolygonStamped poly;
    poly.header = msg_disp->header;
    int v = 120;
    float prev_depth = 0.0;
    // TODO: what is this magic number 239? wtf
    for (int v = (int)0; v <= 239; v += 10) {
        for (int u = (int)width - 1; u >= 0; u--) {
            float depth_value =
                this->baseline * this->fx /
                fg_msg->image.at<float>(v, u);  // free_msg->image.at<cv::Vec4f>(v,u)[0];
            float depth_diff = fabs(depth_value - prev_depth);
            prev_depth = depth_value;
            if (!std::isnan(depth_value) && !std::isinf(depth_value) && depth_diff < 0.5) {
                marker.color.r = 1.0 * (fg_msg->image.at<float>(v, u) - this->pixel_error) /
                                 fg_msg->image.at<float>(v, u);
                marker.color.g = 1.0 * (this->pixel_error) / fg_msg->image.at<float>(v, u);
                geometry_msgs::msg::Point gm_p;
                gm_p.x = (u - this->cx) * depth_value / this->fx;
                gm_p.y = (v - this->cy) * depth_value / this->fy;
                gm_p.z = depth_value;
                marker.points.push_back(gm_p);

                depth_value = this->baseline * this->fx / bg_msg->image.at<float>(v, u);
                gm_p.x = (u - this->cx) * depth_value / this->fx;
                gm_p.y = (v - this->cy) * depth_value / this->fy;
                gm_p.z = depth_value;
                marker.points.push_back(gm_p);

            } else {
                marker_arr.markers.push_back(marker);
                marker.points.clear();
                marker.id++;
            }
        }
    }
    marker_arr.markers.push_back(marker);
    //this->expansion_poly_pub->publish(marker_arr);
}

void DisparityExpansionNode::publish_expansion_cloud(
    const stereo_msgs::msg::DisparityImage::ConstSharedPtr& msg_disp,
    const cv_bridge::CvImageConstPtr& cv_ptrdisparity, const cv_bridge::CvImagePtr& fg_msg,
    const cv_bridge::CvImagePtr& bg_msg) {
    PointCloud::Ptr cloud(new PointCloud);
    cloud->header.frame_id = msg_disp->header.frame_id;
    cloud->height = 1;
    cloud->width = 1;
    cloud->is_dense = false;
    int point_counter = 0;
    pcl::PointXYZI pt_fg, pt_bg, pt_free1, pt_free2, pt_real;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker spheres;
    spheres.header = msg_disp->header;
    spheres.ns = "spheres";
    spheres.id = 0;
    spheres.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::msg::Marker::ADD;
    spheres.scale.x = this->expansion_radius*2.f;
    spheres.scale.y = this->expansion_radius*2.f;
    spheres.scale.z = this->expansion_radius*2.f;
    spheres.color.r = 0.0f;
    spheres.color.g = 1.0f;
    spheres.color.b = 0.0f;
    spheres.color.a = 1.0f;

    for (int v = (int)this->height - 1; v >= 0; v -= 4) {
        for (int u = (int)this->width - 1; u >= 0; u -= 4) {
            // foreground
            // free_msg->image.at<cv::Vec4f>(v,u)[0];
            float depth_value = this->baseline * this->fx / fg_msg->image.at<float>(v, u);
            pt_fg.x = (u - this->cx) * depth_value / this->fx;
            pt_fg.y = (v - this->cy) * depth_value / this->fy;
            pt_fg.z = depth_value;
            pt_fg.intensity = 220;

            // background
            // free_msg->image.at<cv::Vec4f>(v,u)[1];
            depth_value = this->baseline * this->fx / bg_msg->image.at<float>(v, u);
            pt_bg.x = (u - this->cx) * depth_value / this->fx;
            pt_bg.y = (v - this->cy) * depth_value / this->fy;
            pt_bg.z = depth_value;
            pt_bg.intensity = 120;

            // actual depth
            // new_disparity_bridgePtr->image.at<float>(v,u);
            depth_value = this->baseline * this->fx / cv_ptrdisparity->image.at<float>(v, u);
            pt_real.x = (u - this->cx) * depth_value / this->fx;
            pt_real.y = (v - this->cy) * depth_value / this->fy;
            pt_real.z = depth_value;
            pt_real.intensity = 170;  //*disparity32F.at<float>(v,u)/200;
	    
	    geometry_msgs::msg::Point p;
	    p.x = pt_real.x;
	    p.y = pt_real.y;
	    p.z = pt_real.z;
	    if(std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z))
	      spheres.points.push_back(p);

            point_counter++;
            cloud->points.push_back(pt_fg);
            point_counter++;
            cloud->points.push_back(pt_bg);
            point_counter++;
            cloud->points.push_back(pt_real);
        }
    }
    cloud->width = point_counter;

    cloud->header.stamp = rclcpp::Time(msg_disp->header.stamp).nanoseconds();
    cloud->header.stamp = pcl_conversions::toPCL(rclcpp::Time(msg_disp->header.stamp));

    sensor_msgs::msg::PointCloud2 cloud_PC2;
    pcl::toROSMsg(*cloud, cloud_PC2);
    this->expansion_cloud_pub->publish(cloud_PC2);

    marker_array.markers.push_back(spheres);    
    this->expansion_poly_pub->publish(marker_array);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DisparityExpansionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
