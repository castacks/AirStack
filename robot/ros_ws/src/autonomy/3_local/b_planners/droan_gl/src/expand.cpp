#include <rclcpp/rclcpp.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <airstack_common/vislib.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <airstack_common/tflib.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>
#include <EGL/egl.h>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <vector>
#include <mutex>


struct GraphNode {
  int fg_index;
  int bg_index;
  tf2::Stamped<tf2::Transform> tf;
};

struct alignas(16) Vec3 {
  float x, y, z;
};

struct alignas(16) Vec4 {
  float x, y, z, w;
};

struct alignas(16) State {
  Vec3 pos;
  Vec3 vel;
  Vec3 acc;
  Vec3 jerk;
  Vec3 collision;

  State(){}
  State(airstack_msgs::msg::Odometry odom){
    pos.x = odom.pose.position.x;
    pos.y = odom.pose.position.y;
    pos.z = odom.pose.position.z;

    vel.x = odom.twist.linear.x;
    vel.y = odom.twist.linear.y;
    vel.z = odom.twist.linear.z;

    acc.x = odom.acceleration.x;
    acc.y = odom.acceleration.y;
    acc.z = odom.acceleration.z;

    jerk.x = odom.jerk.x;
    jerk.y = odom.jerk.y;
    jerk.z = odom.jerk.z;
  }
};

struct TrajectoryParams {
  float vel_desired[3];
  float vel_max;
};

struct alignas(16) CommonInit {
  State initial_state;
  int traj_count;
  int traj_size;
  float dt;
};

struct mat4 {
  float data[16];

  mat4(){
    std::memset(&data[0], 0, sizeof(data));
  }
  
  mat4(const tf2::Stamped<tf2::Transform>& stamped_tf){
    //const tf2::Transform& tf = stamped_tf.getTransform();
    const tf2::Vector3& t = stamped_tf.getOrigin();
    tf2::Matrix3x3 R = stamped_tf.getBasis();

    data[0]  = R[0][0];
    data[1]  = R[1][0];
    data[2]  = R[2][0];
    data[3]  = 0.0f;

    data[4]  = R[0][1];
    data[5]  = R[1][1];
    data[6]  = R[2][1];
    data[7]  = 0.0f;

    data[8]  = R[0][2];
    data[9]  = R[1][2];
    data[10] = R[2][2];
    data[11] = 0.0f;

    data[12] = t.x();
    data[13] = t.y();
    data[14] = t.z();
    data[15] = 1.0f;
  }

    mat4(const tf2::Transform& stamped_tf){
    const tf2::Vector3& t = stamped_tf.getOrigin();
    tf2::Matrix3x3 R = stamped_tf.getBasis();

    data[0]  = R[0][0];
    data[1]  = R[1][0];
    data[2]  = R[2][0];
    data[3]  = 0.0f;

    data[4]  = R[0][1];
    data[5]  = R[1][1];
    data[6]  = R[2][1];
    data[7]  = 0.0f;

    data[8]  = R[0][2];
    data[9]  = R[1][2];
    data[10] = R[2][2];
    data[11] = 0.0f;

    data[12] = t.x();
    data[13] = t.y();
    data[14] = t.z();
    data[15] = 1.0f;
  }
};

struct alignas(16) CollisionInfo {
  mat4 state_tf;
  float fx, fy, cx, cy;
  float baseline;
  int width, height;
  int limit;
  float scale;
  float expansion_radius;
  int graph_nodes;
};


class DisparityExpanderNode : public rclcpp::Node {
public:
  DisparityExpanderNode()
    : Node("disparity_expander_node"), fx_(0.0f) {
    disp_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>(
								      "/robot_1/sensors/front_stereo/disparity", 10,
								      std::bind(&DisparityExpanderNode::onDisparity,
										this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
								     "/robot_1/sensors/front_stereo/right/camera_info", 10,
								     std::bind(&DisparityExpanderNode::onCameraInfo,
									       this, std::placeholders::_1));
    look_ahead_sub = create_subscription<airstack_msgs::msg::Odometry>("/robot_1/trajectory_controller/look_ahead", 10,
								       std::bind(&DisparityExpanderNode::look_ahead_callback,
										 this, std::placeholders::_1));
    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    fg_pub_ = create_publisher<sensor_msgs::msg::Image>("foreground_expanded", 10);
    bg_pub_ = create_publisher<sensor_msgs::msg::Image>("background_expanded", 10);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 10);
    traj_debug_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("traj_debug", 1);

    initGL();
    horizProg_ = createComputeShader(ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/disparity_expand_horizontal.cs");
    vertProg_  = createComputeShader(ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/disparity_expand_vertical.cs");

    timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1./5.),
    				 std::bind(&DisparityExpanderNode::timer_callback, this));
    
    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    look_ahead_frame = airstack::get_param(this, "look_ahead_frame", std::string("look_ahead_point"));
    graph_nodes = airstack::get_param(this, "graph_nodes", 10);
    expansion_radius = airstack::get_param(this, "expansion_radius", 2.0);
    dt = airstack::get_param(this, "dt", 0.2);
    ht = airstack::get_param(this, "ht", 10.0);
    downsample_scale = airstack::get_param(this, "downsample_scale", 2);
    visualize = airstack::get_param(this, "visualize", false);
    
    scale = 1000000.0;
    current_layer = 0;
    fx_ = -1.;
    look_ahead_valid = false;
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    if(fx_ > 0.f)
      return;
    
    fx_ = static_cast<float>(msg->k[0])/downsample_scale;
    fy_ = static_cast<float>(msg->k[4])/downsample_scale;
    cx_ = static_cast<float>(msg->k[2])/downsample_scale;
    cy_ = static_cast<float>(msg->k[5])/downsample_scale;
    width_ = msg->width/downsample_scale;
    height_ = msg->height/downsample_scale;
    baseline_ = (msg->p[3] / -msg->p[0]); // assuming right camera
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
			 "Camera intrinsics loaded: fx=%.2f fy=%.2f baseline=%.3f",
			 fx_, fy_, baseline_);

    cv::Mat none;
    setupTextures(none, msg->width, msg->height, texIn, fgHoriz, bgHoriz, fgFinal, bgFinal);
  }

  void onDisparity(const stereo_msgs::msg::DisparityImage::SharedPtr msg) {
    if (fx_ <= 0.0f) return;
    
    // udpate graph
    GraphNode node;
    
    try{
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer->lookupTransform(target_frame, msg->header.frame_id,
				     rclcpp::Time(msg->header.stamp), rclcpp::Duration::from_seconds(0.1));
      tf2::fromMsg(t, node.tf);
    }
    catch(tf2::TransformException& ex){
      RCLCPP_ERROR_STREAM(get_logger(), "Transform exception in render_spheres: " << ex.what());
      return;
    }

    if(graph.size() < graph_nodes){
      node.fg_index = graph.size();
      //node.fg_index = graph.size()*2;
      //node.bg_index = graph.size()*2 + 1;
      graph.push_front(node);
    }
    else{
      node.fg_index = graph.back().fg_index;
      //node.bg_index = graph.back().bg_index;
      graph.pop_back();
      graph.push_front(node);
    }

    static std::vector<GLuint64> times;
    int times_limit = 10;

    cv::Mat disp = cv_bridge::toCvCopy(msg->image, "32FC1")->image;
    int width = disp.cols;
    int height = disp.rows;
    //disp = 0.f;
    //disp.at<float>(cy_, cx_) = 2.f;

    // Convert float disparity to int (×10000)
    //cv::Mat disp_i;
    //disp.convertTo(disp_i, CV_32S, scale);

    GLint zero_int = 0;
    GLint max_int = std::numeric_limits<int>::max();
    //glClearTexImage(texIn, 0, GL_RED_INTEGER, GL_INT, &zero_int);
    glClearTexImage(fgHoriz, 0, GL_RED_INTEGER, GL_INT, &zero_int);
    glClearTexImage(bgHoriz, 0, GL_RED_INTEGER, GL_INT, &max_int);
    //glClearTexImage(fgFinal, 0, GL_RED_INTEGER, GL_INT, &zero_int);
    glClearTexSubImage(fgFinal,               // texture name
		       0,                     // mip level
		       0, 0, current_layer,   // xoffset, yoffset, zoffset (layer index = array slice)
		       width_, height_, 1,      // clear 1 layer only (depth = 1)
		       GL_RED_INTEGER,        // pixel format (matches R32I / RGBA32I etc.)
		       GL_INT,                // pixel type (signed int)
		       &zero_int              // pointer to the data to clear with
		       );
    glClearTexImage(bgFinal, 0, GL_RED_INTEGER, GL_INT, &max_int);

    glBindTexture(GL_TEXTURE_2D, texIn);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_FLOAT, disp.ptr<float>());
    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_INT, disp_i.ptr<int>());
    glBindTexture(GL_TEXTURE_2D, 0);

    GLuint query;
    glGenQueries(1, &query);
    glBeginQuery(GL_TIME_ELAPSED, query);
    
    int work_group_size_x = 16;
    int work_group_size_y = 16;
    
    // Pass 1: Horizontal
    glUseProgram(horizProg_);
    glBindImageTexture(0, texIn, 0, GL_FALSE, 0, GL_READ_ONLY,  GL_R32F);
    glBindImageTexture(1, fgHoriz, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glBindImageTexture(2, bgHoriz, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glUniform1f(glGetUniformLocation(horizProg_, "baseline"), baseline_);
    glUniform1f(glGetUniformLocation(horizProg_, "fx"), fx_);
    glUniform1f(glGetUniformLocation(horizProg_, "fy"), fy_);
    glUniform1f(glGetUniformLocation(horizProg_, "cx"), fx_);
    glUniform1f(glGetUniformLocation(horizProg_, "cy"), cy_);
    glUniform1f(glGetUniformLocation(horizProg_, "expansion_radius"), expansion_radius);
    glUniform1f(glGetUniformLocation(horizProg_, "discontinuityThresh"), 1.0f);
    glUniform1f(glGetUniformLocation(horizProg_, "scale"), scale);
    glUniform1i(glGetUniformLocation(horizProg_, "downsample_scale"), downsample_scale);
    glDispatchCompute((width_ + work_group_size_x-1) / work_group_size_x, (height_ + work_group_size_y-1) / work_group_size_y, 1);
    
    glEndQuery(GL_TIME_ELAPSED);
    GLuint64 horizontal_time = 0;
    glGetQueryObjectui64v(query, GL_QUERY_RESULT, &horizontal_time);
    
    //glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    work_group_size_x = 16;
    work_group_size_y = 16;

    glBeginQuery(GL_TIME_ELAPSED, query);
    
    // Pass 2: Vertical
    glUseProgram(vertProg_);
    glBindImageTexture(0, fgHoriz, 0, GL_FALSE, 0, GL_READ_ONLY,  GL_R32I);
    glBindImageTexture(1, bgHoriz, 0, GL_FALSE, 0, GL_READ_ONLY,  GL_R32I);
    glBindImageTexture(2, fgFinal, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32I);
    glBindImageTexture(3, bgFinal, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glUniform1f(glGetUniformLocation(vertProg_, "baseline"), baseline_);
    glUniform1f(glGetUniformLocation(vertProg_, "fx"), fx_);
    glUniform1f(glGetUniformLocation(vertProg_, "fy"), fy_);
    glUniform1f(glGetUniformLocation(vertProg_, "cx"), fx_);
    glUniform1f(glGetUniformLocation(vertProg_, "cy"), cy_);
    glUniform1f(glGetUniformLocation(vertProg_, "expansion_radius"), expansion_radius);
    glUniform1f(glGetUniformLocation(vertProg_, "discontinuityThresh"), 1.0f);
    glUniform1f(glGetUniformLocation(vertProg_, "scale"), scale);
    glUniform1i(glGetUniformLocation(vertProg_, "downsample_scale"), downsample_scale);
    glUniform1i(glGetUniformLocation(vertProg_, "layer"), current_layer);
    glDispatchCompute((width_ + work_group_size_x-1) / work_group_size_x, (height_ + work_group_size_y-1) / work_group_size_y, 1);

    glEndQuery(GL_TIME_ELAPSED);
    GLuint64 vertical_time = 0;
    glGetQueryObjectui64v(query, GL_QUERY_RESULT, &vertical_time);
    GLuint64 total_time = horizontal_time + vertical_time;
    times.push_back(total_time);
    if(times.size() > times_limit)
      times.erase(times.begin());
    GLuint64 average_time = 0;
    for(int i = 0; i < times.size(); i++)
      average_time += times[i];
    average_time /= times.size();
    RCLCPP_INFO(get_logger(), "h v times: %.3f ms, %.3f ms, %.3f ms, %.3f ms",
		horizontal_time / 1e6, vertical_time / 1e6, total_time / 1e6, average_time / 1e6);
    
    //glMemoryBarrier(GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);


    if(visualize)
      publishResults(msg->image.header, fgFinal, bgFinal, width_, height_);

    //glDeleteTextures(1, &texIn);
    //glDeleteTextures(1, &fgHoriz);
    //glDeleteTextures(1, &bgHoriz);
    //glDeleteTextures(1, &fgFinal);
    //glDeleteTextures(1, &bgFinal);
    glDeleteQueries(1, &query);

    current_layer = (current_layer + 1) % graph_nodes;
  }

  void initGL() {
    if(!glfwInit()) {
      throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(640, 480, "Offscreen", nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    GLint maxWorkGroupCount[3];
    GLint maxWorkGroupSize[3];
    GLint maxInvocations;

    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 0, &maxWorkGroupCount[0]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 1, &maxWorkGroupCount[1]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_COUNT, 2, &maxWorkGroupCount[2]);

    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 0, &maxWorkGroupSize[0]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 1, &maxWorkGroupSize[1]);
    glGetIntegeri_v(GL_MAX_COMPUTE_WORK_GROUP_SIZE, 2, &maxWorkGroupSize[2]);

    glGetIntegerv(GL_MAX_COMPUTE_WORK_GROUP_INVOCATIONS, &maxInvocations);

    std::cout << "Max work group count: "
	      << maxWorkGroupCount[0] << ", "
	      << maxWorkGroupCount[1] << ", "
	      << maxWorkGroupCount[2] << std::endl;

    std::cout << "Max work group size: "
	      << maxWorkGroupSize[0] << ", "
	      << maxWorkGroupSize[1] << ", "
	      << maxWorkGroupSize[2] << std::endl;

    std::cout << "Max total invocations per group: "
	      << maxInvocations << std::endl;

    glGenQueries(1, &elapsed_query);

    std::string traj_comp_filename = ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/trajectory.cs";
    traj_shader = createComputeShader(traj_comp_filename);

    std::string collision_comp_filename = ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/collision.cs";
    collision_shader = createComputeShader(collision_comp_filename);
  }

  GLuint createComputeShader(const std::string &file) {
    std::ifstream f(file);
    std::string src((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    const char *c = src.c_str();
    
    RCLCPP_INFO_STREAM(get_logger(), file);
    
    GLuint s = glCreateShader(GL_COMPUTE_SHADER);
    glShaderSource(s, 1, &c, nullptr);
    glCompileShader(s);
    GLint ok;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
      char log[1024];
      glGetShaderInfoLog(s, 1024, nullptr, log);
      RCLCPP_ERROR(get_logger(), "Shader compile error: %s", log);
    }
    GLuint p = glCreateProgram();
    glAttachShader(p, s);
    glLinkProgram(p);

    GLint success = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &success);
    if (!success) {
      GLchar info[1024];
      glGetProgramInfoLog(p, sizeof(info), NULL, info);
      std::cerr << "Link failed: " << info << std::endl;
    }
    
    glDeleteShader(s);
    return p;
  }

  void setupTextures(const cv::Mat &disp_i, int w, int h,
		     GLuint &texIn, GLuint &fgH, GLuint &bgH,
		     GLuint &fgF, GLuint &bgF) {
    auto initTex = [&](GLuint &t, GLenum fmt, const void *data, int width, int height) {
		     glGenTextures(1, &t);
		     glBindTexture(GL_TEXTURE_2D, t);
		     glTexStorage2D(GL_TEXTURE_2D, 1, fmt, width, height);
		     if (data)
		       glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_INT, data);
		   };

    //initTex(texIn, GL_R32I, disp_i.ptr<int>(), w, h);
    glGenTextures(1, &texIn);
    glBindTexture(GL_TEXTURE_2D, texIn);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, w, h);
    //glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, w, h);

    std::vector<int> zeros(width_ * height_, 0);
    std::vector<int> maxv(width_ * height_, std::numeric_limits<int>::max());
    //initTex(fgH, GL_R32I, zeros.data(), width_, height_);
    glGenTextures(1, &fgH);
    glBindTexture(GL_TEXTURE_2D, fgH);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, width_, height_);
    
    //initTex(bgH, GL_R32I, maxv.data(), width_, height_);
    glGenTextures(1, &bgH);
    glBindTexture(GL_TEXTURE_2D, bgH);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, width_, height_);
    
    //initTex(fgF, GL_R32I, zeros.data(), width_, height_);
    glGenTextures(1, &fgF);
    glBindTexture(GL_TEXTURE_2D_ARRAY, fgF);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_R32I, width_, height_, graph_nodes);
    //glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, width_, height_);
    
    //initTex(bgF, GL_R32I, maxv.data(), width_, height_);
    glGenTextures(1, &bgF);
    glBindTexture(GL_TEXTURE_2D, bgF);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, width_, height_);

    //for(float p = -80.f; p < 80.f; p += 5.f){
    //for(float y = 0.f; y < 360.f; y += 5.f){
    for(float p = -15.f; p < 16.f; p += 15.f){
      for(float y = 0.f; y < 360.f; y += 15.f){
	float yaw = y*M_PI/180.f;
	float pitch = p*M_PI/180.f;

	TrajectoryParams params;
	params.vel_desired[0] = sin(yaw);
	params.vel_desired[1] = cos(yaw);
	params.vel_desired[2] = sin(pitch);//0.f;
	params.vel_max = 2.f;

	traj_params.push_back(params);
      }
    }
    
    glGenBuffers(1, &common_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, common_ubo);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(CommonInit), nullptr, GL_DYNAMIC_DRAW);

    glGenBuffers(1, &params_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, params_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, traj_params.size()*sizeof(TrajectoryParams), traj_params.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &traj_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, traj_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, traj_params.size()*get_traj_size()*sizeof(Vec4), nullptr, GL_DYNAMIC_COPY);
    
    glGenBuffers(1, &transform_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, transform_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, graph_nodes*sizeof(mat4), nullptr, GL_DYNAMIC_COPY);

    glGenBuffers(1, &collision_info_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, collision_info_ubo);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(CollisionInfo), nullptr, GL_DYNAMIC_DRAW);
  }

  void publishResults(const std_msgs::msg::Header &hdr,
		      GLuint fgTex, GLuint bgTex, int w, int h) {
    /*
    cv::Mat fg_i(h, w, CV_32S);
    cv::Mat bg_i(h, w, CV_32S);
    //glBindTexture(GL_TEXTURE_2D_ARRAY, fgTex);
    //glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_INT, fg_i.data);
    glGetTextureSubImage(fgTex,
			 0,              // mip level
			 0, 0, current_layer, // x,y,z offset (z = layer index)
			 w, h, 1,
			 GL_RED_INTEGER,
			 GL_INT,
			 w * h * sizeof(int),
			 fg_i.data);
    glBindTexture(GL_TEXTURE_2D, bgTex);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_INT, bg_i.data);

    cv::Mat fg_f, bg_f;
    fg_i.convertTo(fg_f, CV_32F, 1.0 / scale);
    bg_i.convertTo(bg_f, CV_32F, 1.0 / scale);

    auto fg_msg = cv_bridge::CvImage(hdr, "32FC1", fg_f).toImageMsg();
    auto bg_msg = cv_bridge::CvImage(hdr, "32FC1", bg_f).toImageMsg();
    fg_pub_->publish(*fg_msg);
    bg_pub_->publish(*bg_msg);
    */
    
    pcl::PointCloud<pcl::PointXYZI> fg_bg_cloud;

    std::vector<float> data(w*h*graph_nodes);
    glBindTexture(GL_TEXTURE_2D_ARRAY, fgTex);
    glGetTexImage(GL_TEXTURE_2D_ARRAY, 0, GL_RED_INTEGER, GL_INT, data.data());

    //for(const GraphNode& node : graph){
    for(int i = 0; i < graph.size(); i++){
      const GraphNode& node = graph[i];
      cv::Mat image(h, w, CV_32S, data.data() + node.fg_index*w*h);
      
      if(i == current_layer){
	cv::Mat image_f;
	image.convertTo(image_f, CV_32F, 1./scale);
	auto fg_msg = cv_bridge::CvImage(hdr, "32FC1", image_f).toImageMsg();
	fg_pub_->publish(*fg_msg);
      }
      
      pcl::PointXYZI p;
      for(int y = 0; y < image.rows; y++){
	for(int x = 0; x < image.cols; x++){
	  float disp = ((float)image.at<int>(y, x))/scale;
	  float depth = baseline_*fx_/disp;
	  tf2::Vector3 v((x-cx_)*depth/fx_, (y-cy_)*depth/fy_, depth);
	  tf2::Vector3 v_world = node.tf*v;
	  p.x = v_world.x();
	  p.y = v_world.y();
	  p.z = v_world.z();
	  p.intensity = node.fg_index;
	  
	  if(disp > 0.f && std::isfinite(disp))
	    fg_bg_cloud.points.push_back(p);
	}
      }
    }
    
    /*
    pcl::PointXYZI p;
    for(int y = 0; y < fg_f.rows; y++){
      for(int x = 0; x < fg_f.cols; x++){
	float disp = fg_f.at<float>(y, x);
	float depth = baseline_*fx_/disp;
	tf2::Vector3 v((x-cx_)*depth/fx_, (y-cy_)*depth/fy_, depth);
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	p.intensity = 1;
	
	if(disp > 0.f && std::isfinite(disp))
	  fg_bg_cloud.points.push_back(p);
      }
    }
    for(int y = 0; y < bg_f.rows; y++){
      for(int x = 0; x < bg_f.cols; x++){
	float disp = bg_f.at<float>(y, x);
	float depth = baseline_*fx_/disp;
	tf2::Vector3 v((x-cx_)*depth/fx_, (y-cy_)*depth/fy_, depth);
	p.x = v.x();
	p.y = v.y();
	p.z = v.z();
	p.intensity = 0;
	
	if(disp > 0.f && std::isfinite(disp))
	  fg_bg_cloud.points.push_back(p);
      }
    }
    */
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(fg_bg_cloud, output);
    output.header = hdr;
    output.header.frame_id = target_frame;
    fg_bg_cloud_pub_->publish(output);
  }

  void timer_callback(){
    if(!look_ahead_valid || fx_ <= 0.f)
      return;

    airstack_msgs::msg::Odometry look_ahead_odom;
    if(!tflib::transform_odometry(tf_buffer, look_ahead, look_ahead_frame, look_ahead_frame, &look_ahead_odom))
      return;
    tf2::Stamped<tf2::Transform> look_ahead_tf;
    try{
      geometry_msgs::msg::TransformStamped t;
      t = tf_buffer->lookupTransform(target_frame, look_ahead_frame,
				     look_ahead_odom.header.stamp, rclcpp::Duration::from_seconds(0.1));
      tf2::fromMsg(t, look_ahead_tf);
    }
    catch(tf2::TransformException& ex){
      RCLCPP_ERROR_STREAM(get_logger(), "Transform exception in render_spheres: " << ex.what());
      return;
    }
    mat4 look_ahead_mat4(look_ahead_tf);
    
    CommonInit ci;
    ci.initial_state = State(look_ahead_odom);
    ci.traj_count = traj_params.size();
    ci.traj_size = get_traj_size();
    ci.dt = dt;

    //RCLCPP_INFO_STREAM(get_logger(), "initial state: " << ci.initial_state.pos.x << " " << ci.initial_state.pos.y << " " << ci.initial_state.pos.z << " odom " << look_ahead_odom.pose.position.x << " " << look_ahead_odom.pose.position.y << " " << look_ahead_odom.pose.position.z);

    glMemoryBarrier(GL_ALL_BARRIER_BITS);    

    glUseProgram(traj_shader);
    
    //printUniformBlockLayout(traj_shader, "CommonInit");
    //RCLCPP_INFO_STREAM(get_logger(), offsetof(CommonInit, initial_state) << "|" << sizeof(State) << " " << offsetof(CommonInit, traj_count) << " " << offsetof(CommonInit, traj_size) << " " << offsetof(CommonInit, dt));
    
    glBindBuffer(GL_UNIFORM_BUFFER, common_ubo);
    CommonInit* ptr = (CommonInit*)glMapBufferRange(GL_UNIFORM_BUFFER, 0, sizeof(CommonInit),
						    GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    
    *ptr = ci;
    glUnmapBuffer(GL_UNIFORM_BUFFER);
    
    
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, common_ubo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, params_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, traj_ssbo);
    

    gl_tic();
    glDispatchCompute((traj_params.size() + 255) / 256, 1, 1);
    float traj_gen_elapsed = gl_toc();
    RCLCPP_INFO_STREAM(get_logger(), "TRAJ GEN TIMING: " << traj_gen_elapsed);
    //glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    // collision checking
    glUseProgram(collision_shader);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, traj_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, transform_ssbo);
    
    // TODO do this only when one is updated, probably in the function where they get updated
    // TODO see if mapping a smaller region, ie only the updated mat4, is better
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, transform_ssbo);
    mat4* transform_ptr = (mat4*)glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0,
						  graph_nodes*sizeof(mat4),
						  GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    for(int i = 0; i < std::min(graph_nodes, (int)graph.size()); i++)
      transform_ptr[graph[i].fg_index] = mat4(graph[i].tf.inverse());
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

    // TODO maybe just pad the trajectory points buffer so that the shader doesn't need to check the limit
    //glUniform1i(glGetUniformLocation(collision_shader, "limit"), traj_params.size() * get_traj_size());
    //glUniformMatrix4fv(glGetUniformLocation(collision_shader, "state_tf"), 1, GL_FALSE, &look_ahead_mat4.data[0]);

    // TODO really only have to set this once
    CollisionInfo collision_info;
    collision_info.state_tf = look_ahead_mat4;
    collision_info.fx = fx_;
    collision_info.fy = fy_;
    collision_info.cx = cx_;
    collision_info.cy = cy_;
    collision_info.baseline = baseline_;
    collision_info.width = width_;
    collision_info.height = height_;
    collision_info.limit = traj_params.size() * get_traj_size();
    collision_info.scale = scale;
    collision_info.expansion_radius = expansion_radius;
    collision_info.graph_nodes = graph_nodes;
    glBindBuffer(GL_UNIFORM_BUFFER, collision_info_ubo);
    CollisionInfo* collision_info_ptr = (CollisionInfo*)glMapBufferRange(GL_UNIFORM_BUFFER, 0, sizeof(CollisionInfo),
									 GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    
    *collision_info_ptr = collision_info;
    glUnmapBuffer(GL_UNIFORM_BUFFER);
    glBindBufferBase(GL_UNIFORM_BUFFER, 3, collision_info_ubo);

    //glActiveTexture(GL_TEXTURE0);
    //glBindTexture(GL_TEXTURE_2D_ARRAY, fgFinal);
    glBindImageTexture(0, fgFinal, 0, GL_TRUE, 0, GL_READ_WRITE, GL_R32I);

    gl_tic();
    glDispatchCompute((traj_params.size() * get_traj_size() + 255) / 256, 1, 1);
    float traj_collision_check_elapsed = gl_toc();
    RCLCPP_INFO_STREAM(get_logger(), "TRAJ COLLISION CHECK TIMING: " << traj_collision_check_elapsed);
    //glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
    glMemoryBarrier(GL_ALL_BARRIER_BITS);

    // trajectory visualization
    //if(visualize){
      glBindBuffer(GL_SHADER_STORAGE_BUFFER, traj_ssbo);
      Vec4* output_data = (Vec4*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
      std::vector<Vec4> output_states(traj_params.size()*get_traj_size());
      memcpy(output_states.data(), output_data, output_states.size() * sizeof(Vec4));
      glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);

      //RCLCPP_INFO_STREAM(get_logger(), "output states: " << output_states.size() << " params: " << traj_params.size());
    
      traj_markers.overwrite();
      vis::Marker& free_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
      free_markers.set_namespace("free");
      free_markers.set_color(0., 1., 0.);
      free_markers.set_scale(0.1, 0.1, 0.1);
      vis::Marker& collision_markers = traj_markers.add_points(target_frame, look_ahead.header.stamp);
      collision_markers.set_namespace("collision");
      collision_markers.set_color(1., 0., 0.);
      collision_markers.set_scale(0.1, 0.1, 0.1);
      RCLCPP_INFO_STREAM(get_logger(), "traj " << ci.traj_count << " " << ci.traj_size);
      for(int i = 0; i < output_states.size(); i++){
	Vec4& state = output_states[i];
	//RCLCPP_INFO_STREAM(get_logger(), "i " << i << " " << state.pos.x << " " << state.pos.y << " " << state.pos.z);

	if(state.w >= 0.5)
	  free_markers.add_point(state.x, state.y, state.z);
	else
	  collision_markers.add_point(state.x, state.y, state.z);
      }
      //RCLCPP_INFO_STREAM(get_logger(), sizeof(Vec3) << " " << sizeof(State) << " " << sizeof(TrajectoryParams) << " " << sizeof(CommonInit));
    
      traj_debug_pub_->publish(traj_markers.get_marker_array());
      //}
  }

  void check_gl_error(){
    GLenum e = glGetError();
    if(e != GL_NO_ERROR) RCLCPP_INFO(get_logger(), "GL error: 0x%x", e);
    else RCLCPP_INFO(get_logger(), "GL error: ok");
  }

  int get_traj_size(){
    return ht/dt;// * 5;
  }

  void gl_tic(){
    glBeginQuery(GL_TIME_ELAPSED, elapsed_query);
  }
  
  float gl_toc(){
    glEndQuery(GL_TIME_ELAPSED);
    GLuint64 elapsed_ns = 0;
    glGetQueryObjectui64v(elapsed_query, GL_QUERY_RESULT, &elapsed_ns);
    float elapsed_ms = elapsed_ns / 1e6;
    return elapsed_ms;
  }

  void look_ahead_callback(const airstack_msgs::msg::Odometry::SharedPtr msg) {
    look_ahead = *msg;
    look_ahead_valid = true;
  }


  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
  tf2_ros::Buffer* tf_buffer;
  tf2_ros::TransformListener* tf_listener;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fg_pub_, bg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr traj_debug_pub_;

  rclcpp::TimerBase::SharedPtr timer;

  GLFWwindow *window_;
  GLuint horizProg_, vertProg_;
  float fx_, fy_, cx_, cy_, baseline_;
  int width_, height_;
  int downsample_scale;
  float scale;
  float expansion_radius;

  int current_layer, graph_nodes;
  
  GLuint texIn, fgHoriz, bgHoriz, fgFinal, bgFinal;

  std::deque<GraphNode> graph;
  std::string target_frame;

  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::string look_ahead_frame;
  std::vector<TrajectoryParams> traj_params;
  float dt, ht;
  GLuint traj_shader, collision_shader;
  GLuint common_ubo, collision_info_ubo;
  GLuint params_ssbo, traj_ssbo, transform_ssbo;
  
  GLuint elapsed_query;

  vis::MarkerArray traj_markers;
  bool visualize;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparityExpanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
