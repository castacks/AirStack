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

class DisparityExpanderNode : public rclcpp::Node {
public:
  DisparityExpanderNode()
    : Node("disparity_expander_node"), fx_(0.0f) {
    disp_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>(
								      "/robot_1/sensors/front_stereo/disparity", 10,
								      std::bind(&DisparityExpanderNode::onDisparity, this, std::placeholders::_1));

    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
								     "/robot_1/sensors/front_stereo/right/camera_info", 10,
								     std::bind(&DisparityExpanderNode::onCameraInfo, this, std::placeholders::_1));

    fg_pub_ = create_publisher<sensor_msgs::msg::Image>("foreground_expanded", 10);
    bg_pub_ = create_publisher<sensor_msgs::msg::Image>("background_expanded", 10);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 10);

    initGL();
    horizProg_ = createComputeShader(ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/disparity_expand_horizontal.cs");
    vertProg_  = createComputeShader(ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/disparity_expand_vertical.cs");

    scale = 1000000.0;
    downsample_scale = 2.f;
  }

private:
  void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
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
    glClearTexImage(fgFinal, 0, GL_RED_INTEGER, GL_INT, &zero_int);
    glClearTexImage(bgFinal, 0, GL_RED_INTEGER, GL_INT, &max_int);

    glBindTexture(GL_TEXTURE_2D, texIn);
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED, GL_FLOAT, disp.ptr<int>());
    //glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_RED_INTEGER, GL_INT, disp_i.ptr<int>());
    glBindTexture(GL_TEXTURE_2D, 0);

    GLuint query;
    glGenQueries(1, &query);
    glBeginQuery(GL_TIME_ELAPSED, query);
    
    int work_group_size_x = 16;
    int work_group_size_y = 16;
    
    // Pass 1: Horizontal
    glUseProgram(horizProg_);
    glBindImageTexture(0, texIn, 0, GL_FALSE, 0, GL_READ_ONLY,  GL_R32I);
    glBindImageTexture(1, fgHoriz, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glBindImageTexture(2, bgHoriz, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glUniform1f(glGetUniformLocation(horizProg_, "baseline"), baseline_);
    glUniform1f(glGetUniformLocation(horizProg_, "fx"), fx_);
    glUniform1f(glGetUniformLocation(horizProg_, "fy"), fy_);
    glUniform1f(glGetUniformLocation(horizProg_, "cx"), fx_);
    glUniform1f(glGetUniformLocation(horizProg_, "cy"), cy_);
    glUniform1f(glGetUniformLocation(horizProg_, "expansion_radius"), 2.f);
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
    glBindImageTexture(2, fgFinal, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glBindImageTexture(3, bgFinal, 0, GL_FALSE, 0, GL_READ_WRITE, GL_R32I);
    glUniform1f(glGetUniformLocation(vertProg_, "baseline"), baseline_);
    glUniform1f(glGetUniformLocation(vertProg_, "fx"), fx_);
    glUniform1f(glGetUniformLocation(vertProg_, "fy"), fy_);
    glUniform1f(glGetUniformLocation(vertProg_, "cx"), fx_);
    glUniform1f(glGetUniformLocation(vertProg_, "cy"), cy_);
    glUniform1f(glGetUniformLocation(vertProg_, "expansion_radius"), 2.f);
    glUniform1f(glGetUniformLocation(vertProg_, "discontinuityThresh"), 1.0f);
    glUniform1f(glGetUniformLocation(vertProg_, "scale"), scale);
    glUniform1i(glGetUniformLocation(vertProg_, "downsample_scale"), downsample_scale);
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


    publishResults(msg->image.header, fgFinal, bgFinal, width_, height_);

    glDeleteTextures(1, &texIn);
    glDeleteTextures(1, &fgHoriz);
    glDeleteTextures(1, &bgHoriz);
    glDeleteTextures(1, &fgFinal);
    glDeleteTextures(1, &bgFinal);
    glDeleteQueries(1, &query);
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
    glBindTexture(GL_TEXTURE_2D, fgF);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, width_, height_);
    
    //initTex(bgF, GL_R32I, maxv.data(), width_, height_);
    glGenTextures(1, &bgF);
    glBindTexture(GL_TEXTURE_2D, bgF);
    glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32I, width_, height_);
  }

  void publishResults(const std_msgs::msg::Header &hdr,
		      GLuint fgTex, GLuint bgTex, int w, int h) {
    cv::Mat fg_i(h, w, CV_32S);
    cv::Mat bg_i(h, w, CV_32S);
    glBindTexture(GL_TEXTURE_2D, fgTex);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_INT, fg_i.data);
    glBindTexture(GL_TEXTURE_2D, bgTex);
    glGetTexImage(GL_TEXTURE_2D, 0, GL_RED_INTEGER, GL_INT, bg_i.data);

    cv::Mat fg_f, bg_f;
    fg_i.convertTo(fg_f, CV_32F, 1.0 / scale);
    bg_i.convertTo(bg_f, CV_32F, 1.0 / scale);

    auto fg_msg = cv_bridge::CvImage(hdr, "32FC1", fg_f).toImageMsg();
    auto bg_msg = cv_bridge::CvImage(hdr, "32FC1", bg_f).toImageMsg();
    fg_pub_->publish(*fg_msg);
    bg_pub_->publish(*bg_msg);

    pcl::PointCloud<pcl::PointXYZI> fg_bg_cloud;
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
    
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(fg_bg_cloud, output);
    output.header = hdr;
    fg_bg_cloud_pub_->publish(output);
  }

  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disp_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr fg_pub_, bg_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;

  GLFWwindow *window_;
  GLuint horizProg_, vertProg_;
  float fx_, fy_, cx_, cy_, baseline_;
  int width_, height_;
  int downsample_scale;
  float scale;
  
  GLuint texIn, fgHoriz, bgHoriz, fgFinal, bgFinal;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparityExpanderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
