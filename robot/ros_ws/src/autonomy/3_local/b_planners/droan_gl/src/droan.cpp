#include <rclcpp/rclcpp.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <airstack_msgs/msg/odometry.hpp>
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

struct CameraIntrinsics {
  float fx, fy, cx, cy, baseline;
  int width, height;
  bool valid = false;
};

struct Mesh {
  unsigned int VAO, VBO, EBO;
  size_t index_count;
};

struct GraphNode {
  int fg_index;
  int bg_index;
  tf2::Stamped<tf2::Transform> tf;
};

class DisparitySphereRenderer : public rclcpp::Node {
public:
  DisparitySphereRenderer()
    : Node("disparity_sphere_renderer") 
  {
    disparity_sub_ = create_subscription<stereo_msgs::msg::DisparityImage>("/robot_1/sensors/front_stereo/disparity", 10, 
									   std::bind(&DisparitySphereRenderer::disparity_callback,
										     this, std::placeholders::_1));
    caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>("/robot_1/sensors/front_stereo/right/camera_info", 10,
								     std::bind(&DisparitySphereRenderer::camera_info_callback,
									       this, std::placeholders::_1));
    tracking_point_sub = create_subscription<airstack_msgs::msg::Odometry>("/robot_1/trajectory_controller/tracking_point", 10,
									   std::bind(&DisparitySphereRenderer::tracking_point_callback,
										     this, std::placeholders::_1));
    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    rendered_pub_ = create_publisher<sensor_msgs::msg::Image>("/disparity_spheres_image", 10);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 10);
	

    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    graph_nodes = airstack::get_param(this, "graph_nodes", 5);
    total_layers = graph_nodes*2;
  }

  ~DisparitySphereRenderer() {
    glfwTerminate();
  }

private:
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rendered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;
  tf2_ros::TransformListener* tf_listener;
  tf2_ros::Buffer* tf_buffer;

  
  pcl::PointCloud<pcl::PointXYZI> fg_bg_cloud;

  std::mutex data_mutex_;
  cv::Mat disparity_image_;
  CameraIntrinsics intrinsics_;

  GLFWwindow* window_;
  GLuint sphere_vao_, sphere_vbo_, instance_vbo_;
  GLuint framebuffer_, color_tex_, depth_tex_;
  GLuint shader_program_;
  Mesh sphere_mesh;

  GLuint traj_shader;

  std::string target_frame;
  int graph_nodes;
  int total_layers;

  std::deque<GraphNode> graph;

  void init_opengl() {
    // TODO auto detect whether or not to use egl or glfw based on whether egl is able to find a non software renderer
    //*
    if(!glfwInit()) {
      throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(640, 480, "Offscreen", nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    //*/

    /*
    static const EGLint configAttribs[] = {
					   EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
					   EGL_BLUE_SIZE, 8,
					   EGL_GREEN_SIZE, 8,
					   EGL_RED_SIZE, 8,
					   EGL_DEPTH_SIZE, 24,
					   EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
					   EGL_NONE
    };

    static const EGLint pbufferAttribs[] = {
					    EGL_WIDTH, intrinsics_.width,
					    EGL_HEIGHT, intrinsics_.height,
					    EGL_NONE,
    };
    EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    EGLint major, minor;
    eglInitialize(eglDpy, &major, &minor);
    EGLint numConfigs;
    EGLConfig eglCfg;
    eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs);
    EGLSurface eglSurf = eglCreatePbufferSurface(eglDpy, eglCfg, 
						 pbufferAttribs);
    eglBindAPI(EGL_OPENGL_API);
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, 
					 NULL);
    eglMakeCurrent(eglDpy, eglSurf, eglSurf, eglCtx);
    if(!gladLoadGLLoader((GLADloadproc)eglGetProcAddress))
      std::cout << "Failed to initialize GLAD!" << std::endl;
    //*/

    // from now on use your OpenGL context
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << std::endl;
    std::cout << "Vendor:   " << glGetString(GL_VENDOR)   << std::endl;
    std::cout << "Version:  " << glGetString(GL_VERSION)  << std::endl;
    GLint maxVertexUniforms = 0;
    glGetIntegerv(GL_MAX_VERTEX_UNIFORM_VECTORS, &maxVertexUniforms);
    std::cout << "Max vertex uniforms: " << maxVertexUniforms << std::endl;
    GLint maxFragmentUniforms = 0;
    glGetIntegerv(GL_MAX_FRAGMENT_UNIFORM_VECTORS, &maxFragmentUniforms);
    std::cout << "Max fragment uniforms: " << maxFragmentUniforms << std::endl;

    // Create framebuffer
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    glGenTextures(1, &color_tex_);
    glBindTexture(GL_TEXTURE_2D_ARRAY, color_tex_);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_R32F, intrinsics_.width, intrinsics_.height, total_layers);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_tex_, 0);
    
    glGenTextures(1, &depth_tex_);
    glBindTexture(GL_TEXTURE_2D_ARRAY, depth_tex_);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY,
		   1,
		   GL_DEPTH_COMPONENT32F,
		   intrinsics_.width, intrinsics_.height,
		   total_layers);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_tex_, 0);

    
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      throw std::runtime_error("Framebuffer not complete");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    shader_program_ = create_shader();

    std::string traj_comp_filename = ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/trajectory.cs";
    GLuint traj_cs = compile_shader(GL_COMPUTE_SHADER, traj_comp_filename);
    traj_shader = glCreateProgram();
    glAttachShader(traj_shader, traj_cs);
    glLinkProgram(traj_shader);
    glDeleteShader(traj_cs);
    
    glViewport(0,0,intrinsics_.width,intrinsics_.height);
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
  }

  GLuint create_shader() {
    std::string expansion_vert_filename = ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/expansion.vert";
    std::string expansion_frag_filename = ament_index_cpp::get_package_share_directory("droan_gl") + "/shaders/expansion.frag";
    GLuint vs = compile_shader(GL_VERTEX_SHADER, expansion_vert_filename);
    GLuint fs = compile_shader(GL_FRAGMENT_SHADER, expansion_frag_filename);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
  }

  GLuint compile_shader(GLenum shader_type, std::string filename){
    std::ifstream file(filename);
    //RCLCPP_INFO_STREAM(get_logger(), "SHADER FILENAME: " << filename);
    if (!file.is_open())
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to open shader file: " << filename);
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string buffer_str = buffer.str();
    const char* shader_source = buffer_str.c_str();

    //RCLCPP_INFO_STREAM(get_logger(), "SHADER: " << shader_source);
    
    GLuint id = glCreateShader(shader_type);
    glShaderSource(id, 1, &shader_source, nullptr);
    glCompileShader(id);

    int success;
    glGetShaderiv(id, GL_COMPILE_STATUS, &success);
    if (!success) {
      GLint logLength = 0;
      glGetShaderiv(id, GL_INFO_LOG_LENGTH, &logLength);

      std::vector<GLchar> infoLog(logLength);
      glGetShaderInfoLog(id, logLength, nullptr, infoLog.data());

      RCLCPP_ERROR_STREAM(get_logger(), "Shader compilation failed:\n" << infoLog.data());
    }

    return id;
  }

  void create_instanced_sphere() {
    std::string mesh_filename = ament_index_cpp::get_package_share_directory("droan_gl") + "/config/half_sphere.dae";
    RCLCPP_INFO_STREAM(get_logger(), "mesh filename: " << mesh_filename);
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(mesh_filename,
					     aiProcess_Triangulate);
    if(!scene || !scene->HasMeshes()){
      RCLCPP_ERROR_STREAM(get_logger(), "Model load failed: " << importer.GetErrorString());
      exit(1);
    }
    if(scene->mNumMeshes > 1)
      RCLCPP_ERROR_STREAM(get_logger(), "Model has " << scene->mNumMeshes << ". Only loading the first one.");
    
    aiMesh* mesh = scene->mMeshes[0];
    RCLCPP_INFO_STREAM(get_logger(), "Loaded mesh " << 0 << " with " << mesh->mNumVertices << " vertices");

    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    
    for(unsigned i = 0; i < mesh->mNumVertices; i++){
        vertices.push_back(mesh->mVertices[i].x*0.5);
        vertices.push_back(mesh->mVertices[i].y*0.5);
        vertices.push_back(mesh->mVertices[i].z*0.5);
    }
    for(unsigned i = 0; i < mesh->mNumFaces; i++)
      for(unsigned j = 0; j < mesh->mFaces[i].mNumIndices; j++)
	indices.push_back(mesh->mFaces[i].mIndices[j]);
    sphere_mesh.index_count = indices.size();

    glGenVertexArrays(1, &sphere_mesh.VAO);
    glBindVertexArray(sphere_mesh.VAO);

    glGenBuffers(1, &sphere_mesh.VBO);
    glBindBuffer(GL_ARRAY_BUFFER, sphere_mesh.VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &sphere_mesh.EBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, sphere_mesh.EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &instance_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);
  }
  
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    intrinsics_.fx = msg->k[0];
    intrinsics_.fy = msg->k[4];
    intrinsics_.cx = msg->k[2];
    intrinsics_.cy = msg->k[5];
    intrinsics_.baseline = -msg->p.at(3) / msg->p.at(0);
    intrinsics_.width = msg->width;
    intrinsics_.height = msg->height;

    if(!intrinsics_.valid){
      init_opengl();
      create_instanced_sphere();
    }
    
    intrinsics_.valid = true;
  }
  
  void disparity_callback(const stereo_msgs::msg::DisparityImage::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, "32FC1");
    disparity_image_ = cv_ptr->image.clone();
    
    if(!intrinsics_.valid || disparity_image_.empty()) return;

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
      node.fg_index = graph.size()*2;
      node.bg_index = graph.size()*2 + 1;
      graph.push_front(node);
    }
    else{
      node.fg_index = graph.back().fg_index;
      node.bg_index = graph.back().bg_index;
      graph.pop_back();
      graph.push_front(node);
    }


    std::vector<float> offsets;
    for(int y=0; y<disparity_image_.rows; y++){
      for(int x=0; x<disparity_image_.cols; x++){
	float d = disparity_image_.at<float>(y,x);
	if(d <= 0) continue;

	float Z = intrinsics_.baseline * intrinsics_.fx / d;
	float X = (x - intrinsics_.cx) * Z / intrinsics_.fx;
	float Y = (y - intrinsics_.cy) * Z / intrinsics_.fy;
	offsets.push_back(X);
	offsets.push_back(Y);
	offsets.push_back(Z);
      }
    }

    glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
    glBufferData(GL_ARRAY_BUFFER, offsets.size()*sizeof(float), offsets.data(), GL_DYNAMIC_DRAW);

    // foreground
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D_ARRAY, color_tex_);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_tex_, 0, node.fg_index);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_tex_, 0, node.fg_index);
    glClearDepth(1.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDepthFunc(GL_LESS);
    //glCullFace(GL_BACK);

    glUseProgram(shader_program_);
    glm::mat4 proj = glm::perspective(2.f*atan(((float)intrinsics_.height)/(2.f*intrinsics_.fy)),
				      ((float)intrinsics_.width)/((float)intrinsics_.height),
				      0.01f, 100.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(0,0,0), glm::vec3(0,0,1), glm::vec3(0,1,0));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "proj"), 1, GL_FALSE, &proj[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, &view[0][0]);
    glUniform1f(glGetUniformLocation(shader_program_, "baseline_fx"), intrinsics_.baseline * intrinsics_.fx);
    glUniform1i(glGetUniformLocation(shader_program_, "tex_array"), 0);
    glUniform1f(glGetUniformLocation(shader_program_, "sign"), 1.f);
    
    glBindVertexArray(sphere_mesh.VAO);
    glDrawElementsInstanced(GL_TRIANGLES, sphere_mesh.index_count, GL_UNSIGNED_INT, 0, offsets.size()/3);

    // background
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_tex_, 0, node.bg_index);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depth_tex_, 0, node.bg_index);
    glClearDepth(0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDepthFunc(GL_GREATER);
    //glCullFace(GL_FRONT);
    glUniform1f(glGetUniformLocation(shader_program_, "sign"), -1.f);
    
    glBindVertexArray(sphere_mesh.VAO);
    glDrawElementsInstanced(GL_TRIANGLES, sphere_mesh.index_count, GL_UNSIGNED_INT, 0, offsets.size()/3);
    
    publish_texture();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  void publish_texture() {
    fg_bg_cloud.points.clear();
    
    std::vector<float> data(intrinsics_.width * intrinsics_.height * total_layers);
    glBindTexture(GL_TEXTURE_2D_ARRAY, color_tex_);
    glGetTexImage(GL_TEXTURE_2D_ARRAY, 0, GL_RED, GL_FLOAT, data.data());
    
    for(const GraphNode& node : graph){
      std::vector<int> indices{node.fg_index, node.bg_index};
      tf2::Transform tf = node.tf.inverse();
      for(const int& index : indices){
	cv::Mat image(intrinsics_.height, intrinsics_.width, CV_32FC1, data.data() + index*intrinsics_.width*intrinsics_.height);
	cv::flip(image, image, 1);
	pcl::PointXYZI p;
      
	for(int y = 0; y < image.rows; y++){
	  for(int x = 0; x < image.cols; x++){
	    float disp = image.at<float>(y, x);
	    float depth = intrinsics_.baseline*intrinsics_.fx/disp;
	    tf2::Vector3 v((x-intrinsics_.cx)*depth/intrinsics_.fx, (y-intrinsics_.cy)*depth/intrinsics_.fy, depth);
	    tf2::Vector3 v_world = node.tf*v;
	    p.x = v_world.x();
	    p.y = v_world.y();
	    p.z = v_world.z();
	    p.intensity = (index+1)%2;

	    if(disp > 0.f && std::isfinite(disp))
	      fg_bg_cloud.points.push_back(p);
	  }
	}
      }
    }

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(fg_bg_cloud, output);
    // TODO handle transforms correctly
    output.header.stamp = this->now();
    output.header.frame_id = "map";
    fg_bg_cloud_pub_->publish(output);

    
    int selected_layer = 1;
    cv::Mat image(intrinsics_.height, intrinsics_.width, CV_32FC1, data.data() + selected_layer*intrinsics_.width*intrinsics_.height);
    cv::flip(image, image, 1);

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = this->now();
    out_msg.header.frame_id = "camera_frame";
    out_msg.encoding = "32FC1";
    out_msg.image = image;

    rendered_pub_->publish(*out_msg.toImageMsg());
  }

  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg) {
    
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparitySphereRenderer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
