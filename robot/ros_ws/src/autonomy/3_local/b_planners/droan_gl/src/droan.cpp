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

void printUniformBlockLayout(GLuint program, const std::string& blockName)
{
    GLuint blockIndex = glGetUniformBlockIndex(program, blockName.c_str());
    if (blockIndex == GL_INVALID_INDEX) {
        std::cerr << "Block " << blockName << " not found in shader.\n";
        return;
    }

    // Get the number of active uniforms in the block
    GLint numUniforms = 0;
    glGetActiveUniformBlockiv(program, blockIndex, GL_UNIFORM_BLOCK_ACTIVE_UNIFORMS, &numUniforms);

    std::vector<GLint> uniformIndices(numUniforms);
    glGetActiveUniformBlockiv(program, blockIndex, GL_UNIFORM_BLOCK_ACTIVE_UNIFORM_INDICES, uniformIndices.data());

    std::cout << "=== Layout of UBO: " << blockName << " ===\n";

for (int i = 0; i < numUniforms; ++i) {
    GLuint uniformIndex = uniformIndices[i];

    // Query uniform name
    char nameBuf[128];
    GLsizei nameLen;
    glGetActiveUniformName(program, uniformIndex, sizeof(nameBuf), &nameLen, nameBuf);

    // Query properties individually
    GLint offset = 0;
    GLint arrayStride = 0;
    GLint matrixStride = 0;
    GLint type = 0;

    glGetActiveUniformsiv(program, 1, &uniformIndex, GL_UNIFORM_OFFSET, &offset);
    glGetActiveUniformsiv(program, 1, &uniformIndex, GL_UNIFORM_ARRAY_STRIDE, &arrayStride);
    glGetActiveUniformsiv(program, 1, &uniformIndex, GL_UNIFORM_MATRIX_STRIDE, &matrixStride);
    glGetActiveUniformsiv(program, 1, &uniformIndex, GL_UNIFORM_TYPE, &type);

    std::cout << "  " << nameBuf
              << " @ offset=" << offset
              << ", arrayStride=" << arrayStride
              << ", matrixStride=" << matrixStride
              << ", type=0x" << std::hex << type << std::dec << "\n";
}
}

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

struct alignas(16) Vec3 {
  float x, y, z;
};

struct alignas(16) State {
  Vec3 pos;
  Vec3 vel;
  Vec3 acc;
  Vec3 jerk;

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
    look_ahead_sub = create_subscription<airstack_msgs::msg::Odometry>("/robot_1/trajectory_controller/look_ahead", 10,
								       std::bind(&DisparitySphereRenderer::look_ahead_callback,
										 this, std::placeholders::_1));
    tf_buffer = new tf2_ros::Buffer(get_clock());
    tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    rendered_pub_ = create_publisher<sensor_msgs::msg::Image>("/disparity_spheres_image", 10);
    fg_bg_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("fg_bg_cloud", 10);

    

    target_frame = airstack::get_param(this, "target_frame", std::string("map"));
    look_ahead_frame = airstack::get_param(this, "look_ahead_frame", std::string("look_ahead_point"));
    graph_nodes = airstack::get_param(this, "graph_nodes", 5);
    dt = airstack::get_param(this, "dt", 0.2);
    ht = airstack::get_param(this, "ht", 2.0);
    
    total_layers = graph_nodes*2;
    look_ahead_valid = false;
    opengl_inited = false;

    timer = rclcpp::create_timer(this, get_clock(), rclcpp::Duration::from_seconds(1./5.),
				 std::bind(&DisparitySphereRenderer::timer_callback, this));
  }

  ~DisparitySphereRenderer() {
    glfwTerminate();
  }

private:
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr look_ahead_sub;
  tf2_ros::TransformListener* tf_listener;
  tf2_ros::Buffer* tf_buffer;
  
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rendered_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fg_bg_cloud_pub_;

  rclcpp::TimerBase::SharedPtr timer;
  
  pcl::PointCloud<pcl::PointXYZI> fg_bg_cloud;

  bool opengl_inited;
  
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

  bool look_ahead_valid;
  airstack_msgs::msg::Odometry look_ahead;
  std::string look_ahead_frame;
  std::vector<TrajectoryParams> traj_params;
  float dt, ht;
  GLuint common_ubo, params_ssbo, output_ssbo;

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

    // trajectory generation init
    for(float y = 0.f; y < 360.f; y += 15.f){
      float yaw = y*M_PI/180.f;

      TrajectoryParams params;
      params.vel_desired[0] = sin(yaw);
      params.vel_desired[1] = cos(yaw);
      params.vel_desired[2] = 0.f;
      params.vel_max = 2.f;

      traj_params.push_back(params);
    }
    
    glGenBuffers(1, &common_ubo);
    glBindBuffer(GL_UNIFORM_BUFFER, common_ubo);
    glBufferData(GL_UNIFORM_BUFFER, sizeof(CommonInit), nullptr, GL_DYNAMIC_DRAW);

    CommonInit ci;
    std::cout << "c1" << std::endl;
    glBindBuffer(GL_UNIFORM_BUFFER, common_ubo);
    std::cout << "c2" << std::endl;
    CommonInit* ptr = (CommonInit*)glMapBufferRange(GL_UNIFORM_BUFFER, 0, sizeof(CommonInit),
						    GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    std::cout << "c3 " << ptr << std::endl;
    
    *ptr = ci;
    std::cout << "c4" << std::endl;
    glUnmapBuffer(GL_UNIFORM_BUFFER);
    std::cout << "c5" << std::endl;

    glGenBuffers(1, &params_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, params_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, traj_params.size()*sizeof(TrajectoryParams), traj_params.data(), GL_STATIC_DRAW);

    glGenBuffers(1, &output_ssbo);
    glBindBuffer(GL_SHADER_STORAGE_BUFFER, output_ssbo);
    glBufferData(GL_SHADER_STORAGE_BUFFER, traj_params.size()*get_traj_size()*sizeof(State), nullptr, GL_DYNAMIC_COPY);

    opengl_inited = true;
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

  void look_ahead_callback(const airstack_msgs::msg::Odometry::SharedPtr msg) {
    look_ahead = *msg;
    look_ahead_valid = true;
  }

  void timer_callback(){
    if(!look_ahead_valid || !opengl_inited)
      return;

    airstack_msgs::msg::Odometry look_ahead_odom;
    if(!tflib::transform_odometry(tf_buffer, look_ahead, look_ahead_frame, look_ahead_frame, &look_ahead_odom))
      return;
    
    CommonInit ci;
    ci.initial_state = State(look_ahead_odom);
    ci.traj_count = traj_params.size();
    ci.traj_size = get_traj_size();
    ci.dt = dt;

    glUseProgram(traj_shader);
    
    printUniformBlockLayout(traj_shader, "CommonInit");
    RCLCPP_INFO_STREAM(get_logger(), offsetof(CommonInit, initial_state) << "|" << sizeof(State) << " " << offsetof(CommonInit, traj_count) << " " << offsetof(CommonInit, traj_size) << " " << offsetof(CommonInit, dt));
    
    glBindBuffer(GL_UNIFORM_BUFFER, common_ubo);
    CommonInit* ptr = (CommonInit*)glMapBufferRange(GL_UNIFORM_BUFFER, 0, sizeof(CommonInit),
						    GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT);
    
    *ptr = ci;
    glUnmapBuffer(GL_UNIFORM_BUFFER);
    
    
    glBindBufferBase(GL_UNIFORM_BUFFER, 0, common_ubo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, params_ssbo);
    glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, output_ssbo);
    
    
    glDispatchCompute((traj_params.size() + 255) / 256, 1, 1);
    glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

    glBindBuffer(GL_SHADER_STORAGE_BUFFER, output_ssbo);
    State* output_data = (State*)glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_ONLY);
    std::vector<State> output_states(traj_params.size()*get_traj_size());
    memcpy(output_states.data(), output_data, output_states.size() * sizeof(State));
    glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
    
    RCLCPP_INFO_STREAM(get_logger(), "traj " << ci.traj_count << " " << ci.traj_size);
    for(int i = 0; i < output_states.size(); i++)
      RCLCPP_INFO_STREAM(get_logger(), "i " << output_states[i].pos.x << " " << output_states[i].pos.y << " " << output_states[i].pos.z);
    RCLCPP_INFO_STREAM(get_logger(), sizeof(Vec3) << " " << sizeof(State) << " " << sizeof(TrajectoryParams) << " " << sizeof(CommonInit));
  }

  int get_traj_size(){
    return ht/dt;
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparitySphereRenderer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
