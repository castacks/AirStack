#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <stereo_msgs/msg/disparity_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

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


class DisparitySphereRenderer : public rclcpp::Node {
public:
  DisparitySphereRenderer()
    : Node("disparity_sphere_renderer") 
  {
    disparity_sub_ = this->create_subscription<stereo_msgs::msg::DisparityImage>(
										 "/robot_1/sensors/front_stereo/disparity", 10, 
										 std::bind(&DisparitySphereRenderer::disparity_callback, this, std::placeholders::_1));

    caminfo_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
									   "/robot_1/sensors/front_stereo/right/camera_info", 10,
									   std::bind(&DisparitySphereRenderer::camera_info_callback, this, std::placeholders::_1));

    rendered_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/disparity_spheres_image", 10);
  }

  ~DisparitySphereRenderer() {
    glfwTerminate();
  }

private:
  rclcpp::Subscription<stereo_msgs::msg::DisparityImage>::SharedPtr disparity_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rendered_pub_;

  std::mutex data_mutex_;
  cv::Mat disparity_image_;
  CameraIntrinsics intrinsics_;

  GLFWwindow* window_;
  GLuint sphere_vao_, sphere_vbo_, instance_vbo_;
  GLuint framebuffer_, color_tex_;
  GLuint shader_program_;
  Mesh sphere_mesh;

  void init_opengl() {
    /*
    if(!glfwInit()) {
      throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    window_ = glfwCreateWindow(640, 480, "Offscreen", nullptr, nullptr);
    glfwMakeContextCurrent(window_);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    //*/

    //*
    static const EGLint configAttribs[] = {
					   EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
					   EGL_BLUE_SIZE, 8,
					   EGL_GREEN_SIZE, 8,
					   EGL_RED_SIZE, 8,
					   EGL_DEPTH_SIZE, 8,
					   EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
					   EGL_NONE
    };

    static const EGLint pbufferAttribs[] = {
					    EGL_WIDTH, intrinsics_.width,
					    EGL_HEIGHT, intrinsics_.height,
					    EGL_NONE,
    };
    std::cout << "1" << std::endl;
    // 1. Initialize EGL
    EGLDisplay eglDpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    std::cout << "2" << std::endl;

    EGLint major, minor;

    eglInitialize(eglDpy, &major, &minor);
    std::cout << "3" << std::endl;

    // 2. Select an appropriate configuration
    EGLint numConfigs;
    EGLConfig eglCfg;

    eglChooseConfig(eglDpy, configAttribs, &eglCfg, 1, &numConfigs);
    std::cout << "4" << std::endl;

    // 3. Create a surface
    EGLSurface eglSurf = eglCreatePbufferSurface(eglDpy, eglCfg, 
						 pbufferAttribs);
    std::cout << "5" << std::endl;

    // 4. Bind the API
    eglBindAPI(EGL_OPENGL_API);
    std::cout << "6" << std::endl;

    // 5. Create a context and make it current
    EGLContext eglCtx = eglCreateContext(eglDpy, eglCfg, EGL_NO_CONTEXT, 
					 NULL);
    std::cout << "7" << std::endl;

    eglMakeCurrent(eglDpy, eglSurf, eglSurf, eglCtx);
    std::cout << "8" << std::endl;

    if(!gladLoadGLLoader((GLADloadproc)eglGetProcAddress))
      std::cout << "Failed to initialize GLAD!" << std::endl;
    std::cout << "9" << std::endl;
    //*/

    // from now on use your OpenGL context
    std::cout << "Renderer: " << glGetString(GL_RENDERER) << "\n";
    std::cout << "Vendor:   " << glGetString(GL_VENDOR)   << "\n";
    std::cout << "Version:  " << glGetString(GL_VERSION)  << "\n";

    // Create framebuffer
    glGenFramebuffers(1, &framebuffer_);
    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);

    glGenTextures(1, &color_tex_);
    glBindTexture(GL_TEXTURE_2D_ARRAY, color_tex_);
    glTexStorage3D(GL_TEXTURE_2D_ARRAY, 1, GL_RGBA8, intrinsics_.width, intrinsics_.height, 1);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, color_tex_, 0);

    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
      throw std::runtime_error("Framebuffer not complete");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    shader_program_ = create_shader();
  }

  GLuint create_shader() {
    const char* vs_src = R"glsl(
        #version 430
        layout(location=0) in vec3 aPos;
        layout(location=1) in vec3 offset;
        uniform mat4 proj;
        uniform mat4 view;
        void main() {
            gl_Position = proj * view * vec4(aPos + offset, 1.0);
        })glsl";

    const char* fs_src = R"glsl(
        #version 430
        out vec4 FragColor;
        void main() { FragColor = vec4(1,0,0,1); })glsl";

    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vs_src, nullptr);
    glCompileShader(vs);

    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fs_src, nullptr);
    glCompileShader(fs);

    GLuint prog = glCreateProgram();
    glAttachShader(prog, vs);
    glAttachShader(prog, fs);
    glLinkProgram(prog);

    glDeleteShader(vs);
    glDeleteShader(fs);
    return prog;
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
        vertices.push_back(mesh->mVertices[i].x*0.05);
        vertices.push_back(mesh->mVertices[i].y*0.05);
        vertices.push_back(mesh->mVertices[i].z*0.05);
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
    /*
    std::vector<float> vertices;
    const int lat = 10, lon = 10;
    for(int i=0;i<=lat;i++){
      float theta = i * M_PI / lat;
      for(int j=0;j<=lon;j++){
	float phi = j * 2*M_PI / lon;
	float x = sin(theta)*cos(phi);
	float y = cos(theta);
	float z = sin(theta)*sin(phi);
	vertices.push_back(x*0.05f);
	vertices.push_back(y*0.05f);
	vertices.push_back(z*0.05f);
      }
    }

    glGenVertexArrays(1, &sphere_vao_);
    glBindVertexArray(sphere_vao_);

    glGenBuffers(1, &sphere_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, sphere_vbo_);
    glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(0);

    glGenBuffers(1, &instance_vbo_);
    glBindBuffer(GL_ARRAY_BUFFER, instance_vbo_);
    glVertexAttribPointer(1,3,GL_FLOAT,GL_FALSE,3*sizeof(float),(void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribDivisor(1,1);
    */
  }

  void disparity_callback(const stereo_msgs::msg::DisparityImage::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg->image, "32FC1");
    disparity_image_ = cv_ptr->image.clone();

    render_spheres();
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

  void render_spheres() {
    if(!intrinsics_.valid || disparity_image_.empty()) return;

    std::vector<float> offsets;
    for(int y=0; y<disparity_image_.rows; y+=5){
      for(int x=0; x<disparity_image_.cols; x+=5){
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

    glBindFramebuffer(GL_FRAMEBUFFER, framebuffer_);
    glViewport(0,0,intrinsics_.width,intrinsics_.height);
    glClearColor(0,0,0,1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    glUseProgram(shader_program_);
    glm::mat4 proj = glm::perspective(2*atan(((float)intrinsics_.height)/(2.f*intrinsics_.fy)),
				      ((float)intrinsics_.width)/((float)intrinsics_.height),
				      0.01f, 100.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(0,0,0), glm::vec3(0,0,1), glm::vec3(0,1,0));
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "proj"), 1, GL_FALSE, &proj[0][0]);
    glUniformMatrix4fv(glGetUniformLocation(shader_program_, "view"), 1, GL_FALSE, &view[0][0]);

    //glBindVertexArray(sphere_vao_);
    //glDrawArraysInstanced(GL_TRIANGLES, 0, 121, offsets.size()/3);
    glBindVertexArray(sphere_mesh.VAO);
    glDrawElementsInstanced(GL_TRIANGLES, sphere_mesh.index_count, GL_UNSIGNED_INT, 0, offsets.size()/3);

    
    publish_texture();
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }

  void publish_texture() {
    glBindTexture(GL_TEXTURE_2D_ARRAY, color_tex_);
    cv::Mat image(intrinsics_.height,intrinsics_.width,CV_8UC4);
    glGetTexImage(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA, GL_UNSIGNED_BYTE, image.data);

    cv_bridge::CvImage out_msg;
    out_msg.header.stamp = this->now();
    out_msg.header.frame_id = "camera_frame";
    out_msg.encoding = "rgba8";
    out_msg.image = image;

    rendered_pub_->publish(*out_msg.toImageMsg());
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DisparitySphereRenderer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
