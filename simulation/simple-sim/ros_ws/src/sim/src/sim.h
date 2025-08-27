// main.cpp
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include <iostream>
#include <chrono>

#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include "drone.h"
#include "level.h"
#include "shader_utils.h"
#include "mesh_fbx.h"
#include "mesh_collada.h"

class Sim {
private:
  
  float sim_time;
  bool fast_mode;
  
  float image_fov;
  int image_width;
  int image_height;
  float image_width_f;
  float image_height_f;
  float baseline;

  GLFWwindow* window;

  GLuint VBO, VAO;
  GLuint shaderProgram;
  GLuint fbx_shader_program;
  GLuint collada_shader_program;
  GLuint texture;
  GLuint fbo, tex, rbo;

  float lastFrame;
  bool render_fbx;
  FBXMesh* world_mesh;
  ColladaMesh* drone_mesh;
  
public:
  Drone drone;
  int screen_width, screen_height;
  
  Sim(float image_fov, int image_width, int image_height, float baseline, std::string model_filename,
      float model_scale, float model_x_offset, float model_y_offset, float model_z_offset, int screen_width, int screen_height);
  ~Sim();
  float step(std::vector<unsigned char>& left_image_bytes, std::vector<unsigned char>& right_image_bytes);

  float get_fx(){
    float fovx = 2 * atan(tan(glm::radians(image_fov) / 2.) * (image_width_f/image_height_f));
    return (image_width_f / 2.0) / tan(fovx / 2.);
    //return get_fy() * (image_width_f / image_height_f);
  }
  
  float get_fy(){
    return (image_height_f / 2.0) / tan(glm::radians(image_fov) / 2.);
    //return image_height_f / (2.0f * std::tan(glm::radians(image_fov) / 2.0f));
  }
  
  float get_cx(){
    return image_width_f/2.f;
  }
  
  float get_cy(){
    return image_height_f/2.f;
  }

  void increment_time(float dt);
  void set_fast_mode(bool fast_mode);
  
};


void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
  //glViewport(0, 0, width, height);
  Sim* sim = static_cast<Sim*>(glfwGetWindowUserPointer(window));
  sim->screen_width = width;
  sim->screen_height = height;
}


Sim::Sim(float image_fov, int image_width, int image_height, float baseline, std::string model_filename,
	 float model_scale, float model_x_offset, float model_y_offset, float model_z_offset, int screen_width, int screen_height)
  : image_fov(image_fov)
  , image_width(image_width)
  , image_height(image_height)
  , baseline(baseline)
  , sim_time(0.f)
  , fast_mode(false)
  , screen_width(screen_width)
  , screen_height(screen_height){
  
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  image_width_f = image_width;
  image_height_f = image_height;

  window = glfwCreateWindow(screen_width, screen_height, "Drone Simulation", NULL, NULL);
  if (!window) {
    std::cerr << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    exit(1);
  }
  glfwSetWindowUserPointer(window, this);
  glfwMakeContextCurrent(window);
  glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    exit(1);
  }

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_CULL_FACE);

  glGenVertexArrays(1, &VAO);
  glGenBuffers(1, &VBO);

  glBindVertexArray(VAO);

  glBindBuffer(GL_ARRAY_BUFFER, VBO);
  glBufferData(GL_ARRAY_BUFFER, sizeof(cubeVertices), cubeVertices, GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
  glEnableVertexAttribArray(0);

  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  shaderProgram = createShaderProgram("vertex_shader.glsl", "fragment_shader.glsl");
  fbx_shader_program = createShaderProgram("fbx_vertex.glsl", "fbx_fragment.glsl");
  collada_shader_program = createShaderProgram("collada_vertex.glsl", "collada_fragment.glsl");

  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  /*
  int width, height, nrChannels;
  unsigned char* data = stbi_load("crate.jpg", &width, &height, &nrChannels, 0);
  if (data) {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
    glGenerateMipmap(GL_TEXTURE_2D);
  } else {
    std::cerr << "Failed to load texture" << std::endl;
  }
  stbi_image_free(data);
  */
  // framebuffer
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);

  // color
  glGenTextures(1, &tex);
  glBindTexture(GL_TEXTURE_2D, tex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0, GL_RGB, GL_UNSIGNED_BYTE, NULL);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);


  // depth
  glGenRenderbuffers(1, &rbo);
  glBindRenderbuffer(GL_RENDERBUFFER, rbo);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, image_width, image_height);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);
    
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    std::cerr << "Framebuffer not complete!\n";
    exit(1);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDrawBuffer(GL_BACK);
    
  lastFrame = 0.0f;
  render_fbx = true;

  world_mesh = NULL;
  if(render_fbx){
    auto start = std::chrono::high_resolution_clock::now();
    world_mesh = new FBXMesh(model_filename, model_scale, model_x_offset, model_y_offset, model_z_offset);
    double elapsed_seconds = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
    std::cout << "FBX load time: " << elapsed_seconds << " seconds" << std::endl;
  }
    //world_mesh = new FBXMesh("/data/fire_academy/fire_academy_no_box.fbx", 0.01);
  
  auto start = std::chrono::high_resolution_clock::now();
  drone_mesh = new ColladaMesh("/models/model.dae");
  double elapsed_seconds = std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start).count();
  std::cout << "Collada load time: " << elapsed_seconds << " seconds" << std::endl;
}

float Sim::step(std::vector<unsigned char>& left_image_bytes, std::vector<unsigned char>& right_image_bytes){
  glfwPollEvents();
  float currentFrame = glfwGetTime();
  float deltaTime = currentFrame - lastFrame;
  lastFrame = currentFrame;
  drone.applyInput(window, deltaTime);
  if(drone.paused)
    return -1.f;
  sim_time += 1./60.;//deltaTime; // TODO have a fixed dt and multiply an int by that
  

  //std::cout << (1./deltaTime) << std::endl;
  glViewport(0, 0, screen_width, screen_height);

  drone.update(deltaTime);

  glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glm::vec3 camOffset(0, 1, 2);
  glm::mat4 rot = glm::rotate(glm::mat4(1.0f), glm::radians(drone.yaw), glm::vec3(0,1,0));
  glm::vec3 camPos = drone.position + glm::vec3(rot * glm::vec4(camOffset, 0.0f));
  glm::mat4 view = glm::lookAt(camPos, drone.position, glm::vec3(0,1,0));
	
  //glm::mat4 view = glm::lookAt(drone.position, drone.position + drone.getForward(), glm::vec3(0.0f, 1.0f, 0.0f));
  glm::mat4 projection = glm::perspective(glm::radians(image_fov), ((float)screen_width) / ((float)screen_height), 0.1f, 10000.0f);
  glm::mat4 model = glm::mat4(1.0f);

  /*
  model = glm::inverse(drone.get_left_camera_mat(baseline));
  model = glm::scale(model, glm::vec3(0.1f));
  glUseProgram(shaderProgram);
  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
  glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
  glBindVertexArray(VAO);
  glBindTexture(GL_TEXTURE_2D, texture);
  glDrawArrays(GL_TRIANGLES, 0, 36);
  model = glm::mat4(1.0f);
  */
  
  if(render_fbx){
    glUseProgram(fbx_shader_program);
    glUniformMatrix4fv(glGetUniformLocation(fbx_shader_program, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(fbx_shader_program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
    glUniformMatrix4fv(glGetUniformLocation(fbx_shader_program, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform1i(glGetUniformLocation(fbx_shader_program,"diffuse"),0);
    world_mesh->draw();
  }
	
  //model = glm::scale(glm::mat4(1.0f), glm::vec3(10.0f));
  model = glm::translate(model, drone.position);
  model *= glm::toMat4(drone.orientation);
  //model = glm::rotate(model, glm::radians(-drone.yaw), glm::vec3(0,1,0));
  //model = glm::scale(model, glm::vec3(10.0f));
  glUseProgram(collada_shader_program);
  glUniformMatrix4fv(glGetUniformLocation(collada_shader_program, "view"), 1, GL_FALSE, glm::value_ptr(view));
  glUniformMatrix4fv(glGetUniformLocation(collada_shader_program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
  glUniformMatrix4fv(glGetUniformLocation(collada_shader_program, "model"), 1, GL_FALSE, glm::value_ptr(model));
  drone_mesh->draw_except(51);
  glm::mat4 spin_model = model;
  if(drone.armed || !drone.on_ground)
     spin_model *= glm::toMat4(glm::angleAxis(sim_time*1000.f, glm::vec3(0, 1, 0)));
  glUniformMatrix4fv(glGetUniformLocation(collada_shader_program, "model"), 1, GL_FALSE, glm::value_ptr(spin_model));
  drone_mesh->draw_only(51);


  // offscreen
  for(int i = 0; (i < 2) && !fast_mode; i++){
    glViewport(0, 0, image_width, image_height);
    projection = glm::perspective(glm::radians(image_fov), image_width_f / image_height_f, 0.1f, 10000.0f);
    
    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    model = glm::mat4(1.0f);

    if(render_fbx){
      glUseProgram(fbx_shader_program);
      if(i == 0)
	view = drone.get_left_camera_mat(baseline);
      else
	view = drone.get_right_camera_mat(baseline);
      glUniformMatrix4fv(glGetUniformLocation(fbx_shader_program, "view"), 1, GL_FALSE, glm::value_ptr(view));
      glUniformMatrix4fv(glGetUniformLocation(fbx_shader_program, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
      glUniformMatrix4fv(glGetUniformLocation(fbx_shader_program, "model"), 1, GL_FALSE, glm::value_ptr(model));
      glUniform1i(glGetUniformLocation(fbx_shader_program,"diffuse"),0);
      world_mesh->draw();
    }

    // save image
    std::vector<unsigned char> pixels(image_width * image_height * 3);
    glReadPixels(0, 0, image_width, image_height, GL_RGB, GL_UNSIGNED_BYTE, pixels.data());
    //std::vector<unsigned char> flipped(image_width * image_height * 3);
    if(i == 0)
      left_image_bytes.resize(image_width * image_height * 3);
    else
      right_image_bytes.resize(image_width * image_height * 3);
    for (int y = 0; y < image_height; ++y)
      if(i == 0)
	memcpy(&left_image_bytes[y * image_width * 3], &pixels[(image_height - 1 - y) * image_width * 3], image_width * 3);
      else
	memcpy(&right_image_bytes[y * image_width * 3], &pixels[(image_height - 1 - y) * image_width * 3], image_width * 3);
      //memcpy(&flipped[y * image_width * 3], &pixels[(image_height - 1 - y) * image_width * 3], image_width * 3);
    /*
    std::string output_filename = std::string("output") + std::to_string(i) + ".png";
    if(i==0)
      stbi_write_png(output_filename.c_str(), image_width, image_height, 3, left_image_bytes.data(), image_width * 3);
    else
      stbi_write_png(output_filename.c_str(), image_width, image_height, 3, right_image_bytes.data(), image_width * 3);
    */
  }
	
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDrawBuffer(GL_BACK);

  glfwSwapBuffers(window);
	
  //return !glfwWindowShouldClose(window);
  return sim_time;
}

void Sim::increment_time(float dt){
  sim_time += dt;
}
void Sim::set_fast_mode(bool fast_mode){
  this->fast_mode = fast_mode;

  if(fast_mode)
    glfwSwapInterval(0);
  else
    glfwSwapInterval(1);
}

Sim::~Sim(){
  glDeleteVertexArrays(1, &VAO);
  glDeleteBuffers(1, &VBO);
  glDeleteProgram(shaderProgram);
  
  glfwTerminate();
}
