// drone.cpp
#include "drone.h"
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <GLFW/glfw3.h>
#include <algorithm>
#include <iostream>

Drone::Drone(){
  init();
}

void Drone::init(){
  position = glm::vec3(0.f, 0.f, 0.f);
  velocity = glm::vec3(0.f, 0.f, 0.f);
  yaw = 0.f;
  pitch = 0.f;
  roll = 0.f;
  angularVelocity = glm::vec3(0.f, 0.f, 0.f);
  mass = 1.f;
  inertia = 1.f;
  offboard = false;
}

glm::vec3 Drone::getForward() const {
    return orientation * glm::vec3(0.0f, 0.0f, -1.0f);
}

glm::vec3 Drone::getUp() const {
    return orientation * glm::vec3(0.0f, 1.0f, 0.0f);
}

glm::vec3 Drone::getRight() const {
    return orientation * glm::vec3(1.0f, 0.0f, 0.0f);
}

glm::mat4 Drone::get_left_camera_mat(float baseline) {
  glm::mat4 m = glm::mat4(1.f);
  m = glm::translate(m, position);
  m *= glm::toMat4(orientation);

  float pitch = -15.f;
  float yaw = 0.f;
  float roll = 0.f;
  glm::quat qYaw = glm::angleAxis(glm::radians(yaw), glm::vec3(0, 1, 0));
  glm::quat qPitch = glm::angleAxis(glm::radians(pitch), glm::vec3(1, 0, 0));
  glm::quat qRoll = glm::angleAxis(glm::radians(roll), glm::vec3(0, 0, 1));
  glm::quat cam_orientation = qYaw * qPitch * qRoll;

  glm::vec3 cam_translation(-baseline/2., 0.3, -0.2);


  glm::vec3 camWorldPos = position +  orientation*cam_translation;
  glm::quat camWorldRot = orientation * cam_orientation;
  glm::mat4 camWorldMat = glm::translate(glm::mat4(1.0f), camWorldPos)
    * glm::mat4_cast(camWorldRot);
  return glm::inverse(camWorldMat);
}

glm::mat4 Drone::get_right_camera_mat(float baseline) {
  glm::mat4 m = glm::mat4(1.f);
  m = glm::translate(m, position);
  m *= glm::toMat4(orientation);

  float pitch = -15.f;
  float yaw = 0.f;
  float roll = 0.f;
  glm::quat qYaw = glm::angleAxis(glm::radians(yaw), glm::vec3(0, 1, 0));
  glm::quat qPitch = glm::angleAxis(glm::radians(pitch), glm::vec3(1, 0, 0));
  glm::quat qRoll = glm::angleAxis(glm::radians(roll), glm::vec3(0, 0, 1));
  glm::quat cam_orientation = qYaw * qPitch * qRoll;

  glm::vec3 cam_translation(baseline/2., 0.3, -0.2);


  glm::vec3 camWorldPos = position +  orientation*cam_translation;
  glm::quat camWorldRot = orientation * cam_orientation;
  glm::mat4 camWorldMat = glm::translate(glm::mat4(1.0f), camWorldPos)
    * glm::mat4_cast(camWorldRot);
  return glm::inverse(camWorldMat);
}

void Drone::applyInput(GLFWwindow* window, float deltaTime) {
    if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS)
      init();
    
    if(offboard)
      return;
    
    constexpr float pitchRollSpeed = 45.0f; // degrees per second max change
    constexpr float maxPitch = 30.0f;        // max pitch angle degrees
    constexpr float maxRoll = 45.0f;         // max roll angle degrees
    constexpr float yawSpeed = 90.0f;        // degrees per second

    // Smooth pitch control: W/S
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) {
        pitch += pitchRollSpeed * deltaTime;
    } else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {
        pitch -= pitchRollSpeed * deltaTime;
    } else {
        // Auto-level pitch back to 0 smoothly
        pitch -= pitch * 5.0f * deltaTime; 
    }
    pitch = std::clamp(pitch, -maxPitch, maxPitch);

    // Smooth roll control: A/D
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
        roll += pitchRollSpeed * deltaTime;
    } else if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) {
        roll -= pitchRollSpeed * deltaTime;
    } else {
        // Auto-level roll back to 0 smoothly
        roll -= roll * 5.0f * deltaTime;
    }
    roll = std::clamp(roll, -maxRoll, maxRoll);

    // Yaw control: Q/E
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {
        yaw -= yawSpeed * deltaTime;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS) {
        yaw += yawSpeed * deltaTime;
    }

    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
      thrustPower = -15.f;
    }
    else
      thrustPower = 0.f;

    // Update orientation from yaw, pitch, roll (in degrees converted to radians)
    glm::quat qYaw = glm::angleAxis(glm::radians(yaw), glm::vec3(0, 1, 0));
    glm::quat qPitch = glm::angleAxis(glm::radians(pitch), glm::vec3(1, 0, 0));
    glm::quat qRoll = glm::angleAxis(glm::radians(roll), glm::vec3(0, 0, 1));

    // Note: roll * pitch * yaw order for aerospace convention (ZYX) might be different, adjust if needed
    orientation = qYaw * qPitch * qRoll;

    //orientation = glm::quat(glm::vec3(glm::radians(pitch), glm::radians(yaw), glm::radians(roll))); 

    // Thrust vector is always forward (negative Z in local space)
    thrustVector = glm::vec3(0.0f, -1.0f, 0.0f);
}

void Drone::set_command(float roll, float pitch, float yaw, float thrust){
  this->roll = glm::degrees(roll);
  this->pitch = glm::degrees(pitch);
  this->yaw = glm::degrees(yaw);
  this->thrustPower = thrust;
  
  // Update orientation from yaw, pitch, roll (in degrees converted to radians)
  glm::quat qYaw = glm::angleAxis(yaw, glm::vec3(0, 1, 0));
  glm::quat qPitch = glm::angleAxis(pitch, glm::vec3(1, 0, 0));
  glm::quat qRoll = glm::angleAxis(roll, glm::vec3(0, 0, 1));

  // Note: roll * pitch * yaw order for aerospace convention (ZYX) might be different, adjust if needed
  orientation = qYaw * qPitch * qRoll;
  //orientation = glm::quat(glm::vec3(glm::radians(this->pitch), glm::radians(this->yaw), glm::radians(this->roll))); 

  // Thrust vector is always forward (negative Z in local space)
  thrustVector = glm::vec3(0.0f, -1.0f, 0.0f);
}

void Drone::update(float deltaTime) {
    // Rotate thrust vector into world space by orientation
    glm::vec3 worldThrust = orientation * thrustVector * thrustPower;

    // Gravity force (simple)
    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    //glm::vec3 gravity(0.0f, 0.0f, 0.0f);

    // Net acceleration
    glm::vec3 acceleration = (worldThrust / mass) + gravity;

    //std::cout << "thrust: " << worldThrust.x << " " << worldThrust.y << " " << worldThrust.z << std::endl;
    //std::cout << "accel: " << acceleration.x << " " << acceleration.y << " " << acceleration.z << std::endl;

    velocity += acceleration * deltaTime;
    position += velocity * deltaTime;

    if(position.y < 0.f){
      position.y = 0.f;
      
      velocity.x = 0.f;
      velocity.y = 0.f;
      velocity.z = 0.f;

      roll = 0.f;
      pitch = 0.f;
    }

    //std::cout << position.x << " " << position.y << " " << position.z << std::endl;
    
    // Damping
    //velocity *= 0.98f;
}
