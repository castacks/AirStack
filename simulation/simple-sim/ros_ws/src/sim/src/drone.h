#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <GLFW/glfw3.h>

class Drone {
public:
  glm::vec3 position;
  glm::vec3 velocity;
  glm::quat orientation;

  float yaw;    // degrees
  float pitch;  // degrees
  float roll;   // degrees

  glm::vec3 thrustVector;
  float thrustPower;

  float mass;
  float inertia;
  glm::vec3 angularVelocity;

  bool offboard;

  Drone();
  void init();

  glm::vec3 getForward() const;
  glm::vec3 getUp() const;
  glm::vec3 getRight() const;

  void applyInput(GLFWwindow* window, float deltaTime);
  void update(float deltaTime);

  void set_command(float roll, float pitch, float yaw, float thrust);

  glm::mat4 get_left_camera_mat(float baseline);
  glm::mat4 get_right_camera_mat(float baseline);
};
