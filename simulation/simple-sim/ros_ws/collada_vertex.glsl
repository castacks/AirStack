#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 FragPos;
out vec3 Normal;
out vec3 viewPos;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main() {
  FragPos = aPos;
  Normal = aNormal;
  viewPos = vec3(view[3][3], -view[3][2], -view[3][1]);
  gl_Position = projection * view * model * vec4(aPos, 1.0);
}
