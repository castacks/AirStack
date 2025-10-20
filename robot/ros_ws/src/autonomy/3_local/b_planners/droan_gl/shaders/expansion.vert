#version 430

layout(location=0) in vec3 aPos;
layout(location=1) in vec3 offset;

uniform mat4 proj;
uniform mat4 view;
uniform float baseline_fx;
uniform float sign;

out float disparity;
out float back_disparity;
out vec2 fragUV;

void main() {
  vec3 viewDir = sign*normalize(-offset); //camera pos - offset, but camera pos is 0,0,0
  vec3 up = vec3(0.0, 1.0, 0.0);
  vec3 right = normalize(cross(up, viewDir));
  vec3 newUp = normalize(cross(viewDir, right));
  mat3 rot = mat3(right, newUp, viewDir);
  
  vec3 viewDir_bg = -sign*normalize(-offset); //camera pos - offset, but camera pos is 0,0,0
  vec3 up_bg = vec3(0.0, 1.0, 0.0);
  vec3 right_bg = normalize(cross(up_bg, viewDir_bg));
  vec3 newUp_bg = normalize(cross(viewDir_bg, right_bg));
  mat3 rot_bg = mat3(right_bg, newUp_bg, viewDir_bg);
            
  vec4 eye_pos = view * vec4(rot*aPos + offset, 1.0);
  disparity = baseline_fx / -eye_pos.z;

  vec4 eye_pos_bg = view * vec4(rot_bg*aPos + offset, 1.0);
  back_disparity = baseline_fx / -eye_pos_bg.z;
            
  gl_Position = proj * eye_pos;
  fragUV = (gl_Position.xy / gl_Position.w) * 0.5 + 0.5;
}
