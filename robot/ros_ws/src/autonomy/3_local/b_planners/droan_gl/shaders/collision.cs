#version 460 core
layout(local_size_x = 256) in;

struct State {
  vec3 pos;
  vec3 vel;
  vec3 acc;
  vec3 jerk;
  vec3 collision;
};

layout(binding = 0) uniform sampler2DArray tex_array;

layout(std430, binding = 1) buffer TrajectoryPoints {
  State points[];
};

layout(std430, binding = 2) buffer ImageTransforms {
  mat4 image_tfs[];
};


layout(std140, binding = 3) uniform CollisionInfo {
  mat4 state_tf;
  float fx, fy, cx, cy;
  float baseline;
  int width, height;
  int limit;
};

void main() {
  uint index = gl_GlobalInvocationID.x;
  
  if(index < limit){
    vec4 pos_map = state_tf*vec4(points[index].pos, 1);
    points[index].pos = pos_map.xyz;
    //points[index].collision.x = index%2 > 0.5 ? 0. : 1.;
    
    mat4 map_to_cam_tf = inverse(image_tfs[0]);
    vec4 pos_cam = map_to_cam_tf*pos_map;

    float u = pos_cam.x*fx/pos_cam.z + cx;
    float v = pos_cam.y*fy/pos_cam.z + cy;
    float disp = baseline*fx/pos_cam.z;

    float fg_disp = texelFetch(tex_array, ivec3(width-1-u, v, 0), 0).x;
    float bg_disp = texelFetch(tex_array, ivec3(width-1-u, v, 1), 0).x;

    points[index].collision.x = 1.;

    if(u >= 0 && v >=0 && u < width && v < height && disp < fg_disp && disp > bg_disp)
      points[index].collision.x = 0.;
  }
}
