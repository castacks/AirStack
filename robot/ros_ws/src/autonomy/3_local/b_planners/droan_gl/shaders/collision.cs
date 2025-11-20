#version 460 core
layout(local_size_x = 256) in;

struct State {
  vec3 pos;
  vec3 vel;
  vec3 acc;
  vec3 jerk;
  vec3 collision;
};

layout(binding = 0, r32i) uniform iimage2DArray tex_array;

layout(std430, binding = 1) buffer TrajectoryPoints {
  vec4 points[];
};

layout(std430, binding = 2) buffer ImageTransforms {
  mat4 image_tfs[];
};

uniform mat4 state_tf;
uniform float fx;
uniform float fy;
uniform float cx;
uniform float cy;
uniform float baseline;
uniform int width, height;
uniform int limit;
uniform float scale;
uniform float expansion_radius;
uniform int graph_nodes;

void main() {
  uint index = gl_GlobalInvocationID.x;
  if(index >= limit)
    return;
  
  vec4 pos_map = state_tf*points[index];
  
  float collision = 0.;
  for(int i = 0; i < graph_nodes; i++){
    mat4 map_to_cam_tf = image_tfs[i];
    vec4 pos_cam = map_to_cam_tf*pos_map;

    if(pos_cam.z < 0.){
      collision += 1000.;
      break;
    }
    
    float u = pos_cam.x*fx/pos_cam.z + cx;
    float v = pos_cam.y*fy/pos_cam.z + cy;
    float disp = baseline*fx/pos_cam.z*scale;
    
    //float fg_depth = baseline*fx/(float(imageLoad(tex_array, ivec3(u, v, 2*i)).x));
    //float diff = pos_cam.z - fg_depth;
    
    bool in_image_bounds = u >= 0 && v >= 0 && u < width && v < height;
    if(!in_image_bounds){
      collision += 1000.;
      break;
    }
    
    float fg_disp = float(imageLoad(tex_array, ivec3(u, v, 2*i)).x);
    float bg_disp = float(imageLoad(tex_array, ivec3(u, v, 2*i+1)).x);
    
    //if(u >= 0 && v >= 0 && u < width && v < height && diff > 0 /* && diff < (2*expansion_radius)*/){
    
    // unseen
    if(!in_image_bounds || pos_cam.z < 0. || (in_image_bounds && bg_disp < 2000000000. && disp < bg_disp))
      collision += 1000.;
    // collision
    else if(in_image_bounds && disp < fg_disp && disp > bg_disp)
      collision += 1.;
    // seen
    else
      collision += 1000000.;
  }
  
  points[index] = vec4(pos_map.xyz, collision);
}
