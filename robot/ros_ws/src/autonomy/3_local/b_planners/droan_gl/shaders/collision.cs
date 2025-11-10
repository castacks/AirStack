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
  float scale;
  float expansion_radius;
  int graph_nodes;
};

void main() {
  uint index = gl_GlobalInvocationID.x;
  
  if(index < limit){
    vec4 pos_map = state_tf*vec4(points[index].pos, 1);
    points[index].pos = pos_map.xyz;
    
    points[index].collision.x = 1.;
    for(int i = 0; i < graph_nodes; i++){
      mat4 map_to_cam_tf = inverse(image_tfs[i]);
      vec4 pos_cam = map_to_cam_tf*pos_map;

      float u = pos_cam.x*fx/pos_cam.z + cx;
      float v = pos_cam.y*fy/pos_cam.z + cy;
      float disp = baseline*fx/pos_cam.z;
    
      float fg_depth = baseline*fx/(float(imageLoad(tex_array, ivec3(u, v, i)).x)/scale);
      float diff = pos_cam.z - fg_depth;
      
      if(u >= 0 && v >= 0 && u < width && v < height && diff > 0 && diff < (2*expansion_radius)){
	points[index].collision.x = 0.;
	//break;
      }
    }
  }
}
