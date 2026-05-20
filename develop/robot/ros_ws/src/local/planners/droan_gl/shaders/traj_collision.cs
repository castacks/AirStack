#version 450 core
layout(local_size_x = 256) in;

struct State {
  vec3 pos;
  vec3 vel;
  vec3 acc;
  vec3 jerk;
  vec3 collision;
};

struct TrajectoryParams {
  vec3 vel_desired;
  float vel_max;
};


layout(binding = 0, r32i) uniform iimage2DArray tex_array;

layout(std140, binding = 1) uniform CommonInit {
  State initial_state;
  int traj_count;
  int traj_size;
  float dt;
};

layout(std430, binding = 2) buffer ParamsBuffer {
  TrajectoryParams params[];
};

layout(std430, binding = 3) buffer TrajectoryPoints {
  vec4 points[];
};

layout(std430, binding = 4) buffer ImageTransforms {
  mat4 image_tfs[];
};

layout(std140, binding = 5) uniform CollisionInfo {
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
  uint traj_index = gl_GlobalInvocationID.x;
  TrajectoryParams p = params[traj_index];
  
  float k_v = 8.165;
  float k_a = 8.1096;
  float k_j = 4.027;
  float snap_lim = 100.f;
  
  State state = initial_state;

  for(int i = 0; i < traj_size; i++){
    vec3 snap = -k_v*(state.vel - p.vel_desired) - k_a*state.acc - k_j*state.jerk;
    snap = clamp(snap, -snap_lim, snap_lim);
    state.jerk += snap*dt;
    state.acc += state.jerk*dt;
    state.vel += state.acc*dt;
    state.pos += state.vel*dt;

    vec4 pos_map = state_tf*vec4(state.pos, 1);
    
    float collision = 1.;
    for(int j = 0; j < graph_nodes; j++){
      mat4 map_to_cam_tf = image_tfs[j];
      vec4 pos_cam = map_to_cam_tf*pos_map;
      
      float u = pos_cam.x*fx/pos_cam.z + cx;
      float v = pos_cam.y*fy/pos_cam.z + cy;
      float disp = baseline*fx/pos_cam.z;

      float fg_disp = float(imageLoad(tex_array, ivec3(u, v, 2*j)).x)/scale;
      float bg_disp = float(imageLoad(tex_array, ivec3(u, v, 2*j+1)).x)/scale;
      
      if(u >= 0 && v >= 0 && u < width && v < height && disp < fg_disp && disp > bg_disp){
	collision = 0.;
	break;
      }
    }
    
    points[traj_index * traj_size + i] = vec4(pos_map.xyz, collision);
  }

  /*
  for(int j = 0; j < graph_nodes; j++){
    state = initial_state;
      mat4 map_to_cam_tf = image_tfs[j];
    for(int i = 0; i < traj_size; i++){
      vec3 snap = -k_v*(state.vel - p.vel_desired) - k_a*state.acc - k_j*state.jerk;
      snap = clamp(snap, -snap_lim, snap_lim);
      state.jerk += snap*dt;
      state.acc += state.jerk*dt;
      state.vel += state.acc*dt;
      state.pos += state.vel*dt;
	
      vec4 pos_map = state_tf*vec4(state.pos, 1);
      vec4 pos_cam = map_to_cam_tf*pos_map;
	
      float u = pos_cam.x*fx/pos_cam.z + cx;
      float v = pos_cam.y*fy/pos_cam.z + cy;
      float disp = baseline*fx/pos_cam.z;
	
      float fg_disp = float(imageLoad(tex_array, ivec3(u, v, 2*j)).x)/scale;
      float bg_disp = float(imageLoad(tex_array, ivec3(u, v, 2*j+1)).x)/scale;
	
      float collision = 1.;
      if(u >= 0 && v >= 0 && u < width && v < height && disp < fg_disp && disp > bg_disp){
	collision = 0.;
      }
	
      points[traj_index * traj_size + i] = vec4(pos_map.xyz, collision);
    }
  }
  */
}
