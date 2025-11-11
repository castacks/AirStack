#version 460 core
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

layout(std140, binding = 0) uniform CommonInit {
  State initial_state;
  int traj_count;
  int traj_size;
  float dt;
};

layout(std430, binding = 1) buffer ParamsBuffer {
  TrajectoryParams params[];
};

layout(std430, binding = 2) buffer OutputBuffer {
  State points[];
};

void main() {
  uint traj_index = gl_GlobalInvocationID.x;
  TrajectoryParams p = params[traj_index];
  
  float k_v = 8.165;
  float k_a = 8.1096;
  float k_j = 4.027;
  float snap_lim = 100.f;
  
  State state = initial_state;
  //vec3 prev_pos = state.pos;
  //vec3 up = vec3(0, 0, 1);
  //float scale = 1;
  
  //for(int i = 0; i < traj_size/5; i++){
  for(int i = 0; i < traj_size; i++){
    vec3 snap = -k_v*(state.vel - p.vel_desired) - k_a*state.acc - k_j*state.jerk;
    snap = clamp(snap, -snap_lim, snap_lim);
    state.jerk += snap*dt;
    state.acc += state.jerk*dt;
    state.vel += state.acc*dt;
    state.pos += state.vel*dt;

    points[traj_index * traj_size + i] = state;
    
    /*
    vec3 dir = normalize(state.pos - prev_pos);
    vec3 pos_up = state.pos + scale*up;
    vec3 pos_down = state.pos - scale*up;
    vec3 x = scale*cross(dir, up);
    vec3 pos_left = state.pos + x;
    vec3 pos_right = state.pos - x;
    
    // TODO remove
    state.collision = vec3(0, 0, 0);
    
    prev_pos = state.pos;
    points[traj_index * traj_size + i*5] = state;
    state.pos = pos_up;
    points[traj_index * traj_size + i*5+1] = state;
    state.pos = pos_down;
    points[traj_index * traj_size + i*5+2] = state;
    state.pos = pos_left;
    points[traj_index * traj_size + i*5+3] = state;
    state.pos = pos_right;
    points[traj_index * traj_size + i*5+4] = state;
    state.pos = prev_pos;
    */
  }
}
