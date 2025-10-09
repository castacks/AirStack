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
  
  for(int i = 0; i < traj_size; i++){
    vec3 snap = -k_v*(state.vel - p.vel_desired) - k_a*state.acc - k_j*state.jerk;
    snap = clamp(snap, -snap_lim, snap_lim);
    state.jerk += snap*dt;
    state.acc += state.jerk*dt;
    state.vel += state.acc*dt;
    state.pos += state.vel*dt;

    // TODO remove
    state.collision = vec3(0, 0, 0);
    
    points[traj_index * traj_size + i] = state;
  }
}
