#version 460 core
layout(local_size_x = 256) in;

struct State {
  vec3 pos;
  vec3 vel;
  vec3 acc;
  vec3 jerk;
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
  uint trajID = gl_GlobalInvocationID.x;
  TrajectoryParams p = params[trajID];

  State state = initial_state;
  
  for(int i = 0; i < traj_size; i++){
    state.pos += p.vel_desired * dt;
    state.pos.x = i;
    state.pos.y = traj_count;
    state.pos.z = traj_size;
    points[trajID * traj_size + i] = state;
  }
}
