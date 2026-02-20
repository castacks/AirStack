#version 450 core
layout(local_size_x = 256) in;

struct State {
  vec3 pos;
  vec3 vel;
  vec3 acc;
  vec3 jerk;
  vec3 collision;
};

struct TrajectoryPoint {
  vec4 v1;
  vec4 v2;
};

struct TrajectoryParams {
  vec3 vel_desired;
  float vel_max;
};

layout(std140, binding = 0) uniform CommonInit {
  State initial_state;
};

layout(std430, binding = 1) buffer ParamsBuffer {
  TrajectoryParams params[];
};

layout(std430, binding = 2) buffer OutputBuffer {
  TrajectoryPoint points[];
};

uniform int traj_count;
uniform int traj_size;
uniform float dt;
uniform float seen_radius;

void main() {
  uint traj_index = gl_GlobalInvocationID.x;
  TrajectoryParams p = params[traj_index];
  if(traj_index >= traj_count)
    return;
  
  float k_v = 8.165;
  float k_a = 8.1096;
  float k_j = 4.027;
  float snap_lim = 100.f;
  State state = initial_state;
  vec3 initial_pos = initial_state.pos;
  
  for(int i = 0; i < traj_size; i++){
    vec3 snap = -k_v*(state.vel - p.vel_desired) - k_a*state.acc - k_j*state.jerk;
    snap = clamp(snap, -snap_lim, snap_lim);
    state.jerk += snap*dt;
    state.acc += state.jerk*dt;
    state.vel += state.acc*dt;
    state.pos += state.vel*dt;

    // use this to check if it should be seen because its close to the drone's position
    // TODO what if vel is 0?
    float is_seen = distance(state.pos, initial_pos) < seen_radius ? -1. : 1.;
    
    TrajectoryPoint tp;
    tp.v1 = vec4(state.pos, 1);
    tp.v2 = vec4(is_seen, 0, 0, length(state.vel));
    points[traj_index * traj_size + i] = tp;
  }
}
