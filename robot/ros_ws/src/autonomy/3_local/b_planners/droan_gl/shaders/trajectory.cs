#version 460 core
layout(local_size_x = 256) in;

struct InitConditions {
  vec3 pos;
  vec3 vel;
  vec3 acc;
  vec3 jerk;
};

struct TrajectoryParams {
  vec3 desiredVel;
  vec3 desiredAcc;
  float weight;
  int id;
};

struct TrajectoryPoint {
  vec3 pos;
  float t;
};

layout(std140, binding = 0) uniform Common {
  InitConditions init;
  int numPoints;
  float dt;
};

layout(std430, binding = 1) buffer ParamsBuffer {
  TrajectoryParams params[];
};

layout(std430, binding = 2) buffer OutputBuffer {
  TrajectoryPoint points[];
};

void main() {
  uint trajID = gl_GlobalInvocationID.x;
  TrajectoryParams p = params[trajID];

  for (int i = 0; i < numPoints; ++i) {
    float t = float(i) * dt;

    vec3 pos = init.pos
      + init.vel * t
      + 0.5 * init.acc * t * t
      + (1.0 / 6.0) * init.jerk * t * t * t;

    // Add influence from desired velocity/acceleration
    pos += p.desiredVel * t * 0.1;
    pos += 0.5 * p.desiredAcc * t * t * 0.05;

    points[trajID * numPoints + i].pos = pos;
    points[trajID * numPoints + i].t   = t;
  }
}
