// Standalone check of include/pid_controller/attitude_feedforward.hpp (no ROS):
//   g++ -std=c++17 -Iinclude test/test_attitude_feedforward.cpp -o /tmp/t && /tmp/t
#include <cassert>
#include <cmath>
#include <cstdio>

#include "pid_controller/attitude_feedforward.hpp"

using namespace pid_controller;

static void quat_from_rpy(double r, double p, double y, double q[4]) {  // ZYX, (x, y, z, w)
  const double cr = std::cos(r / 2), sr = std::sin(r / 2), cp = std::cos(p / 2), sp = std::sin(p / 2),
               cy = std::cos(y / 2), sy = std::sin(y / 2);
  q[0] = sr * cp * cy - cr * sp * sy;
  q[1] = cr * sp * cy + sr * cp * sy;
  q[2] = cr * cp * sy - sr * sp * cy;
  q[3] = cr * cp * cy + sr * sp * sy;
}

static bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) < tol; }

int main() {
  double q[4];
  // same yaw: the Euler roll / pitch come back exactly
  for (double r : {-0.4, -0.1, 0.0, 0.2, 0.5})
    for (double p : {-0.45, 0.0, 0.1, 0.3})
      for (double y : {-2.5, 0.0, 1.0, 3.0}) {
        quat_from_rpy(r, p, y, q);
        const TiltAngles t = feedforward_tilt(q[0], q[1], q[2], q[3], y);
        assert(near(t.roll, r, 1e-9) && near(t.pitch, p, 1e-9));
      }
  // pure pitch (nose down, accelerate along the reference heading) seen from a vehicle
  // yawed 90 deg left of it: the thrust now points to the vehicle's RIGHT -> positive roll
  quat_from_rpy(0.0, 0.3, 0.0, q);
  TiltAngles t = feedforward_tilt(q[0], q[1], q[2], q[3], M_PI / 2);
  assert(near(t.pitch, 0.0, 1e-9) && near(t.roll, 0.3, 1e-9));
  // 180 deg off: forward becomes backward
  t = feedforward_tilt(q[0], q[1], q[2], q[3], M_PI);
  assert(near(t.pitch, -0.3, 1e-9) && near(t.roll, 0.0, 1e-9));
  // clamp and thrust compensation
  const TiltAngles c = combine_tilt({0.3, -0.3}, {0.5, -0.1}, 0.6);
  assert(near(c.roll, 0.6) && near(c.pitch, -0.4));
  assert(near(tilt_compensated_thrust(0.5, 0.0, 0.0), 0.5));
  assert(near(tilt_compensated_thrust(0.5, std::acos(0.8), 0.0), 0.625));
  assert(near(tilt_compensated_thrust(0.5, 1.5, 0.0), 1.0));  // floored at cos 0.5
  std::puts("attitude_feedforward: all checks passed");
  return 0;
}
