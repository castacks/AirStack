#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <pid_controller/admission.hpp>

namespace pid_controller {

// Works with PIDInfo; no ROS runtime is needed to exercise the control math.
template<class Info>
void reset_history(Info &s) {
  s.integral = s.derivative = s.error = s.dt = 0.0;
  s.p_component = s.i_component = s.d_component = s.ff_component = 0.0;
  s.control = 0.0;
}

template<class Info>
double step(Info &s, double dt) {
  const double previous_error = s.error;
  s.error = s.target - s.measured;
  s.dt = dt;
  s.p_component = s.p * s.error;
  s.derivative = dt > 0.0
    ? s.d_alpha * s.derivative + (1.0 - s.d_alpha) * (s.error - previous_error) / dt
    : 0.0;
  s.d_component = s.d * s.derivative;
  s.ff_component = s.ff * s.ff_value;
  const double base = s.p_component + s.d_component + s.ff_component + s.constant;
  if (s.i == 0.0) {
    s.integral = 0.0;
  } else if (dt > 0.0) {
    const double increment = s.i * s.error * dt;
    const double candidate = base + s.i * s.integral + increment;
    // Freeze only increments that drive further into final-output saturation.
    // An increment in the opposite direction must be allowed to unwind.
    if (!((candidate > s.max && increment > 0.0) ||
          (candidate < s.min && increment < 0.0))) {
      s.integral += s.error * dt;
    }
  }
  s.i_component = s.i * s.integral;
  s.control = std::clamp(base + s.i_component, s.min, s.max);
  return s.control;
}

class SampleClock {
 public:
  void reset() { initialized_ = false; }
  template<class Info>
  double next(int64_t now_ns, Info &s) {
    double dt = initialized_ ? static_cast<double>(now_ns - previous_ns_) / 1e9 : 0.0;
    if (initialized_ && dt <= 0.0) {
      reset_history(s);
      dt = 0.0;
    }
    initialized_ = true;
    previous_ns_ = now_ns;
    return dt;
  }
 private:
  bool initialized_ = false;
  int64_t previous_ns_ = 0;
};

class ControlAuthority {
 public:
  explicit ControlAuthority(double timeout) : timeout_(timeout) {}
  void armed(bool value, double now) { armed_ = value; armed_at_ = now; }
  void control(bool value, double now) { control_ = value; control_at_ = now; }
  bool active(double now) const {
    return armed_ && control_ && fresh(armed_at_, now) && fresh(control_at_, now);
  }
  uint32_t failure_reasons(double now) const {
    return (!armed_ ? DISARMED : 0u) | (!control_ ? NO_CONTROL : 0u) |
      (!fresh(armed_at_, now) ? ARMED_RECEIPT_INVALID : 0u) |
      (!fresh(control_at_, now) ? CONTROL_RECEIPT_INVALID : 0u);
  }
  double armed_age(double now) const { return now - armed_at_; }
  double control_age(double now) const { return now - control_at_; }
 private:
  bool fresh(double stamp, double now) const {
    return std::isfinite(stamp) && now >= stamp && now - stamp <= timeout_;
  }
  double timeout_;
  bool armed_ = false, control_ = false;
  double armed_at_ = -INFINITY, control_at_ = -INFINITY;
};
}  // namespace pid_controller
