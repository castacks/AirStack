#pragma once

#include <cstdint>
#include <cmath>

namespace pid_controller {
enum AdmissionReason : uint32_t {
  DISARMED = 1u << 0, NO_CONTROL = 1u << 1,
  ARMED_RECEIPT_INVALID = 1u << 2, CONTROL_RECEIPT_INVALID = 1u << 3,
  ODOM_MISSING = 1u << 4, ODOM_BEFORE_ACTIVATION = 1u << 5,
  ODOM_RECEIPT_EXPIRED = 1u << 6, TRACKING_RECEIPT_EXPIRED = 1u << 7,
  TRACKING_FUTURE = 1u << 8, TRACKING_STALE = 1u << 9,
  ODOM_FUTURE = 1u << 10, ODOM_STALE = 1u << 11,
  TRACKING_TF_FAILED = 1u << 12, ODOM_TF_FAILED = 1u << 13,
  TRACKING_VELOCITY_INVALID = 1u << 14
};
inline bool horizontal_reference_valid(double x, double y) {
  return std::isfinite(x) && std::isfinite(y);
}
inline uint32_t stamp_reasons(double age, double timeout, uint32_t future, uint32_t stale) {
  return !std::isfinite(age) ? stale : (age < 0.0 ? future : (age > timeout ? stale : 0u));
}
}  // namespace pid_controller
