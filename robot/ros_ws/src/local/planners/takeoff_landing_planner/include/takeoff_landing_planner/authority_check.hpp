#pragma once

#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace authority_check {
using Clock = std::chrono::steady_clock;
// Bits correspond to the six original guard conjuncts, in order.
struct Snapshot {
  bool armed, control;
  Clock::time_point armed_received, control_received, now, after;
  double max_age_s;
  unsigned reason_mask;
  bool valid() const { return reason_mask == 0; }
};
inline Snapshot evaluate(bool armed, bool control, Clock::time_point armed_received,
  Clock::time_point control_received, Clock::time_point now, Clock::time_point after,
  double max_age_s)
{
  unsigned reasons = 0;
  if (!armed) reasons |= 1;
  if (!control) reasons |= 2;
  if (!(armed_received > after)) reasons |= 4;
  if (!(control_received > after)) reasons |= 8;
  if (!(std::chrono::duration<double>(now - armed_received).count() <= max_age_s)) reasons |= 16;
  if (!(std::chrono::duration<double>(now - control_received).count() <= max_age_s)) reasons |= 32;
  return {armed, control, armed_received, control_received, now, after, max_age_s, reasons};
}
inline std::string json(const Snapshot &s, const char *kind, const char *phase)
{
  std::ostringstream out;
  out << std::setprecision(17) << std::boolalpha;
  auto number = [&out](double value) {
    if (std::isfinite(value)) out << value; else out << "null";
  };
  out << "{\"schema\":\"takeoff-authority/v1\",\"record_kind\":\"" << kind
      << "\",\"phase\":\"" << phase << "\",\"armed\":" << s.armed
      << ",\"has_control\":" << s.control << ",\"valid\":" << s.valid()
      << ",\"reason_mask\":" << s.reason_mask
      << ",\"armed_receipt_seen\":" << (s.armed_received != Clock::time_point{})
      << ",\"control_receipt_seen\":" << (s.control_received != Clock::time_point{})
      << ",\"armed_age_s\":";
  if (s.armed_received == Clock::time_point{}) out << "null";
  else number(std::chrono::duration<double>(s.now - s.armed_received).count());
  out << ",\"control_age_s\":";
  if (s.control_received == Clock::time_point{}) out << "null";
  else number(std::chrono::duration<double>(s.now - s.control_received).count());
  out << ",\"armed_received_steady_s\":";
  if (s.armed_received == Clock::time_point{}) out << "null";
  else number(std::chrono::duration<double>(s.armed_received.time_since_epoch()).count());
  out << ",\"control_received_steady_s\":";
  if (s.control_received == Clock::time_point{}) out << "null";
  else number(std::chrono::duration<double>(s.control_received.time_since_epoch()).count());
  out << ",\"steady_now_s\":"; number(std::chrono::duration<double>(s.now.time_since_epoch()).count());
  out << ",\"required_after_s\":"; number(std::chrono::duration<double>(s.after.time_since_epoch()).count());
  out << ",\"max_age_s\":"; number(s.max_age_s);
  out << "}";
  return out.str();
}
}  // namespace authority_check
