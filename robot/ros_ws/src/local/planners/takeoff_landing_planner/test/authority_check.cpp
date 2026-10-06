#include <takeoff_landing_planner/authority_check.hpp>
#include <cstdlib>
#include <iostream>
#include <limits>
using namespace authority_check;
int main() {
  const auto now = Clock::time_point(std::chrono::seconds(10));
  unsigned cases = 0;
  for (bool armed : {false, true}) for (bool control : {false, true})
    for (double a : {0., 9., 9.5, 9.500001, 10., 11.})
    for (double c : {0., 9., 9.5, 9.500001, 10., 11.})
    for (double after : {0., 9.5, 10.})
    for (double limit : {0., .5, 1., std::numeric_limits<double>::quiet_NaN()}) {
      const auto at = Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(a)));
      const auto ct = Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(c)));
      const auto bound = Clock::time_point(std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(after)));
      const bool original = armed && control && at > bound && ct > bound &&
        std::chrono::duration<double>(now-at).count() <= limit &&
        std::chrono::duration<double>(now-ct).count() <= limit;
      if (evaluate(armed, control, at, ct, now, bound, limit).valid() != original) return 1;
      ++cases;
    }
  const auto edge = now-std::chrono::milliseconds(500);
  if (!evaluate(true,true,edge,edge,now,{},.5).valid()) return 2;
  if (evaluate(true,true,edge-std::chrono::nanoseconds(1),edge,now,{},.5).reason_mask != 16) return 3;
  if (evaluate(true,true,edge,edge,now,edge,.5).reason_mask != 12) return 4;
  auto missing = evaluate(false,false,{}, {}, now,{},.5);
  if (missing.reason_mask != 63 || json(missing,"periodic_observation","passive").find("\"armed_age_s\":null") == std::string::npos) return 5;
  std::cout << cases << " original-guard equivalence cases; exact boundaries and missing receipts passed\n";
}
