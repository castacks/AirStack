#include <gtest/gtest.h>
#include <limits>
#include <pid_controller/control_state.hpp>

struct Info {
  double p=.2, i=.1, d=0., ff=0., d_alpha=0., constant=.71;
  double min=-.5, max=1., target=0., measured=0., ff_value=0.;
  double integral=0., derivative=0., error=0., dt=0.;
  double p_component=0., i_component=0., d_component=0., ff_component=0., control=0.;
};

TEST(ControlState, ConstantHeadroomPreventsWindup) {
  Info s; s.target=1.;
  for(int n=0; n<1000; ++n) pid_controller::step(s,.1);
  EXPECT_LE(s.i_component,.09 + 1e-12); EXPECT_LE(s.control,1.);
}
TEST(ControlState, UpperSaturationAllowsUnwind) {
  Info s; s.integral=5.; s.target=-.1;
  pid_controller::step(s,1.); EXPECT_DOUBLE_EQ(s.integral,4.9);
}
TEST(ControlState, LowerSaturationBlocksWorseningAndAllowsUnwind) {
  Info s; s.constant=-.4; s.integral=-5.; s.target=-1.;
  pid_controller::step(s,1.); EXPECT_DOUBLE_EQ(s.integral,-5.);
  s.target=.1; pid_controller::step(s,1.); EXPECT_DOUBLE_EQ(s.integral,-4.9);
}
TEST(ControlState, DerivativeAndFeedforwardConsumeHeadroom) {
  Info s; s.p=0.; s.d=.2; s.ff=.1; s.ff_value=1.; s.target=1.;
  pid_controller::step(s,1.); EXPECT_DOUBLE_EQ(s.integral,0.); EXPECT_DOUBLE_EQ(s.control,1.);
}
TEST(ControlState, ZeroGainClearsIntegral) {
  Info s; s.integral=5.; s.i=0.; pid_controller::step(s,.1);
  EXPECT_DOUBLE_EQ(s.integral,0.); EXPECT_DOUBLE_EQ(s.control,.71);
}
TEST(ControlState, ResetClearsHistoryWithoutChangingGains) {
  Info s; s.integral=5.; s.derivative=2.; s.error=3.; s.control=1.;
  pid_controller::reset_history(s);
  EXPECT_DOUBLE_EQ(s.integral,0.); EXPECT_DOUBLE_EQ(s.derivative,0.);
  EXPECT_DOUBLE_EQ(s.error,0.); EXPECT_DOUBLE_EQ(s.control,0.);
  EXPECT_DOUBLE_EQ(s.constant,.71); EXPECT_DOUBLE_EQ(s.i,.1);
  s.target=.1; s.d=1.; pid_controller::step(s,0.);
  EXPECT_DOUBLE_EQ(s.i_component,0.); EXPECT_DOUBLE_EQ(s.d_component,0.);
  EXPECT_DOUBLE_EQ(s.control,.73);
}
TEST(ControlState, AuthorityRequiresBothFreshInputs) {
  pid_controller::ControlAuthority a(.5); EXPECT_FALSE(a.active(0.));
  a.armed(true,1.); EXPECT_FALSE(a.active(1.));
  a.control(true,1.); EXPECT_TRUE(a.active(1.1));
  EXPECT_FALSE(a.active(1.6)); EXPECT_FALSE(a.active(.9));
  a.armed(true,2.); EXPECT_FALSE(a.active(2.));
  a.control(true,2.); EXPECT_TRUE(a.active(2.));
  a.armed(false,2.1); EXPECT_FALSE(a.active(2.1));
  a.armed(true,2.2); a.control(false,2.2); EXPECT_FALSE(a.active(2.2));
}
TEST(ControlState, ClockRollbackAndDuplicateClearHistory) {
  Info s; pid_controller::SampleClock clock;
  EXPECT_DOUBLE_EQ(clock.next(1000000000,s),0.);
  EXPECT_DOUBLE_EQ(clock.next(1100000000,s),.1);
  s.integral=5.; s.derivative=10.;
  EXPECT_DOUBLE_EQ(clock.next(900000000,s),0.);
  EXPECT_DOUBLE_EQ(s.integral,0.); EXPECT_DOUBLE_EQ(s.derivative,0.);
  s.integral=5.;
  EXPECT_DOUBLE_EQ(clock.next(900000000,s),0.); EXPECT_DOUBLE_EQ(s.integral,0.);
  clock.reset(); EXPECT_DOUBLE_EQ(clock.next(2000000000,s),0.);
}
TEST(ControlState, EqualityAcceptsIntegralAndSaturatedBaseCanUnwind) {
  Info s; s.p=0.; s.constant=.9; s.target=1.;
  pid_controller::step(s,1.); EXPECT_DOUBLE_EQ(s.i_component,.1);
  EXPECT_DOUBLE_EQ(s.control,1.);
  s.constant=2.; s.target=-1.;
  pid_controller::step(s,1.); EXPECT_DOUBLE_EQ(s.integral,0.);
  EXPECT_DOUBLE_EQ(s.control,1.);
}

TEST(Admission, StampBoundsAreUnchanged) {
  using namespace pid_controller;
  EXPECT_EQ(stamp_reasons(0., .5, TRACKING_FUTURE, TRACKING_STALE), 0u);
  EXPECT_EQ(stamp_reasons(.5, .5, TRACKING_FUTURE, TRACKING_STALE), 0u);
  EXPECT_EQ(stamp_reasons(-.03, .5, TRACKING_FUTURE, TRACKING_STALE), TRACKING_FUTURE);
  EXPECT_EQ(stamp_reasons(.500001, .5, ODOM_FUTURE, ODOM_STALE), ODOM_STALE);
  EXPECT_EQ(stamp_reasons(std::numeric_limits<double>::quiet_NaN(), .5,
                         ODOM_FUTURE, ODOM_STALE), ODOM_STALE);
}
TEST(Admission, AuthorityMaskMatchesGateAndReportsMultipleFailures) {
  using namespace pid_controller;
  ControlAuthority a(.5);
  EXPECT_EQ(a.failure_reasons(0.), DISARMED|NO_CONTROL|ARMED_RECEIPT_INVALID|CONTROL_RECEIPT_INVALID);
  a.armed(true, 1.); a.control(true,1.);
  EXPECT_TRUE(a.active(1.5)); EXPECT_EQ(a.failure_reasons(1.5), 0u);
  EXPECT_FALSE(a.active(1.500001));
  EXPECT_EQ(a.failure_reasons(1.500001), ARMED_RECEIPT_INVALID|CONTROL_RECEIPT_INVALID);
  EXPECT_EQ(a.failure_reasons(.9), ARMED_RECEIPT_INVALID|CONTROL_RECEIPT_INVALID);
  a.armed(false,2.); a.control(false,2.);
  EXPECT_EQ(a.failure_reasons(2.), DISARMED|NO_CONTROL);
}
