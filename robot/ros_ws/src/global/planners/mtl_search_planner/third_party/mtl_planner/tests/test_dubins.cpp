// Dubins paths: the geometry every measured route length rests on.
#include <cmath>

#include "mtl/core/dubins.hpp"
#include "mtl/core/numeric.hpp"
#include "test_util.hpp"

using namespace mtl;
using namespace mtl::core;

int main() {
    // --- a straight shot is exactly the straight-line distance -------------
    {
        const DubinsPath p = DubinsPath::connect(Pose2{Vec2(0, 0), 0.0}, Pose2{Vec2(100, 0), 0.0}, 50.0);
        CHECK_NEAR(p.length(), 100.0, 1e-9);
        CHECK_NEAR(p.interpolate(50.0).xy.x(), 50.0, 1e-9);
        CHECK_NEAR(p.interpolate(50.0).xy.y(), 0.0, 1e-9);
    }

    // --- never shorter than the straight line, never below the lower bound --
    {
        const double R = 40.0;
        for (double ang = 0.0; ang < 6.2; ang += 0.37) {
            for (double d = 10.0; d < 400.0; d += 57.0) {
                const Pose2 a{Vec2(0, 0), 0.0};
                const Pose2 b{Vec2(d * std::cos(ang), d * std::sin(ang)), ang};
                const DubinsPath p = DubinsPath::connect(a, b, R);
                CHECK(p.valid());
                CHECK(p.length() >= (b.xy - a.xy).norm() - 1e-9);

                // The sampled endpoint must land on the requested pose.
                const Pose2 e = p.interpolate(p.length());
                CHECK_NEAR(e.xy.x(), b.xy.x(), 1e-6);
                CHECK_NEAR(e.xy.y(), b.xy.y(), 1e-6);
                CHECK_NEAR(std::abs(wrapPi(e.theta - b.theta)), 0.0, 1e-6);
            }
        }
    }

    // --- a U-turn in place costs pi*R, the classic RSR/LSL bound -----------
    {
        const double R = 25.0;
        const DubinsPath p =
            DubinsPath::connect(Pose2{Vec2(0, 0), 0.0}, Pose2{Vec2(0, 2 * R), kPi}, R);
        CHECK_NEAR(p.length(), kPi * R, 1e-6);
    }

    // --- the waypoint builder: monotone arc, no duplicate samples ----------
    {
        Path2 wp(4, 2);
        wp << 0, 0, 500, 0, 500, 500, 0, 500;
        DubinsParams dop;
        dop.stepSize = 2.5;
        Path2 track;
        VecX  arc;
        computeDubinsWaypoints(wp, 100.0, dop, track, arc);

        CHECK(track.rows() > 100);
        CHECK(arc.size() == track.rows());
        for (Index i = 1; i < arc.size(); ++i) CHECK(arc(i) > arc(i - 1));
        // The arc must be at least the straight-line polyline through the
        // waypoints: turns only ever add length.
        CHECK(arc(arc.size() - 1) >= polylineLength(wp) - 1e-6);

        // dubinsLength must agree with the builder it wraps, since the budget
        // bisection compares the two.
        CHECK_NEAR(dubinsLength(wp, 100.0, dop), arc(arc.size() - 1), 1e-9);

        // Every sample steps by about the requested resolution.
        for (Index i = 1; i < track.rows(); ++i)
            CHECK((track.row(i) - track.row(i - 1)).norm() <= dop.stepSize + 1e-6);
    }

    // --- the bisector heading rule: no 360 loop at a sharp corner ----------
    // A hairpin with a 100 m radius used to produce a full circle when each
    // waypoint's heading pointed at the next one.  Half the corner angle per
    // leg keeps it under a half-turn per leg.
    {
        Path2 wp(3, 2);
        wp << 0, 0, 600, 0, 0, 40;  // out and almost straight back
        DubinsParams dop;
        Path2 track;
        VecX  arc;
        computeDubinsWaypoints(wp, 100.0, dop, track, arc);
        const double euclid = polylineLength(wp);
        // A 360 loop at the corner would add 2*pi*R = 628 m on top.
        CHECK(arc(arc.size() - 1) < euclid + 2.0 * kPi * 100.0);
    }

    return test::report("test_dubins");
}
