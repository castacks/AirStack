// =============================================================================
//  mtl_search_planner/search_problem.hpp
//
//  The ROS-free adapter between an AirStack MTL scenario file and the vendored
//  mtl::planner. Nothing in here includes rclcpp, so it is unit-tested (and
//  usable from the `mtl_search_plan` CLI) without a ROS installation.
//
//  FRAMES — the one place they are converted:
//
//    mission NED   the scenario file: n North, e East, d Down [m]; origin =
//                  the Isaac world origin.
//    mtl           the planner's x East, y North over [0, mapSize]; anchored on
//                  the search area's south-west corner (Frame, identical to
//                  cpp_planner/apps/mtl_plan_json.cpp).
//    world ENU     Isaac / AirStack world: x = e, y = n, z = -d = height.
//    map           one robot's odometry frame = its PX4/MAVROS local origin =
//                  the robot's home (spawn):  p_map = p_worldENU - home_ENU.
//
//  Yaw: mtl yaw is counter-clockwise from East, which IS ENU yaw — no
//  conversion between mtl, world and map.
// =============================================================================
#ifndef MTL_SEARCH_PLANNER_SEARCH_PROBLEM_HPP
#define MTL_SEARCH_PLANNER_SEARCH_PROBLEM_HPP

#include <string>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/planner.hpp"
#include "mtl/types.hpp"
#include "mtl_search_planner/json_mini.hpp"

namespace mtl_search {

/// mission NED <-> mtl planning frame (south-west-corner anchored).
struct Frame {
    double nMin = 0.0;  ///< mission-NED north of the map's y = 0 edge
    double eMin = 0.0;  ///< mission-NED east  of the map's x = 0 edge

    mtl::Vec2 toMtl(double n, double e) const { return {e - eMin, n - nMin}; }
    void fromMtl(double x, double y, double& n, double& e) const {
        n = y + nMin;
        e = x + eMin;
    }
    /// mtl yaw is CCW from East; NED yaw is CW from North.
    static double yawToNed(double yawMtl) { return mtl::kPi / 2.0 - yawMtl; }
};

/// World ENU from mission NED.
inline mtl::Vec3 nedToEnu(double n, double e, double d = 0.0) { return {e, n, -d}; }

struct AgentSpec {
    std::string name;
    mtl::Vec2   startNed = mtl::Vec2::Zero();
    mtl::Vec2   homeNed  = mtl::Vec2::Zero();
    /// World-ENU position of this agent's map-frame origin (its home).
    mtl::Vec3 homeEnu(double up = 0.0) const { return {homeNed.y(), homeNed.x(), up}; }
};

/// Everything the planner needs, parsed once from the scenario JSON.
struct SearchProblem {
    std::string            name;
    std::uint64_t          seed = 0;
    Frame                  frame;
    mtl::PlannerParams     params;
    mtl::CellSet           cells;
    std::vector<AgentSpec> agents;
    std::vector<mtl::Vec2> startsMtl;
    jsonmini::Value        raw;   ///< the parsed file (for provenance in plan.json)

    int agentIndex(const std::string& agentName) const;  ///< -1 when absent
};

/// Parse a scenario (`mtl.scenario/1`). @throws std::runtime_error with a named cause.
SearchProblem parseScenario(const std::string& jsonText);
SearchProblem loadScenario(const std::string& path);

/// Run the planner (deterministic for a given SearchProblem).
mtl::PlanningResult solve(const SearchProblem& problem);

// -----------------------------------------------------------------------------
/// One sample of an agent's flight-ready track, in that agent's MAP frame.
struct TrackSample {
    double t = 0.0;        ///< [s] planner timeline
    double arc = 0.0;      ///< [m] flown ground-track arc length from the first sample
    double x = 0.0, y = 0.0, z = 0.0;     ///< aircraft position [m, map]
    double yaw = 0.0;      ///< [rad] ENU heading of the ground track
    double speed = 0.0;    ///< [m/s] planned ground speed at this sample
    double bx = 0.0, by = 0.0, bz = 0.0;  ///< scheduled boresight ground point [m, map]
    double roll = 0.0;     ///< [rad] planned airframe roll (fixed-wing model; advisory)
    double pitch = 0.0;    ///< [rad] planned airframe pitch (the scheduler's nudge)
    double gimbalPhi = 0.0;///< [rad] planned 1-DOF cross-track gimbal angle (+ right)
};

struct AgentTrack {
    std::string name;
    int         index = -1;
    mtl::Vec3   homeEnu = mtl::Vec3::Zero();   ///< world ENU of the map origin
    std::vector<TrackSample> samples;
    std::vector<long> servicedCells;  ///< global cells the gimbal is scheduled to observe
    std::vector<long> plannedCells;   ///< cells the route was drawn through
    double budget = 0.0;
    double flownLength = 0.0;
    double flightTime = 0.0;
    bool   feasible = true;
    bool   scheduled = false;  ///< single-axis gimbal scheduler ran
    bool   singleAxis = true;
    double tilt = 0.0;         ///< [rad] forward mount tilt
    double fov = 0.0;          ///< [rad] full cone
    double speedMps = 0.0;
    double minTurnRadius = 0.0;
    double altitude = 0.0;
    double dt = 0.1;
    double gimbalMax = 0.0;    ///< [rad]
    double gimbalRate = 0.0;   ///< [rad/s]
    double pitchNudgeMax = 0.0;///< [rad]
    std::string routeNote, extensionNote;

    double totalArc() const { return samples.empty() ? 0.0 : samples.back().arc; }
};

/// Extract agent `agentIndex` from a team result, convert it to that agent's
/// map frame (`homeUp` = height of the map origin above the ground plane) and
/// trim the hover padding the team timeline appends after it finishes.
AgentTrack buildAgentTrack(const mtl::PlanningResult& result, const SearchProblem& problem,
                           int agentIndex, double homeUp = 0.0);

/// The whole team plan as `mtl.plan/1` JSON (mission NED), the interchange the
/// MTL testbed and MATLAB exporter share — so a flown AirStack plan can be
/// replayed or plotted by the existing tooling.
std::string teamPlanJson(const mtl::PlanningResult& result, const SearchProblem& problem);

/// One agent's track (map frame + mission NED) as JSON, for the run directory.
std::string agentTrackJson(const AgentTrack& track, const SearchProblem& problem);

}  // namespace mtl_search

#endif  // MTL_SEARCH_PLANNER_SEARCH_PROBLEM_HPP
