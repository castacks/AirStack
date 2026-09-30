// =============================================================================
//  mtl_search_planner/search_problem.hpp
//
//  The ROS-free adapter between an AirStack MTL scenario file and the two
//  vendored planners:
//
//    planner.type = orienteering   mtl::planner (cpp_planner): cells -> clusters
//                                  -> budgeted orienteering -> Dubins -> gimbal
//                                  schedule -> run-out.  The default.
//    planner.type = curve          mtl::curve::Planner (cpp_curve_planner): one
//                                  continuous curve per aircraft, exactly the
//                                  budget long, with a sinusoidal cross-track
//                                  gimbal sweep.
//
//  Nothing in here includes rclcpp, so it is unit-tested (and usable from the
//  `mtl_search_plan` CLI) without a ROS installation.
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
//  conversion between mtl, world and map.  (mtl::curve uses the same frame.)
//
//  GIMBAL ANGLE CONVENTION (host = mtl::planner's):  phi is the 1-DOF
//  cross-track gimbal angle, + looks RIGHT, and the combined cross-track tilt
//  off the mount axis is  crossAngle = roll + phi  (roll + = right wing down):
//      look = [x y] + h tan(theta) u + h tan(roll + phi) / cos(theta) v_right,
//      theta = tilt - pitch.
//  mtl::curve reports gimbalAngle = alpha, the LEVEL-frame sweep angle, + LEFT,
//  and gimbalCmd = alpha + roll.  So crossAngle = -alpha and
//      phi = -alpha - roll = -gimbalCmd,
//  which is what buildAgentTrack writes into TrackSample::gimbalPhi for a curve
//  plan (test_search_problem.cpp rebuilds every boresight from it).
// =============================================================================
#ifndef MTL_SEARCH_PLANNER_SEARCH_PROBLEM_HPP
#define MTL_SEARCH_PLANNER_SEARCH_PROBLEM_HPP

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/planner.hpp"
#include "mtl/types.hpp"
#include "mtl_curve/params.hpp"
#include "mtl_curve/planner.hpp"
#include "mtl_curve/types.hpp"
#include "mtl_search_planner/json_mini.hpp"

namespace mtl_search {

/// Which planner flies the mission (scenario `planner.type`).
enum class PlannerType { Orienteering, Curve };
/// "orienteering" | "curve"
const char* toString(PlannerType type);
/// @throws std::runtime_error on anything but "orienteering" | "curve".
PlannerType plannerTypeFrom(const std::string& name);

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
    /// info_aware.report_both (default true): with the ORIENTEERING planner flown,
    /// also plan its OTHER mode (solveAlternative) so the run folder and the
    /// report carry both outcomes.
    bool                   reportAlternative = true;

    /// planner.type: which planner is FLOWN (default orienteering).
    PlannerType            plannerType = PlannerType::Orienteering;
    /// planner.compare_orienteering (default true): with the CURVE planner flown,
    /// also plan the orienteering planner in BOTH modes (plain and info_aware)
    /// for the report.  Never flown.
    bool                   compareOrienteering = true;
    /// The curve planner's parameters: mtlc_plan's paramsFromScenario() +
    /// curveFromScenario() (the optional "curve" block, auto-scaled), NOT yet
    /// finalised / validated (the Planner does that, and throws on an infinite
    /// budget).  Always parsed, so a bad curve block fails early.
    mtl::curve::PlannerParams curveParams;
    /// The same host cells in mtl::curve's type (mtlc_plan's cellsFromScenario()).
    mtl::curve::CellSet    curveCells;
    std::vector<mtl::curve::Vec2> curveStarts;
    /// airstack.follower.gimbal_law: "open_loop" (default) | "aim_point" - the
    /// single-axis gimbal law the follower flies (SearchPlan.gimbal_law).
    std::string            gimbalLaw = "open_loop";

    int agentIndex(const std::string& agentName) const;  ///< -1 when absent
};

/// Parse a scenario (`mtl.scenario/1`). @throws std::runtime_error with a named cause.
SearchProblem parseScenario(const std::string& jsonText);
SearchProblem loadScenario(const std::string& path);

// -----------------------------------------------------------------------------
//  Planning.  Every call is deterministic for a given SearchProblem, so every
//  robot of the team, solving the same problem, gets the same plan.
// -----------------------------------------------------------------------------

/// The ORIENTEERING planner in the mode the scenario selects (info_aware.enabled:
/// the information-aware abstraction search; otherwise the plain planner).
/// This is the flown plan when planner.type = orienteering.
mtl::PlanningResult solve(const SearchProblem& problem);

/// The orienteering planner in the OTHER info_aware mode (report only).
mtl::PlanningResult solveAlternative(const SearchProblem& problem);

/// The orienteering planner with info_aware forced on or off (report only).
mtl::PlanningResult solveOrienteering(const SearchProblem& problem, bool infoAware);

/// A curve plan and the FINALISED parameters it was made with (plan.json needs
/// the derived fields: budget, stand-off, representation).
struct CurvePlan {
    mtl::curve::PlanningResult result;
    mtl::curve::PlannerParams  params;
};

/// The curve planner (mtl::curve::Planner::planFromCells, as mtlc_plan does).
/// Takes tens of seconds at the reference scale.
/// @throws std::runtime_error on an infinite budget; std::invalid_argument on
///         any other parameter set the planner rejects.
CurvePlan solveCurve(const SearchProblem& problem);

/// Either planner's result.  A std::variant rather than a conversion of the
/// curve result into mtl::PlanningResult: the two AgentPlan / TeamInfo types
/// carry different diagnostics (routes and a gimbal schedule vs curves and a
/// sweep), and converting would either drop the curve's or fake the route's.
/// Each result keeps everything its planner produced (plan.json writes it
/// all); buildAgentTrack() maps both onto the SAME TrackSample fields, so the
/// node, follower, logger and scripts never branch on the planner type.
struct SearchResult {
    std::variant<mtl::PlanningResult, CurvePlan> plan;
    double planningSeconds = 0.0;   ///< wall time of the planner call

    PlannerType type() const {
        return plan.index() == 0 ? PlannerType::Orienteering : PlannerType::Curve;
    }
    const mtl::PlanningResult* orienteering() const { return std::get_if<mtl::PlanningResult>(&plan); }
    const CurvePlan*           curve() const { return std::get_if<CurvePlan>(&plan); }

    // planner-agnostic views (the node's markers and logs)
    std::size_t numAgents() const;
    mtl::Index  numCells() const;
    mtl::Vec2   cellCenterMtl(mtl::Index i) const;          ///< mtl frame
    const mtl::Path3& droneTrackMtl(std::size_t agent) const;  ///< [x y h], mtl frame
    double teamInfo() const;          ///< belief mass the team services / sweeps
    double teamInfoTotal() const;
    double teamInfoFraction() const;
};

/// The plan that is FLOWN: planner.type selects solve() or solveCurve().
SearchResult solveFlown(const SearchProblem& problem);

/// The report-only plans for the side-by-side comparison (never flown):
///   planner.type = orienteering + info_aware.report_both   -> {the other mode}
///   planner.type = curve + planner.compare_orienteering    -> {plain, info_aware}
///   otherwise                                              -> {}
std::vector<SearchResult> solveComparisons(const SearchProblem& problem);

/// File-name suffix of a comparison plan in a run folder:
///   orienteering flown -> "_alt"         (plan_alt.json, track_alt.json; as before)
///   curve flown        -> "_alt_<mode>"  (plan_alt_plain.json, track_alt_info_aware.json, ...)
std::string comparisonSuffix(const SearchProblem& problem, const SearchResult& comparison);

/// "info_aware" or "plain": the mode an orienteering result was planned in.
std::string plannerMode(const mtl::PlanningResult& result);
/// "plain" | "info_aware" | "curve".
std::string plannerMode(const SearchResult& result);

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
    double pitch = 0.0;    ///< [rad] planned airframe pitch (the scheduler's nudge; 0 for a curve)
    double gimbalPhi = 0.0;///< [rad] planned 1-DOF cross-track gimbal angle (+ right, crossAngle = roll + phi)
};

/// Curve-planner diagnostics carried by an AgentTrack (curve plans only).
struct CurveTrackInfo {
    std::string representation, endpointMode, initStrategy;
    double sweepAmplitude = 0.0;   ///< [rad] alphaMax
    double sweepFreq = 0.0;        ///< [Hz]
    double sweepPeakRate = 0.0;    ///< [rad/s] alphaMax * 2 pi f (vs gimbalRate)
    double swathHalfWidth = 0.0;   ///< [m] calibrated single-pass P >= 0.5 half-width
    double standOff = 0.0;         ///< [m] h tan(tilt)
    double maxCurvature = 0.0;     ///< [1/m] discrete, on the flown track
    double endpointError = 0.0;    ///< [m]
    double fastObjective = 0.0;    ///< the agent's last fast (team) residual
    int    optimizerIters = 0;
    std::string optimizerExit;
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
    std::string plannerMode = "plain";  ///< "plain" | "info_aware" | "curve" (see plannerMode())
    PlannerType plannerType = PlannerType::Orienteering;
    CurveTrackInfo curve;               ///< filled for plannerType == Curve only
    std::string gimbalLaw = "open_loop";///< SearchProblem::gimbalLaw, for SearchPlan.gimbal_law

    double totalArc() const { return samples.empty() ? 0.0 : samples.back().arc; }
};

/// Extract agent `agentIndex` from a team result, convert it to that agent's
/// map frame (`homeUp` = height of the map origin above the ground plane) and
/// trim the hover padding the team timeline appends after it finishes.
AgentTrack buildAgentTrack(const mtl::PlanningResult& result, const SearchProblem& problem,
                           int agentIndex, double homeUp = 0.0);
/// The same for a curve plan: bx/by from `sensor`, roll/pitch from `rpy`,
/// speed = V, z = the agent's planned (staggered) altitude, yaw = the curve
/// tangent the boresight was built on, gimbalPhi = -gimbalCmd (see the
/// convention above).
AgentTrack buildAgentTrack(const CurvePlan& plan, const SearchProblem& problem,
                           int agentIndex, double homeUp = 0.0);
AgentTrack buildAgentTrack(const SearchResult& result, const SearchProblem& problem,
                           int agentIndex, double homeUp = 0.0);

/// The whole team plan as `mtl.plan/1` JSON (mission NED), the interchange the
/// MTL testbed and MATLAB exporter share — so a flown AirStack plan can be
/// replayed or plotted by the existing tooling.  A curve plan is written as
/// mtlc_plan writes it (samples.gimbal, curve diagnostics, meta.curve) plus
/// meta.planner_type / meta.planner_mode.
std::string teamPlanJson(const mtl::PlanningResult& result, const SearchProblem& problem);
std::string teamPlanJson(const CurvePlan& plan, const SearchProblem& problem);
std::string teamPlanJson(const SearchResult& result, const SearchProblem& problem);

/// One agent's track (map frame + mission NED) as JSON, for the run directory.
std::string agentTrackJson(const AgentTrack& track, const SearchProblem& problem);

}  // namespace mtl_search

#endif  // MTL_SEARCH_PLANNER_SEARCH_PROBLEM_HPP
