// =============================================================================
//  ipp_baselines_search_planner/receding.hpp — receding horizon for one agent, shared
//  by the ia-tigris baselines (the IA-TIGRIS port's glue, the ipp_planners_node.h loop).
//
//  The sortie is ONE growing track (world ENU), republished under one plan_id:
//
//    t = 0      the planner's solve from the start pose with the whole budget
//    every replan_period_s (or when the drone nears the end of the track):
//      1. fold the looks FLOWN since the last replan into the belief (as misses);
//      2. commit point: s* = progress + lookahead + speed * (planning time + margin),
//         SNAPPED FORWARD to the end of the segment it lies on, i.e. the next node of the
//         current plan (ipp_planners_node.h find_expected_waypoint_reached_after_time_to_plan
//         + get_coords_of_waypoint_num: plan from segments[k].getState(0)). The track up to
//         that node is never changed;
//      3. apply the committed-but-not-yet-flown looks to a planning copy;
//      4. the planner solves from that node with budget - arc(node);
//      5. the newest plan replaces the tail (append_path_msg always publishes the newest
//         best path; none of these planners keeps a tree between solves).
//    The published track is cut at the mission budget (planners that overshoot it -
//    greedy's last segment, rect_coverage - keep their own path internally).
// =============================================================================
#ifndef IPP_BASELINES_SEARCH_PLANNER_RECEDING_HPP
#define IPP_BASELINES_SEARCH_PLANNER_RECEDING_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ipp_baselines_search_planner/belief.hpp"
#include "ipp_baselines_search_planner/planner_core.hpp"
#include "ipp_baselines_search_planner/scenario.hpp"

namespace ipp_baselines_search {

/// One sample of the track, world ENU (z = height above ground).
struct TrackSample {
    double t = 0.0, arc = 0.0;
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
    double bx = 0.0, by = 0.0, bz = 0.0;  ///< boresight ground point (swept when the gimbal is actuated)
    double phi = 0.0;                     ///< cross-track gimbal angle [rad, + right]; 0 = body-fixed
};

struct HorizonParams {
    bool   receding = true;             ///< false: one-shot plan over the whole budget
    double initialPlanningTime = 5.0;   ///< [s] first solve (drone hovering)
    double replanPlanningTime = 5.0;    ///< [s] in-flight solves (max_planning_time)
    double replanPeriod = 5.0;          ///< [s] (= planning time: seconds_until_auto_replan 0.1)
    double commitMarginTime = 3.0;      ///< [s] extra commit distance beyond planning time
    double lookahead = 14.4;            ///< [m] follower carrot distance (1.2 * R_min)
    double minReplanBudget = 10.0;      ///< [m] stop replanning below this
    double endTriggerMargin = 6.0;      ///< [m] replan early when this close to the commit horizon
    double minReplanSpacing = 1.0;      ///< [s] no early (horizon) replan sooner than this after the last
    double gridRes = 50.0;              ///< [m] planning grid (= tigris_search / ia_tigris_search; released map_resolution: 30)
    bool   snapToNode = true;           ///< commit snapped forward to the next plan node (ipp_planners_node.h)
    double startYaw = 1e9;              ///< [rad] initial heading; >= 1e8 -> towards the area centre
    double budgetOverride = -1.0;       ///< [m] > 0 replaces the scenario budget
};

struct ReplanRecord {
    int index = 0;
    std::string trigger;
    double progressArc = 0.0, commitArc = 0.0, budgetLeft = 0.0;
    double timeLimit = 0.0, seconds = 0.0;
    int iterations = 0, treeSize = 0, nodes = 0;
    bool improved = false;
    double segmentLength = 0.0;
    PathReward segmentReward;          ///< both rewards of the new segment (planning belief, yardstick)
    PathReward keptReward;             ///< both rewards of the tail it replaced (same belief)
    double commitTargetArc = 0.0;      ///< s* before snapping to a plan node
    double residualMassFlown = 0.0;    ///< planner's residual belief after the flown looks
    double trackLength = 0.0;          ///< total track after this replan
    double cutAtBudget = 0.0;          ///< [m] of the new plan beyond the budget (not published)
    int flownLooks = 0;
    std::map<std::string, double> extra;  ///< planner-specific (PlanResult::extra)
};

class RecedingHorizon {
public:
    RecedingHorizon(const Scenario& sc, int agentIndex, const Camera& cam, const BaselineParams& bp,
                    const HorizonParams& hp);

    /// Initial solve from `start` (world ENU, yaw ENU). Returns false if the planner found nothing.
    bool start(const Pose2& start);
    /// Replan due now? (period elapsed or the commit horizon reaches the end of the track)
    bool due(double progressArc, double secondsSinceLastReplan) const;
    /// Steps 1-5 above. Returns true when the track changed.
    bool replan(double progressArc, const std::vector<Look>& flownLooks, const std::string& trigger);
    /// Fold flown looks into the belief without replanning (end of the sortie).
    void absorb(const std::vector<Look>& flownLooks);

    const std::vector<TrackSample>& track() const { return track_; }
    double totalArc() const { return track_.empty() ? 0.0 : track_.back().arc; }
    double budget() const { return budget_; }
    bool exhausted() const { return done_; }
    int revision() const { return revision_; }
    const std::vector<ReplanRecord>& records() const { return records_; }
    const PlanningGrid& grid() const { return grid_; }
    const BeliefState& belief() const { return belief_; }
    const PlannerSetup& setup() const { return setup_; }
    const BaselineParams& baselineParams() const { return bp_; }
    const HorizonParams& horizonParams() const { return hp_; }
    SearchPlanner& planner() { return *planner_; }
    const std::vector<TreeNode>& currentPlan() const { return plan_; }
    double currentPlanArc0() const { return planArc0_; }
    double commitDistance() const;

    /// Look of track sample k (weight = sample spacing / speed / dt_ref).
    Look sampleLook(std::size_t k) const;
    /// Planned looks with arc in (a0, a1] (the CLI's stand-in for the flown looks).
    std::vector<Look> plannedLooks(double a0, double a1) const;

private:
    /// Track samples of `path` from `from`: every speed*dt, exactly at every node, cut at the budget.
    std::vector<TrackSample> sampleSegment(const std::vector<TreeNode>& path, const TrackSample& from,
                                           double* cut) const;
    std::size_t indexAtArc(double arc) const;
    void record(ReplanRecord r, const PlanResult& res);
    std::vector<TreeNode> tailFrom(const TrackSample& commit) const;

    Scenario sc_;
    int agent_;
    PlannerSetup setup_;
    BaselineParams bp_;
    HorizonParams hp_;
    PlanningGrid grid_;
    std::unique_ptr<SearchPlanner> planner_;
    std::unique_ptr<RewardEvaluator> eval_;
    BeliefState belief_;
    std::vector<TrackSample> track_;
    std::vector<ReplanRecord> records_;
    std::vector<TreeNode> plan_;   ///< the plan the track's tail was sampled from (uncut)
    double planArc0_ = 0.0;        ///< track arc at plan_[0]
    double budget_ = 0.0;
    int revision_ = -1;
    int replanCount_ = 0;
    bool done_ = false;
};

/// The agent's scenario start (world ENU) with HorizonParams::startYaw, or facing
/// the area centre when startYaw is unset.
Pose2 defaultStartPose(const Scenario& sc, int agentIndex, const HorizonParams& hp);

// ---- run-folder JSON (same layouts the MTL / TIGRIS tooling reads) -------------------
struct GimbalLimits {
    bool   locked = true;
    double maxRad = 1e-6;
    double pitchNudgeMaxRad = 1e-6;
};
constexpr double kGimbalLockedRad = 1e-6;
constexpr double kGimbalTravelRad = 1.3962634015954636;  ///< 80 deg mount travel
constexpr double kPitchNudgeRad = 0.08726646259971647;   ///< 5 deg
GimbalLimits gimbalLimits(const Camera& cam, bool lockGimbal);

struct TrackMeta {
    std::string planId, rewardMode;
    double budget = 0.0;
    int revision = 0;
    bool gimbalLocked = true;
    double gimbalMaxRad = kGimbalLockedRad;
    double pitchNudgeMaxRad = kGimbalLockedRad;
    void setGimbal(const GimbalLimits& g) {
        gimbalLocked = g.locked;
        gimbalMaxRad = g.maxRad;
        pitchNudgeMaxRad = g.pitchNudgeMaxRad;
    }
};

std::vector<int> servicedCells(const Scenario& sc, const RecedingHorizon& rh);
/// `mtl.agent_track/1` (map frame + mission NED); the gimbal block stays under "tigris".
std::string agentTrackJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta);
/// `mtl.plan/1`-shaped single-agent plan (mission NED).
std::string planJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta);
/// `ipp_baselines.replans/1` (the ia_tigris_replans.json layout + "planner" + planner fields).
std::string replansJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta);

}  // namespace ipp_baselines_search

#endif  // IPP_BASELINES_SEARCH_PLANNER_RECEDING_HPP
