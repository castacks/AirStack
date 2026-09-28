// =============================================================================
//  tigris_search_planner/receding.hpp — receding-horizon TIGRIS for one agent.
//
//  The sortie is ONE growing track (world ENU), republished under one plan_id:
//
//    t = 0      plan root -> best from the start pose with the whole budget
//    every replan_period_s (or when the drone nears the end of the track):
//      1. fold the looks FLOWN since the last replan into the belief
//         (measured pose + gimbal on the robot; the planned looks in the CLI);
//      2. commit point s_c = progress + lookahead + speed * (planning time + margin):
//         the track up to s_c is never changed (the follower is already chasing
//         a carrot `lookahead` ahead, and the drone keeps flying while we plan);
//      3. apply the committed-but-not-yet-flown looks to a planning copy;
//      4. TIGRIS from the pose at s_c with budget - arc(s_c);
//      5. if it found a path, track = track[0 .. s_c] + new path (arc and time
//         continue, so the follower's progress index stays valid).
//
//  The belief TIGRIS plans over is maintained OUTSIDE the tree search, as in the
//  original system (IPP::replan receives the map; ipp.cpp never updates it from
//  observations). Every look counts as a MISS (no detector runs; the planner never
//  sees ground truth): MATCHED = prior * P(missed), the logger's residual belief;
//  ORIGINAL = TIGRIS's Bayes update, the looks of one replan interval applied as one
//  pass (each cell once, at its best range).
// =============================================================================
#ifndef TIGRIS_SEARCH_PLANNER_RECEDING_HPP
#define TIGRIS_SEARCH_PLANNER_RECEDING_HPP

#include <memory>
#include <string>
#include <vector>

#include "tigris_search_planner/belief.hpp"
#include "tigris_search_planner/scenario.hpp"
#include "tigris_search_planner/tigris.hpp"

namespace tigris_search {

/// One sample of the track, world ENU (z = height above ground).
struct TrackSample {
    double t = 0.0, arc = 0.0;
    double x = 0.0, y = 0.0, z = 0.0, yaw = 0.0;
    double bx = 0.0, by = 0.0, bz = 0.0;  ///< boresight ground point of the body-fixed camera
};

struct HorizonParams {
    bool   receding = true;             ///< false: one-shot plan over the whole budget
    double initialPlanningTime = 5.0;   ///< [s] first solve (drone hovering)
    double replanPlanningTime = 5.0;    ///< [s] in-flight solves (TIGRIS planning_time)
    double replanPeriod = 5.0;          ///< [s]
    double commitMarginTime = 1.0;      ///< [s] extra commit distance beyond planning time
    double lookahead = 14.4;            ///< [m] follower carrot distance (1.2 * R_min)
    double minReplanBudget = 10.0;      ///< [m] stop replanning below this
    double endTriggerMargin = 6.0;      ///< [m] replan early when this close to the commit horizon
    double minReplanSpacing = 1.0;      ///< [s] no early (horizon) replan sooner than this after the last
    double gridRes = 4.0;               ///< [m] planning grid
    double startYaw = 1e9;              ///< [rad] initial heading; >= 1e8 -> towards the area centre
    double budgetOverride = -1.0;       ///< [m] > 0 replaces the scenario budget
};

struct ReplanRecord {
    int index = 0;
    std::string trigger;
    double progressArc = 0.0, commitArc = 0.0, budgetLeft = 0.0;
    double timeLimit = 0.0, seconds = 0.0;
    int iterations = 0, treeSize = 0, pruned = 0, nodes = 0;
    bool improved = false;
    double segmentLength = 0.0;
    PathReward segmentReward;          ///< both rewards of the new segment (planning belief)
    double residualMassFlown = 0.0;    ///< planner's residual belief after the flown looks
    double trackLength = 0.0;          ///< total track after this replan
    int flownLooks = 0;
};

class RecedingHorizon {
public:
    RecedingHorizon(const Scenario& sc, int agentIndex, const Camera& cam, const TigrisParams& tp,
                    const HorizonParams& hp);

    /// Initial solve from `start` (world ENU, yaw ENU). Returns false if TIGRIS found nothing.
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
    const TigrisParams& tigrisParams() const { return tp_; }
    const HorizonParams& horizonParams() const { return hp_; }
    double commitDistance() const;

    /// Look of track sample k (weight = sample spacing / speed / dt_ref).
    Look sampleLook(std::size_t k) const;
    /// Planned looks with arc in (a0, a1] (the CLI's stand-in for the flown looks).
    std::vector<Look> plannedLooks(double a0, double a1) const;

private:
    std::vector<TrackSample> sampleSegment(const std::vector<TreeNode>& path, const TrackSample& from) const;
    std::size_t indexAtArc(double arc) const;
    void record(ReplanRecord r, const PlanResult& res);

    Scenario sc_;
    int agent_;
    PlannerSetup setup_;
    TigrisParams tp_;
    HorizonParams hp_;
    PlanningGrid grid_;
    std::unique_ptr<TigrisPlanner> planner_;
    std::unique_ptr<RewardEvaluator> eval_;
    BeliefState belief_;
    std::vector<TrackSample> track_;
    std::vector<ReplanRecord> records_;
    double budget_ = 0.0;
    int revision_ = -1;
    int replanCount_ = 0;
    bool done_ = false;
};

/// The agent's scenario start (world ENU) with HorizonParams::startYaw, or facing
/// the area centre when startYaw is unset.
Pose2 defaultStartPose(const Scenario& sc, int agentIndex, const HorizonParams& hp);

// ---- run-folder JSON (same layouts the MTL tooling reads) --------------------
struct TrackMeta {
    std::string planId, rewardMode, sampler;
    double budget = 0.0;
    int revision = 0;
    bool gimbalLocked = true;
};

/// Scenario cells whose centre falls inside any footprint of the track.
std::vector<int> servicedCells(const Scenario& sc, const RecedingHorizon& rh);
/// `mtl.agent_track/1` (map frame + mission NED) — read by analyze_*_run.py and mtl_foxglove.py.
std::string agentTrackJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta);
/// `mtl.plan/1`-shaped single-agent plan (mission NED), for tools that expect plan.json.
std::string planJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta);
/// `tigris.replans/1`: parameters and one record per solve.
std::string replansJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta);

}  // namespace tigris_search

#endif  // TIGRIS_SEARCH_PLANNER_RECEDING_HPP
