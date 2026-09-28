// =============================================================================
//  tigris_search_planner/tigris.hpp — TIGRIS: Informative path planning via
//  sampling-based tree search (Moon, Chatterjee, Scherer, ICRA 2023), single agent.
//
//  A line-by-line port of tigris/src/ipp.cpp (IPP::replan, planner_type OURS)
//  without ROS 1 / OMPL:
//
//    root = start pose; root.info = 0 (ipp.cpp scores the root before setting its state); best = root
//    until the time limit:
//      s        = informedConfig()   (cell drawn with probability ~ its one-look
//                                     reward, backed off along a random yaw so the
//                                     cell sits at viewPointGoal of the FOV)
//                 or randomConfig() (the paper's uniform benchmark sampler)
//      nearest  = nn.nearest(s)                 d = |dxyz| + |dpsi|  (XYZPsi space)
//      m        = steer(nearest, s)             cut at extend_dist or the budget; records
//                                               the straight part of the edge
//      m.info   = informationGain(root -> m)    (nodes + straight edges, re-walked from the root)
//      if !prune(m) and !nearest.closed: add m; if m.info > best.info: best = m
//      for every tree node q within extend_radius of m (q != nearest, q != m, q open):
//        a = steer(q, m.pose); a.info = gain(root -> a)
//        if !prune(a): add a; if m.info > best.info: best = m     <- as in ipp.cpp
//    return root -> best
//
//  prune(m): some tree node within prune_radius has cost <= m.cost and info >= m.info.
//  closed:   a node whose cut hit the budget; it is never extended.
//
//  What differs from the ROS 1 code, and only because AirStack forces it:
//    * OMPL's GNAT is replaced by an exact spatial hash (same nearest / radius results);
//    * the trochoid steer has no wind in AirStack, which makes it a Dubins path;
//    * the camera footprint is the configured cone (see belief.hpp) instead of the
//      rectangular frustum;
//    * the RNG is seeded (mission seed + replan index) instead of std::random_device,
//      so a run can be reproduced.
//  Kept as in the original, deliberately: the near-node loop compares the NEW sample's
//  node (motion_feasible) with the best path, not the rewired node; nodes that were
//  pruned are still used as the rewiring target; turning arcs earn no reward. The
//  optional geofence (bounds_margin_m) is off by default (TIGRIS's only collision
//  check is counter-detection, radius 0 in every launch file). MATCHED mode is an
//  addition (the evaluator's metric) and changes only the reward function.
// =============================================================================
#ifndef TIGRIS_SEARCH_PLANNER_TIGRIS_HPP
#define TIGRIS_SEARCH_PLANNER_TIGRIS_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "tigris_search_planner/belief.hpp"
#include "tigris_search_planner/dubins.hpp"

namespace tigris_search {

struct TigrisParams {
    double extendDist = 60.0;     ///< [m] steer cut (TIGRIS 750 m at 5 km; scaled 1/12.5)
    double extendRadius = 20.0;   ///< [m] near-node rewiring radius (TIGRIS 251 m; scaled)
    double pruneRadius = 60.0;    ///< [m] (TIGRIS 750 m; scaled)
    double rewardStep = 2.0;      ///< [m] look spacing when scoring an edge
    double viewPointGoal = 0.6;   ///< informed sampler: where in the FOV the sampled cell should sit
    double psiWeight = 1.0;       ///< nearest-neighbour metric weight on |dpsi| [m/rad]
    double boundsMargin = -1.0;   ///< [m] geofence around the area (< 0 = off, as in TIGRIS)
    bool   informedSampler = true;
    int    maxIterations = 0;     ///< > 0: stop after this many samples (deterministic runs)
    std::uint64_t seed = 21;
    RewardParams reward;
};

/// Everything fixed for one agent (world ENU, z = height above ground).
struct PlannerSetup {
    Camera camera;
    DetectionModel det;
    double speed = 6.0;          ///< [m/s] used for the look weights dt / dt_ref
    double turnRadius = 12.0;    ///< [m]
    double dubinsStep = 0.5;     ///< [m] steer walk resolution
    double altitude = 30.0;      ///< [m] above ground
    double xMin = -200, xMax = 200, yMin = -200, yMax = 200;  ///< search area
};

struct TreeNode {
    Pose2 pose;
    int parent = -1;
    double cost = 0.0;       ///< path length from the root [m]
    double info = 0.0;       ///< reward of root -> node in the ACTIVE mode
    bool closed = false;     ///< hit the budget: never expanded
    DubinsPath edge;         ///< parent -> node (cut at edgeLen)
    double edgeLen = 0.0;
    bool inTree = false;     ///< added to the nearest-neighbour structure (not pruned)
    bool hasEdge = false;    ///< a straight segment was recorded (Motion::start_edge / end_edge)
    Pose2 edgeStart, edgeEnd;
};

struct PlanResult {
    std::vector<TreeNode> path;   ///< root .. best (path[0] is the root)
    PathReward reward;            ///< both rewards of the best path
    double cost = 0.0;            ///< its length [m]
    int iterations = 0;
    int treeSize = 0;
    int nodesPruned = 0;
    double seconds = 0.0;
    bool improved() const { return path.size() > 1; }
};

class TigrisPlanner {
public:
    TigrisPlanner(const PlanningGrid& grid, const PlannerSetup& setup, const TigrisParams& params);

    /// One TIGRIS solve from `start` with path-length `budget` over `belief`.
    /// `replanIndex` offsets the RNG seed so every replan is reproducible.
    PlanResult plan(const Pose2& start, double budget, double timeLimit, const BeliefState& belief,
                    int replanIndex = 0);

    /// Looks along one edge (every rewardStep, weight = step / speed / dt_ref).
    std::vector<Look> edgeLooks(const DubinsPath& edge, double len) const;
    /// Both rewards of a node chain (as a sequence of passes) over `belief`.
    PathReward scorePath(const std::vector<TreeNode>& path, const BeliefState& belief);
    /// ORIGINAL reward of a node chain the TIGRIS way (MapRepresentation::informationGain).
    void tigrisGain(const std::vector<const TreeNode*>& chain);

    const TigrisParams& params() const { return p_; }
    const PlannerSetup& setup() const { return s_; }

private:
    const PlanningGrid& g_;
    PlannerSetup s_;
    TigrisParams p_;
    RewardEvaluator eval_;
};

}  // namespace tigris_search

#endif  // TIGRIS_SEARCH_PLANNER_TIGRIS_HPP
