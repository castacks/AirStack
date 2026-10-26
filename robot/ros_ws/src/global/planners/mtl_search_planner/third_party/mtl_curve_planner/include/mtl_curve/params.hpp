// =============================================================================
//  mtl_curve/params.hpp
//
//  Single source of truth for every tunable of the parameterized-curve planner
//  - the C++ port of init_curve_params.m (which runs init_params.m first, so
//  the scenario block here carries init_params' reference values).  Each struct
//  owns one stage and carries that stage's defaults, so `PlannerParams{}` gives
//  the reference behaviour of main_curve_planner.m.
//
//  All of it is handed to mtl::curve::Planner's constructor, which calls
//  finalize() and validate() once and then treats the set as read-only for the
//  run.  Derived quantities (budgetDist, sensorStandOff, the curvature limit,
//  the dense step, the control-point box, per-agent altitudes) are computed by
//  finalize(), never set by hand.
//
//  Sections 1-4 are copied from cpp_planner/include/mtl/params.hpp unchanged.
// =============================================================================
#ifndef MTLC_PARAMS_HPP
#define MTLC_PARAMS_HPP

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mtl_curve/types.hpp"

namespace mtl::curve {

// =============================================================================
//  1. PRIOR BELIEF MAP   (copied from cpp_planner)        -> mapgen::generateBeliefMap
// =============================================================================
/// The prior is a sum of numCentroids axis-aligned Gaussian bumps, capped,
/// then floored, then NORMALISED so the whole grid sums to 1.  The fields
/// below therefore only shape the prior; none of them sets its scale.
/// Scenario generation only - the planner never needs this.
struct BeliefMapParams {
    /// Number of Gaussian belief bumps scattered over the map.
    int    numCentroids = 10;
    /// Peak height of each bump before capping.
    double maxPriorPeak = 0.4;
    /// [m] Bounds on each bump's standard deviation, drawn independently in x
    /// and y, so bumps are elliptical and axis-aligned.
    double sigmaMin = 200.0;
    double sigmaMax = 500.0;
    /// Hard ceiling applied after summing (before normalising), which flattens
    /// the tops where bumps overlap.
    double beliefCap = 0.85;
    /// Floor applied after the cap (before normalising).  A non-zero floor
    /// spreads belief mass over the WHOLE map, which raises the cell count
    /// sharply - if you raise this, raise PlannerParams::minimumBeliefMass with it.
    double baseUncertainty = 0.0;
};

// =============================================================================
//  2. CELL EXTRACTION AND MACRO-CLUSTERING   (copied from cpp_planner)
// =============================================================================
struct ClusterParams {
    /// Iteration cap and restart count for the per-K k-means inside
    /// clustering::clusterCells.
    int kmeansMaxIter    = 200;
    int kmeansReplicates = 3;
};

// =============================================================================
//  3. SENSOR DETECTION MODEL   (copied from cpp_planner)   -> eval::updateTargetDetectionProbs
// =============================================================================
/// Probability of detection as a function of slant range r (Moon et al. 2022):
///     f(r) = 1 / (a + exp(b*(r - c)))     for r <= beta
/// a sets the near-field ceiling (f(0) ~ 1/a), b the falloff sharpness, c the
/// range at which the sigmoid breaks.
struct SensorModelParams {
    double a = 1.10;
    double b = 0.01;
    double c = 610.0;
    /// [m] Hard observation cutoff.  Beyond it f(r) is replaced by pOutOfRange.
    double beta = 610.0;
    /// What happens BEYOND beta, per accumulation mode.
    double pOutOfRangeMulti  = 1e-6;
    double pOutOfRangeSingle = 1e-4;
    double pOutOfRangeGrid   = 1e-3;

    double detectionProb(double slantRange) const {
        if (slantRange > beta) return pOutOfRangeMulti;
        return 1.0 / (a + std::exp(b * (slantRange - c)));
    }
};

// =============================================================================
//  4. ORIENTEERING SOLVER      -> planning::solveBudgetedOrienteering
//     (copied from cpp_planner; the curve planner uses it only to ORDER each
//     agent's clusters for the 'clusters' seed)
// =============================================================================
struct OrienteeringParams {
    /// GRASP multi-starts.  Starts 1 and 2 are deterministic, so nStarts <= 2
    /// is fully reproducible; the rest are randomised.
    int nStarts = 8;
    /// Restricted candidate-list size for the randomised starts.  1 = greedy.
    int rclSize = 3;
    /// Ruin-and-recreate iterations.  Leave unset (nullopt) to keep the
    /// solver's own default of 40 AND its automatic thinning above 60
    /// candidate clusters; setting it explicitly DISABLES that thinning,
    /// because an explicit value is treated as a deliberate choice.
    std::optional<int> nPerturb = std::nullopt;
    /// Cap on 2-opt / Or-opt shortening sweeps per local-search call.
    int maxLocalIter = 60;
    /// Seed for the randomised starts.  Kept in a private stream, so this never
    /// disturbs PlannerParams::rngSeed.
    std::uint64_t seed = 1234;
    /// Forced final position.  Unset = OPEN path: the aircraft stops at its last
    /// cluster and does not fly home.  Set it to the launch point for a round
    /// trip, which roughly halves the area reachable on a given budget.
    std::optional<Vec2> endPos = std::nullopt;
    /// Clusters that MUST be visited (indices into this agent's candidate list).
    std::vector<Index> mandatory;
    bool verbose = false;
};


// =============================================================================
//  5. GIMBAL ENVELOPE AND PLATFORM   -> computeSweepParams, computeAirframeRPY
// =============================================================================
/// The single-axis cross-track gimbal and the airframe attitude model.  Field
/// names and defaults match cpp_planner's GimbalSchedulerParams (init_params'
/// sensorOptOpts), so the SAME payload is flown by both planners; the
/// scheduler-only knobs (pitch nudge, repair loop, ...) are not needed here -
/// the curve planner sweeps the gimbal instead of scheduling it.
struct GimbalParams {
    /// [s] Must match the trajectory dt; finalize() keeps them in step.
    double dt = 0.1;
    double gimbalMax      = deg2rad(80);   ///< [rad] gimbal travel from the mount axis
    double gimbalRate     = deg2rad(120);  ///< [rad/s] slew rate (caps the sweep frequency)
    /// [rad] Max |roll + gimbal| off the mount axis (reported, see AgentTrajectory).
    double maxCrossAngle  = deg2rad(85);
    double maxSensorReach = 600.0;         ///< [m] max usable cross-track ground offset
    double aoa            = 0.0;           ///< [rad] trim AoA: gamma = pitch - aoa
    double maxPitch       = deg2rad(35);   ///< [rad] hard pitch stop

    // --- platform (computeAirframeRPY) ---
    bool   computeRoll   = true;   ///< coordinated-turn bank rather than level flight
    bool   computePitch  = true;   ///< pitch = aoa + atan(dh/ds), else pinned to zero
    double maxRoll       = deg2rad(45);
    double rollSign      = 1.0;    ///< +1 or -1: sign convention for bank
    double rollSmoothSec = 1.5;    ///< [s] how quickly the aircraft is assumed to roll
};

// =============================================================================
//  6. SENSOR SWEEP                   -> sensing::computeSweepParams
// =============================================================================
/// The boresight sweeps the mount's cross-track line sinusoidally,
///     alpha(t) = alphaMax * sin(2 pi freq t)
/// with alphaMax the largest angle that respects the slant range (beta *
/// rangeMargin), the gimbal travel and the ground reach, and freq capped by the
/// slew rate.
struct SweepOptions {
    /// [Hz] At 0.25 Hz the aircraft advances 40 m per half-sweep, so every point
    /// of the strip is looked at many times.  Capped automatically by
    /// GimbalParams::gimbalRate.
    double freq = 0.25;
    /// Fraction of beta the boresight slant range may use at full deflection.
    double rangeMargin = 0.98;
    /// Fraction of the feasible amplitude actually swept (1 = all of it).
    double amplitudeFrac = 1.0;
};

// =============================================================================
//  7. SWATH KERNEL CALIBRATION        -> optimization::calibrateSwathKernel
// =============================================================================
/// The fast objective's aircraft-frame look kernel is CALIBRATED from the real
/// detection model and sweep (see calibrateSwathKernel for why and how).
struct KernelParams {
    double tableStep  = 5.0;    ///< [m] aircraft-frame kernel table
    int    nPhase     = 48;     ///< sweep phases averaged
    double gridStep   = 4.0;    ///< [m] raster of the straight-pass calibration
    double trackLen   = 2400.0; ///< [m] length of that straight pass
    double lambdaCap  = 150.0;  ///< log-miss saturation of the profile (= init_curve_params.m)
    double edgeWidth  = 6.0;    ///< [m] logistic width of the swath edge
    double edgeInset  = 12.0;   ///< [m] edge pulled INSIDE the real reach (conservative)
    /// Exploration kernel = accurate + tailWeight * Gaussian(tailSigma) blur.
    double tailSigma  = 100.0;  ///< [m]
    double tailWeight = 0.5;
};

// =============================================================================
//  8. CURVE PARAMETERISATION          -> curve_planning::enforceEndpointConstraints
// =============================================================================
struct CurveParams {
    /// Curvature: kappa(s) is a piecewise-linear spline in arc length and the
    ///   curve is its integral.  Length is EXACTLY the budget and |kappa| <=
    ///   1/minTurnRadius holds everywhere BY CONSTRUCTION (box bounds).
    /// BSpline: the plan's clamped uniform cubic B-spline; length and
    ///   curvature are nonlinear constraints.
    CurveRepresentation representation = CurveRepresentation::Curvature;

    // --- Curvature ---
    double curvatureKnotSpacing = 100.0;  ///< [m] 5000 m -> 51 coefficients

    // --- BSpline ---
    int    numControlPoints = 12;
    int    splineDegree     = 3;          ///< cubic -> C2, curvature-continuous
    /// [m] box on the free control points.  Unset = [-0.2, 1.2] * mapSize
    /// (finalize()), which lets a curve leave the map a little.
    std::optional<Vec2> controlPointBox = std::nullopt;
    /// Soft penalty on curvature excess on top of the hard constraint (plan: 1e5).
    double curvaturePenaltyWeight = 1e5;

    // --- both ---
    double sampleSpacing   = 25.0;  ///< [m] arc spacing of the fast-objective samples
    double curvatureMargin = 0.99;  ///< fraction of maxCurvature the coefficients may use
    /// [rad] launch heading.  Unset = free (optimised).  Set: pinned; the
    /// B-spline then also pins P_2 (plan PHASE 4).
    std::optional<double> initialHeading = std::nullopt;

    /// One mode for the whole team, or one per agent (then numAgents long).
    std::vector<EndpointMode> endpointModes = {EndpointMode::Open};
    /// [m] per-agent destinations, used by EndpointMode::FixedDest.
    std::vector<Vec2> destinations = {Vec2(4500, 4500), Vec2(4500, 1000), Vec2(1000, 4500)};

    /// A centroid counts as passed once within this fraction of the swath
    /// half-width (the gimbal sweeps it from there).
    double captureFrac = 0.35;

    // --- derived by finalize() ---
    double maxCurvature = 0.01;  ///< [1/m] = 1/minTurnRadius
    double denseStep    = 2.0;   ///< [m] = avgDroneSpeed * dt, the flown discretisation

    EndpointMode modeOf(int agent) const {
        if (endpointModes.empty()) return EndpointMode::Open;
        return endpointModes.size() == 1 ? endpointModes.front()
                                         : endpointModes[static_cast<std::size_t>(agent)];
    }
};

// =============================================================================
//  9. OPTIMISER                       -> optimization::optimizeAgentCurve
// =============================================================================
/// L-BFGS on a smooth box transform (v = mid + half*sin(z)), with an
/// augmented-Lagrangian outer loop for the nonlinear constraints, then a
/// Gauss-Newton feasibility projection on the flown discretisation.
struct OptimizerParams {
    int    maxIter     = 150;   ///< per solve (first AL pass)
    int    alOuter     = 6;     ///< augmented-Lagrangian passes
    double alMu0       = 1e3;
    int    lbfgsMemory = 10;
    double ftol        = 1e-7;
    /// GRADUATED OPTIMISATION: exploreIter iterations on the optimistic
    /// long-tailed kernel over the coarse exploration grid first, then refine.
    /// 0 disables.  exploreIterWarm is used for warm re-solves.
    int    exploreIter     = 100;
    int    exploreIterWarm = 60;
    bool   verbose = false;
};

// =============================================================================
// 9b. KERNEL CLUSTERS AND THE GCS SEED      -> gcs::kernelClusters, gcs::planAgentGcs
// =============================================================================
/// Macro-clusters sized by the camera kernel (ClusterMode::Kernel): the same
/// box sum the greedy seed scores with, peeled greedily off the prior.
struct KernelClusterParams {
    double boxFrac     = 0.8;    ///< box half-side / swath half-width (as pickGreedy)
    double minInfoFrac = 0.005;  ///< stop when the best box holds < this of the prior
    double coverFrac   = 0.995;  ///< stop once this fraction of the prior is taken
    int    maxClusters = 1000;
};

/// The GCS seed (InitStrategy::Gcs).  One convex set per cluster: its Voronoi
/// cell cut to a disc of setRadiusFrac * swath half-width (an nPoly-gon) and
/// the map.  The tour through every set of an agent is solved as a convex
/// relaxation (SOCP, SCS) with cutset separation, rounded, re-solved exactly
/// for the order and polished; clusters are dropped (dropRule) until the
/// tour plus Rmin (D - sin D) per turn fits orderBudgetFrac * L.
/// Drop     tour EVERY set of the agent, drop clusters (dropRule) until it fits
/// Orienteering  choose the sets inside the GCS (visit variables + budget)
/// Coverage      a path through swath-sized hexagonal cells of the whole free
///               space, maximising the belief its swath covers (planAgentGcsCoverage)
enum class GcsMode : int { Drop = 0, Orienteering = 1, Coverage = 2 };

struct GcsParams {
    /// Defaults = the recommended settings of the 2026-10-22 revival
    /// (gcs/GCS_REVIVAL_NOTES.md): orienteering mode, 8-NN edges, warm start,
    /// capture-radius sets.
    GcsMode mode = GcsMode::Orienteering;
    /// Orienteering: length tie-break in the objective, per km of route in
    /// units of the agent's largest cluster information.
    double opLengthWeight = 1e-3;
    /// Orienteering: connect each set only to its knn nearest sets (by centre,
    /// symmetrised) instead of to every set.  The relaxation then has O(n k)
    /// edges instead of O(n^2) - the difference between ~1 s and ~1 min on the
    /// 100+ small clusters of an information-aware abstraction.  A route can
    /// still go anywhere through intermediate sets (and collects them on the
    /// way); rounding re-solves any order exactly.  0 = complete graph.
    int    knn = 8;
    /// Orienteering: also round from the orienteering heuristic's order
    /// (solveBudgetedOrienteering on the same clusters and budget).
    bool   warmStart = true;
    /// Weight on the Rmin (D - sin D) turning overhead each route is charged
    /// against the budget (1 = the original; 0 = Euclidean length only, the
    /// budget the 'clusters' seed's orienteering uses).
    double turnWeight = 1.0;
    double setRadiusFrac = 1.0;
    GcsSetRadius setRadiusMode = GcsSetRadius::Capture;
    double captureMinFrac = 0.25;
    int    nPoly         = 16;
    /// Coverage: hexagon circumradius / swath half-width.
    double coverageCellFrac = 0.5;
    /// Coverage: the route point inside each cell is confined to the cell
    /// shrunk about its centre by this factor (1 = the whole cell), so a pass
    /// goes through the cell rather than grazing its edge.
    double coveragePointFrac = 1.0;
    /// Coverage: reward a visit with the cell's OWN belief (prize-collecting,
    /// tight relaxation, best with swath-wide cells) instead of the swath
    /// coverage of every cell it reaches (q_c <= sum_v a_cv p_v).
    bool   coverageOwnOnly = false;
    /// Optimise the best nSeeds distinct GCS routes (coverage mode) as
    /// separate seeds of the multi-start, keeping the best curve.
    int    nSeeds = 1;
    /// Coverage: extra tilings (cellFrac, pointFrac) whose routes are added as
    /// further seeds of the multi-start (structurally different routes).
    std::vector<std::pair<double, double>> coverageExtra;
    /// Coordination: plan each agent's GCS route inside the first sweep, with
    /// the cluster rewards re-scored as the belief the agents already planned
    /// leave unswept (what the greedy seed does with lambdaOther).
    bool   residualRewards = false;
    /// ... and offer each agent EVERY cluster still worth minResidualFrac of
    /// its prior reward, instead of its k-means partition.
    bool   ignorePartition = false;
    double minResidualFrac = 0.2;
    GcsDropRule dropRule = GcsDropRule::Info;
    bool   dropBatch     = false;  ///< drop on the current points until it fits, then re-solve
    int    maxCutRounds  = 15;
    int    nRound        = 30;     ///< random rounding walks
    int    nExact        = 3;      ///< best rounded orders re-solved exactly
    bool   polish        = true;   ///< 2-opt / or-opt + exact re-solve
    int    polishRounds  = 4;
    /// Pull each route point toward its cluster centre: w metres of route per
    /// metre of offset (0 = pure length: the points graze the sets).
    double centerWeight       = 0.0;
    bool   centerWeightByInfo = true;  ///< scale w by info_k / the agent's mean info
    double tol       = 1e-5;       ///< SCS eps_abs = eps_rel
    int    maxIters  = 100000;
    double scale     = 1000.0;     ///< [m] coordinates are divided by this in the conic problems
    std::uint64_t seed = 1;        ///< rounding RNG (private stream)
};

// =============================================================================
// 9b. THE 'sector' SEED                -> planning::solveSectorOrienteering
// =============================================================================
/// Coverage orienteering over a sector grid (InitStrategy::Sector, the default
/// seed).  A sectorSize lattice covers the search area; a visited sector
/// credits the belief in a box of half-side boxFrac x the calibrated swath
/// half-width around its centre; a route scores the belief in the UNION of its
/// boxes (marginal gains), under a Euclidean budget of budgetFrac x the curve
/// length.  The ordered sector centres are flown like the cluster centroids.
/// Defaults are the study's best ("f1.2b1": box 1.2 hw, budget 1.0 L).
struct SectorParams {
    double sectorSize    = 200.0;  ///< [m] sector pitch
    double boxFrac       = 1.2;    ///< box half-side / swath half-width
    double budgetFrac    = 1.0;    ///< Euclidean route budget / curve length
    int    nStarts       = 8;      ///< multi-starts: gain/detour, gain^2/detour, then GRASP
    int    rcl           = 3;      ///< GRASP restricted candidate list
    int    nPerturb      = 40;     ///< ruin-and-recreate iterations, shared by the starts
    int    maxCandidates = 250;    ///< sectors kept as nodes (highest box value first)
    double gridStep      = 50.0;   ///< [m] raster the box sums are taken on
    std::uint64_t seed   = 1234;   ///< private Park-Miller stream
};

// =============================================================================
// 10. TEAM: GRIDS, SEEDING, COORDINATION, REALLOCATION
//                                    -> curve_planning::reallocateCurveClusters
// =============================================================================
struct TeamParams {
    /// [m] h_a = droneAltitude + a * altitudeStagger (plan 3.4: 300/325/350 m).
    /// Each altitude gets its own kernel.  0 = like-for-like with cpp_planner.
    double altitudeStagger = 25.0;
    /// [m] cell size of the fast-objective grid (MATLAB fastEvalDownsample = 25
    /// pixels of 1 m) and of the exploration grid.
    double fastGridStep    = 25.0;
    double exploreGridStep = 50.0;
    /// Euclidean budget (fraction of the agent budget) for ORDERING each agent's
    /// clusters with solveBudgetedOrienteering, for the 'clusters' seed.
    double orderBudgetFrac = 1.0;
    /// Passes over the team.  Pass 1 seeds and optimises every agent (each
    /// against the agents already planned: Gauss-Seidel); every further pass
    /// re-optimises each agent warm (exploreIterWarm) against the others' final
    /// swaths.  Default 1 = ONE optimisation per seed; 2 adds the warm re-solve
    /// (the previous default; for a single agent it is a polish worth ~0.5%).
    int    coordinationSweeps = 1;
    /// Seeds for each agent's first solve, each optimised, best one kept.
    /// Default: the sector seed alone (one optimisation).  The previous default
    /// was {Clusters, Greedy}.
    std::vector<InitStrategy> initStrategies = {InitStrategy::Sector};
    /// The 'sector' seed (coverage orienteering over a sector grid).
    SectorParams sector;
    /// Corridor: keep the optimised curve near its seed route with the penalty
    /// corridorMu * mean((dist to the seed / corridorScale)^2) (0 = off).
    double corridorMu = 0.0;
    double corridorScale = 500.0;   ///< [m]
    bool   corridorGcsOnly = true;  ///< only GCS seeds get the corridor
    bool   corridorKeep = true;     ///< keep it in the later sweeps (false: released after sweep 1)
    /// Where the macro-clusters come from (Radius = cpp_planner's).
    ClusterMode clusterMode = ClusterMode::Radius;
    KernelClusterParams kernelClusters;
    GcsParams gcs;

    /// Slack audit + cluster reallocation between agents after the static
    /// solve (each trial is another optimisation).  Off by default.
    bool   reallocate    = false;
    int    reallocRounds = 2;
    /// An agent whose own clusters are swept to this fraction has slack.
    double slackMarginThreshold   = 0.85;
    /// A cluster is unserviced when more than this fraction of its mass is left.
    double unservicedResidualFrac = 0.5;
    /// Low-yield length: information density below this fraction of the team mean.
    double slackYieldFrac         = 0.1;
    int    maxCandidatesPerCluster = 2;
    int    maxReallocTrials        = 4;
    double reallocEps              = 1e-4;  ///< required team-objective improvement
    std::vector<ReallocStrategy> reallocInitStrategies = {ReallocStrategy::Path,
                                                          ReallocStrategy::Direct};
    /// Path seed: [m] waypoint spacing along the optimised curve.
    double reallocPathWpSpacing = 100.0;
    /// Path seed: [m] length slack kept when repairing the budget (turning
    /// at each join, beyond the turn the estimate already counts).  Unset =
    /// 0.5 * pi * minTurnRadius per join.  The
    /// closed endpoint modes add 2.5 * pi * minTurnRadius of homing slack.
    std::optional<double> reallocPathMargin = std::nullopt;
    /// Path seed: re-optimise with the warm settings (exploreIterWarm); the
    /// seed already sits in the optimised curve's basin.  false = cold solve.
    bool   reallocPathWarm = true;
    /// Path seed: refine iterations of its solve (0 = optimizer.maxIter).
    /// The seed keeps most of the optimised curve, so half the usual solve
    /// loses next to nothing (benchmark: residual 0.0577 vs 0.0573 over six
    /// seeds) and halves the cost of a trial.
    int    reallocPathMaxIter = 75;
    /// Ordering solver settings: a lighter copy of cpp_planner's (2 deterministic
    /// starts, no ruin-and-recreate).
    OrienteeringParams orienteering = [] {
        OrienteeringParams o;
        o.nStarts = 2;
        o.nPerturb = 0;
        return o;
    }();
};

// =============================================================================
// 10b. INFORMATION-AWARE CLUSTERS  -> mapping::clusterByPeaks, curve_planning::info_aware
// =============================================================================
/// INFORMATION-AWARE ABSTRACTION SEARCH for the curve planner (off by default:
/// the plain radius clusters).  The port of the orienteering planner's
/// PlannerParams::infoAware (cpp_planner, AirStack mtl_search_planner), with
/// one change: a candidate abstraction is scored CHEAPLY, by the fast-model
/// team objective of the cluster-dependent SEED curves it produces ('clusters'
/// = orienteering order + pursuit, 'gcs' = graph-of-convex-sets route), not by
/// a full curve optimisation.  The winner (or the optimizeTopK best) is then
/// solved by the ordinary pipeline, so the curve, its length and curvature
/// guarantee are exactly the plain planner's.
///   1. PEAKS: every cell climbs to its highest lattice neighbour; basins
///      whose saddle is at least `persistence` x the lower peak are merged.
///   2. LEVELS: each basin is cut, densest cells first, at the cumulative-mass
///      fractions of a levelSets entry; each (basin, level) set is split under
///      the reach (k-means, k raised until every cell is within reach).
///   3. CANDIDATES: the plain clusters, k-means and every level set at
///      maxClusterRadius and at reachScales x the swath half-width, each with
///      `restarts` extra orienteering seeds; scored by the seed objective.
///   4. MOVES: PEEL / SPLIT / MERGE on the best candidate's visited clusters,
///      kept only if the seed objective falls.
struct InfoAwareParams {
    bool enabled = false;
    std::vector<std::vector<double>> levelSets = {{1.0}, {0.5, 1.0}, {0.35, 0.7, 1.0}};
    double persistence = 0.6;
    /// Reaches tried, as multiples of the calibrated swath half-width (the
    /// curve planner's detection reach); maxClusterRadius is always tried too.
    std::vector<double> reachScales = {1.0};
    bool includeKmeans = true;   ///< k-means clusters at each extra reach as candidates
    /// Extra orienteering / GCS-rounding seeds per candidate (0 = one).
    int  restarts = 0;
    int  maxMoves = 12;
    bool splitMerge = true;
    double peelKeep = 0.6;
    /// Candidates (distinct, best seed score first) that get the FULL solve;
    /// the lowest optimised team objective wins.  1 = cheapest.
    int  optimizeTopK = 1;
    /// Score every candidate with the cheap 'clusters' seed (orienteering
    /// order + pursuit) even when the flown seeds include 'gcs', whose route
    /// costs a conic solve per candidate.  false = score with the planner's
    /// own cluster-dependent seeds.
    bool scoreWithClusters = false;
    /// Score a candidate after a SHORT optimisation of its seeds instead of on
    /// the raw seeds: scoreExploreIters iterations on the exploration kernel,
    /// then scoreIters on the accurate one (0 / 0 = raw seed, the cheapest).
    /// The optimiser moves curves far from their seeds, so the raw seed
    /// objective predicts the optimised one only loosely; a short solve
    /// predicts it better at ~0.3-0.5 s per candidate.  Defaults = the tuned
    /// setting of the 2026-10-22 single-agent study (bench_ia_gcs/REPORT.md):
    /// reachScales {1}, scoreIters 40.
    int  scoreIters = 40;
    int  scoreExploreIters = 0;
};

// =============================================================================
// 10b. QUADROTOR CONVERSION (body-fixed camera)    -> quadrotor::generateQuadTrajectories
// =============================================================================
/// The port of init_curve_params.m section 8 (quadOpts) / quadResolveOpts.m.
///
/// The curve is planned for a fixed wing whose 1-DOF gimbal sweeps the
/// boresight across track.  A quadrotor with the camera FIXED to its body has
/// no gimbal: it points the camera by pointing the body, and its roll / pitch
/// ARE the direction of its thrust, i.e. of its acceleration.  Using the
/// differential flatness of the quadrotor (flat outputs x, y, z, yaw) the
/// conversion re-plans every agent so that the camera boresight lands on the
/// gimbal's ground points while the vehicle stays on the curve, inside the
/// attitude / rate / speed limits below.
///
/// Default vehicle: a 6-9 kg survey quad (DJI M300/M350, Freefly Astro,
/// large-PX4 class) flown inside a survey autopilot's envelope.
enum class CameraMount {
    Matched,  ///< tilt = sensorTiltAngle + cruise trim pitch: looks where the gimbal centre does
    Fixed,    ///< tilt = camTiltFixed
    Nadir     ///< straight down the body
};

struct QuadParams {
    /// Run the conversion in Planner::plan (PlanningResult::quad).  OFF by
    /// default in C++ so a host that only flies fixed wings pays nothing; the
    /// MATLAB reference (init_curve_params) turns it on.
    bool enabled = false;

    // --- vehicle -------------------------------------------------------------
    double speed            = 6.0;            ///< [m/s] cruise; the curve's timeline is stretched by avgDroneSpeed/speed
    double maxSpeed         = 17.0;           ///< [m/s]
    double maxTilt          = deg2rad(30);    ///< [rad] thrust axis from vertical
    double maxRollPitchRate = deg2rad(60);    ///< [rad/s] |(p, q)|
    double maxYawRate       = deg2rad(45);    ///< [rad/s] (PX4 MPC_YAWRAUTO_MAX)
    double maxYawAcc        = deg2rad(60);    ///< [rad/s^2]
    double thrustToWeight   = 2.0;
    double minThrustFrac    = 0.3;            ///< [x g]
    /// Isotropic quadratic drag a_drag = (c_d |v| + k_l) v, c_d calibrated so
    /// that dragSpecSpeed needs dragSpecTilt (0.0107 1/m: 2.2 deg trim at 6 m/s).
    double dragSpecSpeed = 23.0;
    double dragSpecTilt  = deg2rad(30);
    std::optional<double> dragQuad = std::nullopt;  ///< [1/m] explicit c_d
    double dragLin = 0.0;                           ///< [1/s] linear (rotor) drag
    double gravity = 9.81;

    // --- camera --------------------------------------------------------------
    CameraMount cameraMount = CameraMount::Matched;
    std::optional<double> camTiltFixed = std::nullopt;  ///< [rad] Fixed mount (default: sensorTiltAngle)
    double maxLookAngle    = deg2rad(75);    ///< [rad] boresight off-nadir hard cap ...
    double lookRangeMargin = 0.98;           ///< ... and acos(h / (margin * beta))
    double lookPlanMargin  = deg2rad(0.25);  ///< [rad] planned this far inside the look cap

    /// Yaw is a flat output: with a forward-tilted camera it swings the
    /// boresight across track without moving the vehicle.  false = yaw locked
    /// to the track heading (roll / pitch only; the reach shrinks to ~h tan(maxTilt)).
    bool useYaw = true;

    // --- spline / discretisation -----------------------------------------------
    int    splineDegree = 5;                 ///< quintic: snap-continuous, smooth body rates
    double knotSpacing  = 0.5;               ///< [s]
    double dtOpt        = 0.25;              ///< [s] residual sampling in the optimisation
    std::optional<double> dtOut = std::nullopt;  ///< [s] output step (default: PlannerParams::dt)

    // --- objective (1-sigma tolerances) ------------------------------------------
    double sigmaBore     = 5.0;              ///< [m] boresight ground point vs the gimbal's
    double sigmaCross    = 5.0;              ///< [m] cross-track deviation from the curve
    double sigmaAlong    = 10.0;             ///< [m] along-track (timing) deviation
    double sigmaYawReg   = deg2rad(180);     ///< [rad] weak pull of yaw toward the track
    double sigmaSnap     = 2.0;              ///< [m/s^4]
    double sigmaYawAcc   = deg2rad(30);      ///< [rad/s^2]
    double sigmaStartPos = 0.1;              ///< [m]
    double sigmaStartVel = 0.05;             ///< [m/s]

    // --- constraints (squared hinges, escalated until the true limits hold) -----
    double limitMargin     = 0.97;
    double hingeScaleAngle = deg2rad(0.25);
    double hingeScaleRate  = deg2rad(1.0);
    double hingeScaleSpeed = 0.1;
    int    penaltyRounds   = 4;
    double penaltyGrowth   = 10.0;

    // --- Levenberg-Marquardt ----------------------------------------------------
    int    maxIter = 80;
    double tolFun  = 1e-7;
    double lambda0 = 1e-3;
    bool   verbose = false;

    // --- derived by quadrotor::resolveQuadParams ----------------------------------
    double dragQuadResolved = 0.0;   ///< [1/m]
    double trimPitch        = 0.0;   ///< [rad] nose-down pitch that holds `speed`
    double camTilt          = 0.0;   ///< [rad] camera tilt from body -z toward body +x
    bool   resolved         = false;
};

// =============================================================================
// 11. THE WHOLE PARAMETER SET
// =============================================================================
struct PlannerParams {
    // --- run control -------------------------------------------------------
    std::uint64_t rngSeed = 21;
    bool          verbose = true;

    // --- environment -------------------------------------------------------
    double mapSize  = 5000.0;  ///< [m] side of the square search area
    double cellSize = 1.0;     ///< [m] belief-grid resolution

    // --- team --------------------------------------------------------------
    int numAgents = 3;
    int kmeansReplicatesAgents = 5;

    // --- aircraft ----------------------------------------------------------
    double droneAltitude = 300.0;  ///< [m] cruise altitude of agent 0 (see TeamParams::altitudeStagger)
    double avgDroneSpeed = 20.0;   ///< [m/s] constant ground speed
    double dt            = 0.1;    ///< [s] trajectory time step
    double minTurnRadius = 100.0;  ///< [m]

    // --- cell extraction and macro-clustering (same meaning as cpp_planner) --
    double targetCellSize    = 200.0;
    double minimumBeliefMass = 5e-5;
    double maxClusterRadius  = 500.0;
    ClusterParams cluster;

    // --- sensor payload and detection model --------------------------------
    double            fov = deg2rad(60);
    SensorModelParams sensor;
    double detectionThreshold = 0.95;
    /// [rad] mount tilt off nadir toward the forward horizon (init_params: 50 deg).
    double sensorTiltAngle = deg2rad(50);

    // --- endurance budget --------------------------------------------------
    double maxFlightTime     = 250.0;  ///< [s] per agent
    double maxFlightDistance = kInf;   ///< [m] per agent
    /// Optional per-agent override of the derived budget [m].
    std::vector<double> perAgentBudgetDist;

    GimbalParams    gimbal;
    SweepOptions    sweep;
    KernelParams    kernel;
    CurveParams     curve;
    OptimizerParams optimizer;
    TeamParams      team;
    InfoAwareParams infoAware;
    QuadParams      quad;

    // --- derived -------------------------------------------------------------
    double budgetDist() const {
        return std::min(maxFlightDistance, maxFlightTime * avgDroneSpeed);
    }
    double sensorStandOff() const { return droneAltitude * std::tan(sensorTiltAngle); }
    double agentAltitude(int a) const {
        return droneAltitude + static_cast<double>(a) * team.altitudeStagger;
    }

    /// Resolve every derived field and push the shared ones down.
    void finalize();
    /// Throw std::invalid_argument on a set that cannot produce a trajectory.
    void validate() const;
};

}  // namespace mtl::curve

#endif  // MTLC_PARAMS_HPP
