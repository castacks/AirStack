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
    double lambdaCap  = 15.0;   ///< log-miss saturation of the profile
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
    double orderBudgetFrac = 0.8;
    /// Gauss-Seidel passes over the team, each agent optimised with the others'
    /// swaths frozen in the joint objective.
    int    coordinationSweeps = 2;
    /// Multi-start seeds for each agent's first solve (best one kept).
    std::vector<InitStrategy> initStrategies = {InitStrategy::Clusters, InitStrategy::Greedy};

    bool   reallocate    = true;
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
    std::vector<ReallocStrategy> reallocInitStrategies = {ReallocStrategy::Insert,
                                                          ReallocStrategy::Direct};
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
