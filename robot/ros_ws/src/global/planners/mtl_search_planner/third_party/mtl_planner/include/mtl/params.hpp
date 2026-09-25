// =============================================================================
//  mtl/params.hpp
//
//  Single source of truth for every tunable in the pipeline - the C++ port of
//  init_params.m.  Each struct owns one stage of the pipeline and carries the
//  defaults that stage used to hold in-file, so a caller can construct a
//  Planner with nothing but `PlannerParams{}` and get the reference behaviour.
//
//  All of it is handed to Planner's constructor, which calls finalize() and
//  validate() once and then treats the set as read-only for the run.  Derived
//  quantities (budgetDist, sensorStandOff, extendForLateralCoverage) are
//  computed by finalize(), never set by hand.
// =============================================================================
#ifndef MTL_PARAMS_HPP
#define MTL_PARAMS_HPP

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "mtl/types.hpp"

namespace mtl {

// kPi, deg2rad and rad2deg live in mtl/types.hpp, so the numeric helpers can
// use them without pulling in the whole parameter set.

// =============================================================================
//  1. PRIOR BELIEF MAP        -> mapgen::generateBeliefMap
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
//  2. CELL EXTRACTION AND MACRO-CLUSTERING
// =============================================================================
struct ClusterParams {
    /// Iteration cap and restart count for the per-K k-means inside
    /// clustering::clusterCells.
    int kmeansMaxIter    = 200;
    int kmeansReplicates = 3;
};

// =============================================================================
//  3. ROUTING GEOMETRY
// =============================================================================
struct DubinsParams {
    /// [m] Arc-length spacing at which each Dubins segment is sampled.  This is
    /// the resolution of every measured route length in the budgeted planner,
    /// so it trades planning accuracy against planning cost.
    double stepSize = 2.5;
};

// =============================================================================
//  4. SENSOR DETECTION MODEL   -> eval::updateTargetDetectionProbs
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
//  5. ORIENTEERING SOLVER      -> planning::solveBudgetedOrienteering
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
//  6. MACRO ROUTE RE-COSTING   -> planning::planBudgetedMacroRoute
// =============================================================================
/// The orienteering solver reasons in straight lines; the aircraft flies Dubins
/// arcs, which are longer by a route-dependent factor.  This loop shrinks the
/// Euclidean budget handed to the solver until the MEASURED arc fits the real
/// budget, so the budget is a hard guarantee rather than an estimate.
struct MacroRouteParams {
    /// Calibration iterations.  Typically converges in 3 to 5.
    int    maxCalib = 12;
    /// Stop when the feasible / infeasible scale bracket is this tight.
    double scaleTol = 0.01;
    bool   verbose  = false;
};

// =============================================================================
//  7. LEFTOVER-BUDGET CELL REFINEMENT  -> planning::refineWithCellAnchors
// =============================================================================
/// Clusters are all-or-nothing, so the last few hundred metres of budget will
/// never buy a whole one.  This pass offers leftover CELL centres as anchors,
/// priced by mass per metre of detour, and is gimbal-aware on both halves of
/// that price: the aircraft only has to come within gimbalReach of a cell, and
/// once there it sweeps every leftover cell within gimbalReach of the anchor.
struct CellAnchorParams {
    /// [m] How far off the track the gimbal can service a cell.  Left unset it
    /// defaults to PlannerParams::maxClusterRadius - the radius clustering
    /// already guarantees a cluster is sweepable from - so anchors and clusters
    /// stay one kind of object.  0 restores overfly-the-cell behaviour.
    std::optional<double> gimbalReach = std::nullopt;
    /// Stand-off ladder tried for each candidate, as fractions of gimbalReach.
    ///   1  stop the aircraft the instant the cell enters gimbal reach
    ///   0  fly right over the cell - dearer, but centres a bigger ball on it
    std::vector<double> standoffTiers = {1.0, 0.5, 0.0};
    /// Attach, for zero extra distance, leftover cells already within reach of
    /// a waypoint the route flies anyway.
    bool   harvestFree = true;
    /// Cap on ANCHOR WAYPOINTS added.  Inf = as many as fit.  The free harvest
    /// is not capped - it costs nothing.
    double maxAdd = kInf;
    /// Cap on real Dubins length measurements, the expensive part of the pass.
    int    maxTrials = 40;
    /// Ignore an anchor whose whole reach ball is worth less than this fraction
    /// of the mean candidate mass.  Judged on the ball, not the seed cell.
    double minGainFrac = 0.0;
    /// [m] An anchor landing this close to a row the route already flies is
    /// merged into that row instead of duplicating it.
    double snapTol = 1.0;
    bool   verbose = false;
};

// =============================================================================
//  8. THE BUDGET OPTION SET    -> planning::allocateBudgetedTeam / planAgentSortie
// =============================================================================
struct BudgetParams {
    /// Spend leftover budget on cell anchors (the pass above).
    bool cellRefine = true;
    /// Re-offer clusters nobody reached to the agents with budget to spare.
    /// This turns the k-means partition from a hard constraint into a start.
    bool reallocate    = true;
    int  reallocRounds = 2;
    /// Re-plan passes in the reserve bisection that makes the FLOWN arc fit.
    int  maxOuterIter = 4;
    /// Initial hold-back, as a fraction of the budget, for the lateral-coverage
    /// extension, whose cost the route planner cannot see.
    double reserveFrac0 = 0.10;
    /// Relative overrun tolerated before a plan is called infeasible.
    double tol = 0.005;
    /// Shortlist size for the cell refinement: how many leftover cells are
    /// offered to it at all, ranked by reach-ball mass per metre of detour.
    int maxCellCandidates = 60;

    OrienteeringParams orienteering;
    MacroRouteParams   macroRoute;
    CellAnchorParams   cellAnchor;
    bool               verbose = false;
};

// =============================================================================
//  9. LATERAL COVERAGE EXTENSION -> trajectory::extendTrajForLateralCoverage
// =============================================================================
/// With a single-axis gimbal a centre can only be observed as it crosses the
/// swept cross-track line.  The Dubins route stops dead at the last centroid,
/// so cells of the final cluster ahead of that heading never cross it.  This
/// pass appends parallel sweep lanes that carry the aircraft past exactly those
/// centres and nothing else.
struct ExtensionParams {
    enum class Scope {
        Tail,  ///< only the final macro-cluster(s), the ones the route abandons
        All    ///< every centre this agent owns; expect roughly double the sortie
    };
    Scope scope        = Scope::Tail;
    /// How many trailing macro-clusters Scope::Tail covers.
    int   tailClusters = 1;

    // --- sensor envelope the LANE PLANNER assumes ---
    /// [m] Max usable cross-track ground offset.
    double maxSensorReach = 600.0;
    /// [rad] Gimbal travel from the mount axis, and max |roll + gimbal| off it.
    double gimbalMax     = deg2rad(80);
    double maxCrossAngle = deg2rad(85);
    /// Fraction of the reach the lanes plan inside.  NOTE the scheduler uses
    /// 0.97, so the lane planner is deliberately the more conservative of the
    /// two by about 12%.
    double reachMargin = 0.85;

    // --- how far to fly on.  THE knob. ---
    /// [m] Straight run-out flown past the end of the nominal route, holding the
    /// heading the route ended on.  A SET distance, deliberately not derived from
    /// the cells: the extension then costs the same on every sortie and the
    /// budget left for the route itself does not move around underneath the
    /// planner.
    double extendDist = 300.0;
    /// [deg] Pitch the audit is willing to assume, so a centre a few metres
    /// short of the line is not counted as a miss.
    double alongTolDeg = 2.5;
    /// [m] Along-track stand-off of the swept line.  DERIVED from the mount
    /// tilt by PlannerParams::finalize(); 0 = nadir.
    double alongOffset = 0.0;

    // --- sampling and caps ---
    double stepSize     = 2.5;   ///< [m] sampling of the appended path
    double maxExtraDist = kInf;  ///< [m] hard cap on the run-out
    bool   verbose      = false;
};

// =============================================================================
// 10. TRAJECTORY GENERATION   -> trajectory::generateTrajectories
// =============================================================================
struct TrajectoryParams {
    /// [s] Floor on the simulated sortie duration, so a degenerate two-waypoint
    /// route still produces a usable trajectory.
    double minSimTime = 10.0;
    /// [s] Dwell appended at the final waypoint.  A fixed wing cannot hover and
    /// every centre is observed in passing, so this is 0 by default.
    double hoverTime = 0.0;
    /// The gimbal sweep window for a macro-cluster opens while the aircraft is
    /// within this factor times maxClusterRadius of that cluster's centroid.
    double activationRadiusFactor = 1.5;
};

// =============================================================================
// 11. GIMBAL SCHEDULER        -> sensing::optimizeDroneSensorTraj
// =============================================================================
/// ONLY READ WHEN PlannerParams::singleAxisGimbal IS TRUE.  With a multi-axis
/// gimbal there is nothing to schedule, and only the platform block below is
/// used (by sensing::computeAirframeRPY).
struct GimbalSchedulerParams {
    // --- timing and tolerance ---
    /// [s] Must match the trajectory dt; finalize() keeps them in step.
    double dt = 0.1;
    /// false = maxPitchChangeRate is per TIME STEP.  true = per second.
    bool   pitchRateIsPerSecond = false;
    /// [m] How close the boresight must come to a centre for it to count as
    /// observed, and the "free" along-track band.  Pushing this below the
    /// ground-track sample spacing makes hits impossible to achieve.
    double targetTol = 5.0;
    /// [rad] Coverage metrics only - the scheduler aims the boresight, it does
    /// not integrate the footprint.
    double fov = deg2rad(60);

    // --- the mount ---
    /// [rad] Hard stop on |tilt - pitch|.  Past this the boresight is at the
    /// horizon and the ground intersection runs away, so the solver throws
    /// rather than quietly saturating.
    double maxLookAngle = deg2rad(85);
    /// [m] Boresight range budget.  A tilt costs range as 1/cos(theta), so a
    /// real sensor usually runs out of RANGE before the geometry runs out of
    /// reach.  Inf = geometry-limited only.
    double maxSlantRange = 610.0;

    // --- sensor envelope ---
    double gimbalMax     = deg2rad(80);   ///< [rad] gimbal travel from the mount axis
    double gimbalRate    = deg2rad(120);  ///< [rad/s] slew rate
    /// [rad] Max |roll + gimbal| off the mount axis.  Bank and gimbal rotate
    /// about the same axis and add, so a banked turn eats gimbal travel.
    double maxCrossAngle = deg2rad(85);
    double maxSensorReach = 600.0;        ///< [m] max usable cross-track ground offset
    double reachMargin    = 0.97;         ///< fraction of it the scheduler plans inside
    double aoa            = 0.0;          ///< [rad] trim AoA: gamma = pitch - aoa

    // --- step 1: candidate service instants ---
    /// [rad] THE knob.  How much AIRFRAME pitch a service may spend to slide
    /// off the instant the geometry hands it.  Measured from ZERO pitch, not
    /// from the tilt - the tilt is free, pitch is not.
    double pitchNudgeMax = deg2rad(5);
    /// [steps] Stride of the candidate walk, and the cap on candidates kept per
    /// crossing.  Together they bound the size of the scheduling problem.
    int nudgeStep      = 2;
    int maxCandPerPass = 24;

    // --- step 2: reach repair (bending the track, or climbing, for reach) ---
    /// Master switch for the yaw/climb hill-climbing loop.  OFF by default, so
    /// every parameter from here down to altGainDist is dead while it is false.
    bool   enableRepairLoop = false;
    bool   allowPathRepair  = true;
    double minTurnRadius    = 100.0;      ///< [m] curvature bound the yaw repair respects
    double repairBankMax    = deg2rad(20);
    double repairYawMax     = deg2rad(35);
    int    nRepair          = 6;
    double repairDamping    = 0.7;
    bool   allowClimb       = true;
    /// [m] Altitude box.  Both bounds are widened automatically to include the
    /// nominal cruise altitude, so they can never exclude the planned cruise.
    double hMin = 150.0;
    double hMax = 450.0;
    double climbPitch  = deg2rad(4);      ///< [rad] slope of the climb ramp
    double altGainDist = 300.0;           ///< [m] lookahead of the altitude tracker
    double maxPitch    = deg2rad(35);     ///< [rad] hard pitch stop (safety, not a target)

    // --- step 3: schedule ---
    /// Accept-only-if-better outer passes over schedule -> pitch -> re-schedule.
    int    nSchedule   = 6;
    double dwellSec    = 0.1;   ///< [s] time the boresight sits on a centre
    double nadirGapSec = 12.0;  ///< [s] longer gaps park the gimbal centred
    /// Peak / mean slope of the smoothstep joining two service knots; the
    /// scheduler checks each pair against gimbalRate*dt/slewPeak, which makes
    /// the finished profile rate-feasible by construction.
    double slewPeak = 1.5;

    // --- platform (also read by computeAirframeRPY in multi-axis mode) ---
    bool   computeRoll   = true;   ///< coordinated-turn bank rather than level flight
    bool   computePitch  = true;   ///< pitch = aoa + atan(dh/ds), else pinned to zero
    double maxRoll       = deg2rad(45);
    double rollSign      = 1.0;    ///< +1 or -1: sign convention for bank
    double rollSmoothSec = 1.5;    ///< [s] how quickly the aircraft is assumed to roll
    bool   verbose       = false;

    /// [rad] Mount tilt off nadir toward the forward horizon.  Set by
    /// PlannerParams::finalize() from PlannerParams::sensorTiltAngle.
    double tiltAngle = 0.0;
};

// =============================================================================
// 12. THE WHOLE PARAMETER SET
// =============================================================================
struct PlannerParams {
    // --- run control -------------------------------------------------------
    /// Seed for the planner's RNG stream (k-means restarts, and the belief-map
    /// and target sampling when the map generator shares it).
    std::uint64_t rngSeed = 21;
    /// Master console-output switch, threaded into every stage by finalize().
    bool verbose = true;

    // --- environment -------------------------------------------------------
    double mapSize  = 5000.0;  ///< [m] side of the square search area
    double cellSize = 1.0;     ///< [m] belief-grid resolution

    // --- team --------------------------------------------------------------
    int numAgents = 3;
    /// Restart count for the k-means that partitions macro-clusters between
    /// agents.  More restarts = a more stable partition run to run.
    int kmeansReplicatesAgents = 5;

    // --- aircraft ----------------------------------------------------------
    double droneAltitude = 300.0;  ///< [m] cruise altitude held by the trajectory generator
    double avgDroneSpeed = 20.0;   ///< [m/s] constant ground speed
    double dt            = 0.1;    ///< [s] simulation and trajectory time step
    double minTurnRadius = 100.0;  ///< [m] Dubins minimum turning radius
    /// [rad per TIME STEP] Max change in pitch per step (per second if
    /// gimbal.pitchRateIsPerSecond).
    double maxPitchChangeRate = deg2rad(1);

    // --- cell extraction and macro-clustering ------------------------------
    /// [m] Side of the blocks the map is diced into.  A block is kept as a
    /// valid cell when its BELIEF MASS (the probability the target is in it,
    /// on the prior normalised to sum to 1) beats minimumBeliefMass.
    double targetCellSize    = 200.0;
    /// Per-cell probability threshold.  5e-5 keeps every 200 m cell holding more
    /// than 0.005% of the map's belief - ~400-490 cells retaining ~99.8% of the
    /// reference prior, about what the retired mean-belief threshold of 0.005
    /// kept.  Scales with targetCellSize^2.
    double minimumBeliefMass = 5e-5;
    /// [m] Valid cells are clustered with K raised until every cell is within
    /// this radius of its centroid.  This sets the coarseness of the whole
    /// abstraction: bigger means fewer, fatter clusters and a blunter route.
    double maxClusterRadius = 500.0;
    ClusterParams cluster;
    DubinsParams  dubins;

    // --- sensor payload and detection model --------------------------------
    /// [rad] Full cone angle of the camera.  The ground footprint is a circle
    /// of radius slantRange*tan(fov/2) centred on the boresight intersection.
    double            fov = deg2rad(60);
    SensorModelParams sensor;
    /// Cumulative detection probability at which a target is declared FOUND.
    double detectionThreshold = 0.95;

    // --- sensor mount ------------------------------------------------------
    /// true  = 1-DOF cross-track gimbal.  A cell can only be seen at the
    ///         instant it crosses the swept line, which is what makes the
    ///         scheduling machinery necessary.
    /// false = multi-axis gimbal: the nominal trajectories are flown as planned
    ///         and the airframe attitude comes from computeAirframeRPY.
    bool   singleAxisGimbal = true;
    /// [rad] Tilt of the camera away from NADIR toward the forward horizon.
    /// Positive stands the swept line off h*tan(tilt) metres AHEAD, so a cell
    /// is observed BEFORE the aircraft draws level with it.  Costs slant range
    /// (as 1/cos) and cross-track reach (as cos), and costs NO altitude - the
    /// bracket is a mount, not a manoeuvre.  Only used in single-axis mode.
    double sensorTiltAngle = deg2rad(50);

    // --- endurance budget --------------------------------------------------
    double maxFlightTime     = 250.0;  ///< [s] per agent.  Inf = unlimited.
    double maxFlightDistance = kInf;   ///< [m] per agent.  Inf = unlimited.
    /// Optional per-agent override of the derived budget, in metres.  Empty =
    /// every agent gets budgetDist().
    std::vector<double> perAgentBudgetDist;

    BudgetParams          budget;
    ExtensionParams       extension;
    TrajectoryParams      traj;
    GimbalSchedulerParams gimbal;

    /// Print the full A-E geometry audit table for every agent.
    bool verifyGeometry        = true;
    bool verifyGeometryVerbose = true;

    // --- derived (set by finalize(), never by hand) -------------------------
    /// [m] Whichever of maxFlightDistance / maxFlightTime*speed is tighter.
    double budgetDist() const {
        return std::min(maxFlightDistance, maxFlightTime * avgDroneSpeed);
    }
    /// [m] Along-track stand-off of the swept line at cruise altitude.
    double sensorStandOff() const {
        return droneAltitude * std::tan(sensorTiltAngle);
    }
    /// Only a single-axis mount needs the lateral-coverage help.
    bool extendForLateralCoverage() const { return singleAxisGimbal; }
    bool budgeted() const { return std::isfinite(budgetDist()); }

    /// Resolve every derived field and push the shared ones (dt, verbose, the
    /// mount tilt, the turn radius, the reach) down into the sub-structs, so
    /// the stages cannot disagree about a value that has only one meaning.
    /// Planner's constructor calls this; call it yourself only if you build the
    /// sub-structs directly.
    void finalize();

    /// Throw std::invalid_argument on a set that cannot produce a trajectory.
    void validate() const;
};

}  // namespace mtl

#endif  // MTL_PARAMS_HPP
