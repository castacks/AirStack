// =============================================================================
//  mtl/types.hpp
//
//  Basic value types shared by the planner, the map generator and the
//  evaluation library.  Everything here is plain data: no algorithm in this
//  header, so a host simulation can include it to talk to the planner without
//  pulling in the solver.
//
//  Conventions (world frame, matching the MATLAB pipeline)
//    x East, y North, z Up.  Angles in radians.
//    yaw   +ve counter-clockwise from East, the heading of the ground track.
//    roll  +ve right wing down.
//    pitch +ve nose up; pitch is the flight-path angle plus trim AoA.
//    gimbal phi +ve looks right; mount tilt tau +ve looks forward.
// =============================================================================
#ifndef MTL_TYPES_HPP
#define MTL_TYPES_HPP

#include <Eigen/Core>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace mtl {

using Index = Eigen::Index;

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using VecX = Eigen::VectorXd;
using MatX = Eigen::MatrixXd;

/// N-by-2 list of planar points (waypoints, cell centres, ground tracks).
using Path2 = Eigen::Matrix<double, Eigen::Dynamic, 2>;
/// N-by-3 list of spatial points (drone trajectory [x y h], attitude [r p y]).
using Path3 = Eigen::Matrix<double, Eigen::Dynamic, 3>;

inline constexpr double kInf = std::numeric_limits<double>::infinity();
inline constexpr double kPi  = 3.14159265358979323846;

inline constexpr double deg2rad(double d) { return d * kPi / 180.0; }
inline constexpr double rad2deg(double r) { return r * 180.0 / kPi; }

// -----------------------------------------------------------------------------
/// A sampled belief field over a regular grid spanning [0, mapSize] in x and y.
///
/// `values(r, c)` is the prior probability at (x = c * cellSize, y = r * cellSize),
/// i.e. rows index y and columns index x, exactly as MATLAB's meshgrid layout.
///
/// The reference prior (mapgen::generateBeliefMap) is a probability MASS
/// function over the grid: values.sum() == 1, so values(r, c) is the probability
/// that the target is in that pixel.  Consumers that need probabilities
/// (mapping::extractValidCells, eval::computeResidualBelief) normalise a host
/// grid that does not sum to 1 themselves, so a host may pass any non-negative
/// field.
// -----------------------------------------------------------------------------
struct BeliefField {
    MatX   values;            ///< (mapSize/cellSize + 1)^2 grid of prior belief (sums to 1)
    double mapSize  = 0.0;    ///< [m] side of the square search area
    double cellSize = 1.0;    ///< [m] grid resolution

    Index  rows() const { return values.rows(); }
    Index  cols() const { return values.cols(); }
    /// [m] metres per grid pixel in x (the grid spans mapSize over cols()-1 steps)
    double gridResX() const { return mapSize / static_cast<double>(values.cols() - 1); }
    double gridResY() const { return mapSize / static_cast<double>(values.rows() - 1); }
    double x(Index c) const { return static_cast<double>(c) * gridResX(); }
    double y(Index r) const { return static_cast<double>(r) * gridResY(); }
    bool   empty() const { return values.size() == 0; }
};

// -----------------------------------------------------------------------------
/// The valid cells extracted from a belief field: the points the sensor must be
/// aimed at, each carrying the belief mass that makes it worth aiming at.
///
/// All arrays are M long and share one ordering.
// -----------------------------------------------------------------------------
struct CellSet {
    Path2 centers;                 ///< M-by-2 cell centres [m]
    VecX  mass;                    ///< belief mass: P(target in cell), sum of normalised pixels (the threshold test)
    VecX  massNorm;                ///< mass / sum(mass)
    VecX  meanBelief;              ///< mean normalised belief per pixel in the cell
    VecX  peakBelief;              ///< max normalised belief in the cell
    VecX  nPix;                    ///< grid pixels the cell block covered
    VecX  area;                    ///< [m^2] physical area of the cell block
    double totalMapMass  = 0.0;    ///< belief mass over the WHOLE map (1 for a normalised prior)
    double retainedMass  = 0.0;    ///< belief mass over the kept cells
    double minBeliefMass = 0.0;    ///< the threshold the cells were filtered with
    double cellSize      = 0.0;    ///< [m] side of the block the map was diced into
    Vec2   gridRes       = Vec2::Zero();

    Index size() const { return centers.rows(); }
    bool  empty() const { return centers.rows() == 0; }
};

// -----------------------------------------------------------------------------
/// The macro-cluster abstraction the route planner reasons about.
// -----------------------------------------------------------------------------
struct ClusterSet {
    Path2                           centroids;    ///< K-by-2 macro waypoints
    std::vector<int>                cellCluster;  ///< M-by-1 cluster index per cell (0-based)
    VecX                            reward;       ///< K-by-1 summed mass per cluster
    std::vector<std::vector<Index>> cellIdx;      ///< K lists of the cells each cluster owns
    double                          maxRadius = 0.0;  ///< achieved max cell-to-centroid distance

    Index size() const { return centroids.rows(); }
    bool  empty() const { return centroids.rows() == 0; }
};

// -----------------------------------------------------------------------------
/// A ground-truth target.  Ground truth only: the planner never sees these.
// -----------------------------------------------------------------------------
struct Target {
    Vec2   pose          = Vec2::Zero();
    double detectionProb = 0.0;
    double pMissTotal    = 1.0;
    double detectionTime = std::numeric_limits<double>::quiet_NaN();  ///< NaN = never detected
    bool   detected() const { return detectionTime == detectionTime; }
};

// -----------------------------------------------------------------------------
/// The score a route is judged by: aggregate prior belief put under the sensor.
// -----------------------------------------------------------------------------
struct InfoScore {
    double info             = 0.0;   ///< J - aggregate mass serviced
    double infoTotal        = 0.0;   ///< aggregate mass over ALL valid cells (the ceiling)
    double infoFraction     = 0.0;
    double infoRealized     = std::numeric_limits<double>::quiet_NaN();
    double realizedFraction = std::numeric_limits<double>::quiet_NaN();
    double infoMapFraction  = 0.0;   ///< includes the sub-threshold prior
    double totalMapMass     = std::numeric_limits<double>::quiet_NaN();
    Index  nServiced        = 0;
    Index  nTotal           = 0;
    Index  nRealized        = 0;
    double routeLength      = std::numeric_limits<double>::quiet_NaN();
    double budget           = std::numeric_limits<double>::quiet_NaN();
    double budgetUsed       = std::numeric_limits<double>::quiet_NaN();
    double efficiency       = std::numeric_limits<double>::quiet_NaN();  ///< info per metre
};

// -----------------------------------------------------------------------------
/// What the budgeted orienteering solver did.
// -----------------------------------------------------------------------------
struct OrienteeringInfo {
    double             reward         = 0.0;
    double             rewardTotal    = 0.0;
    double             rewardFraction = 0.0;
    double             length         = 0.0;   ///< Euclidean tour length
    double             budget         = kInf;
    double             budgetUsed     = 0.0;
    std::vector<char>  selected;               ///< K-by-1 flags
    std::vector<Index> dropped;
    std::vector<Index> unreachable;            ///< pruned as individually infeasible
    bool               feasible  = true;
    int                nStarts   = 0;
    int                bestStart = 0;
    std::vector<double> startRewards;
    std::string        note;
};

/// What the Dubins re-costing loop around the solver did.
struct MacroRouteInfo {
    double             reward         = 0.0;
    double             rewardTotal    = 0.0;
    double             rewardFraction = 0.0;
    double             euclidLength   = 0.0;
    double             flownLength    = 0.0;   ///< MEASURED Dubins arc
    double             budget         = kInf;
    double             budgetUsed     = 0.0;
    std::vector<char>  selected;
    std::vector<Index> dropped;
    std::vector<Index> unreachable;
    int                calibIters   = 0;
    double             scale        = 1.0;
    bool               feasible     = true;
    bool               budgetActive = false;
    OrienteeringInfo   orienteering;
    std::string        note;
};

/// What the leftover-budget cell refinement bought.
struct CellRefineInfo {
    double             reward       = 0.0;   ///< mass added
    double             flownLength  = 0.0;
    double             lengthBefore = 0.0;
    double             ratio        = 1.0;   ///< Dubins inflation of the route
    bool               feasible     = true;
    int                nAdded       = 0;     ///< anchor WAYPOINTS spliced in
    int                nHarvested   = 0;     ///< cells attached to rows already flown
    int                nCellsServed = 0;
    int                nTrials      = 0;     ///< real length measurements spent
    double             gimbalReach  = 0.0;
    std::vector<Index> anchorSeeds;
    std::vector<Index> rejected;
    std::vector<Index> candidates;           ///< the shortlist that was offered
    std::string        note = "disabled";
};

/// One sweep lane appended by the lateral-coverage extension.
/// What the lateral-coverage extension did.
struct ExtensionInfo {
    bool               enabled        = false;
    bool               applied        = false;
    std::string        scope;
    double             arcNominalEnd  = 0.0;  ///< arc length where the nominal route ended
    std::vector<Index> candidateIdx;          ///< centres allowed to drive the extension
    std::vector<Index> residualBefore;        ///< never abeam on the nominal track
    std::vector<Index> residualAfter;         ///< still never abeam after extending
    double             extendDist = 0.0;  ///< [m] straight run-out that was flown
    double             extraDist  = 0.0;  ///< [m] total appended (same thing here)
    double             reachEff   = 0.0;
    double             alongTol   = 0.0;
    int                passes    = 0;
    Index              nNeverAbeamAllBefore = -1;
    Index              nNeverAbeamAllAfter  = -1;
    std::string        note = "extension disabled";
};

// -----------------------------------------------------------------------------
/// Why a centre the single-axis gimbal was asked to observe was not observed.
// -----------------------------------------------------------------------------
enum class MissReason : int {
    Observed       = 0,
    NeverOnLine    = 1,  ///< passed within reach but never on the boresight line
    OutOfReach     = 2,  ///< the track never passed close enough
    DoubleBooked   = 3   ///< reachable, but every instant collided with another service
};

/// Everything the gimbal scheduler can tell you about its own solution.
struct GimbalDiagnostics {
    Index  nTargets       = 0;
    Index  nTargetsHit    = 0;
    double targetCoverage = 1.0;
    VecX   targetErr;                    ///< closest the boresight came to each centre [m]
    double targetErrMax   = 0.0;
    std::vector<Index> targetServiceStep;
    std::vector<Index> targetMissedIdx;
    Path2  targetXY;
    double targetTol      = 0.0;

    // --- the mount ---
    double tiltAngle       = 0.0;
    double tiltAngleDeg    = 0.0;
    double nadirOffsetM    = 0.0;        ///< where a centred gimbal looks, on average
    VecX   lookAngle;                    ///< theta = tau - pitch, per step
    double lookAngleMaxDeg = 0.0;
    VecX   slantRange;
    double slantRangeMax   = 0.0;
    double slantRangeBudget = kInf;

    // --- how each centre fared ---
    Index  nObservableNominal = 0;
    Index  nReachableAbeam    = 0;
    Index  nOnBoresightLine   = 0;
    std::vector<Index>      nCandidates;
    std::vector<MissReason> missReason;
    Index  nMissNeverAbeam   = 0;
    Index  nMissOutOfReach   = 0;
    Index  nMissDoubleBooked = 0;
    VecX   closestApproachM;
    VecX   reachAtClosestM;
    Index  nFreeServices     = 0;
    VecX   serviceBias;
    VecX   servicePitchDeg;
    double servicePitchMaxDeg = 0.0;

    // --- platform ---
    double altMin = 0.0, altMax = 0.0, altRmsDev = 0.0;
    int    altBoxLimited = 0;
    double pitchMaxDeg = 0.0, pitchRateMaxDeg = 0.0, pitchRateLimitDeg = 0.0;
    double rollMaxDeg = 0.0, gimbalMaxDeg = 0.0, gimbalRateMaxDeg = 0.0;
    double crossAngleMaxDeg = 0.0, gimbalRateClipDeg = 0.0;
    double yawRepairMaxDeg = 0.0, pathShiftMax = 0.0;
    Index  nClimbFor = 0;

    // --- per-step signals (N long) ---
    VecX   gimbalAngle;                  ///< phi
    VecX   crossAngle;                   ///< alpha = roll + phi
    VecX   roll;
    VecX   lookForward;                  ///< h*tan(theta)
    VecX   lookCross;
    VecX   altError;
    VecX   footprintRadius;

    // --- boresight presentability ---
    double lookSpeedMax    = 0.0;
    double lookPathLength  = 0.0;
    double lookPathRatio   = 0.0;

    double hMin = 0.0, hMax = 0.0;       ///< the altitude box actually used
    double aoa  = 0.0;
    double maxPitch = 0.0, gimbalMax = 0.0, gimbalRate = 0.0, maxCrossAngle = 0.0;
    double maxRoll = 0.0, maxLookAngle = 0.0, maxSlantRange = kInf, dt = 0.1;
    std::string note;
};

// -----------------------------------------------------------------------------
/// One agent's planned sortie: the route, the sensor task list, the trajectory
/// and the audit trail that says what the budget could and could not buy.
// -----------------------------------------------------------------------------
struct AgentPlan {
    int    agentId = 0;

    Path2  droneRoute;                   ///< (1+n)-by-2 macro route, row 0 = launch point
    Path2  sensorTargets;                ///< S-by-2 cell centres in service order
    std::vector<Index> sensorMacroRow;   ///< S-by-1 route row each centre belongs to
    std::vector<Index> sensorCellIdx;    ///< S-by-1 global cell index of each centre

    Path3  droneTraj;                    ///< N-by-3 [x y h]
    Path2  sensorTraj;                   ///< N-by-2 boresight ground point
    VecX   timeVec;                      ///< N-by-1 [s]
    ExtensionInfo extInfo;

    std::vector<Index> selClusters;      ///< global cluster ids in visit order
    std::vector<Index> cellAnchors;      ///< cells the refinement pass got serviced
    std::vector<Index> servicedCellIdx;  ///< global cells the sensor is routed through
    std::vector<Index> droppedClusters;

    double flownLength = 0.0;
    double flightTime  = 0.0;
    double budget      = kInf;
    double budgetUsed  = 0.0;
    bool   feasible    = true;

    MacroRouteInfo routeInfo;
    CellRefineInfo refineInfo;
    int    outerIters = 0;
    double reserve    = 0.0;
    InfoScore score;

    bool grounded() const { return sensorTargets.rows() == 0; }
};

// -----------------------------------------------------------------------------
/// What the team as a whole achieved, de-duplicated across agents.
// -----------------------------------------------------------------------------
struct TeamInfo {
    double info         = 0.0;
    double infoTotal    = 0.0;
    double infoFraction = 0.0;
    std::vector<Index>  servicedCellIdx;
    std::vector<Index>  unservicedCellIdx;
    std::vector<Index>  reachedClusters;
    std::vector<Index>  unreachedClusters;
    std::vector<double> flownLength;     ///< per agent
    std::vector<double> budgetUsed;      ///< per agent, as a fraction
    std::vector<double> budgets;         ///< per agent
    std::vector<Index>  reallocated;     ///< clusters that changed hands
    int    rounds = 0;
    InfoScore score;
};

// -----------------------------------------------------------------------------
/// One agent's flight-ready output, padded onto the team-wide timeline.
// -----------------------------------------------------------------------------
struct AgentTrajectory {
    Path3 drone;                         ///< N-by-3 [x y h]
    Path2 sensor;                        ///< N-by-2 boresight ground point
    Path3 rpy;                           ///< N-by-3 [roll pitch yaw]
    std::vector<Index> realizedCellIdx;  ///< global cells the gimbal actually observed
    Path2 missedTargets;                 ///< centres the scheduler could not observe
    GimbalDiagnostics diagnostics;       ///< only meaningful in single-axis mode
    bool  scheduled = false;             ///< true when the gimbal scheduler ran
};

// -----------------------------------------------------------------------------
/// Everything one call to Planner::plan() produces.
// -----------------------------------------------------------------------------
struct PlanningResult {
    CellSet    cells;                    ///< the valid cells the planner worked from
    ClusterSet clusters;                 ///< the macro abstraction it built
    std::vector<int>       agentOfCluster;   ///< K-by-1 k-means partition (0-based agent)

    std::vector<AgentPlan>       plans;
    std::vector<AgentTrajectory> trajectories;
    TeamInfo   team;

    VecX  timeVec;                       ///< team-wide timeline, all agents padded onto it
    Index numSteps() const { return timeVec.size(); }

    Path2 reachedCentroids;              ///< clusters the team actually flies to
    Path2 droppedCentroids;              ///< clusters the budget refused
    Path2 servicedCenters;               ///< cells the sensor is routed through
    Path2 droppedCenters;                ///< cells nobody services

    /// Global cell indices the gimbal actually observed, de-duplicated.
    std::vector<Index> realizedCellIdx;
};

}  // namespace mtl

#endif  // MTL_TYPES_HPP
