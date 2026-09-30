// =============================================================================
//  mtl_curve/types.hpp
//
//  Basic value types shared by the curve planner, the map generator and the
//  evaluation library.  Everything here is plain data: no algorithm in this
//  header, so a host simulation can include it to talk to the planner without
//  pulling in the solver.
//
//  BeliefField, CellSet, ClusterSet, Target and OrienteeringInfo are copied
//  verbatim from cpp_planner (mtl/types.hpp), in namespace mtl::curve so the
//  two packages can be linked into one host.  Everything below the "CURVE
//  PLANNER" banner is new.
//
//  Conventions (world frame, matching the MATLAB pipeline)
//    x East, y North, z Up.  Angles in radians.
//    yaw   +ve counter-clockwise from East, the heading of the ground track.
//    roll  +ve right wing down.
//    pitch +ve nose up; pitch is the flight-path angle plus trim AoA.
//    gimbal phi +ve looks right; mount tilt tau +ve looks forward.
// =============================================================================
#ifndef MTLC_TYPES_HPP
#define MTLC_TYPES_HPP

#include <Eigen/Core>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace mtl::curve {

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


// =============================================================================
//  CURVE PLANNER
// =============================================================================

enum class CurveRepresentation : int { Curvature = 0, BSpline = 1 };
enum class EndpointMode : int { Open = 0, ReturnHome = 1, FixedDest = 2 };
/// Seeds for an agent's first solve: pursue the ordered cluster centroids and
/// then continue greedily ('clusters', the plan), or greedy from the start.
enum class InitStrategy : int { Clusters = 0, Greedy = 1 };
/// Seeds tried when a slack agent claims a pooled cluster.
enum class ReallocStrategy : int { Insert = 0, Direct = 1 };

inline const char* toString(CurveRepresentation r) {
    return r == CurveRepresentation::Curvature ? "curvature" : "bspline";
}
inline const char* toString(EndpointMode m) {
    switch (m) {
        case EndpointMode::ReturnHome: return "return_home";
        case EndpointMode::FixedDest:  return "fixed_dest";
        default:                       return "open";
    }
}
inline const char* toString(InitStrategy s) { return s == InitStrategy::Clusters ? "clusters" : "greedy"; }
inline const char* toString(ReallocStrategy s) { return s == ReallocStrategy::Insert ? "insert" : "direct"; }

// -----------------------------------------------------------------------------
/// The cross-track sweep an agent's single-axis gimbal flies
/// (sensing::computeSweepParams):  alpha(t) = alphaMax * sin(2 pi freq t),
/// boresight at along = h tan(tau), cross = h tan(alpha) / cos(tau).
// -----------------------------------------------------------------------------
struct SweepParams {
    double tiltAngle = 0.0;         ///< [rad] mount tilt tau
    double alphaMax  = 0.0;         ///< [rad] sweep amplitude
    double freq      = 0.0;         ///< [Hz]
    double standOff  = 0.0;         ///< [m] h tan(tau)
    double crossMax  = 0.0;         ///< [m] cross-track offset at full deflection
    double halfWidthNominal = 0.0;  ///< [m] geometric limit (the plan's W_half for this mount)
    double peakRate  = 0.0;         ///< [rad/s] alphaMax * 2 pi freq
};

// -----------------------------------------------------------------------------
/// The fast objective's look kernel in the AIRCRAFT frame
/// (optimization::calibrateSwathKernel): log-miss deposited per metre flown on
/// a pixel at along-track offset a and cross-track offset d (left +).
// -----------------------------------------------------------------------------
struct SwathKernel {
    MatX   T;                  ///< nd-by-na table, T(i, j) at d = d0 + i*step, a = a0 + j*step
    double a0 = 0.0, d0 = 0.0, step = 5.0;
    double Rk = 0.0;           ///< [m] support radius about the aircraft (the stencil radius)
    double reach = 0.0;        ///< [m] physical horizontal reach sqrt(beta^2 - h^2)
    double halfWidth = 0.0;    ///< [m] one straight pass still detects with P >= 0.5 out to here
    double fitMaxErrP = 0.0;   ///< straight-line model error, in miss probability
    VecX   d, lambdaEff, lambdaModel;  ///< straight-line profiles (exact / fast model)
    double h = 0.0;            ///< [m] altitude it was calibrated for
    SweepParams sweep;
    bool   explore = false;    ///< the optimistic long-tailed exploration kernel
    /// [m] bounding box of the table's non-zero entries (a tilted mount sees
    /// nothing behind the aircraft) - lets the deposit skip half the stencil.
    double aLo = 0.0, aHi = 0.0, dLo = 0.0, dHi = 0.0;
    Index  na() const { return T.cols(); }
    Index  nd() const { return T.rows(); }
};

/// The accurate kernel and its exploration twin.
struct SwathKernelSet {
    SwathKernel accurate;
    SwathKernel explore;
};

// -----------------------------------------------------------------------------
/// Block-summed (mass-preserving) coarse copy of the prior the fast objective
/// integrates over (optimization::buildFastGrid).  Pixel (r, c) sits at
/// (x0 + c*hg, y0 + r*hg); rows index y like BeliefField.
// -----------------------------------------------------------------------------
struct FastGrid {
    MatX   prior;               ///< ny-by-nx, sums to 1 on a normalised prior
    double x0 = 0.0, y0 = 0.0;  ///< [m] centre of pixel (0, 0)
    double hg = 25.0;           ///< [m] pixel size
    Index  nx = 0, ny = 0;
    std::vector<int> six, siy;  ///< stencil offsets reaching Rk about a centre
    double Rk = 0.0;
    int    downsample = 1;      ///< belief pixels per coarse pixel, per side
    double x(Index c) const { return x0 + static_cast<double>(c) * hg; }
    double y(Index r) const { return y0 + static_cast<double>(r) * hg; }
    bool   empty() const { return prior.size() == 0; }
};

// -----------------------------------------------------------------------------
/// A curve's decision vector, bounds and endpoint regime
/// (curve_planning::enforceEndpointConstraints).
///
///   Curvature  v = [c (Nk); theta0].  kappa(s) = sum_m c_m hat_m(s),
///              theta(s) = theta0 + int kappa, gamma = pStart + int [cos, sin].
///              M equal segments of the budget length; sample i at the midpoint
///              of segment i.
///   BSpline    v = free control points, vec'ed [x; y].
// -----------------------------------------------------------------------------
struct CurveRep {
    CurveRepresentation type = CurveRepresentation::Curvature;
    double L = 0.0;                   ///< [m] budget length
    Vec2   pStart = Vec2::Zero();
    std::optional<Vec2> pGoal;        ///< set in ReturnHome / FixedDest
    EndpointMode mode = EndpointMode::Open;
    double kappaMax = 0.01;
    Index  nv = 0;
    VecX   lb, ub;                    ///< box bounds on v (lb == ub: fixed)

    // --- Curvature ---
    Index Nk = 0;
    VecX  knots;                      ///< Nk uniform knots on [0, L]
    Index M = 0;                      ///< optimisation segments

    // --- BSpline ---
    Index Nc = 0;
    int   p = 3;
    VecX  knotVec;                    ///< clamped uniform knot vector
    std::vector<char> fixed;          ///< Nc flags: pinned control points
    Path2 Pfix;                       ///< Nc-by-2, values of the pinned ones
    Index Mu = 0;                     ///< coverage samples (midpoints of Mu u-steps)
    double du = 0.0;
    MatX  B0, B1, B2;                 ///< Mu-by-Nc basis and derivatives at the samples
    MatX  BK1, BK2;                   ///< curvature-constraint set (~5 m)
    double kappaCon = 0.0;            ///< curvatureMargin * kappaMax
    MatX  B1s;                        ///< Simpson nodes (501) for the length
    VecX  wS;
    double curvaturePenaltyWeight = 0.0;

    bool hasEq() const { return pGoal.has_value(); }
};

// -----------------------------------------------------------------------------
/// A curve sampled for the objective or the trajectory
/// (curve_planning::evalParametricCurve).
// -----------------------------------------------------------------------------
struct CurveSamples {
    Path2 pts;        ///< M-by-2 sample points
    Path2 tan;        ///< unit tangents
    Path2 nrm;        ///< unit LEFT normals [-t_y, t_x]
    VecX  ds;         ///< arc length each sample represents (sums to the length)
    VecX  kappa;      ///< signed curvature at the samples
    VecX  s;          ///< arc length at each sample
    double L = 0.0;   ///< total arc length
    Vec2  endPt = Vec2::Zero();

    // Curvature internals
    VecX  theta;      ///< segment headings
    MatX  Phi;        ///< M-by-Nk integrals of the hat basis at the samples
    double dsSeg = 0.0;
    // BSpline internals
    Path2 r1, r2;     ///< r'(u), r''(u)
    VecX  sp;         ///< |r'|
    double du = 0.0;
    Path2 Pc;         ///< all control points
};

/// What one optimizeAgentCurve call did.
struct OptimizeInfo {
    double J0 = 0.0, J = 0.0;  ///< fast objective before / after
    int    iters = 0, fevals = 0;
    double ceqMax = 0.0, cinMax = 0.0;
    int    projSteps = 0;
    double projErrBefore = 0.0, projErrAfter = 0.0;
    double timeSec = 0.0;
    std::string exitMsg;
};

/// The service audit and slack the reallocation rounds are driven by.
struct ClusterAudit {
    VecX priorMass, residMass, residFrac;   ///< K-by-1 on the fast grid
    MatX distToCurve;                        ///< K-by-N centroid-to-curve distance [m]
    std::vector<char>  centroidInSwath;      ///< the plan's test, reported
    std::vector<Index> unserviced;           ///< residFrac > unservicedResidualFrac
    VecX slack;                              ///< low-yield length fraction per agent
    VecX ownCoverage;                        ///< share of each agent's own clusters swept
    std::vector<char> isSlack;
    double teamJ = 0.0;                      ///< fast team residual
};

/// One reallocation trial.
struct ReallocTrial {
    int    round = 0;
    Index  cluster = -1;
    int    agent = -1;
    double Jold = 0.0, Jnew = 0.0;
    bool   accepted = false;
    std::string strategy;
};

// -----------------------------------------------------------------------------
/// One agent's planned curve and the audit trail behind it.
// -----------------------------------------------------------------------------
struct AgentPlan {
    int    agentId = 0;
    Vec2   start = Vec2::Zero();
    CurveRep rep;                      ///< the representation (decision vector layout)
    VecX   v;                          ///< the optimised curve parameters
    VecX   vStatic;                    ///< the curve before reallocation
    std::vector<Index> waypointClusters;  ///< cluster order the seed pursued (global ids)
    std::string initStrategy;          ///< the seed that won ("clusters" / "greedy" / realloc)
    double altitude = 0.0;
    SweepParams sweep;
    double swathHalfWidth = 0.0;       ///< [m] calibrated single-pass P >= 0.5 half-width

    double budget = 0.0;
    double flownLength = 0.0;          ///< [m] measured on the flown 2 m track
    double flightTime  = 0.0;
    double budgetUsed  = 0.0;
    double maxKappa    = 0.0;          ///< [1/m] discrete, on the flown track
    double endpointError = 0.0;        ///< [m] to pGoal (0 in Open mode)
    bool   feasible = true;            ///< length, curvature and endpoint within tolerance
    OptimizeInfo lastOpt;
    Path2  seedPath;                   ///< the flown seed polyline (diagnostic)
    /// Global cells whose CENTRE this agent's swath alone detects with P >= 0.5
    /// on the fast model (the swath definition) - the counterpart of
    /// cpp_planner's serviced cells, for hosts that report per-cell coverage.
    std::vector<Index> servicedCellIdx;
};

// -----------------------------------------------------------------------------
/// One agent's flight-ready output, padded onto the team-wide timeline.  The
/// drone / sensor / rpy layout is cpp_planner's AgentTrajectory, so the copied
/// eval library scores it unchanged.
// -----------------------------------------------------------------------------
struct AgentTrajectory {
    Path3 drone;           ///< N-by-3 [x y h]
    Path2 sensor;          ///< N-by-2 boresight ground point
    Path3 rpy;             ///< N-by-3 [roll pitch yaw]
    VecX  gimbalAngle;     ///< [rad] sweep angle alpha (level frame, + left)
    VecX  gimbalCmd;       ///< [rad] alpha + roll: what the gimbal is commanded
    Index steps = 0;       ///< samples before padding
    double length = 0.0;   ///< [m] flown polyline length
    double maxKappaDiscrete = 0.0;
    double maxGimbalCmdDeg = 0.0, maxRollDeg = 0.0;
    Vec2  startPt = Vec2::Zero(), endPt = Vec2::Zero();
};

/// What the team solve did.
struct TeamInfo {
    std::vector<double>      Jhist;       ///< fast team objective after each stage
    std::vector<std::string> stageNames;
    double Jstatic = 0.0;                 ///< after the coordination sweeps
    double Jfinal  = 0.0;                 ///< after reallocation
    std::vector<ReallocTrial> log;
    ClusterAudit audit;                   ///< the final audit
    /// Cells whose centre the TEAM detects with P >= 0.5 (fast model), their
    /// complement, and the prior mass they carry (cpp_planner's TeamInfo names).
    std::vector<Index> servicedCellIdx, unservicedCellIdx;
    double info = 0.0, infoTotal = 0.0, infoFraction = 0.0;
    int    acceptedTrials() const {
        int n = 0;
        for (const ReallocTrial& t : log) n += t.accepted ? 1 : 0;
        return n;
    }
};

// -----------------------------------------------------------------------------
/// Everything one call to Planner::plan() produces.
// -----------------------------------------------------------------------------
struct PlanningResult {
    CellSet    cells;
    ClusterSet clusters;
    std::vector<int> agentOfCluster;       ///< K-by-1 k-means partition (0-based agent)

    std::vector<AgentPlan>       plans;
    std::vector<AgentTrajectory> trajectories;
    TeamInfo team;

    VecX  timeVec;                         ///< team-wide timeline
    Index numSteps() const { return timeVec.size(); }

    /// The static-partition curves (before reallocation) flown the same way,
    /// for the static-vs-reallocated comparison.  Empty when reallocation is
    /// off or changed nothing.
    std::vector<AgentTrajectory> staticTrajectories;

    FastGrid grid;                         ///< the fast grid the plan was optimised on
    MatX     teamLambda;                   ///< its final team log-miss (fast model)
};
}  // namespace mtl::curve

#endif  // MTLC_TYPES_HPP
