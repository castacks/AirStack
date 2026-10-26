// =============================================================================
//  mtl_curve/gcs/gcs.hpp
//
//  Graph-of-convex-sets seed - the port of functions/gcs/*.m.
//
//    kernelClusters   macro-clusters sized by the camera kernel: greedy peeling
//                     of the box sum the greedy seed scores with (pickGreedy)
//    convexSets       one polytope per cluster: Voronoi cell ∩ swath disc ∩ map
//    solveConic       SCS-standard-form conic program -> SCS
//    fixedOrderRoute  shortest polyline through sets in a GIVEN order (SOCP)
//    tour             the order AND the points: convex relaxation of the
//                     visit-every-set GCS (perspective SOCP), cutset separation,
//                     rounding, exact fixed-order re-solve, 2-opt/or-opt polish
//    planAgentGcs     one agent: tour through its clusters, drop clusters until
//                     the route (with turning) fits the budget
//
//  Built only with MTLC_ENABLE_GCS (the default); without SCS, solveConic
//  throws std::runtime_error.
// =============================================================================
#ifndef MTLC_GCS_GCS_HPP
#define MTLC_GCS_GCS_HPP

#include <optional>
#include <vector>

#include <Eigen/SparseCore>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::gcs {

// ---- kernel clusters ---------------------------------------------------------
struct KernelClusters {
    Path2 centroids;              ///< K-by-2, the best look point of each box
    VecX  info;                   ///< K-by-1, prior mass the box took
    Eigen::MatrixXi pixelCluster; ///< ny-by-nx cluster of each fast pixel, -1 = never taken
    double boxHalf = 0.0;         ///< [m] half-side of the box
};

/// repeat: B = boxSum(R, n), n = round(boxFrac hw / hg); c = argmax B (column-
/// major, as MATLAB); info = B(c); zero R in the box; until the best box holds
/// < minInfoFrac of the prior, coverFrac is taken, or maxClusters.
KernelClusters kernelClusters(const FastGrid& G, double hw, const KernelClusterParams& p);

// ---- convex sets ---------------------------------------------------------------
struct ConvexSet {
    Eigen::Matrix<double, Eigen::Dynamic, 2> H;   ///< unit outward normals
    VecX  h;                                      ///< H x <= h
    Path2 V;                                      ///< ccw vertices
    Vec2  c = Vec2::Zero();                       ///< the cluster centre
    double cheb = 0.0;                            ///< inscribed radius about c
};

/// Voronoi cell of centroid k ∩ inscribed nPoly-gon of radius r ∩ [xmin xmax ymin ymax].
std::vector<ConvexSet> convexSets(const Path2& centroids, double radius, const Eigen::Vector4d& bounds,
                                  int nPoly);
/// Same with one radius per centroid.
std::vector<ConvexSet> convexSets(const Path2& centroids, const VecX& radii, const Eigen::Vector4d& bounds,
                                  int nPoly);

// ---- conic solve -----------------------------------------------------------------
/// min c'x  s.t.  A x + s = b,  s in {0}^z x R+^l x Q^q[0] x ...
struct ConicProblem {
    VecX c;
    Eigen::SparseMatrix<double> A;   ///< column-major
    VecX b;
    int  z = 0, l = 0;
    std::vector<int> q;
};
struct ConicResult {
    VecX   x;
    bool   ok = false;
    int    status = 0;
    double obj = kInf;
    double time = 0.0;
};
ConicResult solveConic(const ConicProblem& P, const GcsParams& opts);

// ---- routes ----------------------------------------------------------------------
struct FixedOrderResult {
    Path2  pts;          ///< n-by-2, row k in sets[order[k]]
    double len = 0.0;    ///< route length (start .. pts .. goal)
};
/// minimise sum_k |x_k - x_(k-1)| (+ goal leg) + sum_k w_k |x_k - c_k|,  x_k in sets[order[k]]
/// centerWeights: indexed like `sets` (empty = none).
FixedOrderResult fixedOrderRoute(const std::vector<ConvexSet>& sets, const std::vector<Index>& order,
                                 const Vec2& pStart, const std::optional<Vec2>& pGoal,
                                 const std::vector<double>& centerWeights, const GcsParams& opts);

struct TourResult {
    std::vector<Index> order;   ///< indices into the sets passed
    Path2  pts;
    double len = 0.0;           ///< route length
    double lb  = 0.0;           ///< relaxation optimum (objective incl. centre pull)
    double gap = 0.0;
    int    nCuts = 0, nSolves = 0;
    double time = 0.0;
    bool   ok = true;
};
TourResult tour(const std::vector<ConvexSet>& sets, const Vec2& pStart, const std::optional<Vec2>& pGoal,
                const std::vector<double>& centerWeights, const GcsParams& opts);

struct AgentGcsResult {
    std::vector<Index> clusters;   ///< visit order (indices into all sets)
    Path2  wp;
    double len = 0.0;              ///< with turning overhead
    double lenEuclid = 0.0;
    double lb = 0.0, gap = 0.0;
    std::vector<Index> dropped;
    int    nSolves = 0;
    double time = 0.0;
    /// Coverage mode: the other distinct rounded routes, best first (for a
    /// multi-start of the curve optimiser, GcsParams::nSeeds).
    std::vector<Path2> altWp;
};
/// sets/info: ALL clusters; ids: this agent's.  budget [m]; Rmin [m].
AgentGcsResult planAgentGcs(const std::vector<ConvexSet>& sets, const VecX& info, const std::vector<Index>& ids,
                            const Vec2& pStart, const std::optional<Vec2>& pGoal, double budget, double Rmin,
                            const GcsParams& opts);

/// ORIENTEERING over convex sets (GcsMode::Orienteering): instead of touring
/// every set and dropping clusters until the route fits, choose WHICH sets to
/// visit inside the GCS.  Each set gets a visit variable p_v in [0, 1] (in- and
/// out-degree = p_v), the relaxed route length is a linear budget constraint
/// sum t_e <= budget, and the objective is  max sum info_v p_v  (minus
/// opLengthWeight per km).  Subtours are cut by generalised cutsets
/// y(delta-(S)) >= p_v.  Rounding walks the flow until the terminal; each walk
/// is re-solved exactly for its order, repaired to the budget (with turning)
/// by dropping the least information per metre, grown by cheapest insertion
/// while it fits, and 2-opt polished.  lb = the relaxation's information (an
/// upper bound), gap = 1 - route info / lb.  `warm`: extra visit orders
/// (global cluster ids, e.g. the orienteering heuristic's) evaluated exactly
/// alongside the flow walks.
AgentGcsResult planAgentGcsOrienteering(const std::vector<ConvexSet>& sets, const VecX& info,
                                        const std::vector<Index>& ids, const Vec2& pStart,
                                        const std::optional<Vec2>& pGoal, double budget, double Rmin,
                                        const GcsParams& opts,
                                        const std::vector<std::vector<Index>>* warm = nullptr);

// ---- coverage GCS (GcsMode::Coverage) -------------------------------------------------
/// The free space tiled into hexagonal convex cells about the swath size:
/// circumradius rho = cellFrac * hw, centres sqrt(3) rho apart, clipped to the
/// map.  Neighbours share an edge.  cover(c, v) = P_det of a straight pass
/// through cell v's centre at cell c's centre (the kernel's straight-pass
/// profile at their distance), kept where >= 0.05.
struct CoverageCells {
    std::vector<ConvexSet> sets;               ///< hexagons (sets[i].c = centre)
    Path2 centers;
    std::vector<std::vector<Index>> nbr;       ///< edge-adjacent cells
    Eigen::SparseMatrix<double> cover;         ///< n-by-n, (c, v)
    Eigen::MatrixXi pixelCell;                 ///< fast-grid pixel -> cell
    double rho = 0.0, spacing = 0.0;
    /// prior mass of each cell (weights W summed over its pixels)
    VecX cellMass(const MatX& W) const;
};
CoverageCells coverageCells(const FastGrid& G, const SwathKernel& K, double cellFrac,
                            const Eigen::Vector4d& bounds);

/// Coverage GCS: a path through ADJACENT cells (a graph of convex sets with
/// edges only between neighbours) that maximises the belief its swath covers,
///     max  sum_c info_c q_c - opLengthWeight * length[km]
///     s.t. q_c <= sum_v cover(c, v) p_v,  0 <= q_c <= 1     (coverage)
///          degree(v) = p_v in [0, 1], source out-degree 1, free end / goal
///          perspective points, conservation, t_e >= |z_e^v - z_e^u|
///          sum_e t_e <= budget,   y(delta-(S)) >= p_v (separated cutsets)
/// A visit is a pass through a swath-sized cell, so what the route sweeps
/// between "targets" counts, and a cell covered by a neighbour's pass is
/// not paid for twice.  Rounded by flow walks, each re-solved exactly for its
/// order (fixed-order SOCP), repaired to the budget (with turning) and grown
/// by cheapest insertion while it fits; the walk covering the most belief
/// (1 - prod(1 - cover), the true saturation) wins.
/// info: per cell (e.g. cellMass of the residual).  R.clusters = cell indices.
AgentGcsResult planAgentGcsCoverage(const CoverageCells& C, const VecX& info, const Vec2& pStart,
                                    const std::optional<Vec2>& pGoal, double budget, double Rmin,
                                    const GcsParams& opts);

/// Route length and the Rmin (D - sin D) turning overhead (free launch/arrival headings).
double routeLength(const Path2& P, const Vec2& pStart, const std::optional<Vec2>& pGoal);
double turnOverhead(const Path2& P, const Vec2& pStart, const std::optional<Vec2>& pGoal, double Rmin);

}  // namespace mtl::curve::gcs

#endif  // MTLC_GCS_GCS_HPP
