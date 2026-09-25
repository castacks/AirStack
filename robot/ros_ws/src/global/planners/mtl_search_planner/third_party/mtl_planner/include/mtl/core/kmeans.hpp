// =============================================================================
//  mtl/core/kmeans.hpp
//
//  k-means over 2-D points, standing in for MATLAB's kmeans(X, k, 'Replicates',
//  R, 'MaxIter', M).  Same contract: squared-Euclidean distance, k-means++
//  seeding, R independent restarts, the restart with the lowest within-cluster
//  sum of squares wins.
//
//  The planner uses it twice, for two different jobs:
//    * clustering::clusterCells raises K until every cell is within
//      maxClusterRadius of its centroid - that radius is the guarantee the
//      gimbal sweep of a cluster covers the cells the cluster owns;
//    * the team partition splits the macro-clusters geographically between
//      agents, which allocateBudgetedTeam then treats as a starting point
//      rather than a constraint.
//
//  An empty cluster is refilled with the point furthest from its own centroid,
//  which is what MATLAB does with its default 'EmptyAction','singleton'.
// =============================================================================
#ifndef MTL_CORE_KMEANS_HPP
#define MTL_CORE_KMEANS_HPP

#include <cstdint>
#include <vector>

#include "mtl/types.hpp"

namespace mtl::core {

struct KMeansResult {
    Path2            centroids;   ///< k-by-2
    std::vector<int> assignment;  ///< n-by-1 cluster index per point (0-based)
    double           sumSquares = 0.0;
    int              iterations = 0;
    bool             converged  = false;
};

struct KMeansOptions {
    int           maxIter    = 200;
    int           replicates = 3;
    std::uint64_t seed       = 0;  ///< seeds the private stream used for k-means++
};

/// Cluster `points` into `k` groups.  `k` is clamped to [1, points.rows()].
KMeansResult kmeans(const Path2& points, int k, const KMeansOptions& opts);

/// Largest distance from any point to its own centroid.
double maxClusterRadius(const Path2& points, const KMeansResult& result);

}  // namespace mtl::core

#endif  // MTL_CORE_KMEANS_HPP
