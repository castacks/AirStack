#include "mtl/mapgen/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

namespace mtl::mapgen {

BeliefField generateBeliefMap(double mapSize, double cellSize, const BeliefMapParams& o,
                              std::uint64_t seed) {
    BeliefField field;
    field.mapSize  = mapSize;
    field.cellSize = cellSize;

    const auto n = static_cast<Index>(std::floor(mapSize / cellSize)) + 1;
    field.values = MatX::Zero(n, n);

    std::mt19937_64                        rng(seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_int_distribution<int>     sig(static_cast<int>(o.sigmaMin),
                                               static_cast<int>(o.sigmaMax));

    // Precompute the axis coordinates once: the grid is separable, so each bump
    // is an outer product of two 1-D Gaussians rather than an n^2 evaluation of
    // exp().  On a 5001 x 5001 grid that is the difference between a second and
    // a minute.
    VecX ax(n);
    for (Index i = 0; i < n; ++i) ax(i) = static_cast<double>(i) * cellSize;

    for (int c = 0; c < o.numCentroids; ++c) {
        const double mux = u01(rng) * mapSize;
        const double muy = u01(rng) * mapSize;
        const auto   sx  = static_cast<double>(sig(rng));
        const auto   sy  = static_cast<double>(sig(rng));

        VecX gx(n), gy(n);
        for (Index i = 0; i < n; ++i) {
            const double dx = ax(i) - mux;
            const double dy = ax(i) - muy;
            gx(i) = std::exp(-0.5 * dx * dx / (sx * sx));
            gy(i) = std::exp(-0.5 * dy * dy / (sy * sy));
        }
        // rows index y, columns index x - the MATLAB meshgrid layout.
        field.values.noalias() += o.maxPriorPeak * (gy * gx.transpose());
    }

    field.values = field.values.cwiseMin(o.beliefCap).cwiseMax(o.baseUncertainty);

    // Normalise to a probability mass function over the grid: the whole map sums
    // to 1, so every downstream mass (a cell's, a cluster's, a route's, the
    // residual after the search) is directly a probability.
    const double total = field.values.sum();
    if (!(total > 0.0))
        throw std::invalid_argument("mtl::mapgen::generateBeliefMap: the prior has no mass");
    field.values /= total;
    return field;
}

std::vector<Target> generateTargetPoses(const BeliefField& belief, int numTargets,
                                        std::uint64_t seed) {
    std::vector<Target> targets;
    if (belief.empty() || numTargets <= 0) return targets;

    // Flatten the map into a CDF.  Column-major, so the linear index maps back
    // to (row, col) the same way MATLAB's does.
    const Index rows = belief.rows();
    const Index cols = belief.cols();
    const Index n    = rows * cols;

    std::vector<double> cdf(static_cast<std::size_t>(n));
    double              acc = 0.0;
    for (Index c = 0; c < cols; ++c) {
        for (Index r = 0; r < rows; ++r) {
            acc += std::max(belief.values(r, c), 0.0);
            cdf[static_cast<std::size_t>(c * rows + r)] = acc;
        }
    }
    if (acc <= 0.0) return targets;
    for (double& v : cdf) v /= acc;

    std::mt19937_64                        rng(seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    targets.reserve(static_cast<std::size_t>(numTargets));
    for (int i = 0; i < numTargets; ++i) {
        const double v  = u01(rng);
        const auto   it = std::lower_bound(cdf.begin(), cdf.end(), v);
        const auto   li = static_cast<Index>(std::distance(cdf.begin(), it));
        const Index  idx = std::min(li, n - 1);
        const Index  r   = idx % rows;
        const Index  c   = idx / rows;

        Target t;
        t.pose = Vec2(belief.x(c), belief.y(r));
        targets.push_back(t);
    }
    return targets;
}

}  // namespace mtl::mapgen
