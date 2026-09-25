#include "mtl/core/kmeans.hpp"

#include <algorithm>
#include <limits>
#include <random>

namespace mtl::core {
namespace {

/// One Lloyd run from a k-means++ seeding.
KMeansResult runOnce(const Path2& pts, int k, int maxIter, std::mt19937_64& rng) {
    const Index n = pts.rows();
    KMeansResult r;
    r.centroids = Path2::Zero(k, 2);
    r.assignment.assign(static_cast<std::size_t>(n), 0);

    // --- k-means++ seeding ---
    std::uniform_int_distribution<Index> pick(0, n - 1);
    r.centroids.row(0) = pts.row(pick(rng));

    VecX d2(n);
    for (Index i = 0; i < n; ++i) d2(i) = (pts.row(i) - r.centroids.row(0)).squaredNorm();

    for (int c = 1; c < k; ++c) {
        const double total = d2.sum();
        Index chosen = 0;
        if (total <= 0.0) {
            chosen = pick(rng);
        } else {
            std::uniform_real_distribution<double> u(0.0, total);
            double target = u(rng);
            double acc    = 0.0;
            chosen        = n - 1;
            for (Index i = 0; i < n; ++i) {
                acc += d2(i);
                if (acc >= target) {
                    chosen = i;
                    break;
                }
            }
        }
        r.centroids.row(c) = pts.row(chosen);
        for (Index i = 0; i < n; ++i)
            d2(i) = std::min(d2(i), (pts.row(i) - r.centroids.row(c)).squaredNorm());
    }

    // --- Lloyd iterations ---
    std::vector<int> counts(static_cast<std::size_t>(k), 0);
    for (int it = 0; it < std::max(1, maxIter); ++it) {
        r.iterations = it + 1;
        bool changed = false;

        for (Index i = 0; i < n; ++i) {
            double best  = std::numeric_limits<double>::infinity();
            int    bestC = 0;
            for (int c = 0; c < k; ++c) {
                const double d = (pts.row(i) - r.centroids.row(c)).squaredNorm();
                if (d < best) {
                    best  = d;
                    bestC = c;
                }
            }
            if (r.assignment[static_cast<std::size_t>(i)] != bestC) {
                r.assignment[static_cast<std::size_t>(i)] = bestC;
                changed = true;
            }
        }

        Path2 sums = Path2::Zero(k, 2);
        std::fill(counts.begin(), counts.end(), 0);
        for (Index i = 0; i < n; ++i) {
            const int c = r.assignment[static_cast<std::size_t>(i)];
            sums.row(c) += pts.row(i);
            ++counts[static_cast<std::size_t>(c)];
        }

        for (int c = 0; c < k; ++c) {
            if (counts[static_cast<std::size_t>(c)] > 0) {
                r.centroids.row(c) = sums.row(c) / counts[static_cast<std::size_t>(c)];
                continue;
            }
            // Empty cluster: re-seed it on the point furthest from its own
            // centroid, the same repair MATLAB's 'singleton' action makes.
            double worst  = -1.0;
            Index  worstI = 0;
            for (Index i = 0; i < n; ++i) {
                const int    ci = r.assignment[static_cast<std::size_t>(i)];
                const double d  = (pts.row(i) - r.centroids.row(ci)).squaredNorm();
                if (d > worst && counts[static_cast<std::size_t>(ci)] > 1) {
                    worst  = d;
                    worstI = i;
                }
            }
            if (worst >= 0.0) {
                --counts[static_cast<std::size_t>(r.assignment[static_cast<std::size_t>(worstI)])];
                r.assignment[static_cast<std::size_t>(worstI)] = c;
                counts[static_cast<std::size_t>(c)] = 1;
                r.centroids.row(c) = pts.row(worstI);
                changed = true;
            }
        }

        if (!changed) {
            r.converged = true;
            break;
        }
    }

    r.sumSquares = 0.0;
    for (Index i = 0; i < n; ++i)
        r.sumSquares +=
            (pts.row(i) - r.centroids.row(r.assignment[static_cast<std::size_t>(i)])).squaredNorm();
    return r;
}

}  // namespace

KMeansResult kmeans(const Path2& points, int k, const KMeansOptions& opts) {
    KMeansResult best;
    const Index  n = points.rows();
    if (n == 0) return best;

    k = std::max(1, std::min<int>(k, static_cast<int>(n)));

    std::mt19937_64 rng(opts.seed);
    best.sumSquares = std::numeric_limits<double>::infinity();
    for (int rep = 0; rep < std::max(1, opts.replicates); ++rep) {
        KMeansResult r = runOnce(points, k, opts.maxIter, rng);
        if (r.sumSquares < best.sumSquares) best = std::move(r);
    }
    return best;
}

double maxClusterRadius(const Path2& points, const KMeansResult& result) {
    double worst = 0.0;
    for (Index i = 0; i < points.rows(); ++i) {
        const int c = result.assignment[static_cast<std::size_t>(i)];
        worst = std::max(worst, (points.row(i) - result.centroids.row(c)).norm());
    }
    return worst;
}

}  // namespace mtl::core
