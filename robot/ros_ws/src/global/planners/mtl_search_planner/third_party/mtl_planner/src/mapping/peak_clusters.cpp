#include "mtl/mapping/peak_clusters.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <tuple>
#include <vector>

#include "mtl/core/kmeans.hpp"

namespace mtl::mapping {
namespace {

/// Union-find over basins, tracking each set's highest peak.
struct Dsu {
    std::vector<int>   up;
    std::vector<Index> top;  ///< cell at the top of the set
    explicit Dsu(const std::vector<Index>& peaks) : up(peaks.size()), top(peaks) {
        std::iota(up.begin(), up.end(), 0);
    }
    int find(int a) {
        while (up[static_cast<std::size_t>(a)] != a) {
            up[static_cast<std::size_t>(a)] = up[static_cast<std::size_t>(up[static_cast<std::size_t>(a)])];
            a = up[static_cast<std::size_t>(a)];
        }
        return a;
    }
};

/// Lattice neighbours of every cell: centres closer than 1.5 cell sizes.  The
/// cells come from a regular dicing, so this is 8-connectivity; a spatial hash
/// keeps it linear in the number of cells.
std::vector<std::vector<Index>> latticeNeighbours(const CellSet& cells) {
    const Index m = cells.size();
    std::vector<std::vector<Index>> nb(static_cast<std::size_t>(m));
    if (m == 0) return nb;

    double cs = cells.cellSize;
    if (!(cs > 0.0)) {
        // No cell size recorded: use the smallest centre spacing.
        cs = kInf;
        for (Index i = 0; i < m; ++i)
            for (Index j = i + 1; j < m; ++j)
                cs = std::min(cs, (cells.centers.row(i) - cells.centers.row(j)).norm());
        if (!std::isfinite(cs) || !(cs > 0.0)) cs = 1.0;
    }
    const double lim = 1.5 * cs;

    const double x0 = cells.centers.col(0).minCoeff();
    const double y0 = cells.centers.col(1).minCoeff();
    auto key = [&](Index i) {
        return std::make_pair(static_cast<long long>(std::floor((cells.centers(i, 0) - x0) / cs + 0.5)),
                              static_cast<long long>(std::floor((cells.centers(i, 1) - y0) / cs + 0.5)));
    };
    std::vector<std::tuple<long long, long long, Index>> keyed;
    keyed.reserve(static_cast<std::size_t>(m));
    for (Index i = 0; i < m; ++i) {
        const auto k = key(i);
        keyed.emplace_back(k.first, k.second, i);
    }
    std::sort(keyed.begin(), keyed.end());
    auto lookup = [&](long long gx, long long gy, std::vector<Index>& out) {
        auto lo = std::lower_bound(keyed.begin(), keyed.end(), std::make_tuple(gx, gy, Index{0}));
        for (auto it = lo; it != keyed.end() && std::get<0>(*it) == gx && std::get<1>(*it) == gy; ++it)
            out.push_back(std::get<2>(*it));
    };

    std::vector<Index> cand;
    for (Index i = 0; i < m; ++i) {
        const auto k = key(i);
        cand.clear();
        for (long long dx = -2; dx <= 2; ++dx)
            for (long long dy = -2; dy <= 2; ++dy) lookup(k.first + dx, k.second + dy, cand);
        for (const Index j : cand) {
            if (j == i) continue;
            if ((cells.centers.row(i) - cells.centers.row(j)).norm() <= lim)
                nb[static_cast<std::size_t>(i)].push_back(j);
        }
        std::sort(nb[static_cast<std::size_t>(i)].begin(), nb[static_cast<std::size_t>(i)].end());
    }
    return nb;
}

}  // namespace

// -----------------------------------------------------------------------------
PeakBasins findPeakBasins(const CellSet& cells, double persistence) {
    PeakBasins out;
    const Index m = cells.size();
    out.basinOfCell.assign(static_cast<std::size_t>(m), -1);
    if (m == 0) return out;

    const std::vector<std::vector<Index>> nb = latticeNeighbours(cells);
    const VecX& w = cells.mass;

    // Steepest ascent: each cell points at its highest neighbour if that is
    // higher than itself.  Ties break on the lower index so the result is
    // deterministic and the pointers cannot cycle.
    auto higher = [&](Index a, Index b) {  // is a strictly above b
        return w(a) > w(b) || (w(a) == w(b) && a < b);
    };
    std::vector<Index> up(static_cast<std::size_t>(m));
    for (Index i = 0; i < m; ++i) {
        Index best = i;
        for (const Index j : nb[static_cast<std::size_t>(i)])
            if (higher(j, best)) best = j;
        up[static_cast<std::size_t>(i)] = best;
    }

    // Follow the pointers to the tops; one raw basin per local maximum.
    std::vector<int> rawOf(static_cast<std::size_t>(m), -1);
    std::vector<Index> peaks;
    for (Index i = 0; i < m; ++i) {
        if (up[static_cast<std::size_t>(i)] == i) {
            rawOf[static_cast<std::size_t>(i)] = static_cast<int>(peaks.size());
            peaks.push_back(i);
        }
    }
    for (Index i = 0; i < m; ++i) {
        Index r = i;
        while (up[static_cast<std::size_t>(r)] != r) r = up[static_cast<std::size_t>(r)];
        rawOf[static_cast<std::size_t>(i)] = rawOf[static_cast<std::size_t>(r)];
    }

    // Saddles: for each lattice edge that crosses two raw basins, its height is
    // the lower of its two cells.  Process the highest first (a merge tree) and
    // merge when the saddle is at least `persistence` x the lower of the two
    // current peaks.
    std::vector<std::tuple<double, int, int>> edges;
    for (Index i = 0; i < m; ++i)
        for (const Index j : nb[static_cast<std::size_t>(i)]) {
            if (j <= i) continue;
            const int a = rawOf[static_cast<std::size_t>(i)], b = rawOf[static_cast<std::size_t>(j)];
            if (a != b) edges.emplace_back(std::min(w(i), w(j)), a, b);
        }
    std::sort(edges.begin(), edges.end(), [](const auto& x, const auto& y) {
        if (std::get<0>(x) != std::get<0>(y)) return std::get<0>(x) > std::get<0>(y);
        return std::make_pair(std::get<1>(x), std::get<2>(x)) < std::make_pair(std::get<1>(y), std::get<2>(y));
    });

    Dsu dsu(peaks);
    for (const auto& [saddle, a0, b0] : edges) {
        const int a = dsu.find(a0), b = dsu.find(b0);
        if (a == b) continue;
        const Index ta = dsu.top[static_cast<std::size_t>(a)];
        const Index tb = dsu.top[static_cast<std::size_t>(b)];
        const double lowerPeak = std::min(w(ta), w(tb));
        if (saddle + 1e-300 < persistence * lowerPeak) continue;
        // merge the lower-topped set into the higher one
        const bool aHigh = higher(ta, tb);
        const int  keep = aHigh ? a : b, gone = aHigh ? b : a;
        dsu.up[static_cast<std::size_t>(gone)] = keep;
    }

    // Dense relabelling, ordered by peak height (basin 0 = the highest peak).
    std::vector<int> roots;
    for (int r = 0; r < static_cast<int>(peaks.size()); ++r)
        if (dsu.find(r) == r) roots.push_back(r);
    std::sort(roots.begin(), roots.end(), [&](int x, int y) {
        return higher(dsu.top[static_cast<std::size_t>(x)], dsu.top[static_cast<std::size_t>(y)]);
    });
    std::vector<int> dense(peaks.size(), -1);
    for (std::size_t k = 0; k < roots.size(); ++k) dense[static_cast<std::size_t>(roots[k])] = static_cast<int>(k);

    out.peakCell.resize(roots.size());
    out.mass.assign(roots.size(), 0.0);
    for (std::size_t k = 0; k < roots.size(); ++k) out.peakCell[k] = dsu.top[static_cast<std::size_t>(roots[k])];
    for (Index i = 0; i < m; ++i) {
        const int b = dense[static_cast<std::size_t>(dsu.find(rawOf[static_cast<std::size_t>(i)]))];
        out.basinOfCell[static_cast<std::size_t>(i)] = b;
        out.mass[static_cast<std::size_t>(b)] += w(i);
    }
    return out;
}

// -----------------------------------------------------------------------------
std::vector<std::vector<Index>> splitUnderRadius(const CellSet& cells,
                                                 const std::vector<Index>& members,
                                                 double maxRadius, const ClusterParams& opts,
                                                 std::uint64_t seed) {
    std::vector<std::vector<Index>> out;
    const auto n = static_cast<Index>(members.size());
    if (n == 0) return out;

    Path2 pts(n, 2);
    for (Index i = 0; i < n; ++i) pts.row(i) = cells.centers.row(members[static_cast<std::size_t>(i)]);

    core::KMeansOptions ko;
    ko.maxIter    = opts.kmeansMaxIter;
    ko.replicates = opts.kmeansReplicates;

    for (int k = 1;; ++k) {
        if (k >= static_cast<int>(n)) {
            for (const Index g : members) out.push_back({g});
            return out;
        }
        ko.seed = seed + static_cast<std::uint64_t>(k) * 7919ULL;
        const core::KMeansResult r = core::kmeans(pts, k, ko);
        if (core::maxClusterRadius(pts, r) <= maxRadius) {
            out.assign(static_cast<std::size_t>(r.centroids.rows()), {});
            for (Index i = 0; i < n; ++i)
                out[static_cast<std::size_t>(r.assignment[static_cast<std::size_t>(i)])].push_back(
                    members[static_cast<std::size_t>(i)]);
            out.erase(std::remove_if(out.begin(), out.end(),
                                     [](const std::vector<Index>& v) { return v.empty(); }),
                      out.end());
            return out;
        }
    }
}

// -----------------------------------------------------------------------------
ClusterSet clusterSetFromMembers(const CellSet& cells,
                                 const std::vector<std::vector<Index>>& members,
                                 const std::vector<int>& basin, const std::vector<int>& level) {
    ClusterSet out;
    const auto K = static_cast<Index>(members.size());
    out.centroids.resize(K, 2);
    out.reward = VecX::Zero(K);
    out.cellIdx.assign(static_cast<std::size_t>(K), {});
    out.cellCluster.assign(static_cast<std::size_t>(cells.size()), -1);
    out.maxRadius = 0.0;

    for (Index k = 0; k < K; ++k) {
        const std::vector<Index>& mem = members[static_cast<std::size_t>(k)];
        Vec2 c = Vec2::Zero();
        for (const Index g : mem) c += cells.centers.row(g).transpose();
        if (!mem.empty()) c /= static_cast<double>(mem.size());
        out.centroids.row(k) = c.transpose();
        for (const Index g : mem) {
            out.cellCluster[static_cast<std::size_t>(g)] = static_cast<int>(k);
            out.cellIdx[static_cast<std::size_t>(k)].push_back(g);
            out.reward(k) += cells.mass(g);
            out.maxRadius = std::max(out.maxRadius, (cells.centers.row(g).transpose() - c).norm());
        }
        std::sort(out.cellIdx[static_cast<std::size_t>(k)].begin(), out.cellIdx[static_cast<std::size_t>(k)].end());
    }
    if (static_cast<Index>(basin.size()) == K) out.basin = basin;
    if (static_cast<Index>(level.size()) == K) out.level = level;
    return out;
}

// -----------------------------------------------------------------------------
ClusterSet clusterByPeaks(const CellSet& cells, const PeakBasins& basins,
                          const std::vector<double>& levelFracs, double maxRadius,
                          const ClusterParams& opts, std::uint64_t seed) {
    std::vector<std::vector<Index>> members;
    std::vector<int> basinOf, levelOf;
    const Index nb = basins.size();
    const auto  nl = static_cast<int>(levelFracs.size());

    for (Index b = 0; b < nb; ++b) {
        // The basin's cells, densest first.
        std::vector<Index> mem;
        for (Index i = 0; i < cells.size(); ++i)
            if (basins.basinOfCell[static_cast<std::size_t>(i)] == static_cast<int>(b)) mem.push_back(i);
        std::stable_sort(mem.begin(), mem.end(),
                         [&](Index x, Index y) { return cells.mass(x) > cells.mass(y); });
        const double total = basins.mass[static_cast<std::size_t>(b)];

        // Cut at the cumulative fractions.  A cell goes to the first level whose
        // cut its cumulative mass (before it) has not yet reached, so a level is
        // never empty unless the one before it swallowed everything.
        std::vector<std::vector<Index>> byLevel(static_cast<std::size_t>(nl));
        double cum = 0.0;
        for (const Index i : mem) {
            int l = 0;
            while (l < nl - 1 && cum >= levelFracs[static_cast<std::size_t>(l)] * total - 1e-15) ++l;
            byLevel[static_cast<std::size_t>(l)].push_back(i);
            cum += cells.mass(i);
        }

        for (int l = 0; l < nl; ++l) {
            const auto& set = byLevel[static_cast<std::size_t>(l)];
            if (set.empty()) continue;
            const std::uint64_t s = seed + 1000003ULL * static_cast<std::uint64_t>(b + 1) +
                                    7919ULL * static_cast<std::uint64_t>(l);
            for (auto& part : splitUnderRadius(cells, set, maxRadius, opts, s)) {
                members.push_back(std::move(part));
                basinOf.push_back(static_cast<int>(b));
                levelOf.push_back(l);
            }
        }
    }

    ClusterSet out = clusterSetFromMembers(cells, members, basinOf, levelOf);

    // Parent: the nearest cluster one level up in the same basin.
    out.parent.assign(static_cast<std::size_t>(out.size()), -1);
    for (Index k = 0; k < out.size(); ++k) {
        if (out.level[static_cast<std::size_t>(k)] == 0) continue;
        double best = kInf;
        for (Index q = 0; q < out.size(); ++q) {
            if (out.basin[static_cast<std::size_t>(q)] != out.basin[static_cast<std::size_t>(k)]) continue;
            if (out.level[static_cast<std::size_t>(q)] != out.level[static_cast<std::size_t>(k)] - 1) continue;
            const double d = (out.centroids.row(q) - out.centroids.row(k)).norm();
            if (d < best) {
                best = d;
                out.parent[static_cast<std::size_t>(k)] = static_cast<int>(q);
            }
        }
    }
    return out;
}

}  // namespace mtl::mapping
