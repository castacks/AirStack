#include "mtl_curve/curve_planning/info_aware.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "mtl_curve/core/kmeans.hpp"
#include "mtl_curve/mapping/cells.hpp"
#include "mtl_curve/mapping/peak_clusters.hpp"

namespace mtl::curve::curve_planning {
namespace {

std::string fracsLabel(const std::vector<double>& f) {
    std::ostringstream o;
    o << "[";
    for (std::size_t i = 0; i < f.size(); ++i) o << (i ? "," : "") << f[i];
    o << "]";
    return o.str();
}

double radiusOf(const CellSet& cells, const std::vector<Index>& mem) {
    if (mem.empty()) return 0.0;
    Vec2 c = Vec2::Zero();
    for (const Index g : mem) c += cells.centers.row(g).transpose();
    c /= static_cast<double>(mem.size());
    double r = 0.0;
    for (const Index g : mem) r = std::max(r, (cells.centers.row(g).transpose() - c).norm());
    return r;
}

ClusterSet rebuild(const CellSet& cells, const std::vector<std::vector<Index>>& mem, const std::vector<int>& basin,
                   const std::vector<int>& level) {
    std::vector<std::vector<Index>> m2;
    std::vector<int> b2, l2;
    for (std::size_t k = 0; k < mem.size(); ++k) {
        if (mem[k].empty()) continue;
        m2.push_back(mem[k]);
        if (basin.size() == mem.size()) b2.push_back(basin[k]);
        if (level.size() == mem.size()) l2.push_back(level[k]);
    }
    return mapping::clusterSetFromMembers(cells, m2, b2, l2);
}

}  // namespace

std::vector<AbstractionCandidate> infoAwareCandidates(const CellSet& cells, const ClusterSet& plain,
                                                      double hw, const PlannerParams& P, Index* nBasins) {
    const InfoAwareParams& ia = P.infoAware;
    std::vector<AbstractionCandidate> out;
    {
        AbstractionCandidate c;
        c.label = "plain";
        c.reach = P.maxClusterRadius;
        c.clusters = plain;
        if (c.clusters.reward.size() != c.clusters.size() ||
            static_cast<Index>(c.clusters.cellIdx.size()) != c.clusters.size())
            mapping::computeClusterRewards(cells.mass, c.clusters);
        out.push_back(std::move(c));
    }
    const mapping::PeakBasins basins = mapping::findPeakBasins(cells, ia.persistence);
    if (nBasins) *nBasins = basins.size();

    std::vector<double> reaches{P.maxClusterRadius};
    for (const double s : ia.reachScales) {
        const double r = s * hw;
        bool dup = false;
        for (const double q : reaches) dup = dup || std::abs(q - r) < 1.0;
        if (!dup && r > 0.0) reaches.push_back(r);
    }
    for (std::size_t ri = 0; ri < reaches.size(); ++ri) {
        const double r = reaches[ri];
        const std::string at = "@" + std::to_string(static_cast<long long>(std::lround(r)));
        if (ri > 0 && ia.includeKmeans) {
            AbstractionCandidate c;
            c.label = "kmeans" + at;
            c.reach = r;
            c.clusters = mapping::clusterCells(cells.centers, r, P.cluster, P.rngSeed, false);
            mapping::computeClusterRewards(cells.mass, c.clusters);
            out.push_back(std::move(c));
        }
        for (const std::vector<double>& ls : ia.levelSets) {
            AbstractionCandidate c;
            c.label = "peaks" + fracsLabel(ls) + at;
            c.reach = r;
            c.clusters = mapping::clusterByPeaks(cells, basins, ls, r, P.cluster, P.rngSeed);
            out.push_back(std::move(c));
        }
    }
    return out;
}

std::vector<std::pair<std::string, ClusterSet>> infoAwareMoves(const ClusterSet& cl, const std::vector<Index>& visited,
                                                               const CellSet& cells, double reach,
                                                               const InfoAwareParams& ia, const ClusterParams& cp,
                                                               std::uint64_t seed) {
    std::vector<std::pair<std::string, ClusterSet>> moves;
    const std::vector<std::vector<Index>>& mem = cl.cellIdx;
    const auto K = static_cast<Index>(mem.size());
    std::vector<int> basin = cl.basin, level = cl.level;
    if (static_cast<Index>(basin.size()) != K) basin.assign(static_cast<std::size_t>(K), -1);
    if (static_cast<Index>(level.size()) != K) level.assign(static_cast<std::size_t>(K), 0);

    std::vector<char> chosen(static_cast<std::size_t>(K), 0);
    for (const Index c : visited)
        if (c >= 0 && c < K) chosen[static_cast<std::size_t>(c)] = 1;
    std::vector<Index> order;
    for (Index k = 0; k < K; ++k)
        if (chosen[static_cast<std::size_t>(k)]) order.push_back(k);
    std::stable_sort(order.begin(), order.end(), [&](Index a, Index b) { return cl.reward(a) > cl.reward(b); });

    for (const Index c : order) {
        const std::vector<Index>& mc = mem[static_cast<std::size_t>(c)];
        // PEEL
        if (mc.size() >= 3) {
            std::vector<Index> s = mc;
            std::stable_sort(s.begin(), s.end(), [&](Index a, Index b) { return cells.mass(a) > cells.mass(b); });
            const double tot = cl.reward(c);
            std::vector<Index> keep, rest;
            double cum = 0.0;
            for (const Index g : s) {
                (cum < ia.peelKeep * tot - 1e-15 || keep.empty() ? keep : rest).push_back(g);
                cum += cells.mass(g);
            }
            if (!rest.empty()) {
                auto m2 = mem;
                auto b2 = basin, l2 = level;
                m2[static_cast<std::size_t>(c)] = keep;
                for (auto& part : mapping::splitUnderRadius(cells, rest, reach, cp, seed + 31ULL * static_cast<std::uint64_t>(c))) {
                    m2.push_back(std::move(part));
                    b2.push_back(basin[static_cast<std::size_t>(c)]);
                    l2.push_back(level[static_cast<std::size_t>(c)] + 1);
                }
                moves.emplace_back("peel", rebuild(cells, m2, b2, l2));
            }
        }
        // SPLIT
        if (mc.size() >= 2) {
            Path2 pts(static_cast<Index>(mc.size()), 2);
            for (std::size_t i = 0; i < mc.size(); ++i) pts.row(static_cast<Index>(i)) = cells.centers.row(mc[i]);
            core::KMeansOptions ko;
            ko.maxIter = cp.kmeansMaxIter;
            ko.replicates = cp.kmeansReplicates;
            ko.seed = seed + 17ULL * static_cast<std::uint64_t>(c);
            const core::KMeansResult r = core::kmeans(pts, 2, ko);
            std::vector<Index> a, b;
            for (std::size_t i = 0; i < mc.size(); ++i) (r.assignment[i] == 0 ? a : b).push_back(mc[i]);
            if (!a.empty() && !b.empty()) {
                auto m2 = mem;
                auto b2 = basin, l2 = level;
                m2[static_cast<std::size_t>(c)] = a;
                m2.push_back(b);
                b2.push_back(basin[static_cast<std::size_t>(c)]);
                l2.push_back(level[static_cast<std::size_t>(c)]);
                moves.emplace_back("split", rebuild(cells, m2, b2, l2));
            }
        }
        // MERGE with the nearest unvisited cluster whose union still fits
        Index best = -1;
        double bestD = kInf;
        for (Index d = 0; d < K; ++d) {
            if (d == c || chosen[static_cast<std::size_t>(d)]) continue;
            const double dist = (cl.centroids.row(c) - cl.centroids.row(d)).norm();
            if (dist >= bestD || dist > 2.0 * reach) continue;
            std::vector<Index> u = mc;
            u.insert(u.end(), mem[static_cast<std::size_t>(d)].begin(), mem[static_cast<std::size_t>(d)].end());
            if (radiusOf(cells, u) <= reach) {
                bestD = dist;
                best = d;
            }
        }
        if (best >= 0) {
            auto m2 = mem;
            m2[static_cast<std::size_t>(c)].insert(m2[static_cast<std::size_t>(c)].end(),
                                                   mem[static_cast<std::size_t>(best)].begin(),
                                                   mem[static_cast<std::size_t>(best)].end());
            m2[static_cast<std::size_t>(best)].clear();
            moves.emplace_back("merge", rebuild(cells, m2, basin, level));
        }
    }
    return moves;
}

}  // namespace mtl::curve::curve_planning
