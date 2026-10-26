// One agent's GCS route trimmed to the budget - port of gcsPlanAgent.m.
#include <algorithm>
#include <chrono>

#include "mtl_curve/gcs/gcs.hpp"

namespace mtl::curve::gcs {

AgentGcsResult planAgentGcs(const std::vector<ConvexSet>& sets, const VecX& info, const std::vector<Index>& ids,
                            const Vec2& pStart, const std::optional<Vec2>& pGoal, double budget, double Rmin,
                            const GcsParams& opts) {
    const auto t0 = std::chrono::steady_clock::now();
    AgentGcsResult R;
    std::vector<Index> kept = ids;
    double infoRef = 0.0;
    for (const Index k : ids) infoRef += info(k);
    infoRef = ids.empty() ? 1.0 : infoRef / static_cast<double>(ids.size());
    auto fullLen = [&](const Path2& P) { return routeLength(P, pStart, pGoal) + turnOverhead(P, pStart, pGoal, Rmin); };

    TourResult T;
    std::vector<Index> order;
    double len = 0.0;
    while (true) {
        std::vector<ConvexSet> sk;
        std::vector<double> wk;
        for (const Index k : kept) {
            sk.push_back(sets[static_cast<std::size_t>(k)]);
            if (opts.centerWeight > 0.0)
                wk.push_back(opts.centerWeightByInfo ? opts.centerWeight * info(k) / std::max(infoRef, 1e-300)
                                                     : opts.centerWeight);
        }
        T = tour(sk, pStart, pGoal, wk, opts);
        R.nSolves += T.nSolves;
        order.clear();
        for (const Index i : T.order) order.push_back(kept[static_cast<std::size_t>(i)]);
        len = T.len + turnOverhead(T.pts, pStart, pGoal, Rmin);
        if (len <= budget || kept.empty()) break;
        // ---- drop (one, or a batch on the current points) ---------------------
        Path2 P = T.pts;
        std::vector<Index> ord = order;
        while (true) {
            std::size_t j = 0;
            if (opts.dropRule == GcsDropRule::Info) {
                for (std::size_t i = 1; i < ord.size(); ++i)
                    if (info(ord[i]) < info(ord[j])) j = i;
            } else {
                double best = kInf;
                for (std::size_t i = 0; i < ord.size(); ++i) {
                    Path2 Pi(P.rows() - 1, 2);
                    for (Index r = 0, q = 0; r < P.rows(); ++r)
                        if (r != static_cast<Index>(i)) Pi.row(q++) = P.row(r);
                    const double saved = len - fullLen(Pi);
                    const double sc = info(ord[i]) / std::max(saved, 1e-9);
                    if (sc < best) {
                        best = sc;
                        j = i;
                    }
                }
            }
            R.dropped.push_back(ord[j]);
            ord.erase(ord.begin() + static_cast<std::ptrdiff_t>(j));
            Path2 P2(P.rows() - 1, 2);
            for (Index r = 0, q = 0; r < P.rows(); ++r)
                if (r != static_cast<Index>(j)) P2.row(q++) = P.row(r);
            P = P2;
            len = fullLen(P);
            if (!opts.dropBatch || len <= budget || ord.empty()) break;
        }
        std::vector<Index> nk;
        for (const Index k : kept)
            if (std::find(R.dropped.begin(), R.dropped.end(), k) == R.dropped.end()) nk.push_back(k);
        kept = nk;
    }
    R.clusters = order;
    R.wp = T.pts;
    R.len = len;
    R.lenEuclid = T.len;
    R.lb = T.lb;
    R.gap = T.gap;
    R.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return R;
}

}  // namespace mtl::curve::gcs
