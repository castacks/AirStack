#include "mtl/planning/info_score.hpp"

#include <algorithm>
#include <cmath>

namespace mtl::planning {
namespace {

double sumOver(const VecX& w, const std::vector<Index>& idx, Index& count) {
    std::vector<char> seen(static_cast<std::size_t>(w.size()), 0);
    double s = 0.0;
    count    = 0;
    for (const Index i : idx) {
        if (i < 0 || i >= w.size()) continue;
        if (seen[static_cast<std::size_t>(i)]) continue;
        seen[static_cast<std::size_t>(i)] = 1;
        s += w(i);
        ++count;
    }
    return s;
}

}  // namespace

InfoScore pathInformationScore(const VecX& cellMass, const std::vector<Index>& serviced,
                               double routeLength, const InfoScoreOptions& opts) {
    InfoScore s;
    s.nTotal      = cellMass.size();
    s.infoTotal   = cellMass.sum();
    s.totalMapMass = opts.totalMapMass;

    Index n = 0;
    s.info      = sumOver(cellMass, serviced, n);
    s.nServiced = n;

    s.infoFraction    = s.info / std::max(s.infoTotal, 1e-300);
    s.infoMapFraction = s.info / std::max(opts.totalMapMass, 1e-300);

    if (opts.observed != nullptr) {
        Index nr = 0;
        s.infoRealized     = sumOver(cellMass, *opts.observed, nr);
        s.nRealized        = nr;
        s.realizedFraction = s.infoRealized / std::max(s.infoTotal, 1e-300);
    }

    s.routeLength = routeLength;
    s.budget      = opts.budget;
    if (std::isfinite(opts.budget) && opts.budget > 0.0) s.budgetUsed = routeLength / opts.budget;
    if (std::isfinite(routeLength) && routeLength > 0.0) s.efficiency = s.info / routeLength;
    return s;
}

}  // namespace mtl::planning
