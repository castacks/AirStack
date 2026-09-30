#include "mtl_curve/optimization/fast_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mtl::curve::optimization {

void setStencil(FastGrid& G, double Rk) {
    G.six.clear();
    G.siy.clear();
    G.Rk = Rk;
    if (!(Rk > 0.0)) return;
    const int n = static_cast<int>(std::ceil(Rk / G.hg)) + 1;
    for (int ox = -n; ox <= n; ++ox) {
        for (int oy = -n; oy <= n; ++oy) {
            if (std::hypot(static_cast<double>(ox), static_cast<double>(oy)) * G.hg <= Rk + G.hg) {
                G.six.push_back(ox);
                G.siy.push_back(oy);
            }
        }
    }
}

FastGrid buildFastGrid(const BeliefField& belief, int downsample, double Rk) {
    if (belief.empty()) throw std::invalid_argument("buildFastGrid: empty belief field");
    const int f = std::max(1, downsample);
    const double total = belief.values.sum();
    if (!(total > 0.0)) throw std::invalid_argument("buildFastGrid: the belief has no mass");
    const Index ny0 = belief.rows(), nx0 = belief.cols();
    const double dx = belief.gridResX();
    FastGrid G;
    G.nx = (nx0 + f - 1) / f;
    G.ny = (ny0 + f - 1) / f;
    G.prior = MatX::Zero(G.ny, G.nx);
    for (Index c = 0; c < nx0; ++c)
        for (Index r = 0; r < ny0; ++r) G.prior(r / f, c / f) += belief.values(r, c);
    G.prior /= total;
    G.hg = static_cast<double>(f) * dx;
    G.x0 = 0.5 * static_cast<double>(f - 1) * dx;
    G.y0 = 0.5 * static_cast<double>(f - 1) * belief.gridResY();
    G.downsample = f;
    setStencil(G, Rk);
    return G;
}

FastGrid rasterizeCells(const CellSet& cells, double mapSize, double hg, double cellSize, double Rk) {
    if (!(hg > 0.0)) throw std::invalid_argument("rasterizeCells: pixel size must be positive");
    FastGrid G;
    G.hg = hg;
    G.nx = G.ny = static_cast<Index>(std::floor(mapSize / hg)) + 1;
    G.x0 = G.y0 = 0.0;
    G.prior = MatX::Zero(G.ny, G.nx);
    const double half = 0.5 * cellSize;
    for (Index i = 0; i < cells.size(); ++i) {
        const double cx = cells.centers(i, 0), cy = cells.centers(i, 1);
        const Index c1 = std::max<Index>(0, static_cast<Index>(std::ceil((cx - half) / hg)));
        const Index c2 = std::min<Index>(G.nx - 1, static_cast<Index>(std::floor((cx + half) / hg - 1e-12)));
        const Index r1 = std::max<Index>(0, static_cast<Index>(std::ceil((cy - half) / hg)));
        const Index r2 = std::min<Index>(G.ny - 1, static_cast<Index>(std::floor((cy + half) / hg - 1e-12)));
        const double m = cells.mass(i);
        if (c1 > c2 || r1 > r2) {
            const Index c = std::min<Index>(G.nx - 1, std::max<Index>(0, static_cast<Index>(std::lround(cx / hg))));
            const Index r = std::min<Index>(G.ny - 1, std::max<Index>(0, static_cast<Index>(std::lround(cy / hg))));
            G.prior(r, c) += m;
            continue;
        }
        const double share = m / static_cast<double>((c2 - c1 + 1) * (r2 - r1 + 1));
        G.prior.block(r1, c1, r2 - r1 + 1, c2 - c1 + 1).array() += share;
    }
    const double total = G.prior.sum();
    if (total > 0.0) G.prior /= total;
    G.downsample = 1;
    setStencil(G, Rk);
    return G;
}

}  // namespace mtl::curve::optimization
