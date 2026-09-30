#include "mtl/planning/coverage_score.hpp"

#include <algorithm>
#include <cmath>

namespace mtl::planning {

CoverageModel::CoverageModel(const CellSet& cells, int subsample, double fov,
                             const SensorModelParams& sensor)
    : fov_(fov), sensor_(sensor) {
    const Index m = cells.size();
    const int   s = std::max(1, subsample);
    const double cs = (cells.cellSize > 0.0) ? cells.cellSize : 1.0;
    pts_.resize(m * s * s, 2);
    w_.resize(m * s * s);
    Index q = 0;
    for (Index i = 0; i < m; ++i) {
        const double share = cells.mass(i) / static_cast<double>(s * s);
        for (int a = 0; a < s; ++a)
            for (int b = 0; b < s; ++b) {
                pts_(q, 0) = cells.centers(i, 0) + ((a + 0.5) / s - 0.5) * cs;
                pts_(q, 1) = cells.centers(i, 1) + ((b + 0.5) / s - 0.5) * cs;
                w_(q)      = share;
                ++q;
            }
    }
    if (q == 0) return;

    bs_  = cs;  // one bucket per cell block
    bx0_ = pts_.col(0).minCoeff() - 1.0;
    by0_ = pts_.col(1).minCoeff() - 1.0;
    nbx_ = static_cast<Index>((pts_.col(0).maxCoeff() - bx0_) / bs_) + 2;
    nby_ = static_cast<Index>((pts_.col(1).maxCoeff() - by0_) / bs_) + 2;
    buckets_.assign(static_cast<std::size_t>(nbx_ * nby_), {});
    for (Index k = 0; k < q; ++k) {
        const auto ix = static_cast<Index>((pts_(k, 0) - bx0_) / bs_);
        const auto iy = static_cast<Index>((pts_(k, 1) - by0_) / bs_);
        buckets_[static_cast<std::size_t>(iy * nbx_ + ix)].push_back(k);
    }
}

VecX CoverageModel::logMiss(const std::vector<AgentTrajectory>& trajectories, int lookStride) const {
    VecX lm = VecX::Zero(pts_.rows());
    if (pts_.rows() == 0) return lm;
    const double tanHalf = std::tan(fov_ / 2.0);
    const double logOut  = std::log1p(-sensor_.pOutOfRangeMulti);
    const int stride = std::max(1, lookStride);

    for (const AgentTrajectory& tr : trajectories) {
        const Index N = std::min(tr.drone.rows(), tr.sensor.rows());
        Index k = 0;
        while (k < N) {
            // Hover padding / a grounded agent: identical states, integrated once.
            Index run = 1;
            while (k + run < N && tr.drone.row(k + run) == tr.drone.row(k) &&
                   tr.sensor.row(k + run) == tr.sensor.row(k))
                ++run;
            const double w = (run > 1) ? static_cast<double>(run)
                                       : static_cast<double>(std::min<Index>(stride, N - k));
            const Index step = (run > 1) ? run : std::min<Index>(stride, N - k);

            const double px = tr.drone(k, 0), py = tr.drone(k, 1), h = tr.drone(k, 2);
            const double lx = tr.sensor(k, 0), ly = tr.sensor(k, 1);
            k += step;

            const double radius =
                std::sqrt((px - lx) * (px - lx) + (py - ly) * (py - ly) + h * h) * tanHalf;
            const double r2 = radius * radius;
            const auto ix1 = std::max<Index>(0, static_cast<Index>(std::floor((lx - radius - bx0_) / bs_)));
            const auto ix2 = std::min<Index>(nbx_ - 1, static_cast<Index>(std::floor((lx + radius - bx0_) / bs_)));
            const auto iy1 = std::max<Index>(0, static_cast<Index>(std::floor((ly - radius - by0_) / bs_)));
            const auto iy2 = std::min<Index>(nby_ - 1, static_cast<Index>(std::floor((ly + radius - by0_) / bs_)));
            for (Index iy = iy1; iy <= iy2; ++iy)
                for (Index ix = ix1; ix <= ix2; ++ix)
                    for (const Index q : buckets_[static_cast<std::size_t>(iy * nbx_ + ix)]) {
                        const double dx = pts_(q, 0) - lx, dy = pts_(q, 1) - ly;
                        if (dx * dx + dy * dy > r2) continue;
                        const double ex = pts_(q, 0) - px, ey = pts_(q, 1) - py;
                        const double slant = std::sqrt(ex * ex + ey * ey + h * h);
                        const double lmiss = (slant > sensor_.beta)
                                                 ? logOut
                                                 : std::log1p(-sensor_.detectionProb(slant));
                        lm(q) += w * lmiss;
                    }
        }
    }
    return lm;
}

double CoverageModel::detectedMass(const std::vector<AgentTrajectory>& trajectories,
                                   int lookStride) const {
    const VecX lm = logMiss(trajectories, lookStride);
    double miss = 0.0;
    for (Index q = 0; q < lm.size(); ++q) miss += w_(q) * std::exp(lm(q));
    return w_.sum() - miss;
}

}  // namespace mtl::planning
