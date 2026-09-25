#include "mtl/sensing/abeam.hpp"

#include <cmath>
#include <vector>

namespace mtl::sensing {

AbeamResult abeamObservable(const Path2& track, const Path2& centers, double reach,
                            double alongTol, double alongOffset) {
    AbeamResult res;
    const Index m = centers.rows();
    res.observable.assign(static_cast<std::size_t>(m), 0);
    res.dmin = VecX::Constant(std::max<Index>(m, 0), kInf);
    if (m == 0 || track.rows() < 2) return res;

    // Segment starts, unit tangents and left normals, skipping zero-length steps.
    std::vector<Vec2>   A, u, v;
    std::vector<double> L;
    A.reserve(static_cast<std::size_t>(track.rows()));
    for (Index i = 0; i + 1 < track.rows(); ++i) {
        const Vec2   d  = track.row(i + 1).transpose() - track.row(i).transpose();
        const double ln = d.norm();
        if (ln <= 1e-9) continue;
        A.push_back(track.row(i).transpose());
        u.push_back(d / ln);
        v.emplace_back(-d.y() / ln, d.x() / ln);  // left normal
        L.push_back(ln);
    }
    if (L.empty()) return res;

    for (Index j = 0; j < m; ++j) {
        const Vec2 t = centers.row(j).transpose();
        bool   crossed = false;
        double best    = kInf;
        bool   anyInReach = false;

        for (std::size_t s = 0; s < L.size(); ++s) {
            const Vec2   r  = t - A[s];
            const double g1 = r.dot(u[s]) - alongOffset;  // along offset at segment start
            const double g2 = g1 - L[s];                  // ... and at its end
            const double c  = std::abs(r.dot(v[s]));      // perpendicular distance to the line
            if (c > reach) continue;
            anyInReach = true;
            if (g1 >= 0.0 && g2 <= 0.0) {  // zero crossing inside this segment
                crossed = true;
                break;
            }
            best = std::min(best, std::min(std::abs(g1), std::abs(g2)));
        }

        if (crossed) {
            res.observable[static_cast<std::size_t>(j)] = 1;
            res.dmin(j) = 0.0;
            ++res.nObservable;
        } else if (anyInReach) {
            res.dmin(j) = best;
            if (best <= alongTol) {
                res.observable[static_cast<std::size_t>(j)] = 1;
                ++res.nObservable;
            }
        }
    }
    return res;
}

}  // namespace mtl::sensing
