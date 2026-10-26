#include <algorithm>
#include <cmath>

#include "mtl_curve/gcs/gcs.hpp"

namespace mtl::curve::gcs {
namespace {

/// Sutherland-Hodgman: keep { x : a'x <= b } of the ccw polygon V.
std::vector<Vec2> clip(const std::vector<Vec2>& V, const Vec2& a, double b) {
    std::vector<Vec2> out;
    const std::size_t n = V.size();
    if (n == 0) return out;
    std::vector<double> f(n);
    for (std::size_t i = 0; i < n; ++i) f[i] = a.dot(V[i]) - b;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        if (f[i] <= 0.0) out.push_back(V[i]);
        if ((f[i] < 0.0 && f[j] > 0.0) || (f[i] > 0.0 && f[j] < 0.0)) {
            const double t = f[i] / (f[i] - f[j]);
            out.push_back(V[i] + t * (V[j] - V[i]));
        }
    }
    std::vector<Vec2> dedup;
    for (std::size_t i = 0; i < out.size(); ++i)
        if ((out[(i + 1) % out.size()] - out[i]).norm() > 1e-9) dedup.push_back(out[i]);
    return dedup;
}

}  // namespace

std::vector<ConvexSet> convexSets(const Path2& centroids, double radius, const Eigen::Vector4d& bounds,
                                  int nPoly) {
    return convexSets(centroids, VecX::Constant(centroids.rows(), radius), bounds, nPoly);
}

std::vector<ConvexSet> convexSets(const Path2& centroids, const VecX& radii, const Eigen::Vector4d& bounds,
                                  int nPoly) {
    const Index K = centroids.rows();
    std::vector<ConvexSet> sets(static_cast<std::size_t>(K));
    for (Index k = 0; k < K; ++k) {
        const Vec2 c = centroids.row(k).transpose();
        const double radius = radii(k);
        std::vector<Vec2> V;
        for (int i = 0; i < nPoly; ++i) {
            const double a = 2.0 * kPi * i / nPoly;
            V.emplace_back(c + radius * Vec2(std::cos(a), std::sin(a)));
        }
        V = clip(V, Vec2(-1, 0), -bounds(0));
        V = clip(V, Vec2(1, 0), bounds(1));
        V = clip(V, Vec2(0, -1), -bounds(2));
        V = clip(V, Vec2(0, 1), bounds(3));
        for (Index j = 0; j < K && !V.empty(); ++j) {
            const Vec2 cj = centroids.row(j).transpose();
            const double d = (cj - c).norm();
            if (!(d > 0.0) || d >= 2.0 * radius) continue;
            const Vec2 a = (cj - c) / d;
            V = clip(V, a, a.dot(0.5 * (c + cj)));
        }
        if (V.size() < 3) V = {c + Vec2(1e-3, 0), c + Vec2(0, 1e-3), c - Vec2(1e-3, 1e-3)};
        ConvexSet& S = sets[static_cast<std::size_t>(k)];
        const Index nv = static_cast<Index>(V.size());
        S.V.resize(nv, 2);
        std::vector<Vec2> Hn;
        std::vector<double> hn;
        for (Index i = 0; i < nv; ++i) {
            S.V.row(i) = V[static_cast<std::size_t>(i)].transpose();
            const Vec2 e = V[static_cast<std::size_t>((i + 1) % nv)] - V[static_cast<std::size_t>(i)];
            Vec2 nrm(e.y(), -e.x());
            const double L = nrm.norm();
            if (L <= 1e-9) continue;
            nrm /= L;
            Hn.push_back(nrm);
            hn.push_back(nrm.dot(V[static_cast<std::size_t>(i)]));
        }
        S.H.resize(static_cast<Index>(Hn.size()), 2);
        S.h.resize(static_cast<Index>(hn.size()));
        for (std::size_t i = 0; i < Hn.size(); ++i) {
            S.H.row(static_cast<Index>(i)) = Hn[i].transpose();
            S.h(static_cast<Index>(i)) = hn[i];
        }
        S.c = c;
        S.cheb = std::max(0.0, (S.h - S.H * c).minCoeff());
    }
    return sets;
}

}  // namespace mtl::curve::gcs
