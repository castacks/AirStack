#include "mtl_curve/core/bspline.hpp"

#include <stdexcept>
#include <vector>

namespace mtl::curve::core {

VecX clampedUniformKnots(Index nCtrl, int p) {
    if (nCtrl < p + 1) throw std::invalid_argument("clampedUniformKnots: need at least degree+1 control points");
    const Index nInner = nCtrl - p + 1;  // linspace(0, 1, Nc - p + 1)
    VecX t(nCtrl + p + 1);
    Index k = 0;
    for (int i = 0; i < p; ++i) t(k++) = 0.0;
    for (Index i = 0; i < nInner; ++i)
        t(k++) = static_cast<double>(i) / static_cast<double>(nInner - 1);
    for (int i = 0; i < p; ++i) t(k++) = 1.0;
    return t;
}

namespace {

/// First derivative of the degree-q basis from the degree-(q-1) basis.
MatX derivOf(const MatX& prev, int q, const VecX& t) {
    const Index m  = t.size();
    const Index nb = m - q - 1;
    MatX D = MatX::Zero(prev.rows(), nb);
    for (Index j = 0; j < nb; ++j) {
        const double d1 = t(j + q) - t(j);
        const double d2 = t(j + q + 1) - t(j + 1);
        if (d1 > 0.0) D.col(j) += prev.col(j) / d1;
        if (d2 > 0.0) D.col(j) -= prev.col(j + 1) / d2;
        D.col(j) *= static_cast<double>(q);
    }
    return D;
}

}  // namespace

void bsplineBasis(const VecX& u, int p, const VecX& t, MatX* N0, MatX* N1, MatX* N2) {
    const Index m  = t.size();
    const Index nu = u.size();
    if (m < p + 2) throw std::invalid_argument("bsplineBasis: knot vector too short");

    // degree 0
    MatX B = MatX::Zero(nu, m - 1);
    Index last = 0;
    for (Index j = 0; j + 1 < m; ++j)
        if (t(j) < t(j + 1)) last = j;
    for (Index i = 0; i < nu; ++i) {
        if (u(i) >= t(m - 1)) {
            B(i, last) = 1.0;
            continue;
        }
        for (Index j = 0; j + 1 < m; ++j)
            if (u(i) >= t(j) && u(i) < t(j + 1)) B(i, j) = 1.0;
    }

    std::vector<MatX> Nq(static_cast<std::size_t>(p) + 1);
    Nq[0] = B;
    for (int q = 1; q <= p; ++q) {
        const MatX& prev = Nq[static_cast<std::size_t>(q) - 1];
        const Index nb = m - q - 1;
        MatX cur = MatX::Zero(nu, nb);
        for (Index j = 0; j < nb; ++j) {
            const double d1 = t(j + q) - t(j);
            const double d2 = t(j + q + 1) - t(j + 1);
            for (Index i = 0; i < nu; ++i) {
                double v = 0.0;
                if (d1 > 0.0) v += (u(i) - t(j)) / d1 * prev(i, j);
                if (d2 > 0.0) v += (t(j + q + 1) - u(i)) / d2 * prev(i, j + 1);
                cur(i, j) = v;
            }
        }
        Nq[static_cast<std::size_t>(q)] = std::move(cur);
    }
    if (N0) *N0 = Nq[static_cast<std::size_t>(p)];
    if (N1) *N1 = p >= 1 ? derivOf(Nq[static_cast<std::size_t>(p) - 1], p, t)
                         : MatX::Zero(nu, m - p - 1);
    if (N2) {
        if (p >= 2) {
            const MatX D1 = derivOf(Nq[static_cast<std::size_t>(p) - 2], p - 1, t);
            *N2 = derivOf(D1, p, t);
        } else {
            *N2 = MatX::Zero(nu, m - p - 1);
        }
    }
}

}  // namespace mtl::curve::core
