#include <algorithm>
#include <cmath>

#include "mtl_curve/gcs/gcs.hpp"

namespace mtl::curve::gcs {
namespace {

/// Sum over a (2n+1)^2 box, zero padded, same size (MATLAB conv2 'same').
MatX boxSum(const MatX& A, int n) {
    const Index ny = A.rows(), nx = A.cols();
    MatX S = MatX::Zero(ny + 1, nx + 1);
    for (Index c = 0; c < nx; ++c)
        for (Index r = 0; r < ny; ++r) S(r + 1, c + 1) = A(r, c) + S(r, c + 1) + S(r + 1, c) - S(r, c);
    MatX B(ny, nx);
    for (Index c = 0; c < nx; ++c) {
        const Index c1 = std::max<Index>(0, c - n), c2 = std::min<Index>(nx - 1, c + n);
        for (Index r = 0; r < ny; ++r) {
            const Index r1 = std::max<Index>(0, r - n), r2 = std::min<Index>(ny - 1, r + n);
            B(r, c) = S(r2 + 1, c2 + 1) - S(r1, c2 + 1) - S(r2 + 1, c1) + S(r1, c1);
        }
    }
    return B;
}

}  // namespace

KernelClusters kernelClusters(const FastGrid& G, double hw, const KernelClusterParams& p) {
    KernelClusters out;
    MatX R = G.prior;
    const double total = R.sum();
    const int n = std::max(1, static_cast<int>(std::lround(p.boxFrac * hw / G.hg)));
    out.pixelCluster = Eigen::MatrixXi::Constant(G.ny, G.nx, -1);
    out.boxHalf = (n + 0.5) * G.hg;
    std::vector<Vec2> cent;
    std::vector<double> info;
    double taken = 0.0;
    while (static_cast<int>(info.size()) < p.maxClusters) {
        const MatX B = boxSum(R, n);
        double mx = -kInf;
        Index rb = 0, cb = 0;
        for (Index c = 0; c < G.nx; ++c)          // column-major, first max (as MATLAB max(B(:)))
            for (Index r = 0; r < G.ny; ++r)
                if (B(r, c) > mx) {
                    mx = B(r, c);
                    rb = r;
                    cb = c;
                }
        if (!(mx > 0.0) || mx < p.minInfoFrac * total) break;
        const Index r1 = std::max<Index>(0, rb - n), r2 = std::min<Index>(G.ny - 1, rb + n);
        const Index c1 = std::max<Index>(0, cb - n), c2 = std::min<Index>(G.nx - 1, cb + n);
        double m = 0.0;
        const int k = static_cast<int>(info.size());
        for (Index c = c1; c <= c2; ++c)
            for (Index r = r1; r <= r2; ++r) {
                m += R(r, c);
                if (R(r, c) > 0.0 && out.pixelCluster(r, c) < 0) out.pixelCluster(r, c) = k;
                R(r, c) = 0.0;
            }
        cent.emplace_back(G.x(cb), G.y(rb));
        info.push_back(m);
        taken += m;
        if (taken >= p.coverFrac * total) break;
    }
    out.centroids.resize(static_cast<Index>(cent.size()), 2);
    out.info.resize(static_cast<Index>(info.size()));
    for (std::size_t k = 0; k < cent.size(); ++k) {
        out.centroids.row(static_cast<Index>(k)) = cent[k].transpose();
        out.info(static_cast<Index>(k)) = info[k];
    }
    return out;
}

}  // namespace mtl::curve::gcs
