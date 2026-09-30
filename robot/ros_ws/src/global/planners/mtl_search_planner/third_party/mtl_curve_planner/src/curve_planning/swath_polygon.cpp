#include "mtl_curve/curve_planning/swath_polygon.hpp"

#include <algorithm>
#include <cmath>

#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"

namespace mtl::curve::curve_planning {

SwathPolygon computeSwathPolygon(const VecX& v, const CurveRep& rep, double halfWidth, const SwathKernel* K,
                                 const FastGrid* G, double ds) {
    const Index M = std::max<Index>(10, static_cast<Index>(std::lround(rep.L / ds)));
    const CurveSamples C = evalParametricCurve(v, rep, M);
    SwathPolygon S;
    S.halfWidth = halfWidth;
    S.center = C.pts;
    S.left = C.pts + halfWidth * C.nrm;
    S.right = C.pts - halfWidth * C.nrm;
    if (G && K) {
        const MatX Lam = optimization::swathKernelDeposit(C.pts, C.tan, C.ds, *G, *K);
        S.mask = (Lam.array() <= std::log(0.5)).matrix();
        S.area = static_cast<double>(S.mask.count()) * G->hg * G->hg;
    }
    return S;
}

}  // namespace mtl::curve::curve_planning
