#include <chrono>
#include <stdexcept>
#include <vector>

#include "mtl_curve/gcs/gcs.hpp"

#if defined(MTLC_HAVE_SCS)
extern "C" {
#include "scs.h"
}
#endif

namespace mtl::curve::gcs {

ConicResult solveConic(const ConicProblem& P, const GcsParams& opts) {
    ConicResult R;
#if defined(MTLC_HAVE_SCS)
    const auto t0 = std::chrono::steady_clock::now();
    Eigen::SparseMatrix<double> A = P.A;
    A.makeCompressed();
    const scs_int m = static_cast<scs_int>(A.rows()), n = static_cast<scs_int>(A.cols());
    std::vector<scs_float> Ax(A.valuePtr(), A.valuePtr() + A.nonZeros());
    std::vector<scs_int> Ai(A.innerIndexPtr(), A.innerIndexPtr() + A.nonZeros());
    std::vector<scs_int> Ap(A.outerIndexPtr(), A.outerIndexPtr() + A.cols() + 1);
    std::vector<scs_float> b(P.b.data(), P.b.data() + P.b.size());
    std::vector<scs_float> c(P.c.data(), P.c.data() + P.c.size());
    std::vector<scs_int> q(P.q.begin(), P.q.end());

    ScsMatrix Am{Ax.data(), Ai.data(), Ap.data(), m, n};
    ScsData d{};
    d.m = m;
    d.n = n;
    d.A = &Am;
    d.P = nullptr;
    d.b = b.data();
    d.c = c.data();
    ScsCone k{};
    k.z = P.z;
    k.l = P.l;
    k.q = q.empty() ? nullptr : q.data();
    k.qsize = static_cast<scs_int>(q.size());
    ScsSettings st{};
    scs_set_default_settings(&st);
    st.verbose = 0;
    st.eps_abs = opts.tol;
    st.eps_rel = opts.tol;
    st.max_iters = opts.maxIters;
    std::vector<scs_float> x(static_cast<std::size_t>(n), 0.0), y(static_cast<std::size_t>(m), 0.0),
        s(static_cast<std::size_t>(m), 0.0);
    ScsSolution sol{x.data(), y.data(), s.data()};
    ScsInfo info{};
    const scs_int flag = scs(&d, &k, &st, &sol, &info);
    R.status = static_cast<int>(flag);
    R.ok = flag == SCS_SOLVED || flag == SCS_SOLVED_INACCURATE;
    R.x = Eigen::Map<const VecX>(x.data(), n);
    R.obj = P.c.dot(R.x);
    R.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
#else
    (void)P;
    (void)opts;
    throw std::runtime_error("mtl::curve::gcs::solveConic: built without SCS (MTLC_ENABLE_GCS=OFF)");
#endif
    return R;
}

}  // namespace mtl::curve::gcs
