#include "mtl_curve/curve_planning/init_spline.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/QR>

#include "mtl_curve/core/bspline.hpp"
#include "mtl_curve/curve_planning/endpoint_constraints.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"

namespace mtl::curve::curve_planning {
namespace {

double wrap(double a) { return std::atan2(std::sin(a), std::cos(a)); }

/// Sum over a (2n+1)^2 box, zero padded, same size (MATLAB conv2 'same').
MatX boxSum(const MatX& A, int n) {
    const Index ny = A.rows(), nx = A.cols();
    MatX S = MatX::Zero(ny + 1, nx + 1);  // summed-area table
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

Vec2 pickGreedy(const Vec2& pos, const std::optional<double>& head, const MatX& lamRun,
                const FastGrid& G, double hw, double capture, double remLen,
                const std::optional<Vec2>& goal) {
    const MatX R = (G.prior.array() * lamRun.array().exp()).matrix();
    const int n = std::max(1, static_cast<int>(std::lround(0.8 * hw / G.hg)));
    const MatX Bm = boxSum(R, n);
    double best = 0.0;
    Vec2 tg = pos;
    bool found = false;
    for (Index c = 0; c < G.nx; ++c) {        // column-major order, as MATLAB max(score(:))
        for (Index r = 0; r < G.ny; ++r) {
            const double X = G.x(c), Y = G.y(r);
            const double dist = std::hypot(X - pos.x(), Y - pos.y());
            if (dist < capture || dist > remLen) continue;
            if (goal && dist + std::hypot(X - goal->x(), Y - goal->y()) > remLen) continue;
            double score = Bm(r, c) / (dist + hw);
            if (head) {
                const double ang = std::abs(wrap(std::atan2(Y - pos.y(), X - pos.x()) - *head));
                score /= 1.0 + 0.3 * ang / kPi;
            }
            if (score > best) {
                best = score;
                tg = Vec2(X, Y);
                found = true;
            }
        }
    }
    if (!found) return goal ? *goal : Vec2(pos + Vec2(1.0, 0.0));
    return tg;
}

MatX depositPath(const Path2& pts, const VecX& th, double ds, const optimization::AgentProblem& prob) {
    Path2 tan(pts.rows(), 2);
    for (Index i = 0; i < pts.rows(); ++i) {
        tan(i, 0) = std::cos(th(i));
        tan(i, 1) = std::sin(th(i));
    }
    return optimization::swathKernelDeposit(pts, tan, VecX::Constant(pts.rows(), ds), *prob.G, *prob.K);
}

/// Least-squares fit of the curvature coefficients (and theta0) to the flown
/// headings th:  min ||theta0 + Phi c - th||^2 + mu ||c - cQI||^2  over the box,
/// by a few active-set passes.  Used for spliced seeds, whose copied arcs the
/// quasi-interpolant would smooth and drift; the hats wholly inside the first
/// nPrefix copied samples (and theta0, when nPrefix > 0) keep the old values,
/// so that stretch of the old curve is reproduced exactly.
VecX fitHeadings(const VecX& th, double ds, const CurveRep& rep, const VecX& vQI, const VecX& vOld, Index nPrefix) {
    const Index N = th.size(), Nk = rep.Nk, nv = Nk + 1;
    VecX s(N);
    for (Index i = 0; i < N; ++i) s(i) = (static_cast<double>(i) + 0.5) * ds;
    MatX A(N, nv);
    A.leftCols(Nk) = hatIntegrals(s, rep.knots);
    A.col(Nk).setOnes();
    const double h = rep.knots(1) - rep.knots(0);
    const double mu = 0.01 * h * h;
    VecX x = vQI;
    std::vector<char> fixed(static_cast<std::size_t>(nv), 0);
    if (nPrefix > 0) {
        for (Index m = 0; m < Nk; ++m)
            if (rep.knots(m) + h <= static_cast<double>(nPrefix) * ds + 1e-9) {
                x(m) = vOld(m);
                fixed[static_cast<std::size_t>(m)] = 1;
            }
        x(Nk) = vOld(Nk);
        fixed[static_cast<std::size_t>(Nk)] = 1;
    }
    for (int pass = 0; pass < 6; ++pass) {
        std::vector<Index> fr;
        for (Index m = 0; m < nv; ++m)
            if (!fixed[static_cast<std::size_t>(m)]) fr.push_back(m);
        if (fr.empty()) break;
        VecX r = th;
        for (Index m = 0; m < nv; ++m)
            if (fixed[static_cast<std::size_t>(m)]) r -= A.col(m) * x(m);
        const Index nf = static_cast<Index>(fr.size());
        MatX Af(N, nf);
        for (Index k = 0; k < nf; ++k) Af.col(k) = A.col(fr[static_cast<std::size_t>(k)]);
        MatX H = Af.transpose() * Af;
        VecX g = Af.transpose() * r;
        for (Index k = 0; k < nf; ++k) {
            const Index m = fr[static_cast<std::size_t>(k)];
            const double w = m < Nk ? mu : 1e-9;
            H(k, k) += w;
            g(k) += w * vQI(m);
        }
        const VecX xf = H.ldlt().solve(g);
        bool clamped = false;
        for (Index k = 0; k < nf; ++k) {
            const Index m = fr[static_cast<std::size_t>(k)];
            x(m) = xf(k);
            if (m < Nk && (x(m) < rep.lb(m) || x(m) > rep.ub(m))) {
                x(m) = std::min(std::max(x(m), rep.lb(m)), rep.ub(m));
                fixed[static_cast<std::size_t>(m)] = 1;
                clamped = true;
            }
        }
        if (!clamped) break;
    }
    return x;
}

}  // namespace

InitResult initParametricSpline(const Vec2& pStart, const Path2& waypoints, double L, EndpointMode mode,
                                const std::optional<Vec2>& pDest, const CurveParams& curve,
                                const optimization::AgentProblem& prob, const Splice* splice) {
    InitResult out;
    out.rep = enforceEndpointConstraints(curve.representation, L, pStart, mode, pDest, curve);
    const CurveRep& rep = out.rep;
    const FastGrid& G = *prob.G;

    const double kMax = rep.kappaMax * 0.95;
    const double Rmin = 1.0 / rep.kappaMax;
    const Index N = std::max<Index>(10, static_cast<Index>(std::lround(L / curve.sampleSpacing)));
    const double ds = L / static_cast<double>(N);
    const double dth = kMax * ds;
    const double hw = prob.K->halfWidth;
    const double capture = curve.captureFrac * hw;

    // ---- splice (reallocation Path seed): copy arcs of an existing curve ------
    const bool useSplice = splice != nullptr && splice->v != nullptr && splice->rep != nullptr &&
                           splice->pieces != nullptr && !splice->pieces->empty() &&
                           rep.type == CurveRepresentation::Curvature &&
                           splice->rep->type == CurveRepresentation::Curvature && splice->rep->M == N;
    VecX thOld;
    Path2 nodeOld;                                   // node j = start of old sample j (N+1 rows)
    if (useSplice) {
        thOld = evalParametricCurve(*splice->v, *splice->rep).theta;
        nodeOld.resize(N + 1, 2);
        nodeOld.row(0) = pStart.transpose();
        for (Index j = 0; j < N; ++j)
            nodeOld.row(j + 1) = nodeOld.row(j) + ds * Eigen::RowVector2d(std::cos(thOld(j)), std::sin(thOld(j)));
    }
    const std::vector<SplicePiece> noPieces;
    const std::vector<SplicePiece>& pieces = useSplice ? *splice->pieces : noPieces;
    std::size_t pc = 0;                              // current piece
    bool copying = false;                            // arc piece: copying (else acquiring)
    Index jp = 0;                                    // arc piece: next old sample
    double turnOffset = 0.0;                         // 2*pi multiple between flown and old headings
    Index pieceSteps = 0;
    double viaBudget = kInf;
    Index copiedFromStart = 0;                       // samples copied before the first deviation
    bool deviated = false;
    auto startPiece = [&]() {
        pieceSteps = 0;
        copying = false;
        if (pc < pieces.size() && !pieces[pc].via) jp = pieces[pc].j0;
    };
    if (useSplice) {
        startPiece();
        if (!pieces[0].via && pieces[0].j0 == 0) copying = true;   // the kept prefix
    }

    const Index nWp = waypoints.rows();
    out.captured.assign(static_cast<std::size_t>(nWp), 0);
    Index qi = 0;
    Vec2 pos = pStart;
    const std::optional<Vec2> goal = rep.pGoal;
    bool homing = false;

    MatX lamRun = prob.lambdaOther ? *prob.lambdaOther : MatX::Zero(G.ny, G.nx);
    Index lastDep = 0;
    Path2 pts(N, 2);
    VecX th(N);
    std::optional<Vec2> target;
    Index tgtSteps = 0;
    double tgtBudget = kInf;
    std::vector<Vec2> greedy;

    double head;
    if (useSplice && copying) {
        head = (*splice->v)(splice->rep->Nk);            // theta(0) of the kept curve
    } else if (curve.initialHeading) {
        head = *curve.initialHeading;
    } else {
        Vec2 tg;
        if (useSplice) tg = pieces[0].via ? pieces[0].pt : Vec2(nodeOld.row(pieces[0].j0).transpose());
        else tg = nWp > 0 ? Vec2(waypoints.row(0).transpose()) : pickGreedy(pos, std::nullopt, lamRun, G, hw, capture, L, goal);
        head = std::atan2(tg.y() - pos.y(), tg.x() - pos.x());
    }

    auto stepTo = [&](Index i, double hm) {
        pts(i, 0) = pos.x() + 0.5 * ds * std::cos(hm);
        pts(i, 1) = pos.y() + 0.5 * ds * std::sin(hm);
        th(i) = hm;
        pos += ds * Vec2(std::cos(hm), std::sin(hm));
    };
    auto pursue = [&](Index i, const Vec2& tg) {
        const double want = std::atan2(tg.y() - pos.y(), tg.x() - pos.x());
        const double turn = std::max(-dth, std::min(dth, wrap(want - head)));
        stepTo(i, head + 0.5 * turn);
        head += turn;
    };

    for (Index i = 0; i < N; ++i) {
        const double remLen = static_cast<double>(N - i) * ds;

        // ---- spliced pieces first ------------------------------------------
        if (pc < pieces.size() && !homing) {
            if (goal && remLen <= (*goal - pos).norm() + (copying ? 0.0 : 2.5 * kPi * Rmin)) {
                homing = true;                           // the goal comes first
                pc = pieces.size();
            }
        }
        if (pc < pieces.size() && !homing) {
            const SplicePiece& P = pieces[pc];
            ++pieceSteps;
            if (P.via) {
                if (pieceSteps == 1) viaBudget = ((P.pt - pos).norm() + 2.0 * kPi * Rmin) / ds * 1.5 + 10.0;
                pursue(i, P.pt);
                deviated = true;
                if ((P.pt - pos).norm() < capture || static_cast<double>(pieceSteps) > viaBudget) {
                    ++pc;
                    startPiece();
                }
                continue;
            }
            if (!copying) {
                // ACQUIRE: track the old arc until on it and aligned, then copy
                Index best = jp;
                double bd = (Vec2(nodeOld.row(jp).transpose()) - pos).norm();
                for (Index j = jp + 1; j <= std::min<Index>(P.j1, jp + 8); ++j) {
                    const double d = (Vec2(nodeOld.row(j).transpose()) - pos).norm();
                    if (d < bd) {
                        bd = d;
                        best = j;
                    }
                }
                jp = best;
                const double herr = wrap(head - thOld(jp));
                if (bd < ds && std::abs(herr) < dth) {
                    copying = true;
                    turnOffset = (head - herr) - thOld(jp);   // a 2*pi multiple
                } else {
                    pursue(i, Vec2(nodeOld.row(std::min<Index>(jp + 3, P.j1 + 1)).transpose()));
                    deviated = true;
                    if (jp >= P.j1) {
                        ++pc;
                        startPiece();
                    }
                    continue;
                }
            }
            // COPY the old heading (the arc is the old one, translated by the
            // small offset left at acquisition)
            const double hm = thOld(jp) + turnOffset;
            stepTo(i, hm);
            head = jp + 1 <= P.j1 ? 0.5 * (thOld(jp) + thOld(jp + 1)) + turnOffset : hm;
            if (!deviated) copiedFromStart = i + 1;
            ++jp;
            if (jp > P.j1) {
                ++pc;
                startPiece();
                deviated = true;
            }
            continue;
        }

        if (goal && !homing && remLen <= (*goal - pos).norm() + 2.5 * kPi * Rmin) homing = true;
        if (homing) {
            target = *goal;
        } else if (!target) {
            while (qi < nWp && (Vec2(waypoints.row(qi).transpose()) - pos).norm() < capture) {
                out.captured[static_cast<std::size_t>(qi)] = 1;
                ++qi;
            }
            if (qi < nWp) {
                target = Vec2(waypoints.row(qi).transpose());
            } else {
                if (i > lastDep) {
                    lamRun += depositPath(pts.middleRows(lastDep, i - lastDep), th.segment(lastDep, i - lastDep), ds, prob);
                    lastDep = i;
                }
                target = pickGreedy(pos, head, lamRun, G, hw, capture, remLen, goal);
                greedy.push_back(*target);
            }
            tgtSteps = 0;
            tgtBudget = ((*target - pos).norm() + 2.0 * kPi * Rmin) / ds * 1.5;
        }

        double turn;
        if (homing && (*goal - pos).norm() < 2.0 * Rmin) {
            turn = dth;                                  // orbit what is left
        } else {
            const double want = std::atan2(target->y() - pos.y(), target->x() - pos.x());
            turn = std::max(-dth, std::min(dth, wrap(want - head)));
        }
        const double hm = head + 0.5 * turn;
        stepTo(i, hm);
        head += turn;

        if (!homing && target) {
            ++tgtSteps;
            const bool hit = (*target - pos).norm() < capture;
            if (hit || static_cast<double>(tgtSteps) > tgtBudget) {
                if (qi < nWp && (*target - Vec2(waypoints.row(qi).transpose())).norm() == 0.0) {
                    out.captured[static_cast<std::size_t>(qi)] = hit ? 1 : 0;
                    ++qi;
                }
                target.reset();
            }
        }
    }
    out.splicePieceReached = static_cast<Index>(pc);
    out.splicePrefix = copiedFromStart;

    out.path.resize(N + 1, 2);
    out.path.row(0) = pStart.transpose();
    out.path.bottomRows(N) = pts;
    out.greedyTargets.resize(static_cast<Index>(greedy.size()), 2);
    for (std::size_t k = 0; k < greedy.size(); ++k) out.greedyTargets.row(static_cast<Index>(k)) = greedy[k].transpose();
    out.endPt = pos;

    // ---- project onto the representation ------------------------------------
    if (rep.type == CurveRepresentation::Curvature) {
        VecX kap = VecX::Zero(N);
        for (Index i = 0; i + 1 < N; ++i) kap(i) = (th(i + 1) - th(i)) / ds;
        const double h = rep.knots(1) - rep.knots(0);
        VecX c(rep.Nk);
        for (Index m = 0; m < rep.Nk; ++m) {
            double sw = 0.0, sk = 0.0;
            for (Index i = 0; i < N; ++i) {
                const double s = (static_cast<double>(i) + 0.5) * ds;
                const double w = std::max(0.0, 1.0 - std::abs(s - rep.knots(m)) / h);
                sw += w;
                sk += w * kap(i);
            }
            c(m) = std::min(std::max(sk / std::max(sw, 1e-300), rep.lb(m)), rep.ub(m));
        }
        VecX v0(rep.nv);
        v0.head(rep.Nk) = c;
        v0(rep.Nk) = th(0) - 0.5 * ds * c(0);
        if (useSplice) {
            v0 = fitHeadings(th, ds, rep, v0, *splice->v, out.splicePrefix);
        } else if (curve.initialHeading) {
            v0(rep.Nk) = *curve.initialHeading;
        } else {
            // the quasi-interpolant smooths the turns and drifts the curve;
            // rotate the launch heading so the fitted curve's centroid direction
            // matches the flown one (cheap, first order)
            const CurveSamples Cf = evalParametricCurve(v0, rep, N);
            const Vec2 m1 = pts.colwise().mean().transpose(), m2 = Cf.pts.colwise().mean().transpose();
            const double a1 = std::atan2(m1.y() - pStart.y(), m1.x() - pStart.x());
            const double a2 = std::atan2(m2.y() - pStart.y(), m2.x() - pStart.x());
            v0(rep.Nk) += wrap(a1 - a2);
        }
        out.v0 = v0;
    } else {
        VecX u(N);
        for (Index i = 0; i < N; ++i) u(i) = (static_cast<double>(i) + 0.5) / static_cast<double>(N);
        MatX B;
        core::bsplineBasis(u, rep.p, rep.knotVec, &B, nullptr, nullptr);
        Index nf = 0;
        for (const char f : rep.fixed) nf += f ? 0 : 1;
        MatX Bf(N, nf);
        MatX rhs = pts;
        Index k = 0;
        for (Index cc = 0; cc < rep.Nc; ++cc) {
            if (rep.fixed[static_cast<std::size_t>(cc)]) {
                rhs.col(0) -= B.col(cc) * rep.Pfix(cc, 0);
                rhs.col(1) -= B.col(cc) * rep.Pfix(cc, 1);
            } else {
                Bf.col(k++) = B.col(cc);
            }
        }
        const MatX Pf = Bf.colPivHouseholderQr().solve(rhs);
        VecX v0(2 * nf);
        v0.head(nf) = Pf.col(0);
        v0.tail(nf) = Pf.col(1);
        out.v0 = v0.cwiseMax(rep.lb).cwiseMin(rep.ub);
    }
    return out;
}

}  // namespace mtl::curve::curve_planning
