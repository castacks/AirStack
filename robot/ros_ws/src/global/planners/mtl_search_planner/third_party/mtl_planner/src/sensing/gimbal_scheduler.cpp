#include "mtl/sensing/gimbal_scheduler.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <vector>

#include "mtl/core/numeric.hpp"

namespace mtl::sensing {
namespace {

using core::clampAbs;

// -----------------------------------------------------------------------------
//  Per-step geometry of the sensor envelope.
// -----------------------------------------------------------------------------
struct Envelope {
    VecX lo, hi;    ///< cross-track GROUND interval the gimbal can reach
    VecX aLo, aHi;  ///< the same as combined cross-track ANGLES
};

/// Cross-track ground interval the gimbal can reach, step by step.
///
/// Bank shifts the whole window, because bank and gimbal turn the boresight
/// about the same axis.  FOUR limits bound it, and all four appear on both
/// sides: the gimbal stop, the combined-tilt stop, the usable ground reach and
/// the slant-range budget.
///
/// ct = cos(tau - p) is where the mount tilt enters.  A tilted boresight meets
/// the ground obliquely, so l = h*tan(alpha)/ct reaches FURTHER out at a longer
/// range slant = h/(ct*cos(alpha)).  Inverting the first is what puts ct in the
/// numerator of aCap: capping the GROUND offset at maxSensorReach caps the ANGLE
/// at atan(reach*ct/h).
Envelope reachEnvelope(const VecX& h, const VecX& roll, const VecX& ct,
                       const GimbalSchedulerParams& o) {
    const Index n = h.size();
    Envelope e;
    e.aLo.resize(n);
    e.aHi.resize(n);
    e.lo.resize(n);
    e.hi.resize(n);

    for (Index k = 0; k < n; ++k) {
        const double hk = std::max(h(k), 1e-6);
        double aCap = std::atan(o.maxSensorReach * ct(k) / hk);
        if (std::isfinite(o.maxSlantRange)) {
            // h/(ct*cos alpha) <= Rmax  ->  |alpha| <= acos( h/(Rmax*ct) )
            const double cA = std::min(1.0, std::max(-1.0, hk / std::max(o.maxSlantRange * ct(k), 1e-9)));
            aCap = std::min(aCap, std::acos(cA));
        }
        double aLo = std::max(std::max(-o.maxCrossAngle, roll(k) - o.gimbalMax), -aCap);
        double aHi = std::min(std::min(o.maxCrossAngle, roll(k) + o.gimbalMax), aCap);
        if (aLo > aHi) {
            // A bank excursion large enough to push the whole window past a stop
            // leaves an inverted interval; collapse those steps instead.
            const double mid = 0.5 * (aLo + aHi);
            aLo = aHi = mid;
        }
        e.aLo(k) = aLo;
        e.aHi(k) = aHi;
        e.lo(k)  = o.reachMargin * hk * std::tan(aLo) / ct(k);
        e.hi(k)  = o.reachMargin * hk * std::tan(aHi) / ct(k);
    }
    return e;
}

/// How much further out than the reach window a cross-track offset is, signed
/// the way the track would have to move to fix it.  Zero inside.
double signedExcess(double c, double lo, double hi) {
    if (c > hi) return c - hi;
    if (c < lo) return c - lo;
    return 0.0;
}

// -----------------------------------------------------------------------------
//  Step 1 output: every instant at which each centre is physically observable.
// -----------------------------------------------------------------------------
struct Candidates {
    std::vector<Index>  k;     ///< service step
    std::vector<double> bias;  ///< along-track bias f must carry at k [m] (0 = free)
    std::vector<double> c;     ///< cross-track ground offset needed [m]
    std::vector<double> phi;   ///< gimbal angle that gives it [rad]
    std::vector<double> cost;  ///< ranking key, sorted ascending
    double excess = 0.0;       ///< signed cross-track shortfall when nothing works
    bool   onLine = false;     ///< the centre is on the boresight line at some instant
};

/// Hold the pitch, altitude and along-track offset mutually consistent, tilt
/// included.
///
/// THE TILT IS FREE.  Altitude obeys dh/ds = tan(p - aoa): the AIRFRAME pitch is
/// the flight-path angle and the bracket is not part of the airframe.  So the
/// tilt appears ONLY in the along-track offset, f = h*tan(tau - p), which means
/// the boresight stands off h*tan(tau) at zero pitch in level flight, for
/// nothing.  That is the whole point of tilting the mount.
///
/// The control variable is f, so a scheduled along-track bias (`pulse`) can be
/// added to whatever f the altitude reference already implies, and the pitch is
/// then read back off the geometry.  Every limit is applied to the PITCH, here,
/// inside the loop: rate, stop, look-angle and the altitude box.  Nothing
/// downstream may touch p or h again.
struct MarchResult {
    VecX h, f, p;
    int  nBox = 0;
};

MarchResult altitudeMarch(const VecX& hRef, const VecX& ds, const VecX& dhds, const VecX& pulse,
                          double pRate, const GimbalSchedulerParams& o) {
    const Index n   = hRef.size();
    const double tau = o.tiltAngle;
    MarchResult r;
    r.h = VecX::Zero(n);
    r.f = VecX::Zero(n);
    r.p = VecX::Zero(n);
    if (n == 0) return r;

    r.h(0) = hRef(0);
    double pPrev = 0.0;
    const double pLoLook = tau - o.maxLookAngle;
    const double pHiLook = tau + o.maxLookAngle;

    for (Index k = 0; k < n; ++k) {
        const double hk   = std::max(r.h(k), 1e-6);
        const double want = dhds(k) + (hRef(k) - r.h(k)) / std::max(o.altGainDist, 1.0);
        const double pTrk = o.aoa + std::atan(want);          // pitch the climb needs
        const double fdes = hk * std::tan(tau - pTrk) + pulse(k);  // plus the schedule's bias
        const double pdes = tau - std::atan(fdes / hk);

        double pk = pPrev + clampAbs(pdes - pPrev, pRate);    // rate limit
        pk = clampAbs(pk, o.maxPitch);                        // pitch stop
        pk = std::min(std::max(pk, pLoLook), pHiLook);        // look-angle stop
        if (k + 1 < n && ds(k + 1) > 1e-9) {                  // altitude box
            const double pLo = o.aoa + std::atan((o.hMin - r.h(k)) / ds(k + 1));
            const double pHi = o.aoa + std::atan((o.hMax - r.h(k)) / ds(k + 1));
            const double pb  = std::min(std::max(pk, pLo), pHi);
            if (std::abs(pb - pk) > 1e-12) ++r.nBox;
            pk = pb;
        }

        r.p(k) = pk;
        pPrev  = pk;
        r.f(k) = hk * std::tan(tau - pk);
        if (k + 1 < n) {
            r.h(k + 1) = (ds(k + 1) <= 1e-9) ? r.h(k)
                                             : r.h(k) + std::tan(pk - o.aoa) * ds(k + 1);
        }
    }
    return r;
}

/// Every instant at which each centre is physically observable.
///
/// The boresight sits on the cross-track line at along-track offset
/// f = h*tan(tau - p), so a centre is under it exactly where its OWN along-track
/// offset equals f.  Every crossing over the whole sortie is a candidate.
///
/// THE WINDOW IS MEASURED IN PITCH, NOT IN METRES.  The pitch that puts the
/// boresight on centre j at step k is pReq = tau - atan((tgt-xy).u / h) - note
/// the tau term drops out at the natural stand-off, so a service there is FREE -
/// and a candidate is admitted when |pReq| is inside pitchNudgeMax.  Writing the
/// test this way is what makes the tilt honest: df/dp = -h*sec^2(tau-p) grows
/// with the tilt, so a fixed metric half-width would under-admit at tau = 0 and
/// over-admit at tau = 40 deg.
std::vector<Candidates> buildCandidates(const Path2& dXY, const Path2& uh, const Path2& vh,
                                        const VecX& h, const VecX& ct, const VecX& roll,
                                        const VecX& f, const Path2& tgt,
                                        const GimbalSchedulerParams& o) {
    const Index M = tgt.rows();
    const Index N = dXY.rows();
    const double tau = o.tiltAngle;
    const Envelope env = reachEnvelope(h, roll, ct, o);
    const double bFree = o.targetTol;  // the sample-spacing floor: inside it, a hit
    const double nudge = o.pitchNudgeMax;

    std::vector<Candidates> C(static_cast<std::size_t>(M));

    VecX along(N), g(N), c(N), ag(N), apReq(N);
    for (Index j = 0; j < M; ++j) {
        Candidates& Cj = C[static_cast<std::size_t>(j)];
        for (Index k = 0; k < N; ++k) {
            const double rx = tgt(j, 0) - dXY(k, 0);
            const double ry = tgt(j, 1) - dXY(k, 1);
            along(k) = rx * uh(k, 0) + ry * uh(k, 1);   // centre's own along-track offset
            g(k)     = along(k) - f(k);                 // +ve: centre is ahead of the boresight
            c(k)     = rx * vh(k, 0) + ry * vh(k, 1);   // +ve: centre is right of the track
            ag(k)    = std::abs(g(k));
            apReq(k) = std::abs(tau - std::atan(along(k) / std::max(h(k), 1e-6)));
        }

        // --- every instant the centre is ON THE BORESIGHT LINE ---
        // Taking the whole set, rather than only the zero crossings of g,
        // catches a centre whose best instant is at the END of the track (the
        // Dubins path stops at the last centroid) and one the track only grazes.
        std::vector<char> inWin(static_cast<std::size_t>(N), 0);
        bool any = false;
        for (Index k = 0; k < N; ++k) {
            inWin[static_cast<std::size_t>(k)] = (apReq(k) <= nudge) || (ag(k) <= bFree);
            any = any || inWin[static_cast<std::size_t>(k)];
        }
        if (!any) {
            Index km = 0;
            apReq.minCoeff(&km);
            Cj.excess = signedExcess(c(km), env.lo(km), env.hi(km));
            continue;
        }
        Cj.onLine = true;

        // Runs of consecutive in-window steps: one pass each.
        std::vector<Index> kk, kBestRun;
        Index s = 0;
        while (s < N) {
            if (!inWin[static_cast<std::size_t>(s)]) {
                ++s;
                continue;
            }
            Index e = s;
            while (e + 1 < N && inWin[static_cast<std::size_t>(e + 1)]) ++e;

            Index kb = s;  // the free instant of this pass
            for (Index k = s; k <= e; ++k)
                if (ag(k) < ag(kb)) kb = k;
            kBestRun.push_back(kb);

            std::vector<Index> ks{kb};
            const Index stride = std::max<Index>(1, o.nudgeStep);
            for (Index k = s; k <= e; k += stride) ks.push_back(k);
            ks.push_back(e);
            std::sort(ks.begin(), ks.end());
            ks.erase(std::unique(ks.begin(), ks.end()), ks.end());

            if (static_cast<int>(ks.size()) > o.maxCandPerPass) {
                // Thin, but never drop kb.
                std::vector<Index> thin{kb};
                const auto nk = static_cast<double>(ks.size());
                for (int q = 0; q < o.maxCandPerPass; ++q) {
                    const double t = (o.maxCandPerPass == 1)
                                         ? 0.0
                                         : static_cast<double>(q) / (o.maxCandPerPass - 1.0);
                    const auto idx = static_cast<std::size_t>(
                        std::lround(t * (nk - 1.0)));
                    thin.push_back(ks[std::min(idx, ks.size() - 1)]);
                }
                std::sort(thin.begin(), thin.end());
                thin.erase(std::unique(thin.begin(), thin.end()), thin.end());
                ks = std::move(thin);
            }
            kk.insert(kk.end(), ks.begin(), ks.end());
            s = e + 1;
        }
        std::sort(kk.begin(), kk.end());
        kk.erase(std::unique(kk.begin(), kk.end()), kk.end());

        // --- keep only what the gimbal can actually reach ---
        std::vector<Index> kg;
        for (const Index k : kk)
            if (c(k) >= env.lo(k) && c(k) <= env.hi(k)) kg.push_back(k);

        if (kg.empty()) {
            // On the line, but out of reach at every one of those instants: hand
            // the smallest signed cross-track shortfall to the yaw/climb repair.
            double exc = kInf;
            for (const Index k0 : kBestRun) {
                const double e0 = signedExcess(c(k0), env.lo(k0), env.hi(k0));
                if (std::abs(e0) < std::abs(exc)) exc = e0;
            }
            Cj.excess = std::isfinite(exc) ? exc : 0.0;
            continue;
        }

        // The cross-track angle needed.  cos(theta) enters because a tilted
        // boresight meets the ground obliquely: the SAME gimbal angle reaches
        // LESS cross-track ground the more the mount is tilted.  Evaluating it
        // at the theta the profile will actually fly (via ct) keeps the
        // candidate and the profile in agreement.
        struct Entry {
            Index  k;
            double bias, c, phi, cost;
        };
        std::vector<Entry> entries;
        entries.reserve(kg.size());
        for (const Index k : kg) {
            const double alpha = std::atan(c(k) * ct(k) / std::max(h(k), 1e-6));
            const double phi   = alpha - roll(k);
            if (std::abs(phi) > o.gimbalMax || std::abs(alpha) > o.maxCrossAngle) continue;

            const double bias = g(k);
            const double marg = std::min(env.hi(k) - c(k), c(k) - env.lo(k));
            // Rank: free instants first, then the ones with the most cross-track
            // room to spare, which are least sensitive to a later repair or a
            // neighbouring service pulling the profile about.
            double payable = apReq(k);
            if (std::abs(bias) <= bFree) payable = 0.0;  // already a hit: costs nothing
            const double cost = payable / std::max(nudge, 1e-9) +
                                0.15 * std::max(0.0, 1.0 - marg / 200.0) +
                                0.01 * std::abs(bias) / std::max(bFree, 1.0);
            entries.push_back(Entry{k, bias, c(k), phi, cost});
        }
        if (entries.empty()) continue;

        std::stable_sort(entries.begin(), entries.end(),
                         [](const Entry& a, const Entry& b) { return a.cost < b.cost; });
        for (const Entry& e : entries) {
            Cj.k.push_back(e.k);
            Cj.bias.push_back(e.bias);
            Cj.c.push_back(e.c);
            Cj.phi.push_back(e.phi);
            Cj.cost.push_back(e.cost);
        }
        Cj.excess = 0.0;
    }
    return C;
}

// -----------------------------------------------------------------------------
//  Step 3: one candidate per centre, gimbal-slew feasible.
// -----------------------------------------------------------------------------
struct Schedule {
    std::vector<Index>  t, nCand, pick, dwell;
    std::vector<double> c, bias, phi;
    std::vector<char>   served;
};

/// Services already placed, kept sorted by step.
struct Placed {
    std::vector<Index>  j, t, d;
    std::vector<double> p;
};

/// Placed services a new one at (t, phi) would collide with.  Only the immediate
/// neighbours need checking: gimbal travel is sequential, so a chain in which
/// every ADJACENT pair is reachable is reachable whole.
std::vector<std::size_t> blockers(Index t, double ph, Index dw, const Placed& P,
                                  const GimbalSchedulerParams& o) {
    std::vector<std::size_t> blk;
    if (P.t.empty()) return blk;
    const double rate = o.gimbalRate * o.dt / std::max(o.slewPeak, 1e-9);

    for (std::size_t i = 0; i < P.t.size(); ++i)
        if (P.t[i] == t) blk.push_back(i);
    if (!blk.empty()) return blk;

    // last placed before t, first placed after t
    std::size_t ib = P.t.size();
    for (std::size_t i = 0; i < P.t.size(); ++i)
        if (P.t[i] < t) ib = i;
    std::size_t ia = P.t.size();
    for (std::size_t i = 0; i < P.t.size(); ++i) {
        if (P.t[i] > t) {
            ia = i;
            break;
        }
    }

    if (ib < P.t.size()) {
        const Index dk = t - P.t[ib] - dw - P.d[ib];
        if (dk < 1 || std::abs(ph - P.p[ib]) > rate * static_cast<double>(dk)) blk.push_back(ib);
    }
    if (ia < P.t.size()) {
        const Index dk = P.t[ia] - t - dw - P.d[ia];
        if (dk < 1 || std::abs(P.p[ia] - ph) > rate * static_cast<double>(dk)) blk.push_back(ia);
    }
    return blk;
}

void insertSvc(Placed& P, Index j, Index t, double ph, Index dw) {
    std::size_t ins = 0;
    while (ins < P.t.size() && P.t[ins] < t) ++ins;
    P.j.insert(P.j.begin() + static_cast<long>(ins), j);
    P.t.insert(P.t.begin() + static_cast<long>(ins), t);
    P.p.insert(P.p.begin() + static_cast<long>(ins), ph);
    P.d.insert(P.d.begin() + static_cast<long>(ins), dw);
}

void dropSvc(Placed& P, std::size_t i) {
    P.j.erase(P.j.begin() + static_cast<long>(i));
    P.t.erase(P.t.begin() + static_cast<long>(i));
    P.p.erase(P.p.begin() + static_cast<long>(i));
    P.d.erase(P.d.begin() + static_cast<long>(i));
}

/// Cheapest candidate of Cj the gimbal can get to, longest dwell first.
bool firstFit(const Candidates& Cj, const Placed& P, const GimbalSchedulerParams& o,
              const std::vector<Index>& dwellTry, std::size_t& ci, Index& dw) {
    for (std::size_t q = 0; q < Cj.k.size(); ++q) {
        for (const Index d : dwellTry) {
            if (blockers(Cj.k[q], Cj.phi[q], d, P, o).empty()) {
                ci = q;
                dw = d;
                return true;
            }
        }
    }
    return false;
}

void setSvc(Schedule& S, Index j, const Candidates& Cj, std::size_t ci, Index dw) {
    const auto u = static_cast<std::size_t>(j);
    S.served[u] = 1;
    S.pick[u]   = static_cast<Index>(ci);
    S.t[u]      = Cj.k[ci];
    S.phi[u]    = Cj.phi[ci];
    S.c[u]      = Cj.c[ci];
    S.bias[u]   = Cj.bias[ci];
    S.dwell[u]  = dw;
}

Schedule scheduleServices(const std::vector<Candidates>& C, const GimbalSchedulerParams& o,
                          Index dwell, Index N) {
    const auto M = static_cast<Index>(C.size());
    Schedule S;
    S.t.assign(static_cast<std::size_t>(M), 0);
    S.nCand.assign(static_cast<std::size_t>(M), 0);
    S.pick.assign(static_cast<std::size_t>(M), 0);
    S.dwell.assign(static_cast<std::size_t>(M), dwell);
    S.c.assign(static_cast<std::size_t>(M), 0.0);
    S.bias.assign(static_cast<std::size_t>(M), 0.0);
    S.phi.assign(static_cast<std::size_t>(M), 0.0);
    S.served.assign(static_cast<std::size_t>(M), 0);
    for (Index j = 0; j < M; ++j)
        S.nCand[static_cast<std::size_t>(j)] = static_cast<Index>(C[static_cast<std::size_t>(j)].k.size());
    if (M == 0) return S;

    // The dwell is cosmetic - it makes the boresight settle visibly on a centre -
    // but it spends slew budget on both sides, and in a dense cluster that is the
    // difference between a service and a miss.  So try the full dwell first and
    // give it up only for a centre that would otherwise go unobserved.
    std::vector<Index> dwellTry{dwell, std::max<Index>(1, dwell / 2), 0};
    std::sort(dwellTry.begin(), dwellTry.end(), std::greater<>());
    dwellTry.erase(std::unique(dwellTry.begin(), dwellTry.end()), dwellTry.end());

    // MOST CONSTRAINED FIRST.  A centre with two usable instants must claim one
    // before a centre with fifty takes the slot.  Ties broken by the cheapest
    // instant's time, so the schedule still builds roughly left to right.
    std::vector<Index> orderJ(static_cast<std::size_t>(M));
    std::iota(orderJ.begin(), orderJ.end(), Index{0});
    std::stable_sort(orderJ.begin(), orderJ.end(), [&](Index a, Index b) {
        const auto& Ca = C[static_cast<std::size_t>(a)];
        const auto& Cb = C[static_cast<std::size_t>(b)];
        const double na = Ca.k.empty() ? kInf : static_cast<double>(Ca.k.size());
        const double nb = Cb.k.empty() ? kInf : static_cast<double>(Cb.k.size());
        if (na != nb) return na < nb;
        const double ta = Ca.k.empty() ? kInf : static_cast<double>(Ca.k.front());
        const double tb = Cb.k.empty() ? kInf : static_cast<double>(Cb.k.front());
        return ta < tb;
    });

    Placed P;
    for (int sweep = 0; sweep < 2; ++sweep) {
        for (const Index j : orderJ) {
            const auto u = static_cast<std::size_t>(j);
            if (S.served[u] || S.nCand[u] == 0) continue;
            std::size_t ci = 0;
            Index       dw = dwellTry.front();
            if (firstFit(C[u], P, o, dwellTry, ci, dw)) {
                insertSvc(P, j, C[u].k[ci], C[u].phi[ci], dw);
                setSvc(S, j, C[u], ci, dw);
            }
        }
        bool allDone = true;
        for (Index j = 0; j < M; ++j) {
            const auto u = static_cast<std::size_t>(j);
            if (!S.served[u] && S.nCand[u] > 0) allDone = false;
        }
        if (allDone) break;
    }

    // --- displace and retry ---
    // A centre that found no slot is often blocked by ONE neighbour that has
    // alternatives of its own.  Lift that neighbour out, seat this centre, and
    // put the neighbour back somewhere else.  Reverted whole if the swap does not
    // pay, so this can only ever add services.
    for (Index j = 0; j < M; ++j) {
        const auto u = static_cast<std::size_t>(j);
        if (S.served[u] || S.nCand[u] == 0) continue;

        bool placed = false;
        for (std::size_t ci = 0; ci < C[u].k.size() && !placed; ++ci) {
            const Index  t  = C[u].k[ci];
            const double ph = C[u].phi[ci];
            for (const Index dw : dwellTry) {
                const std::vector<std::size_t> blk = blockers(t, ph, dw, P, o);
                if (blk.size() != 1) continue;
                const std::size_t b  = blk[0];
                const Index       jb = P.j[b];

                Placed Q = P;
                dropSvc(Q, b);                                  // lift the neighbour
                if (!blockers(t, ph, dw, Q, o).empty()) continue;
                insertSvc(Q, j, t, ph, dw);                     // seat this centre

                std::size_t cb = 0;
                Index       db = dwellTry.front();
                if (!firstFit(C[static_cast<std::size_t>(jb)], Q, o, dwellTry, cb, db))
                    continue;                                   // neighbour homeless: revert

                insertSvc(Q, jb, C[static_cast<std::size_t>(jb)].k[cb],
                          C[static_cast<std::size_t>(jb)].phi[cb], db);
                P = std::move(Q);
                setSvc(S, j, C[u], ci, dw);
                setSvc(S, jb, C[static_cast<std::size_t>(jb)], cb, db);
                placed = true;
                break;
            }
        }
    }

    // Centres with candidates but no slot: park their bookkeeping at the
    // cheapest instant, so the diagnostics still say where they wanted to be.
    for (Index j = 0; j < M; ++j) {
        const auto u = static_cast<std::size_t>(j);
        if (!S.served[u] && S.nCand[u] > 0) {
            S.t[u]   = C[u].k.front();
            S.c[u]   = C[u].c.front();
            S.phi[u] = C[u].phi.front();
        } else if (S.nCand[u] == 0) {
            S.t[u] = 0;
        }
        S.t[u] = std::min(std::max<Index>(S.t[u], 0), N - 1);
    }
    return S;
}

/// Along-track bias profile the scheduled services ask of the pitch.  Zero
/// everywhere except a short bump at each service that needed one, so the pitch
/// is flat wherever the geometry (tilt included) was already obliging.  This is
/// an offset ON TOP OF the tilt's own stand-off, which altitudeMarch supplies.
VecX biasPulse(const Schedule& S, const GimbalSchedulerParams& o, Index N) {
    VecX pulse = VecX::Zero(N);
    std::vector<Index> sv;
    for (std::size_t j = 0; j < S.served.size(); ++j) {
        if (S.served[j] && std::abs(S.bias[j]) > std::max(0.25, 0.5 * o.targetTol))
            sv.push_back(static_cast<Index>(j));
    }
    if (sv.empty()) return pulse;

    std::stable_sort(sv.begin(), sv.end(), [&](Index a, Index b) {
        return S.t[static_cast<std::size_t>(a)] < S.t[static_cast<std::size_t>(b)];
    });

    const auto n = static_cast<Index>(sv.size());
    std::vector<Index>  kk{0};
    std::vector<double> vv{0.0};

    for (Index i = 0; i < n; ++i) {
        const auto  u = static_cast<std::size_t>(sv[i]);
        const Index t = S.t[u];
        const double b = S.bias[u];

        double gapPrev = kInf, gapNext = kInf;
        if (i > 0) gapPrev = static_cast<double>(t - S.t[static_cast<std::size_t>(sv[i - 1])]);
        if (i + 1 < n) gapNext = static_cast<double>(S.t[static_cast<std::size_t>(sv[i + 1])] - t);

        double w = static_cast<double>(std::lround(0.8 / o.dt));
        w = std::min(w, std::floor((gapPrev - 1.0) / 2.0));
        w = std::min(w, std::floor((gapNext - 1.0) / 2.0));
        const auto wi = static_cast<Index>(std::max(1.0, w));

        if (t - wi > kk.back()) {
            kk.push_back(t - wi);
            vv.push_back(0.0);
        }
        if (t > kk.back()) {
            kk.push_back(t);
            vv.push_back(b);
        }
        if (t + wi <= N - 1) {
            kk.push_back(t + wi);
            vv.push_back(0.0);
        }
    }
    if (kk.back() < N - 1) {
        kk.push_back(N - 1);
        vv.push_back(0.0);
    }
    return core::smoothstepProfile(kk, vv, N);
}

/// Pull the knots the scheduler never checked into slew reach.  Service knots
/// are untouchable - they ARE the hits.  Everything else (parking, the two end
/// knots) is decoration and gets clamped to what the gimbal can manage from its
/// neighbours, in both directions.
void relaxFreeKnots(const std::vector<Index>& kk, std::vector<double>& vv,
                    const std::vector<char>& isSvc, const GimbalSchedulerParams& o) {
    const std::size_t n = kk.size();
    if (n < 2) return;
    const double rate = o.gimbalRate * o.dt / std::max(o.slewPeak, 1e-9);
    for (std::size_t i = 1; i < n; ++i) {
        if (isSvc[i]) continue;
        const double B = rate * static_cast<double>(std::max<Index>(kk[i] - kk[i - 1], 1));
        vv[i] = std::max(std::min(vv[i], vv[i - 1] + B), vv[i - 1] - B);
    }
    for (std::size_t i = n - 1; i-- > 0;) {
        if (isSvc[i]) continue;
        const double B = rate * static_cast<double>(std::max<Index>(kk[i + 1] - kk[i], 1));
        vv[i] = std::max(std::min(vv[i], vv[i + 1] + B), vv[i + 1] - B);
    }
}

// -----------------------------------------------------------------------------
//  Step 4: the gimbal-angle track through the scheduled services.
// -----------------------------------------------------------------------------
struct Profile {
    VecX  p, h, phi, alpha, f, l, lLo, lHi;
    Path2 look;
    std::vector<Index> sIdx;
    int   nBox = 0;
    double rateClipped = 0.0;
};

/// Planned in GIMBAL ANGLE, not in ground offset, because the gimbal's limits
/// are the gimbal's: the rate limit and the travel stop both live on phi, and
/// neither is affected by the mount tilt.  Only the MAPPING from phi to ground
/// offset changes, and that happens once, at the end.
Profile buildProfile(const Path2& dXY, const Path2& uh, const Path2& vh, const VecX& h,
                     const VecX& f, const VecX& p, const VecX& ct, const VecX& roll,
                     const Schedule& S, const GimbalSchedulerParams& o, Index nadirGap,
                     int nBox) {
    const Index N = dXY.rows();
    Profile R;

    std::vector<Index> sv;
    for (std::size_t j = 0; j < S.served.size(); ++j)
        if (S.served[j]) sv.push_back(static_cast<Index>(j));
    std::stable_sort(sv.begin(), sv.end(), [&](Index a, Index b) {
        return S.t[static_cast<std::size_t>(a)] < S.t[static_cast<std::size_t>(b)];
    });
    const auto nS = static_cast<Index>(sv.size());

    std::vector<Index>  kk;
    std::vector<double> vv;
    std::vector<char>   isSvc;
    kk.push_back(0);
    vv.push_back(0.0);
    isSvc.push_back(0);
    if (nS > 0 && S.t[static_cast<std::size_t>(sv[0])] == 0) {
        vv[0]    = S.phi[static_cast<std::size_t>(sv[0])];
        isSvc[0] = 1;
    }

    for (Index i = 0; i < nS; ++i) {
        const auto  u  = static_cast<std::size_t>(sv[i]);
        const Index t0 = S.t[u];
        // Park the gimbal centred (phi = 0, straight down the tilted axis)
        // through a long gap.
        if (t0 - kk.back() > nadirGap) {
            const Index k1 = kk.back() + nadirGap / 3;
            const Index k2 = t0 - nadirGap / 3;
            if (k1 > kk.back()) {
                kk.push_back(k1);
                vv.push_back(0.0);
                isSvc.push_back(0);
            }
            if (k2 > kk.back()) {
                kk.push_back(k2);
                vv.push_back(0.0);
                isSvc.push_back(0);
            }
        }
        // The dwell knots are the ones the schedule reserved room for; a service
        // that had to give its dwell up to fit gets the single knot only.
        const Index dwv = S.dwell[u];
        for (const Index kd : {t0 - dwv, t0, t0 + dwv}) {
            if (kd > kk.back() && kd >= 0 && kd <= N - 1) {
                kk.push_back(kd);
                vv.push_back(S.phi[u]);
                isSvc.push_back(1);
            }
        }
    }
    if (N - 1 > kk.back()) {
        if (N - 1 - kk.back() > nadirGap) {
            kk.push_back(kk.back() + nadirGap / 3);
            vv.push_back(0.0);
            isSvc.push_back(0);
        }
        kk.push_back(N - 1);
        vv.push_back(0.0);
        isSvc.push_back(0);
    }

    // The gimbal stop and the combined-tilt stop, applied to the KNOTS - where
    // they are cheap - not to the finished profile.
    const Envelope env = reachEnvelope(h, roll, ct, o);
    for (std::size_t i = 0; i < kk.size(); ++i) {
        const Index k = kk[i];
        vv[i] = std::max(std::min(vv[i], o.gimbalMax), -o.gimbalMax);
        vv[i] = std::max(std::min(vv[i], env.aHi(k) - roll(k)), env.aLo(k) - roll(k));
    }

    // The scheduler checked the slew between consecutive SERVICES.  The parking
    // and end knots were added afterwards and nobody checked those, so a slow
    // gimbal cannot always get back to centre and out again.
    relaxFreeKnots(kk, vv, isSvc, o);

    VecX phi = core::smoothstepProfile(kk, vv, N);

    // smoothstep is monotone between knots, so clamped knots give a clamped
    // profile; this is belt and braces against a pathological bank excursion.
    phi = phi.cwiseMin(o.gimbalMax).cwiseMax(-o.gimbalMax);
    VecX alpha = (phi + roll).cwiseMin(env.aHi).cwiseMax(env.aLo);
    phi = alpha - roll;

    // LAST LINE OF DEFENCE.  The profile is rate-feasible by construction, so
    // this should be a no-op - but a physically impossible gimbal command is
    // worse than a missed centre, so the rate limit wins unconditionally.
    // Anything it costs is then measured by scoreCoverage and reported as a
    // miss, not quietly claimed as a hit.
    const VecX phiLim = core::rateLimit(phi, o.gimbalRate * o.dt);
    R.rateClipped = (phiLim - phi).cwiseAbs().maxCoeff();
    phi   = phiLim;
    alpha = (phi + roll).cwiseMin(env.aHi).cwiseMax(env.aLo);
    phi   = alpha - roll;

    VecX lProf(N);
    for (Index k = 0; k < N; ++k) lProf(k) = h(k) * std::tan(alpha(k)) / ct(k);

    // f carries the tilt AND the pitch: look = xy + f*u + l*v with
    // f = h*tan(tau - p).  Nothing here knows about the tilt separately, which
    // is the point - it is folded into f by altitudeMarch.
    R.look.resize(N, 2);
    for (Index k = 0; k < N; ++k) {
        R.look(k, 0) = dXY(k, 0) + f(k) * uh(k, 0) + lProf(k) * vh(k, 0);
        R.look(k, 1) = dXY(k, 1) + f(k) * uh(k, 1) + lProf(k) * vh(k, 1);
    }

    R.sIdx.assign(static_cast<std::size_t>(N), -1);
    for (Index i = 0; i < nS; ++i) {
        const auto  u = static_cast<std::size_t>(sv[i]);
        const Index d = std::max<Index>(S.dwell[u], 1);
        const Index a = std::max<Index>(0, S.t[u] - d);
        const Index b = std::min<Index>(N - 1, S.t[u] + d);
        for (Index k = a; k <= b; ++k) R.sIdx[static_cast<std::size_t>(k)] = sv[i];
    }

    R.p     = p;
    R.h     = h;
    R.phi   = phi;
    R.alpha = alpha;
    R.f     = f;
    R.l     = lProf;
    R.lLo   = env.lo;
    R.lHi   = env.hi;
    R.nBox  = nBox;
    return R;
}

/// Closest the boresight ever came to each centre.
void scoreCoverage(const Profile& R, const Path2& tgt, const GimbalSchedulerParams& o, VecX& err,
                   std::vector<char>& hit) {
    const Index M = tgt.rows();
    err = VecX::Constant(std::max<Index>(M, 0), kInf);
    hit.assign(static_cast<std::size_t>(M), 0);
    for (Index j = 0; j < M; ++j) {
        double best = kInf;
        for (Index k = 0; k < R.look.rows(); ++k)
            best = std::min(best, (R.look.row(k).transpose() - tgt.row(j).transpose()).norm());
        err(j) = best;
        hit[static_cast<std::size_t>(j)] = (best <= o.targetTol) ? 1 : 0;
    }
}

/// Rebuild the ground track from a heading profile.  Arc length is preserved
/// exactly, so the timeline is untouched.
Path2 integrateHeading(const Vec2& xy0, const VecX& psi, const VecX& ds) {
    const Index n = psi.size();
    Path2 xy(n, 2);
    xy.row(0) = xy0.transpose();
    for (Index k = 1; k < n; ++k) {
        xy(k, 0) = xy(k - 1, 0) + ds(k) * std::cos(psi(k));
        xy(k, 1) = xy(k - 1, 1) + ds(k) * std::sin(psi(k));
    }
    return xy;
}

void frameOf(const VecX& psi, Path2& uh, Path2& vh) {
    const Index n = psi.size();
    uh.resize(n, 2);
    vh.resize(n, 2);
    for (Index k = 0; k < n; ++k) {
        uh(k, 0) = std::cos(psi(k));
        uh(k, 1) = std::sin(psi(k));
        vh(k, 0) = uh(k, 1);   // right of heading
        vh(k, 1) = -uh(k, 0);
    }
}

/// Arc length at each centre's closest approach - where the yaw repair bends.
VecX closestArc(const Path2& dXY, const VecX& arc, const Path2& tgt) {
    const Index M = tgt.rows();
    VecX sJ = VecX::Zero(M);
    for (Index j = 0; j < M; ++j) {
        double best = kInf;
        Index  bk   = 0;
        for (Index k = 0; k < dXY.rows(); ++k) {
            const double d2 = (tgt.row(j) - dXY.row(k)).squaredNorm();
            if (d2 < best) {
                best = d2;
                bk   = k;
            }
        }
        sJ(j) = arc(bk);
    }
    return sJ;
}

// -----------------------------------------------------------------------------
//  STEP 2: reach repair.  Two levers, both behind enableRepairLoop, both
//  accepted ONLY IF THE CANDIDATE COUNT IMPROVES - a cluster has centres on both
//  sides of the track, so bending toward one can strand another.
// -----------------------------------------------------------------------------

/// One evaluation of a (heading perturbation, climb set) pair: rebuild the
/// track, re-derive everything that depends on it, and count how many centres it
/// leaves observable at some instant.
struct State {
    Path2 dXY, uh, vh;
    VecX  yaw, roll, hRef, dhds, h, f, p, ct;
    std::vector<Candidates> C;
    std::vector<char>       feas;
    VecX                    excess;
    Index                   nFeas = 0;
    VecX                    sJ;
};

/// Last resort: reach grows with altitude, so raise the reference into an
/// achievable climb ramp around the centres yaw could not fix.  The cross-track
/// reach at gimbal alpha is h*tan(alpha)/cos(theta), so the altitude a given
/// offset needs SHRINKS with the tilt - a tilted mount buys cross-track reach as
/// well as stand-off.  Hence the ct0 factor.
VecX climbReference(const VecX& hNom, const Path2& dXY, const Path2& vh, const VecX& arc,
                    const VecX& roll, const Path2& tgt, const std::vector<char>& climbFor,
                    const GimbalSchedulerParams& o) {
    VecX         hRef  = hNom;
    const double slope = std::tan(o.climbPitch);
    const double ct0   = std::max(std::cos(o.tiltAngle), 1e-3);

    for (Index j = 0; j < static_cast<Index>(climbFor.size()); ++j) {
        if (!climbFor[static_cast<std::size_t>(j)]) continue;
        double best = kInf;
        Index  k    = 0;
        for (Index i = 0; i < dXY.rows(); ++i) {
            const double d2 = (tgt.row(j) - dXY.row(i)).squaredNorm();
            if (d2 < best) {
                best = d2;
                k    = i;
            }
        }
        const double c = (tgt.row(j) - dXY.row(k)).dot(vh.row(k));
        double aMax = (c >= 0.0) ? std::min(o.maxCrossAngle, roll(k) + o.gimbalMax)
                                 : std::min(o.maxCrossAngle, -roll(k) + o.gimbalMax);
        aMax = std::max(aMax, deg2rad(5)) * o.reachMargin;
        const double hNeed = std::abs(c) * ct0 / std::max(std::tan(aMax), 1e-6);
        const double hTop  = std::min(hNeed * 1.02, o.hMax);
        if (hTop <= hNom(k)) continue;
        for (Index i = 0; i < hRef.size(); ++i)
            hRef(i) = std::max(hRef(i), hTop - std::abs(arc(i) - arc(k)) * slope);
    }
    hRef = hRef.cwiseMax(o.hMin).cwiseMin(o.hMax);
    hRef = core::movingAverage(hRef, std::max(3, static_cast<int>(std::lround(6.0 / o.dt))));
    return hRef.cwiseMax(o.hMin).cwiseMin(o.hMax);
}

State evaluateState(const Path2& dXY0, const VecX& psi0, const VecX& ds, const VecX& arc,
                    const VecX& hNom, const Path2& tgt, const VecX& dPsi,
                    const std::vector<char>& climbFor, const VecX& pulse, double pRate,
                    const GimbalSchedulerParams& o) {
    State st;
    const VecX psi = psi0 + dPsi;
    st.dXY = integrateHeading(dXY0.row(0).transpose(), psi, ds);
    frameOf(psi, st.uh, st.vh);
    st.yaw  = psi;
    st.roll = VecX::Zero(dPsi.size());
    if (o.computeRoll)
        st.roll = core::coordinatedRoll(st.yaw, ds, o.dt, o.maxRoll, o.rollSign, o.rollSmoothSec);

    st.hRef = hNom;
    if (o.allowClimb &&
        std::any_of(climbFor.begin(), climbFor.end(), [](char c) { return c != 0; })) {
        st.hRef = climbReference(hNom, st.dXY, st.vh, arc, st.roll, tgt, climbFor, o);
    }
    st.dhds = core::slopeOf(st.hRef, ds);

    const MarchResult m = altitudeMarch(st.hRef, ds, st.dhds, pulse, pRate, o);
    st.h = m.h;
    st.f = m.f;
    st.p = m.p;
    st.ct.resize(m.p.size());
    for (Index k = 0; k < m.p.size(); ++k)
        st.ct(k) = std::max(std::cos(o.tiltAngle - m.p(k)), 1e-3);

    st.C = buildCandidates(st.dXY, st.uh, st.vh, st.h, st.ct, st.roll, st.f, tgt, o);

    const Index M = tgt.rows();
    st.feas.assign(static_cast<std::size_t>(M), 0);
    st.excess = VecX::Zero(M);
    for (Index j = 0; j < M; ++j) {
        const bool ok = !st.C[static_cast<std::size_t>(j)].k.empty();
        st.feas[static_cast<std::size_t>(j)] = ok ? 1 : 0;
        st.excess(j) = st.C[static_cast<std::size_t>(j)].excess;
        if (ok) ++st.nFeas;
    }
    st.sJ = closestArc(st.dXY, arc, tgt);
    return st;
}

/// Bend the ground track toward the centres the gimbal cannot reach.
///
/// The perturbation is applied to the HEADING, not to the position: the track is
/// then re-integrated at the same speed, so arc length, timeline and sample
/// count are untouched, and the curvature it adds is bounded by construction.
VecX yawRepair(const VecX& dPsi, const VecX& arc, const VecX& ds, const State& st,
               const GimbalSchedulerParams& o) {
    const Index n = dPsi.size();
    const auto  M = static_cast<Index>(st.feas.size());

    // Median non-zero step, for the speed the bank bound is evaluated at.
    std::vector<double> steps;
    for (Index k = 0; k < ds.size(); ++k)
        if (ds(k) > 1e-9) steps.push_back(ds(k));
    const double dsMed = steps.empty() ? 0.0 : core::quantile(steps, 0.5);
    const double v     = dsMed / o.dt;
    const double kapMax =
        std::min(1.0 / std::max(o.minTurnRadius, 1e-6),
                 std::tan(o.repairBankMax) * 9.81 / std::max(v * v, 1e-6));

    VecX add = VecX::Zero(n);
    for (Index j = 0; j < M; ++j) {
        if (st.feas[static_cast<std::size_t>(j)]) continue;
        const double need = st.excess(j);  // signed: how much further out it is
        if (need == 0.0) continue;
        const double amp = o.repairDamping * need * 1.15;
        if (std::abs(amp) < 1e-3) continue;

        const double L = std::max(kPi * std::sqrt(std::abs(amp) / (2.0 * kapMax)),
                                  kPi * std::abs(amp) / (2.0 * o.repairYawMax));
        bool any = false;
        for (Index k = 0; k < n; ++k) {
            if (std::abs(arc(k) - st.sJ(j)) > L) continue;
            any = true;
            const double u = (arc(k) - st.sJ(j)) / L;
            add(k) += amp * kPi * std::sin(kPi * u) / (2.0 * L);
        }
        (void)any;
    }
    return (add.cwiseAbs().maxCoeff() > 0.0) ? (dPsi + add).eval() : dPsi;
}

}  // namespace

GimbalSchedule optimizeDroneSensorTraj(const Path3& dTraj, const Path2& sTraj, const Path2& targets,
                                       double maxPitchChangeRate, const GimbalSchedulerParams& opts) {
    GimbalSchedule out;
    GimbalSchedulerParams o = opts;

    const Index N = dTraj.rows();
    if (sTraj.rows() != N)
        throw std::invalid_argument("optimizeDroneSensorTraj: dTraj and sTraj row counts differ");

    const Path2 dXY0 = dTraj.leftCols(2);
    const VecX  hNom = dTraj.col(2);
    const Path2 tgt  = targets.leftCols(2);
    const Index M    = tgt.rows();

    double pRate = o.pitchRateIsPerSecond ? maxPitchChangeRate * o.dt : maxPitchChangeRate;
    pRate = std::max(pRate, 1e-6);

    if (N > 0) {
        o.hMin = std::min(o.hMin, hNom.minCoeff());
        o.hMax = std::max(o.hMax, hNom.maxCoeff());
    }
    o.pitchNudgeMax = std::min(o.pitchNudgeMax, o.maxPitch);
    o.maxLookAngle  = std::min(std::max(o.maxLookAngle, 0.0), deg2rad(89));

    // The tilt must leave the boresight below the horizon with the pitch stop
    // fully deflected, otherwise f = h*tan(theta) is unbounded and every
    // downstream number is nonsense.  Refuse rather than quietly saturate.
    const double tau = o.tiltAngle;
    if (std::abs(tau) >= o.maxLookAngle) {
        throw std::invalid_argument(
            "optimizeDroneSensorTraj: tiltAngle is at or past the look-angle stop - the "
            "boresight would be at the horizon and never reach the ground");
    }

    auto& dg = out.diagnostics;
    dg.nTargets        = M;
    dg.targetXY        = tgt;
    dg.targetTol       = o.targetTol;
    dg.tiltAngle       = tau;
    dg.tiltAngleDeg    = rad2deg(tau);
    dg.slantRangeBudget = o.maxSlantRange;
    dg.pitchRateLimitDeg = rad2deg(pRate);
    dg.hMin = o.hMin;
    dg.hMax = o.hMax;
    dg.aoa = o.aoa;
    dg.maxPitch = o.maxPitch;
    dg.gimbalMax = o.gimbalMax;
    dg.gimbalRate = o.gimbalRate;
    dg.maxCrossAngle = o.maxCrossAngle;
    dg.maxRoll = o.maxRoll;
    dg.maxLookAngle = o.maxLookAngle;
    dg.maxSlantRange = o.maxSlantRange;
    dg.dt = o.dt;

    // --- trivial / degenerate ---------------------------------------------
    if (N < 3) {
        out.droneTraj  = dTraj;
        out.sensorTraj = sTraj;
        out.rpy        = Path3::Zero(std::max<Index>(N, 0), 3);
        out.sensorIdx.assign(static_cast<std::size_t>(std::max<Index>(N, 0)), -1);
        dg.note           = "trajectory too short";
        dg.targetCoverage = (M > 0) ? 0.0 : 1.0;
        dg.targetErr      = VecX::Constant(std::max<Index>(M, 0), kInf);
        for (Index j = 0; j < M; ++j) {
            dg.targetMissedIdx.push_back(j);
            dg.missReason.push_back(MissReason::OutOfReach);
            dg.nCandidates.push_back(0);
        }
        dg.gimbalAngle = VecX::Zero(std::max<Index>(N, 0));
        dg.crossAngle  = VecX::Zero(std::max<Index>(N, 0));
        dg.roll        = VecX::Zero(std::max<Index>(N, 0));
        if (N > 0) {
            dg.altMin = hNom.minCoeff();
            dg.altMax = hNom.maxCoeff();
        }
        return out;
    }

    const Index dwell    = std::max<Index>(1, static_cast<Index>(std::lround(o.dwellSec / o.dt / 2.0)));
    const Index nadirGap = std::max<Index>(2, static_cast<Index>(std::lround(o.nadirGapSec / o.dt)));

    // === STEP 0: nominal geometry =========================================
    // Arc length is the timeline: everything below perturbs the HEADING and
    // re-integrates at the same speed, so ds, arc and the sample count are
    // invariants and the timeline is never touched.
    VecX ds, arc;
    core::arcOf(dXY0, ds, arc);
    const VecX psi0 = core::headingOf(dXY0, ds);

    // === STEPS 1 AND 2 ====================================================
    // Enumerate the instants each centre is observable, and (optionally) bend
    // the track, then climb, for the centres that have no such instant.  Each
    // pass is ACCEPTED ONLY IF IT IMPROVES THE CANDIDATE COUNT: a cluster has
    // centres on both sides of the track, so bending toward one can strand
    // another.  The repair loop is off by default.
    VecX              dPsi     = VecX::Zero(N);
    std::vector<char> climbFor(static_cast<std::size_t>(M), 0);

    State st = evaluateState(dXY0, psi0, ds, arc, hNom, tgt, dPsi, climbFor, VecX::Zero(N), pRate, o);
    const Index nAbeam0 = st.nFeas;
    if (o.verbose) {
        std::printf(
            "  audit: %lld/%lld centres observable somewhere on the nominal track (tilt %.1f "
            "deg)\n",
            static_cast<long long>(nAbeam0), static_cast<long long>(M), rad2deg(tau));
    }

    if (o.enableRepairLoop) {
        for (int pass = 1; pass <= std::max(1, o.nRepair); ++pass) {
            if (st.nFeas >= M) break;
            bool improved = false;

            // ---- lever 1: bend the track (yaw) ----------------------------
            if (o.allowPathRepair) {
                const VecX dPsiT = yawRepair(dPsi, arc, ds, st, o);
                if ((dPsiT - dPsi).cwiseAbs().maxCoeff() > 0.0) {
                    State stT = evaluateState(dXY0, psi0, ds, arc, hNom, tgt, dPsiT, climbFor,
                                              VecX::Zero(N), pRate, o);
                    if (stT.nFeas > st.nFeas) {
                        dPsi     = dPsiT;
                        st       = std::move(stT);
                        improved = true;
                        if (o.verbose) {
                            std::printf("  pass %d: yaw repair -> %lld/%lld observable (%.1f deg)\n",
                                        pass, static_cast<long long>(st.nFeas),
                                        static_cast<long long>(M),
                                        rad2deg(dPsi.cwiseAbs().maxCoeff()));
                        }
                    }
                }
            }

            // ---- lever 2: climb for reach ---------------------------------
            if (!improved && o.allowClimb) {
                std::vector<char> cf = climbFor;
                bool              changed = false;
                for (Index j = 0; j < M; ++j) {
                    if (!st.feas[static_cast<std::size_t>(j)] && !cf[static_cast<std::size_t>(j)]) {
                        cf[static_cast<std::size_t>(j)] = 1;
                        changed = true;
                    }
                }
                if (changed) {
                    State stT = evaluateState(dXY0, psi0, ds, arc, hNom, tgt, dPsi, cf,
                                              VecX::Zero(N), pRate, o);
                    if (stT.nFeas > st.nFeas) {
                        climbFor = cf;
                        st       = std::move(stT);
                        improved = true;
                        if (o.verbose) {
                            std::printf("  pass %d: climb to %.0f m -> %lld/%lld observable\n", pass,
                                        st.hRef.maxCoeff(), static_cast<long long>(st.nFeas),
                                        static_cast<long long>(M));
                        }
                    }
                }
            }

            if (!improved) break;
        }
    }

    const Path2 dXY  = st.dXY;
    const Path2 uh   = st.uh;
    const Path2 vh   = st.vh;
    const VecX  yaw  = st.yaw;
    const VecX  roll = st.roll;
    const VecX  hRef = st.hRef;
    const VecX  dhds = st.dhds;

    // === STEP 3: schedule, then feed the implied pitch back ================
    // The feedback is the one genuinely coupled part of the problem: pitch
    // changes theta, which changes f, which moves every crossing, which changes
    // the candidates.  Every pass is measured end to end and ACCEPTED ONLY IF
    // COVERAGE IMPROVES, so it cannot diverge.
    VecX pulse = VecX::Zero(N);

    struct BestPass {
        Profile                 R;
        Schedule                S;
        VecX                    err;
        std::vector<char>       hit;
        VecX                    p, h, f, ct;
        int                     nBox = 0;
        std::vector<Candidates> C;
        bool                    valid = false;
    } best;

    for (int it = 0; it < std::max(1, o.nSchedule); ++it) {
        const MarchResult m = altitudeMarch(hRef, ds, dhds, pulse, pRate, o);
        VecX ct(N);
        for (Index k = 0; k < N; ++k) ct(k) = std::max(std::cos(tau - m.p(k)), 1e-3);

        std::vector<Candidates> C = buildCandidates(dXY, uh, vh, m.h, ct, roll, m.f, tgt, o);
        Schedule Sc = scheduleServices(C, o, dwell, N);
        Profile  R  = buildProfile(dXY, uh, vh, m.h, m.f, m.p, ct, roll, Sc, o, nadirGap, m.nBox);

        VecX              errIt;
        std::vector<char> hitIt;
        scoreCoverage(R, tgt, o, errIt, hitIt);

        const auto nHit = std::count(hitIt.begin(), hitIt.end(), 1);
        const auto nBestHit = best.valid ? std::count(best.hit.begin(), best.hit.end(), 1) : -1;
        const double pMax     = m.p.cwiseAbs().maxCoeff();
        const double pBestMax = best.valid ? best.p.cwiseAbs().maxCoeff() : kInf;

        if (!best.valid || nHit > nBestHit || (nHit == nBestHit && pMax < pBestMax - 1e-9)) {
            best.R = R;
            best.S = Sc;
            best.err = errIt;
            best.hit = hitIt;
            best.p = m.p;
            best.h = m.h;
            best.f = m.f;
            best.ct = ct;
            best.nBox = m.nBox;
            best.C = C;
            best.valid = true;
            if (o.verbose) {
                std::printf("  schedule pass %d: %lld/%lld observed, |pitch| %.2f deg\n", it + 1,
                            static_cast<long long>(nHit), static_cast<long long>(M),
                            rad2deg(pMax));
            }
        }
        if (nHit >= M && pMax < 1e-9) break;

        // Next pass: realise the along-track biases this schedule asked for.
        const VecX pulseNew = biasPulse(Sc, o, N);
        if ((pulseNew - pulse).cwiseAbs().maxCoeff() < 1e-12) break;
        pulse = pulseNew;
    }

    const Profile&           R   = best.R;
    const Schedule&          S   = best.S;
    const std::vector<char>& hit = best.hit;
    const VecX&              h   = best.h;
    const VecX&              p   = best.p;

    // --- assemble ----------------------------------------------------------
    out.droneTraj.resize(N, 3);
    out.droneTraj.leftCols(2) = dXY;
    out.droneTraj.col(2)      = R.h;
    out.sensorTraj            = R.look;
    out.rpy.resize(N, 3);
    out.rpy.col(0) = roll;
    out.rpy.col(1) = R.p;
    out.rpy.col(2) = yaw;
    out.sensorIdx  = R.sIdx;

    // --- diagnostics -------------------------------------------------------
    const VecX& gim = R.phi;
    VecX theta(N), slant(N);
    for (Index k = 0; k < N; ++k) {
        theta(k) = tau - p(k);
        slant(k) = std::sqrt((R.look.row(k) - dXY.row(k)).squaredNorm() + h(k) * h(k));
    }

    dg.nTargetsHit    = std::count(hit.begin(), hit.end(), 1);
    dg.targetCoverage = (M > 0) ? static_cast<double>(dg.nTargetsHit) / static_cast<double>(M) : 1.0;
    dg.targetErr      = best.err;
    dg.targetErrMax   = 0.0;
    for (Index j = 0; j < M; ++j)
        if (std::isfinite(best.err(j))) dg.targetErrMax = std::max(dg.targetErrMax, best.err(j));
    dg.targetServiceStep = S.t;
    for (Index j = 0; j < M; ++j)
        if (!hit[static_cast<std::size_t>(j)]) dg.targetMissedIdx.push_back(j);

    dg.nadirOffsetM  = h.mean() * std::tan(tau);
    dg.lookAngle     = theta;
    dg.lookAngleMaxDeg = rad2deg(theta.cwiseAbs().maxCoeff());
    dg.slantRange    = slant;
    dg.slantRangeMax = slant.maxCoeff();

    // --- how each centre was made observable, and WHY a miss missed ---
    //   1 NEVER ON THE BORESIGHT LINE - the along-track offset never comes
    //     within a bounded pitch of f.  The gimbal cannot help; the GROUND TRACK
    //     has to change.
    //   2 OUT OF GIMBAL REACH - on the line at some instant, but further out
    //     cross-track than the envelope allows at every one of them.
    //   3 GIMBAL DOUBLE-BOOKED - reachable, but every instant it could be served
    //     collides with another centre's service.  The only true scheduling miss.
    const Envelope env = reachEnvelope(h, roll, best.ct, o);
    dg.closestApproachM = VecX::Zero(M);
    dg.reachAtClosestM  = VecX::Zero(M);
    dg.nCandidates      = S.nCand;
    dg.missReason.assign(static_cast<std::size_t>(M), MissReason::Observed);

    Index nOnLine = 0, nInReach = 0;
    for (Index j = 0; j < M; ++j) {
        if (best.C[static_cast<std::size_t>(j)].onLine) ++nOnLine;
        if (S.nCand[static_cast<std::size_t>(j)] > 0) ++nInReach;

        double best2 = kInf;
        Index  kca   = 0;
        for (Index k = 0; k < N; ++k) {
            const double d2 = (tgt.row(j) - dXY.row(k)).squaredNorm();
            if (d2 < best2) {
                best2 = d2;
                kca   = k;
            }
        }
        dg.closestApproachM(j) = std::sqrt(best2);
        dg.reachAtClosestM(j)  = std::max(env.hi(kca), -env.lo(kca));

        if (hit[static_cast<std::size_t>(j)]) continue;
        if (S.nCand[static_cast<std::size_t>(j)] > 0) {
            dg.missReason[static_cast<std::size_t>(j)] = MissReason::DoubleBooked;
            ++dg.nMissDoubleBooked;
        } else if (dg.closestApproachM(j) <= dg.reachAtClosestM(j)) {
            dg.missReason[static_cast<std::size_t>(j)] = MissReason::NeverOnLine;
            ++dg.nMissNeverAbeam;
        } else {
            dg.missReason[static_cast<std::size_t>(j)] = MissReason::OutOfReach;
            ++dg.nMissOutOfReach;
        }
    }
    dg.nObservableNominal = nAbeam0;
    dg.nOnBoresightLine   = nOnLine;
    dg.nReachableAbeam    = nInReach;

    // The pitch a service actually costs.  The centre's own along-track offset
    // is f + bias, and the pitch that puts the boresight there is
    // tau - atan(.../h), so a service at the tilt's natural stand-off costs ZERO
    // pitch even though its along-track offset is large.
    dg.serviceBias    = VecX::Zero(M);
    dg.servicePitchDeg = VecX::Zero(M);
    Index nFree = 0, nServed = 0;
    for (Index j = 0; j < M; ++j) {
        const auto  u  = static_cast<std::size_t>(j);
        const Index tk = std::min(std::max<Index>(S.t[u], 0), N - 1);
        const double hAt = std::max(h(tk), 1e-6);
        dg.serviceBias(j)     = S.bias[u];
        dg.servicePitchDeg(j) = rad2deg(std::abs(tau - std::atan((best.f(tk) + S.bias[u]) / hAt)));
        if (S.served[u]) {
            ++nServed;
            if (dg.servicePitchDeg(j) < 0.5) ++nFree;
            dg.servicePitchMaxDeg = std::max(dg.servicePitchMaxDeg, dg.servicePitchDeg(j));
        }
    }
    dg.nFreeServices = nFree;

    // --- platform ---
    dg.altMin    = R.h.minCoeff();
    dg.altMax    = R.h.maxCoeff();
    dg.altRmsDev = std::sqrt((R.h - hNom).array().square().mean());
    dg.altBoxLimited   = best.nBox;
    dg.pitchMaxDeg     = rad2deg(p.cwiseAbs().maxCoeff());
    dg.pitchRateMaxDeg =
        (N > 1) ? rad2deg((p.tail(N - 1) - p.head(N - 1)).cwiseAbs().maxCoeff()) : 0.0;
    dg.rollMaxDeg   = rad2deg(roll.cwiseAbs().maxCoeff());
    dg.gimbalMaxDeg = rad2deg(gim.cwiseAbs().maxCoeff());
    dg.gimbalRateMaxDeg =
        (N > 1) ? rad2deg((gim.tail(N - 1) - gim.head(N - 1)).cwiseAbs().maxCoeff() / o.dt) : 0.0;
    dg.crossAngleMaxDeg   = rad2deg((gim + roll).cwiseAbs().maxCoeff());
    dg.gimbalRateClipDeg  = rad2deg(R.rateClipped);
    dg.yawRepairMaxDeg    = rad2deg(dPsi.cwiseAbs().maxCoeff());
    dg.pathShiftMax       = (dXY - dXY0).rowwise().norm().maxCoeff();
    dg.nClimbFor          = std::count(climbFor.begin(), climbFor.end(), 1);

    // --- per-step signals ---
    dg.gimbalAngle = gim;
    dg.crossAngle  = gim + roll;
    dg.roll        = roll;
    dg.lookForward = VecX::Zero(N);
    dg.lookCross   = VecX::Zero(N);
    dg.footprintRadius = VecX::Zero(N);
    for (Index k = 0; k < N; ++k) {
        dg.lookForward(k) = h(k) * std::tan(theta(k));
        dg.lookCross(k)   = (R.look.row(k) - dXY.row(k)).dot(vh.row(k));
        dg.footprintRadius(k) = slant(k) * std::tan(o.fov / 2.0);
    }
    dg.altError = R.h - hNom;

    // --- presentability of the boresight track ---
    double lookLen = 0.0, segMax = 0.0;
    for (Index k = 1; k < N; ++k) {
        const double s = (R.look.row(k) - R.look.row(k - 1)).norm();
        lookLen += s;
        segMax = std::max(segMax, s);
    }
    dg.lookPathLength = lookLen;
    dg.lookSpeedMax   = segMax / o.dt;
    dg.lookPathRatio  = lookLen / std::max(ds.tail(N - 1).sum(), 1e-9);

    if (o.verbose) {
        std::printf(
            "optimizeDroneSensorTraj: tilt %.1f deg (stand-off %.0f m); %lld/%lld centres observed "
            "(max err %.2f m); %lld observable on the nominal track, |pitch| %.2f deg\n",
            rad2deg(tau), dg.nadirOffsetM, static_cast<long long>(dg.nTargetsHit),
            static_cast<long long>(M), dg.targetErrMax, static_cast<long long>(nAbeam0),
            dg.pitchMaxDeg);
        std::printf(
            "  %lld of %lld services need no pitch at all (max service pitch %.2f deg); alt "
            "%.0f-%.0f m (rms dev %.1f), slant %.0f m max, boresight %.1fx drone path, gimbal rate "
            "max %.0f deg/s\n",
            static_cast<long long>(nFree), static_cast<long long>(nServed), dg.servicePitchMaxDeg,
            dg.altMin, dg.altMax, dg.altRmsDev, dg.slantRangeMax, dg.lookPathRatio,
            dg.gimbalRateMaxDeg);
        if (!dg.targetMissedIdx.empty()) {
            std::printf(
                "  UNOBSERVED %lld: %lld passed within reach but never on the line (ground track), "
                "%lld never passed close enough (altitude/tilt), %lld gimbal double-booked "
                "(scheduling)\n",
                static_cast<long long>(dg.targetMissedIdx.size()),
                static_cast<long long>(dg.nMissNeverAbeam),
                static_cast<long long>(dg.nMissOutOfReach),
                static_cast<long long>(dg.nMissDoubleBooked));
        }
    }
    return out;
}

}  // namespace mtl::sensing
