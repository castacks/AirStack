#include "mtl_curve/planning/sector_orienteering.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <vector>

namespace mtl::curve::planning {
namespace {

/// Park-Miller minimal standard generator (16807 x mod 2^31 - 1): exact in
/// doubles, so the MATLAB port draws the same numbers.
class ParkMiller {
public:
    explicit ParkMiller(std::uint64_t seed) : s_(seed % kM) { if (s_ == 0) s_ = 1; }
    /// uniform integer in [0, n)
    int below(int n) {
        s_ = (s_ * 16807ULL) % kM;
        const int k = static_cast<int>(std::floor(static_cast<double>(s_) / static_cast<double>(kM) * n));
        return std::min(k, n - 1);
    }
private:
    static constexpr std::uint64_t kM = 2147483647ULL;
    std::uint64_t s_;
};

class SectorSolver {
public:
    SectorSolver(const MatX& W, double x0, double y0, double hg, const Eigen::Vector4d& bounds, double hw,
                 const Vec2& start, const std::optional<Vec2>& goal, double L, const SectorParams& p)
        : p_(p), start_(start), goal_(goal) {
        // --- raster: block-sum the weights to ~gridStep ------------------------
        const int f = std::max(1, static_cast<int>(std::lround(p.gridStep / hg)));
        ny_ = (W.rows() + f - 1) / f;
        nx_ = (W.cols() + f - 1) / f;
        hg_ = hg * f;
        x0_ = x0 + 0.5 * (f - 1) * hg;
        y0_ = y0 + 0.5 * (f - 1) * hg;
        w_.assign(static_cast<std::size_t>(nx_ * ny_), 0.0);
        for (Index c = 0; c < W.cols(); ++c)
            for (Index r = 0; r < W.rows(); ++r)
                w_[static_cast<std::size_t>((r / f) * nx_ + c / f)] += std::max(0.0, W(r, c));
        R_ = p.boxFrac * hw;
        budget_ = p.budgetFrac * L;

        // --- sectors over the bounds, valued by their box -----------------------
        const double wx = bounds(1) - bounds(0), wy = bounds(3) - bounds(2);
        const int nsx = std::max(1, static_cast<int>(std::lround(wx / p.sectorSize)));
        const int nsy = std::max(1, static_cast<int>(std::lround(wy / p.sectorSize)));
        const double px = wx / nsx, py = wy / nsy;
        std::vector<Vec2> cen;
        std::vector<double> val;
        for (int i = 0; i < nsy; ++i)
            for (int j = 0; j < nsx; ++j) {
                const Vec2 s(bounds(0) + (j + 0.5) * px, bounds(2) + (i + 0.5) * py);
                double v = 0.0;
                for (const int k : boxPix(s)) v += w_[static_cast<std::size_t>(k)];
                cen.push_back(s);
                val.push_back(v);
            }
        std::vector<int> ord(cen.size());
        std::iota(ord.begin(), ord.end(), 0);
        std::stable_sort(ord.begin(), ord.end(), [&](int a, int b) { return val[a] > val[b]; });
        const double vmax = ord.empty() ? 0.0 : val[static_cast<std::size_t>(ord[0])];
        if (vmax > 0.0)
            for (const int k : ord) {
                if (static_cast<int>(node_.size()) >= p.maxCandidates || val[static_cast<std::size_t>(k)] < 1e-4 * vmax) break;
                node_.push_back(cen[static_cast<std::size_t>(k)]);
                box_.push_back(boxPix(cen[static_cast<std::size_t>(k)]));
            }
        N_ = static_cast<int>(node_.size());
        // distances: index N_ = start, N_ + 1 = goal
        const int M = N_ + 2;
        D_.assign(static_cast<std::size_t>(M * M), 0.0);
        for (int i = 0; i < M; ++i)
            for (int j = 0; j < M; ++j) D_[static_cast<std::size_t>(i * M + j)] = (P(i) - P(j)).norm();
    }

    SectorRoute solve() {
        ParkMiller rng(p_.seed);
        std::vector<int> best;
        double bestR = -1.0, bestL = kInf;
        const int perStart = std::max(1, p_.nPerturb / std::max(1, p_.nStarts));
        if (N_ > 0)
            for (int st = 0; st < p_.nStarts; ++st) {
                std::vector<int> tour;
                fill(tour, st == 0 ? 0 : (st == 1 ? 1 : 2), rng);
                localSearch(tour, rng);
                double R = reward(tour), L = length(tour);
                for (int it = 0; it < perStart; ++it) {     // ruin and recreate
                    std::vector<int> t2 = tour;
                    const int nrem = std::min<int>(1 + rng.below(3), static_cast<int>(t2.size()));
                    for (int q = 0; q < nrem && !t2.empty(); ++q)
                        t2.erase(t2.begin() + rng.below(static_cast<int>(t2.size())));
                    fill(t2, 2, rng);
                    localSearch(t2, rng);
                    const double R2 = reward(t2), L2 = length(t2);
                    if (R2 > R + 1e-12 || (std::abs(R2 - R) < 1e-12 && L2 < L)) {
                        tour = t2;
                        R = R2;
                        L = L2;
                    }
                }
                if (R > bestR + 1e-12 || (std::abs(R - bestR) < 1e-12 && L < bestL)) {
                    best = tour;
                    bestR = R;
                    bestL = L;
                }
            }
        SectorRoute out;
        out.wp.resize(static_cast<Index>(best.size()), 2);
        for (std::size_t k = 0; k < best.size(); ++k) out.wp.row(static_cast<Index>(k)) = node_[static_cast<std::size_t>(best[k])].transpose();
        out.reward = std::max(bestR, 0.0);
        out.length = best.empty() ? 0.0 : bestL;
        out.budget = budget_;
        out.nCandidates = N_;
        return out;
    }

private:
    SectorParams p_;
    Vec2 start_;
    std::optional<Vec2> goal_;
    Index nx_ = 0, ny_ = 0;
    double hg_ = 50.0, x0_ = 0.0, y0_ = 0.0, R_ = 600.0, budget_ = 0.0;
    std::vector<double> w_;
    std::vector<Vec2> node_;
    std::vector<std::vector<int>> box_;
    int N_ = 0;
    std::vector<double> D_;

    Vec2 P(int i) const {
        if (i < N_) return node_[static_cast<std::size_t>(i)];
        if (i == N_) return start_;
        return goal_ ? *goal_ : start_;
    }
    double d(int i, int j) const { return D_[static_cast<std::size_t>(i * (N_ + 2) + j)]; }
    int S() const { return N_; }
    int Gl() const { return N_ + 1; }

    std::vector<int> boxPix(const Vec2& s) const {
        std::vector<int> o;
        const Index q0 = std::max<Index>(0, static_cast<Index>(std::ceil((s.x() - R_ - x0_) / hg_)));
        const Index q1 = std::min<Index>(nx_ - 1, static_cast<Index>(std::floor((s.x() + R_ - x0_) / hg_)));
        const Index r0 = std::max<Index>(0, static_cast<Index>(std::ceil((s.y() - R_ - y0_) / hg_)));
        const Index r1 = std::min<Index>(ny_ - 1, static_cast<Index>(std::floor((s.y() + R_ - y0_) / hg_)));
        for (Index r = r0; r <= r1; ++r)
            for (Index q = q0; q <= q1; ++q) o.push_back(static_cast<int>(r * nx_ + q));
        return o;
    }
    double length(const std::vector<int>& t) const {
        double L = 0.0;
        int prev = S();
        for (const int k : t) {
            L += d(prev, k);
            prev = k;
        }
        if (goal_) L += d(prev, Gl());
        return L;
    }
    double reward(const std::vector<int>& t) const {
        std::vector<char> cov(w_.size(), 0);
        for (const int k : t)
            for (const int px : box_[static_cast<std::size_t>(k)]) cov[static_cast<std::size_t>(px)] = 1;
        double R = 0.0;
        for (std::size_t i = 0; i < cov.size(); ++i)
            if (cov[i]) R += w_[i];
        return R;
    }
    /// length added by inserting node k before position pos (pos == |t|: append)
    double detour(const std::vector<int>& t, int k, int pos) const {
        const int a = pos == 0 ? S() : t[static_cast<std::size_t>(pos - 1)];
        if (pos < static_cast<int>(t.size())) {
            const int b = t[static_cast<std::size_t>(pos)];
            return d(a, k) + d(k, b) - d(a, b);
        }
        return goal_ ? d(a, k) + d(k, Gl()) - d(a, Gl()) : d(a, k);
    }
    // greedy insertion by marginal gain per metre of detour (mode 0), gain^2 /
    // detour (1), or a random pick among the best rcl (2, GRASP)
    void fill(std::vector<int>& t, int mode, ParkMiller& rng) const {
        std::vector<char> in(static_cast<std::size_t>(N_), 0);
        for (const int k : t) in[static_cast<std::size_t>(k)] = 1;
        std::vector<int> cnt(w_.size());
        double L = length(t);
        struct Cand { double score; int k, pos; };
        std::vector<Cand> cands;
        while (true) {
            std::fill(cnt.begin(), cnt.end(), 0);
            for (const int k : t)
                for (const int px : box_[static_cast<std::size_t>(k)]) ++cnt[static_cast<std::size_t>(px)];
            cands.clear();
            for (int k = 0; k < N_; ++k) {
                if (in[static_cast<std::size_t>(k)]) continue;
                int bp = -1;
                double bd = kInf;
                for (int pos = 0; pos <= static_cast<int>(t.size()); ++pos) {
                    const double dd = detour(t, k, pos);
                    if (dd < bd) {
                        bd = dd;
                        bp = pos;
                    }
                }
                if (bp < 0 || L + bd > budget_) continue;
                double g = 0.0;
                for (const int px : box_[static_cast<std::size_t>(k)])
                    if (cnt[static_cast<std::size_t>(px)] == 0) g += w_[static_cast<std::size_t>(px)];
                if (g <= 1e-9) continue;
                const double den = bd + 1.0;
                cands.push_back({mode == 1 ? g * g / den : g / den, k, bp});
            }
            if (cands.empty()) break;
            std::stable_sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.score > b.score; });
            const int pick = mode == 2 ? rng.below(std::min<int>(p_.rcl, static_cast<int>(cands.size()))) : 0;
            const Cand& c = cands[static_cast<std::size_t>(pick)];
            t.insert(t.begin() + c.pos, c.k);
            in[static_cast<std::size_t>(c.k)] = 1;
            L = length(t);
        }
    }
    bool twoOpt(std::vector<int>& t) const {
        bool imp = false;
        const int n = static_cast<int>(t.size());
        for (int i = 0; i + 1 < n; ++i)
            for (int j = i + 1; j < n; ++j) {
                const int a = i == 0 ? S() : t[static_cast<std::size_t>(i - 1)];
                const int nx = j + 1 < n ? t[static_cast<std::size_t>(j + 1)] : (goal_ ? Gl() : -1);
                const int ti = t[static_cast<std::size_t>(i)], tj = t[static_cast<std::size_t>(j)];
                const double before = d(a, ti) + (nx >= 0 ? d(tj, nx) : 0.0);
                const double after = d(a, tj) + (nx >= 0 ? d(ti, nx) : 0.0);
                if (after < before - 1e-6) {
                    std::reverse(t.begin() + i, t.begin() + j + 1);
                    imp = true;
                }
            }
        return imp;
    }
    bool orOpt(std::vector<int>& t) const {
        bool imp = false;
        for (int seg = 1; seg <= 3; ++seg)
            for (int i = 0; i + seg <= static_cast<int>(t.size()); ++i) {
                const double L0 = length(t);
                std::vector<int> s(t.begin() + i, t.begin() + i + seg), rest = t;
                rest.erase(rest.begin() + i, rest.begin() + i + seg);
                double bestL = L0;
                std::vector<int> bestT;
                for (int pos = 0; pos <= static_cast<int>(rest.size()); ++pos)
                    for (int rev = 0; rev < 2; ++rev) {
                        std::vector<int> t2 = rest, ss = s;
                        if (rev) std::reverse(ss.begin(), ss.end());
                        t2.insert(t2.begin() + pos, ss.begin(), ss.end());
                        const double L2 = length(t2);
                        if (L2 < bestL - 1e-6) {
                            bestL = L2;
                            bestT = t2;
                        }
                    }
                if (!bestT.empty()) {
                    t = bestT;
                    imp = true;
                }
            }
        return imp;
    }
    // drop one node and refill; keep it if the union reward improves
    bool replace(std::vector<int>& t, ParkMiller& rng) const {
        bool imp = false;
        double R = reward(t);
        for (int i = 0; i < static_cast<int>(t.size()); ++i) {
            std::vector<int> t2 = t;
            t2.erase(t2.begin() + i);
            fill(t2, 0, rng);
            const double R2 = reward(t2);
            if (R2 > R + 1e-9) {
                t = t2;
                R = R2;
                imp = true;
            }
        }
        return imp;
    }
    void localSearch(std::vector<int>& t, ParkMiller& rng) const {
        for (int it = 0; it < 6; ++it) {
            const bool a = twoOpt(t);
            const bool b = orOpt(t);
            const std::size_t n0 = t.size();
            fill(t, 0, rng);
            const bool c = t.size() > n0;
            const bool r = it < 2 ? replace(t, rng) : false;
            if (!a && !b && !c && !r) break;
        }
    }
};

}  // namespace

SectorRoute solveSectorOrienteering(const MatX& W, double x0, double y0, double hg, const Eigen::Vector4d& bounds,
                                    double swathHalfWidth, const Vec2& start, const std::optional<Vec2>& goal,
                                    double L, const SectorParams& params) {
    const auto t0 = std::chrono::steady_clock::now();
    SectorSolver S(W, x0, y0, hg, bounds, swathHalfWidth, start, goal, L, params);
    SectorRoute r = S.solve();
    r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
}

}  // namespace mtl::curve::planning
