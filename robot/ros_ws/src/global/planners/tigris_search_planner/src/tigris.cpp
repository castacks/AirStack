// =============================================================================
//  tigris.cpp — the TIGRIS tree search (see tigris.hpp for the algorithm and the
//  mapping to tigris/src/ipp.cpp).
// =============================================================================
#include "tigris_search_planner/tigris.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <unordered_map>

namespace tigris_search {
namespace {

constexpr double kPi = 3.14159265358979323846;

/// Uniform xy hash of tree-node indices (replaces OMPL's GNAT).
class NodeHash {
public:
    explicit NodeHash(double bucket) : b_(std::max(bucket, 1.0)) {}
    void clear() { m_.clear(); n_ = 0; }
    void add(int idx, double x, double y) {
        m_[key(ix(x), ix(y))].push_back(idx);
        ++n_;
    }
    int size() const { return n_; }

    /// Nearest by d(a, b) = hypot(dx, dy) + w |dpsi|.
    template <typename Dist>
    int nearest(double x, double y, Dist&& dist) const {
        int best = -1;
        double bestD = std::numeric_limits<double>::infinity();
        const long cx = ix(x), cy = ix(y);
        int seen = 0;
        for (long k = 0;; ++k) {
            for (long i = cx - k; i <= cx + k; ++i) {
                for (long j = cy - k; j <= cy + k; ++j) {
                    if (std::max(std::labs(i - cx), std::labs(j - cy)) != k) continue;
                    const auto it = m_.find(key(i, j));
                    if (it == m_.end()) continue;
                    for (const int idx : it->second) {
                        ++seen;
                        const double d = dist(idx);
                        if (d < bestD) { bestD = d; best = idx; }
                    }
                }
            }
            if (seen >= n_) break;
            if (best >= 0 && bestD <= static_cast<double>(k) * b_) break;
        }
        return best;
    }

    template <typename Dist>
    void within(double x, double y, double r, Dist&& dist, std::vector<int>& out) const {
        out.clear();
        const long k = static_cast<long>(std::ceil(r / b_));
        const long cx = ix(x), cy = ix(y);
        for (long i = cx - k; i <= cx + k; ++i) {
            for (long j = cy - k; j <= cy + k; ++j) {
                const auto it = m_.find(key(i, j));
                if (it == m_.end()) continue;
                for (const int idx : it->second) {
                    if (dist(idx) <= r) out.push_back(idx);
                }
            }
        }
    }

private:
    long ix(double v) const { return static_cast<long>(std::floor(v / b_)); }
    static long long key(long i, long j) { return (static_cast<long long>(i) << 32) ^ (j & 0xffffffffLL); }
    double b_;
    int n_ = 0;
    std::unordered_map<long long, std::vector<int>> m_;
};

}  // namespace

TigrisPlanner::TigrisPlanner(const PlanningGrid& grid, const PlannerSetup& setup, const TigrisParams& params)
    : g_(grid), s_(setup), p_(params), eval_(grid, setup.det, params.reward) {}

std::vector<Look> TigrisPlanner::edgeLooks(const DubinsPath& edge, double len) const {
    std::vector<Look> out;
    if (!(len > 0.0)) return out;
    const double step = std::max(p_.rewardStep, 1e-3);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step - 1e-9)));
    out.reserve(static_cast<std::size_t>(n));
    double prev = 0.0;
    for (int k = 1; k <= n; ++k) {
        const double s = std::min(len, k * step);
        const Pose2 q = edge.sample(s);
        const double w = (s - prev) / s_.speed / s_.det.dtRef;
        prev = s;
        out.push_back(lookFromPose(q.x, q.y, s_.altitude, q.yaw, s_.camera, s_.det, w));
    }
    return out;
}

void TigrisPlanner::tigrisGain(const std::vector<const TreeNode*>& chain) {
    // MapRepresentation::informationGain: root -> leaf, each node's footprint, then the
    // straight part of the edge that ends at that node (edge_coords share the node index).
    for (const TreeNode* n : chain) {
        eval_.tigrisNode(n->pose.x, n->pose.y, s_.altitude, n->pose.yaw, s_.camera);
        if (n->hasEdge) {
            eval_.tigrisEdge(n->edgeStart.x, n->edgeStart.y, n->edgeEnd.x, n->edgeEnd.y, n->edgeStart.yaw,
                             s_.altitude, s_.camera);
        }
    }
}

PathReward TigrisPlanner::scorePath(const std::vector<TreeNode>& path, const BeliefState& belief) {
    PathReward r;
    std::vector<const TreeNode*> chain;
    for (const TreeNode& n : path) chain.push_back(&n);
    eval_.begin(belief);
    tigrisGain(chain);
    r.original = eval_.reward().original;
    eval_.begin(belief);
    for (std::size_t i = 1; i < path.size(); ++i) eval_.pass(edgeLooks(path[i].edge, path[i].edgeLen), false, true);
    r.matched = eval_.reward().matched;
    return r;
}

PlanResult TigrisPlanner::plan(const Pose2& start, double budget, double timeLimit, const BeliefState& belief,
                               int replanIndex) {
    using Clock = std::chrono::steady_clock;
    PlanResult res;
    const RewardMode mode = p_.reward.mode;
    const bool original = mode == RewardMode::ORIGINAL;
    const double h = s_.altitude;

    std::mt19937_64 rng(p_.seed * 1000003ULL + static_cast<std::uint64_t>(replanIndex) * 7919ULL + 17ULL);
    std::uniform_real_distribution<double> uPsi(0.0, 2.0 * kPi);  // disPhi
    std::uniform_real_distribution<double> uX(s_.xMin, s_.xMax), uY(s_.yMin, s_.yMax);

    // ---- IPP::updateInformedConfig: one-look reward of every cell at the nominal range
    const double decl = s_.camera.tilt - p_.viewPointGoal * s_.camera.fov / 2.0;
    const double backOff = h * std::tan(decl);
    const double rNom = h / std::cos(decl);
    std::vector<double> weights(g_.size(), 0.0);
    double wSum = 0.0;
    if (p_.informedSampler) {
        const double pNom = s_.det.prob(rNom);
        for (std::size_t k = 0; k < g_.size(); ++k) {
            double w;
            if (original) {
                double q;
                w = originalCellUpdate(belief.presence[k], rNom, s_.det, p_.reward, &q);
            } else {
                w = belief.residual[k] * pNom;
            }
            weights[k] = std::max(w, 0.0);
            wSum += weights[k];
        }
    }
    const bool informed = p_.informedSampler && wSum > 0.0;
    std::discrete_distribution<std::size_t> cellDist;
    if (informed) cellDist = std::discrete_distribution<std::size_t>(weights.begin(), weights.end());

    // IPP::informedConfig / IPP::randomConfig
    auto sampleConfig = [&]() {
        Pose2 q;
        if (informed) {
            const std::size_t k = cellDist(rng);
            const int i = static_cast<int>(k) / g_.nx, j = static_cast<int>(k) % g_.nx;
            q.yaw = wrapPi(uPsi(rng));
            // ipp.cpp keeps x, y in ints: `int x = ...; x -= delta_dist * cos(psi);` truncates
            q.x = std::trunc(std::trunc(g_.cx(j)) - backOff * std::cos(q.yaw));
            q.y = std::trunc(std::trunc(g_.cy(i)) - backOff * std::sin(q.yaw));
        } else {
            q.x = uX(rng);
            q.y = uY(rng);
            q.yaw = wrapPi(uPsi(rng));
        }
        return q;
    };

    const double margin = p_.boundsMargin;
    auto outside = [&](const Pose2& q) {
        if (margin < 0.0) return false;
        return q.x < s_.xMin - margin || q.x > s_.xMax + margin || q.y < s_.yMin - margin ||
               q.y > s_.yMax + margin;
    };

    // ---- tree: every created node lives in `nodes`; only non-pruned ones are in the hash
    std::vector<TreeNode> nodes;
    std::vector<std::vector<Look>> looks;  // MATCHED: edge looks per node
    nodes.reserve(8192);
    looks.reserve(8192);
    NodeHash hash(std::max(10.0, p_.extendRadius));

    auto dist = [&](const Pose2& a, const Pose2& b) {
        return std::hypot(a.x - b.x, a.y - b.y) + p_.psiWeight * std::fabs(wrapPi(a.yaw - b.yaw));
    };

    // IPP::steer: walk the path from `from` toward `target` in dubins_step samples; stop at
    // extend_dist or the budget (the node goes back to the previous sample; budget -> closed);
    // record the straight segment (start_edge / end_edge) exactly as the ROS 1 loop does.
    auto steer = [&](int from, const Pose2& target, TreeNode* out) {
        const TreeNode& f = nodes[static_cast<std::size_t>(from)];
        DubinsPath dp;
        if (!dubinsShortest(f.pose, target, s_.turnRadius, &dp)) return false;
        const double L = dp.length();
        const double step = std::max(s_.dubinsStep, 1e-3);
        const int nSteps = std::max(1, static_cast<int>(std::ceil(L / step - 1e-9)));
        const double rootCost = f.cost;
        const double eps = 0.0001;
        double oldPsi = f.pose.yaw;
        bool startEdge = false, haveStart = false, haveEnd = false;
        Pose2 eStart, eEnd;
        double dist = 0.0, prevS = 0.0, distS1 = 0.0;  // chord sums, as dist_covered in IPP::steer
        Pose2 s1 = f.pose;
        for (int i = 1; i <= nSteps; ++i) {
            const double sv = std::min(L, i * step);
            const Pose2 s2 = dp.sample(sv);
            if (!outside(s2)) {
                const double dpsi = std::fabs(oldPsi - s2.yaw);
                if (dpsi < eps && !startEdge) {
                    startEdge = true;
                    eStart = s2;
                    haveStart = true;
                } else if (dpsi < eps && startEdge) {
                    eEnd = s2;
                    haveEnd = true;
                } else if (oldPsi != s2.yaw) {
                    startEdge = false;
                    oldPsi = s2.yaw;
                }
                dist += std::hypot(s2.x - s1.x, s2.y - s1.y);
                if (dist >= p_.extendDist || rootCost + dist >= budget) {
                    if (i == 1) return false;
                    out->closed = rootCost + dist >= budget;
                    out->edgeLen = prevS;
                    out->cost = rootCost + distS1;
                    break;
                }
            } else {
                if (i == 1) return false;  // trapped
                out->closed = false;
                out->edgeLen = prevS;
                out->cost = rootCost + dist;
                break;
            }
            prevS = sv;
            distS1 = dist;
            s1 = s2;
            if (i == nSteps) {
                out->edgeLen = sv;
                out->cost = rootCost + dist;
            }
        }
        out->edge = dp;
        out->pose = dp.sample(out->edgeLen);
        out->parent = from;
        out->hasEdge = haveStart && haveEnd;
        out->edgeStart = eStart;
        out->edgeEnd = eEnd;
        return out->edgeLen > 1e-9;
    };

    std::vector<const TreeNode*> chainBuf;
    std::vector<int> idxBuf;
    auto infoOf = [&](const TreeNode& n, const std::vector<Look>& ownLooks) {
        idxBuf.clear();
        for (int q = n.parent; q >= 0; q = nodes[static_cast<std::size_t>(q)].parent) idxBuf.push_back(q);
        eval_.begin(belief);
        if (original) {
            chainBuf.clear();
            for (auto it = idxBuf.rbegin(); it != idxBuf.rend(); ++it) chainBuf.push_back(&nodes[static_cast<std::size_t>(*it)]);
            chainBuf.push_back(&n);
            tigrisGain(chainBuf);
            return eval_.reward().original;
        }
        for (auto it = idxBuf.rbegin(); it != idxBuf.rend(); ++it) {
            eval_.pass(looks[static_cast<std::size_t>(*it)], false, true);
        }
        eval_.pass(ownLooks, false, true);
        return eval_.reward().matched;
    };

    std::vector<int> nearBuf;
    auto pruned = [&](const TreeNode& n) {  // IPP::prune
        hash.within(n.pose.x, n.pose.y, p_.pruneRadius,
                    [&](int idx) { return dist(nodes[static_cast<std::size_t>(idx)].pose, n.pose); }, nearBuf);
        for (const int q : nearBuf) {
            const TreeNode& o = nodes[static_cast<std::size_t>(q)];
            if (o.cost <= n.cost && o.info >= n.info) return true;
        }
        return false;
    };

    auto store = [&](TreeNode&& n, std::vector<Look>&& l, bool add) {
        const int idx = static_cast<int>(nodes.size());
        n.inTree = add;
        if (add) hash.add(idx, n.pose.x, n.pose.y);
        nodes.push_back(std::move(n));
        looks.push_back(std::move(l));
        return idx;
    };

    // root: ipp.cpp evaluates informationGain(motion) BEFORE copying the start state into it,
    // i.e. on an unset (zero) state, so the root's information is ~0; every child's path
    // information does include the root footprint (informationGain walks to the root).
    {
        TreeNode root;
        root.pose = start;
        root.info = 0.0;
        store(std::move(root), {}, true);
    }
    int best = 0;

    const auto t0 = Clock::now();  // the ROS 1 clock starts after updateInformedConfig and the root
    auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now() - t0).count(); };
    std::vector<int> nearNodes;
    while (elapsed() < timeLimit && (p_.maxIterations <= 0 || res.iterations < p_.maxIterations)) {
        ++res.iterations;
        const Pose2 sample = sampleConfig();
        const int nearest = hash.nearest(sample.x, sample.y, [&](int idx) {
            return dist(nodes[static_cast<std::size_t>(idx)].pose, sample);
        });
        if (nearest < 0) continue;
        TreeNode m;
        if (!steer(nearest, sample, &m)) continue;
        std::vector<Look> mLooks = original ? std::vector<Look>() : edgeLooks(m.edge, m.edgeLen);
        m.info = infoOf(m, mLooks);
        const bool addM = !pruned(m) && !nodes[static_cast<std::size_t>(nearest)].closed;
        if (!addM) ++res.nodesPruned;
        const int mIdx = store(std::move(m), std::move(mLooks), addM);
        const TreeNode& mRef = nodes[static_cast<std::size_t>(mIdx)];
        if (addM && mRef.info > nodes[static_cast<std::size_t>(best)].info) best = mIdx;
        const Pose2 mPose = mRef.pose;
        const double mInfo = mRef.info;

        hash.within(mPose.x, mPose.y, p_.extendRadius,
                    [&](int idx) { return dist(nodes[static_cast<std::size_t>(idx)].pose, mPose); }, nearNodes);
        for (const int q : nearNodes) {
            if (q == nearest || q == mIdx || nodes[static_cast<std::size_t>(q)].closed) continue;
            TreeNode a;
            if (!steer(q, mPose, &a)) continue;
            std::vector<Look> aLooks = original ? std::vector<Look>() : edgeLooks(a.edge, a.edgeLen);
            a.info = infoOf(a, aLooks);
            const bool addA = !pruned(a);
            if (!addA) ++res.nodesPruned;
            store(std::move(a), std::move(aLooks), addA);
            // ipp.cpp compares motion_feasible (the sampled node), not add_motion
            if (addA && mInfo > nodes[static_cast<std::size_t>(best)].info) best = mIdx;
        }
    }

    for (int q = best; q >= 0; q = nodes[static_cast<std::size_t>(q)].parent) res.path.push_back(nodes[static_cast<std::size_t>(q)]);
    std::reverse(res.path.begin(), res.path.end());
    res.cost = nodes[static_cast<std::size_t>(best)].cost;
    res.treeSize = hash.size();
    res.reward = scorePath(res.path, belief);
    res.seconds = elapsed();
    return res;
}

}  // namespace tigris_search
