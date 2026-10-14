// =============================================================================
//  ia_tigris.cpp — the IA-TIGRIS tree search (see ia_tigris.hpp for the algorithm
//  and the mapping to ia-tigris/src/ipp_planners/src/Tigris.cpp).
// =============================================================================
#include "ia_tigris_search_planner/ia_tigris.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>

namespace ia_tigris_search {
namespace {

constexpr double kPi = 3.14159265358979323846;
using Clock = std::chrono::steady_clock;

}  // namespace

/// Uniform xy hash of tree-node indices (replaces OMPL's GNAT; exact nearest / radius).
class IaTigrisPlanner::NodeHash {
public:
    explicit NodeHash(double bucket) : b_(std::max(bucket, 1.0)) {}
    void clear() { m_.clear(); n_ = 0; }
    void add(int idx, double x, double y) {
        m_[key(ix(x), ix(y))].push_back(idx);
        ++n_;
    }
    int size() const { return n_; }

    /// Nearest by the planner's d(a, b) = hypot(dx, dy) + w |dpsi|.
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

IaTigrisPlanner::IaTigrisPlanner(const PlanningGrid& grid, const PlannerSetup& setup, const TigrisParams& params)
    : g_(grid), s_(setup), p_(params), eval_(grid, setup.det, params.reward),
      hash_(std::make_unique<NodeHash>(std::max(10.0, params.extendRadius))) {}

IaTigrisPlanner::~IaTigrisPlanner() = default;

// ---- rewards (identical to tigris_search_planner) ----------------------------
std::vector<Look> IaTigrisPlanner::edgeLooks(const DubinsPath& edge, double len, double tStart) const {
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
        out.push_back(lookFromPose(q.x, q.y, s_.altitude, q.yaw, s_.camera, s_.det, w,
                                   s_.camera.phiAt(tStart + s / s_.speed)));
    }
    return out;
}

void IaTigrisPlanner::tigrisGain(const std::vector<const TreeNode*>& chain, double startTime) {
    // SearchMap::search_information_gain_with_hashmaps, root -> leaf: for every node FIRST the
    // straight part of the edge that ends at it (estimate_edge_belief_and_reward_with_hashmap,
    // on the parent's belief), THEN its own footprint (estimate_belief_and_reward_with_hashmap,
    // which sees the edge's updates). Turning arcs earn nothing (no_info_on_turns).
    const Camera& cam = s_.camera;
    std::vector<RewardEvaluator::ViewPose> poses;
    for (const TreeNode* n : chain) {
        if (n->hasEdge) {
            if (!cam.sweep) {
                eval_.tigrisEdge(n->edgeStart.x, n->edgeStart.y, n->edgeEnd.x, n->edgeEnd.y, n->edgeStart.yaw,
                                 s_.altitude, cam);
            } else {
                // swept camera: the swath is the union of the footprints along the straight part
                const double tEdge = startTime + (n->arc - n->edgeLen) / s_.speed;
                const double a = n->edgeStartS, b = std::max(n->edgeEndS, a);
                const double step = std::max(p_.rewardStep, 1e-3);
                const int m = std::max(1, static_cast<int>(std::ceil((b - a) / step - 1e-9)));
                poses.clear();
                for (int k = 0; k <= m; ++k) {
                    const double sv = std::min(b, a + k * step);
                    const Pose2 q = n->edge.sample(sv);
                    poses.push_back({q.x, q.y, q.yaw, cam.phiAt(tEdge + sv / s_.speed)});
                }
                eval_.tigrisEdgeSwept(poses, s_.altitude, cam);
            }
        }
        eval_.tigrisNode(n->pose.x, n->pose.y, s_.altitude, n->pose.yaw, cam,
                         cam.phiAt(startTime + n->arc / s_.speed));
    }
}

PathReward IaTigrisPlanner::scorePath(const std::vector<TreeNode>& path, const BeliefState& belief,
                                      double startTime) {
    PathReward r;
    std::vector<const TreeNode*> chain;
    for (const TreeNode& n : path) chain.push_back(&n);
    eval_.begin(belief);
    tigrisGain(chain, startTime);
    r.original = eval_.reward().original;
    eval_.begin(belief);
    for (std::size_t i = 1; i < path.size(); ++i) {
        const double tEdge = startTime + (path[i].arc - path[i].edgeLen) / s_.speed;
        eval_.pass(edgeLooks(path[i].edge, path[i].edgeLen, tEdge), false, true);
    }
    r.matched = eval_.reward().matched;
    return r;
}

// SearchMap::search_information_gain_with_hashmaps: replay the ancestors' local updates
// over the base belief, then evaluate only this node's own looks on top.
double IaTigrisPlanner::evaluate(TreeNode& n, NodeData& d) {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    eval_.begin(*belief_);
    chainBuf_.clear();
    for (int q = n.parent; q >= 0; q = nodes_[static_cast<std::size_t>(q)].parent) chainBuf_.push_back(q);
    for (auto it = chainBuf_.rbegin(); it != chainBuf_.rend(); ++it) {
        const NodeData& a = data_[static_cast<std::size_t>(*it)];
        if (original) {
            for (const auto& u : a.dPres) eval_.seedPresence(u.first, u.second);
        } else {
            for (const auto& u : a.dRes) eval_.seedResidual(u.first, u.second);
        }
    }
    eval_.markDelta();
    double inc;
    if (original) {
        const std::vector<const TreeNode*> self{&n};
        tigrisGain(self, startTime_);
        inc = eval_.reward().original;
        eval_.delta(&d.dPres, nullptr);
        d.dRes.clear();
    } else {
        eval_.pass(d.looks, false, true);
        inc = eval_.reward().matched;
        eval_.delta(nullptr, &d.dRes);
        d.dPres.clear();
    }
    return (n.parent >= 0 ? nodes_[static_cast<std::size_t>(n.parent)].info : 0.0) + inc;
}

double IaTigrisPlanner::dist(const Pose2& a, const Pose2& b) const {
    return std::hypot(a.x - b.x, a.y - b.y) + p_.psiWeight * std::fabs(wrapPi(a.yaw - b.yaw));
}

// IPP::steer / Planner::extend_from_start_node_toward_goal_node: walk the Dubins path from
// `from` toward `target`; stop at extend_dist or the budget (the node goes back to the
// previous sample; budget -> closed); record the straight segment (start_edge / end_edge).
bool IaTigrisPlanner::steer(int from, const Pose2& target, double budget, TreeNode* out) const {
    const TreeNode& f = nodes_[static_cast<std::size_t>(from)];
    DubinsPath dp;
    if (!dubinsShortest(f.pose, target, s_.turnRadius, &dp)) return false;
    const double L = dp.length();
    const double step = std::max(s_.dubinsStep, 1e-3);
    const int nSteps = std::max(1, static_cast<int>(std::ceil(L / step - 1e-9)));
    const double rootCost = f.cost;
    const double eps = 0.0001;
    const double margin = p_.boundsMargin;
    auto outside = [&](const Pose2& q) {
        if (margin < 0.0) return false;
        return q.x < s_.xMin - margin || q.x > s_.xMax + margin || q.y < s_.yMin - margin ||
               q.y > s_.yMax + margin;
    };
    double oldPsi = f.pose.yaw;
    bool startEdge = false, haveStart = false, haveEnd = false;
    Pose2 eStart, eEnd;
    double eStartS = 0.0, eEndS = 0.0;
    double distC = 0.0, prevS = 0.0, distS1 = 0.0;  // chord sums, as dist_covered
    Pose2 s1 = f.pose;
    for (int i = 1; i <= nSteps; ++i) {
        const double sv = std::min(L, i * step);
        const Pose2 s2 = dp.sample(sv);
        if (!outside(s2)) {
            const double dpsi = std::fabs(oldPsi - s2.yaw);
            if (dpsi < eps && !startEdge) {
                startEdge = true;
                eStart = s2;
                eStartS = sv;
                haveStart = true;
            } else if (dpsi < eps && startEdge) {
                eEnd = s2;
                eEndS = sv;
                haveEnd = true;
            } else if (oldPsi != s2.yaw) {
                startEdge = false;
                oldPsi = s2.yaw;
            }
            distC += std::hypot(s2.x - s1.x, s2.y - s1.y);
            if (distC >= p_.extendDist || rootCost + distC >= budget) {
                if (i == 1) return false;
                out->closed = rootCost + distC >= budget;
                out->edgeLen = prevS;
                out->cost = rootCost + distS1;
                break;
            }
        } else {
            if (i == 1) return false;  // trapped
            out->closed = false;
            out->edgeLen = prevS;
            out->cost = rootCost + distC;
            break;
        }
        prevS = sv;
        distS1 = distC;
        s1 = s2;
        if (i == nSteps) {
            out->edgeLen = sv;
            out->cost = rootCost + distC;
        }
    }
    out->edge = dp;
    out->pose = dp.sample(out->edgeLen);
    out->parent = from;
    out->arc = f.arc + out->edgeLen;
    out->hasEdge = haveStart && haveEnd;
    out->edgeStart = eStart;
    out->edgeEnd = eEnd;
    out->edgeStartS = eStartS;
    out->edgeEndS = eEndS;
    return out->edgeLen > 1e-9;
}

bool IaTigrisPlanner::pruned(const TreeNode& n) {  // Tigris::should_prune
    hash_->within(n.pose.x, n.pose.y, p_.pruneRadius,
                  [&](int idx) { return dist(nodes_[static_cast<std::size_t>(idx)].pose, n.pose); }, nearBuf_);
    for (const int q : nearBuf_) {
        const TreeNode& o = nodes_[static_cast<std::size_t>(q)];
        if (o.cost <= n.cost && o.info >= n.info) return true;
    }
    return false;
}

int IaTigrisPlanner::addNode(TreeNode&& n, NodeData&& d) {
    const int idx = static_cast<int>(nodes_.size());
    n.inTree = true;
    hash_->add(idx, n.pose.x, n.pose.y);
    nodes_.push_back(std::move(n));
    data_.push_back(std::move(d));
    return idx;
}

void IaTigrisPlanner::rebuildHash() {
    hash_->clear();
    for (std::size_t i = 0; i < nodes_.size(); ++i) hash_->add(static_cast<int>(i), nodes_[i].pose.x, nodes_[i].pose.y);
}

// IPP::updateInformedConfig / InfoMapSearch::update_weighted_sampler: one-look reward of every
// cell at the nominal range, on the CURRENT belief (rebuilt every solve).
void IaTigrisPlanner::buildSampler(const BeliefState& belief) {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    const double decl = s_.camera.tilt - p_.viewPointGoal * s_.camera.fov / 2.0;
    const double rNom = s_.altitude / std::cos(decl);
    weights_.assign(g_.size(), 0.0);
    weightSum_ = 0.0;
    if (!p_.informedSampler) return;
    const double pNom = s_.det.prob(rNom);
    for (std::size_t k = 0; k < g_.size(); ++k) {
        double w;
        if (original) {
            double q;
            w = originalCellUpdate(belief.presence[k], rNom, s_.det, p_.reward, &q);
        } else {
            w = belief.residual[k] * pNom;
        }
        weights_[k] = std::max(w, 0.0);
        weightSum_ += weights_[k];
    }
}

// Tigris::reset_tree_to_pose
void IaTigrisPlanner::resetTree(const Pose2& start) {
    nodes_.clear();
    data_.clear();
    hash_->clear();
    lastPath_.clear();
    TreeNode root;
    root.pose = start;
    NodeData d;
    root.info = evaluate(root, d);  // the root's own footprint (IA-TIGRIS scores the root)
    addNode(std::move(root), std::move(d));
    best_ = 0;
    tailEnd_ = -1;
    tailInfo_ = 0.0;
}

// Tigris::recycle_tree_from_node + add_children_to_tree
void IaTigrisPlanner::recycle(int newRoot, double budget, PlanResult* res) {
    const auto t0 = Clock::now();
    const std::size_t oldN = nodes_.size();
    std::vector<std::vector<int>> kids(oldN);
    for (std::size_t i = 0; i < oldN; ++i) {
        if (nodes_[i].parent >= 0) kids[static_cast<std::size_t>(nodes_[i].parent)].push_back(static_cast<int>(i));
    }
    const double c0 = nodes_[static_cast<std::size_t>(newRoot)].cost;
    const double a0 = nodes_[static_cast<std::size_t>(newRoot)].arc;
    // the previous plan's tail: from newRoot to the end of the last best path
    int oldTail = -1;
    for (std::size_t i = 0; i < lastPath_.size(); ++i) {
        if (lastPath_[i] == newRoot) { oldTail = lastPath_.back(); break; }
    }
    std::vector<int> newIdx(oldN, -1);
    std::vector<TreeNode> nn;
    std::vector<NodeData> nd;
    nn.reserve(oldN);
    nd.reserve(oldN);
    // depth-first, children in insertion order (Tigris::add_children_to_tree): parents are
    // placed before their children, and the best node is picked in the same order
    std::vector<int> q{newRoot};
    while (!q.empty()) {
        const int o = q.back();
        q.pop_back();
        TreeNode n = std::move(nodes_[static_cast<std::size_t>(o)]);
        NodeData d = std::move(data_[static_cast<std::size_t>(o)]);
        if (o == newRoot) {
            n.parent = -1;
            n.cost = 0.0;
            n.arc = 0.0;
            n.edgeLen = 0.0;
            n.edge = DubinsPath();
            n.edge.q0 = n.pose;
            n.hasEdge = false;
            n.closed = false;
            d.looks.clear();
        } else {
            n.parent = newIdx[static_cast<std::size_t>(n.parent)];
            n.cost -= c0;
            n.arc -= a0;
            if (n.cost > budget + 1e-6) continue;  // over the new budget: dropped with its subtree
        }
        newIdx[static_cast<std::size_t>(o)] = static_cast<int>(nn.size());
        nn.push_back(std::move(n));
        nd.push_back(std::move(d));
        const auto& ks = kids[static_cast<std::size_t>(o)];
        for (auto it = ks.rbegin(); it != ks.rend(); ++it) q.push_back(*it);
    }
    nodes_.swap(nn);
    data_.swap(nd);
    // every reward again, root to leaf, on the updated belief (UPDATESUBTREE); the best node is
    // updated with a strict > as each child is re-added
    for (std::size_t i = 0; i < nodes_.size(); ++i) nodes_[i].info = evaluate(nodes_[i], data_[i]);
    rebuildHash();
    best_ = 0;
    for (std::size_t i = 1; i < nodes_.size(); ++i) {
        if (nodes_[i].info > nodes_[static_cast<std::size_t>(best_)].info) best_ = static_cast<int>(i);
    }
    tailEnd_ = oldTail >= 0 ? newIdx[static_cast<std::size_t>(oldTail)] : -1;
    tailInfo_ = tailEnd_ >= 0 ? nodes_[static_cast<std::size_t>(tailEnd_)].info : 0.0;
    res->recycled = true;
    res->nodesKept = static_cast<int>(nodes_.size());
    res->nodesDropped = static_cast<int>(oldN - nodes_.size());
    res->tailInfo = tailInfo_;
    res->recycleSeconds = std::chrono::duration<double>(Clock::now() - t0).count();
}

// The TIGRIS main loop with IA-TIGRIS's near-node loop (Tigris::replan_loop /
// extend_neighbors_towards_node).
void IaTigrisPlanner::grow(double budget, double timeLimit, int replanIndex, PlanResult* res) {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    std::mt19937_64 rng(p_.seed * 1000003ULL + static_cast<std::uint64_t>(replanIndex) * 7919ULL + 17ULL);
    std::uniform_real_distribution<double> uPsi(0.0, 2.0 * kPi);
    std::uniform_real_distribution<double> uX(s_.xMin, s_.xMax), uY(s_.yMin, s_.yMax);
    const bool informed = p_.informedSampler && weightSum_ > 0.0;
    std::discrete_distribution<std::size_t> cellDist;
    if (informed) cellDist = std::discrete_distribution<std::size_t>(weights_.begin(), weights_.end());
    const double decl = s_.camera.tilt - p_.viewPointGoal * s_.camera.fov / 2.0;
    const double backOff = s_.altitude * std::tan(decl);

    // Tigris::sample_node: the cell centre backed off along a random yaw
    auto sampleConfig = [&]() {
        Pose2 q;
        if (informed) {
            const std::size_t k = cellDist(rng);
            const int i = static_cast<int>(k) / g_.nx, j = static_cast<int>(k) % g_.nx;
            q.yaw = wrapPi(uPsi(rng));
            q.x = g_.cx(j) - backOff * std::cos(q.yaw);
            q.y = g_.cy(i) - backOff * std::sin(q.yaw);
        } else {
            q.x = uX(rng);
            q.y = uY(rng);
            q.yaw = wrapPi(uPsi(rng));
        }
        return q;
    };
    auto tEdge = [&](const TreeNode& n) { return startTime_ + (n.arc - n.edgeLen) / s_.speed; };
    auto elapsed = [&]() { return std::chrono::duration<double>(Clock::now() - t0_).count(); };
    auto consider = [&](int idx) {  // add_node_to_tree_or_prune: compare the ADDED node
        if (nodes_[static_cast<std::size_t>(idx)].info > nodes_[static_cast<std::size_t>(best_)].info) best_ = idx;
    };

    std::vector<int> nearNodes;
    while (elapsed() < timeLimit && (p_.maxIterations <= 0 || res->iterations < p_.maxIterations)) {
        ++res->iterations;
        const Pose2 sample = sampleConfig();
        const int nearest = hash_->nearest(sample.x, sample.y, [&](int idx) {
            return dist(nodes_[static_cast<std::size_t>(idx)].pose, sample);
        });
        if (nearest < 0) continue;
        TreeNode m;
        if (!steer(nearest, sample, budget, &m)) continue;
        NodeData md;
        if (!original) md.looks = edgeLooks(m.edge, m.edgeLen, tEdge(m));
        m.info = evaluate(m, md);
        const bool addM = m.edgeLen > 0.01 && !nodes_[static_cast<std::size_t>(nearest)].closed && !pruned(m);
        const Pose2 mPose = m.pose;
        int mIdx = -1;
        if (addM) {
            mIdx = addNode(std::move(m), std::move(md));
            consider(mIdx);
        } else {
            ++res->nodesPruned;
        }
        // every open tree node within extend_radius extends toward the sample (also when the
        // sample itself was pruned: it is still the target pose)
        hash_->within(mPose.x, mPose.y, p_.extendRadius,
                      [&](int idx) { return dist(nodes_[static_cast<std::size_t>(idx)].pose, mPose); }, nearNodes);
        for (const int q : nearNodes) {
            if (q == nearest || q == mIdx || nodes_[static_cast<std::size_t>(q)].closed) continue;
            if (dist(nodes_[static_cast<std::size_t>(q)].pose, mPose) < 1e-6) continue;  // colocated
            TreeNode a;
            if (!steer(q, mPose, budget, &a)) continue;
            if (a.edgeLen < 0.01) continue;
            NodeData ad;
            if (!original) ad.looks = edgeLooks(a.edge, a.edgeLen, tEdge(a));
            a.info = evaluate(a, ad);
            if (!pruned(a)) {
                consider(addNode(std::move(a), std::move(ad)));
            } else {
                ++res->nodesPruned;
            }
        }
    }
}

void IaTigrisPlanner::finish(PlanResult* res) {
    // the best path is still the plan we are flying (then the track is not revised)
    res->keptTail = tailEnd_ >= 0 && best_ == tailEnd_;
    lastPath_.clear();
    for (int q = best_; q >= 0; q = nodes_[static_cast<std::size_t>(q)].parent) lastPath_.push_back(q);
    std::reverse(lastPath_.begin(), lastPath_.end());
    res->path.clear();
    for (const int q : lastPath_) res->path.push_back(nodes_[static_cast<std::size_t>(q)]);
    res->cost = nodes_[static_cast<std::size_t>(best_)].cost;
    res->treeSize = hash_->size();
    res->reward = scorePath(res->path, *belief_, startTime_);
    res->seconds = std::chrono::duration<double>(Clock::now() - t0_).count();
}

PlanResult IaTigrisPlanner::plan(const Pose2& start, double budget, double timeLimit, const BeliefState& belief,
                                 int replanIndex, double startTime) {
    PlanResult res;
    belief_ = &belief;
    startTime_ = startTime;
    buildSampler(belief);
    t0_ = Clock::now();
    resetTree(start);
    grow(budget, timeLimit, replanIndex, &res);
    finish(&res);
    return res;
}

PlanResult IaTigrisPlanner::replanFrom(std::size_t pathIndex, double budget, double timeLimit,
                                       const BeliefState& belief, int replanIndex, double startTime) {
    if (!hasTree() || pathIndex >= lastPath_.size()) {
        throw std::logic_error("IaTigrisPlanner::replanFrom: no tree node at that index of the last path");
    }
    PlanResult res;
    belief_ = &belief;
    startTime_ = startTime;
    buildSampler(belief);
    t0_ = Clock::now();
    recycle(lastPath_[pathIndex], budget, &res);
    if (nodes_.empty()) {  // "Tree recycling cleared all nodes. Starting new plan" (cannot happen: the root stays)
        throw std::logic_error("IaTigrisPlanner::replanFrom: recycling removed the root");
    }
    grow(budget, timeLimit, replanIndex, &res);
    finish(&res);
    return res;
}

}  // namespace ia_tigris_search
