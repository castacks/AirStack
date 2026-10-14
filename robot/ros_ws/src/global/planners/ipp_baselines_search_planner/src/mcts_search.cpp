// =============================================================================
//  mcts_search.cpp — MCTSSearch.h / MCTSNode.h (see mcts_search.hpp).
// =============================================================================
#include "ipp_baselines_search_planner/mcts_search.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace ipp_baselines_search {

namespace {
constexpr double kPi = 3.14159265358979323846;
double constrainAngle(double x) {  // MCTSSearch::constrain_angle
    x = std::fmod(x + kPi, 2 * kPi);
    if (x < 0) x += 2 * kPi;
    return x - kPi;
}
}  // namespace

std::unique_ptr<InfoMapCopy> MctsSearchPlanner::copyMap(const InfoMapCopy& from) const {
    // std::make_unique<InfoMapSearch>(*parent_info_map): every array is copied. The arrays are
    // identical in every copy (the hashmap reward never writes the map), so their bytes come
    // from the master; the RNG state comes from `from` (that is what differs between copies).
    const_cast<MctsSearchPlanner*>(this)->copies_++;
    auto c = std::make_unique<InfoMapCopy>();
    c->gen = from.gen;
    if (p_.mctsRetainMapCopies) {  // ROS 1 memory: every node keeps its copy
        c->map = master_.map;
        c->copyMap = master_.copyMap;
        c->priority = master_.priority;
        c->sensorModelId = master_.sensorModelId;
        c->sampler = master_.sampler;
        return c;
    }
    // default: the same copy work, then the arrays are released (nothing ever reads them:
    // sampling uses the master's identical weights with the node's own generator)
    static volatile double sink = 0.0;
    {
        const std::vector<double> a = master_.map, b = master_.copyMap, pr = master_.priority;
        const std::vector<int> id = master_.sensorModelId;
        const std::discrete_distribution<std::size_t> smp = master_.sampler;
        sink = sink + (a.empty() ? 0.0 : a.back()) + (b.empty() ? 0.0 : b.back()) + (pr.empty() ? 0.0 : pr.back()) +
               (id.empty() ? 0 : id.back()) + static_cast<double>(smp.probabilities().size());
    }
    return c;
}

double MctsSearchPlanner::ucb1(const MctsNode& n) const {
    if (n.nVisits == 0) return std::numeric_limits<double>::max();
    return n.value() / (budget_ / s_.speed) +
           p_.mctsExplorationWeight * std::sqrt(std::log(nodes_[0].nVisits) / n.nVisits);
}

int MctsSearchPlanner::selectChildHighestUcb1(int idx) const {
    int best = -1;
    double bestScore = std::numeric_limits<double>::lowest();
    for (const int c : nodes_[static_cast<std::size_t>(idx)].children) {
        const double s = ucb1(nodes_[static_cast<std::size_t>(c)]);
        if (s > bestScore) {
            bestScore = s;
            best = c;
        }
    }
    return best;
}

Pose2 MctsSearchPlanner::primitiveGoal(const Pose2& from, int actionIdx) const {
    const double minAngle = -2 * kPi / 3;  // "-135 degrees" (sic: -120)
    const double maxAngle = 2 * kPi / 3;
    const int numAngles = p_.mctsNumActions / 2;
    const int angleIdx = actionIdx % numAngles;
    const double radius = (actionIdx > angleIdx) ? p_.mctsExtendRadius * 3.0 : p_.mctsExtendRadius;
    const double angle = minAngle + (maxAngle - minAngle) * static_cast<double>(angleIdx) / (numAngles - 1.) + from.yaw;
    Pose2 g;
    g.x = from.x + radius * std::cos(angle);
    g.y = from.y + radius * std::sin(angle);
    g.yaw = constrainAngle(angle);
    return g;
}

// SearchMap::search_information_gain_with_hashmaps: the chain's local updates replayed, then
// this node's edge and footprint; returns (and stores) its search_info = own + parent's
double MctsSearchPlanner::transitionValue(MctsNode& n) {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    eval_.begin(*belief_);
    std::vector<int> chain;
    for (int q = n.t.parent; q >= 0; q = nodes_[static_cast<std::size_t>(q)].t.parent) chain.push_back(q);
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const MctsNode& a = nodes_[static_cast<std::size_t>(*it)];
        if (original) {
            for (const auto& u : a.dPres) eval_.seedPresence(u.first, u.second);
        } else {
            for (const auto& u : a.dRes) eval_.seedResidual(u.first, u.second);
        }
    }
    eval_.markDelta();
    double inc;
    if (original) {
        originalNodeGain(eval_, n.t, startTime_, /*edgeFirst=*/true, p_.useEdgeReward);
        inc = eval_.reward().original;
        eval_.delta(&n.dPres, nullptr);
        n.dRes.clear();
    } else {
        matchedNodeGain(eval_, n.t, startTime_);
        inc = eval_.reward().matched;
        eval_.delta(nullptr, &n.dRes);
        n.dPres.clear();
    }
    const double parentInfo = n.t.parent >= 0 ? nodes_[static_cast<std::size_t>(n.t.parent)].searchInfo : 0.0;
    n.searchInfo = inc + parentInfo;
    return n.searchInfo;
}

// make_feasible_action_node (315-346)
bool MctsSearchPlanner::makeFeasible(int fromIdx, int actionIdx, MctsNode* out) {
    Pose2 goal;
    {
        MctsNode& from = nodes_[static_cast<std::size_t>(fromIdx)];
        if (p_.mctsRolloutType == "informed") {
            // make_informed_action_node: InfoMapSearch::sample_xy on the FROM node's map copy
            const std::size_t cell = master_.sampler(from.map->gen);
            const int ny = g_.ny;
            const int row = static_cast<int>(cell) / ny, col = static_cast<int>(cell) % ny;  // row = x
            goal.x = g_.cx(row);
            goal.y = g_.cy(col);
            std::uniform_real_distribution<double> psi(-kPi, kPi - 0.0001);
            goal.yaw = psi(gen_);
            goal.yaw = wrapPi(goal.yaw);  // enforceBounds
        } else {
            goal = primitiveGoal(from.t.pose, actionIdx);
        }
    }
    TreeNode t;
    const MctsNode& from = nodes_[static_cast<std::size_t>(fromIdx)];
    if (!extend(from.t, fromIdx, goal, std::numeric_limits<double>::max(), &t)) return false;
    out->t = t;
    out->map = copyMap(*from.map);  // feasible_mcts_node->set_info_map(copy of from's map)
    return true;
}

// act_and_transition_to_new_node (274-308)
int MctsSearchPlanner::actAndTransition(int fromIdx, int actionIdx) {
    MctsNode nn;
    bool ok = makeFeasible(fromIdx, actionIdx, &nn);
    int attempts = 1;
    const int maxAttempts = 6;
    while (!ok && attempts < maxAttempts) {
        ok = makeFeasible(fromIdx, actionIdx, &nn);
        attempts++;
        if (attempts == maxAttempts) throw NodeStuckException();  // (sic) also after a success
    }
    const MctsNode& from = nodes_[static_cast<std::size_t>(fromIdx)];
    nn.t.parent = fromIdx;
    nn.name = from.name + "." + std::to_string(actionIdx);
    nn.t.depth = from.t.depth + 1;
    nn.map = copyMap(*from.map);  // new_node->set_info_map(copy of the parent's map)
    const double fromInformation = from.information;
    const double transition = transitionValue(nn);
    nn.edgeValue = transition;
    nn.information = fromInformation + transition;
    if (static_cast<std::size_t>(nn.t.depth) > depthMax_) depthMax_ = static_cast<std::size_t>(nn.t.depth);
    nodes_.push_back(std::move(nn));
    return static_cast<int>(nodes_.size()) - 1;
}

void MctsSearchPlanner::expand(int idx) {
    for (int i = 0; i < p_.mctsNumActions; i++) {
        try {
            const int c = actAndTransition(idx, i);
            nodes_[static_cast<std::size_t>(idx)].children.push_back(c);
        } catch (const NodeStuckException&) {
        }
    }
    ++expansions_;
}

double MctsSearchPlanner::rollout(int idx) {
    const std::size_t keep = nodes_.size();
    int cur = idx, num = 0;
    double value = 0.0;
    std::uniform_int_distribution<int> actionDist(0, p_.mctsNumActions - 1);
    do {
        try {
            const int a = actionDist(gen_);
            const int ni = actAndTransition(cur, a);
            value += nodes_[static_cast<std::size_t>(ni)].information;
            cur = ni;
        } catch (const NodeStuckException&) {
            break;
        }
        num++;
    } while (num < p_.mctsMaxRolloutDepth && cur >= 0);
    nodes_.resize(keep);  // the rollout nodes are deleted
    return value;
}

void MctsSearchPlanner::rolloutAndBackpropagate(int idx) {
    if (nodes_[static_cast<std::size_t>(idx)].nVisits != 0) {
        throw std::invalid_argument("Rollout called on a node that has already been visited");
    }
    const double f = rollout(idx);
    nodes_[static_cast<std::size_t>(idx)].futureValue = f;
    nodes_[static_cast<std::size_t>(idx)].nVisits++;
    int node = idx;
    while (node >= 0 && nodes_[static_cast<std::size_t>(node)].t.parent >= 0) {
        const int parent = nodes_[static_cast<std::size_t>(node)].t.parent;
        nodes_[static_cast<std::size_t>(parent)].futureValue += nodes_[static_cast<std::size_t>(node)].value();
        nodes_[static_cast<std::size_t>(parent)].nVisits += 1;
        node = parent;
    }
}

bool MctsSearchPlanner::setupSolve(const Pose2& start, double budget) {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    // the planner's InfoMapSearch for this solve (fetch_latest_belief + update_weighted_sampler):
    // weights = one-look reward at the nominal range x priority, row (x)-major
    const double rNom = nominalRange();
    const double pNom = s_.det.prob(rNom);
    std::vector<double> w;
    w.reserve(g_.size());
    for (int j = 0; j < g_.nx; ++j) {
        for (int i = 0; i < g_.ny; ++i) {
            const std::size_t k = static_cast<std::size_t>(i * g_.nx + j);
            double v;
            if (original) {
                double q;
                v = originalCellUpdate(belief_->presence[k], rNom, s_.det, p_.reward, &q);
            } else {
                v = belief_->residual[k] * pNom;
            }
            w.push_back(std::max(v, 0.0));
        }
    }
    master_.map = belief_->presence;
    master_.copyMap = belief_->residual;
    master_.priority.assign(g_.size(), 1.0);
    master_.sensorModelId.assign(g_.size(), 0);
    master_.sampler = std::discrete_distribution<std::size_t>(w.begin(), w.end());
    // the global InfoMap's generator never advances (MCTS only samples from copies), so every
    // replan's root copy starts from the same state: seeded with the mission seed alone
    master_.gen.seed(static_cast<std::uint32_t>(p_.seed));
    gen_.seed(static_cast<std::uint32_t>(solveSeed(p_.seed, replanIndex_)));

    nodes_.clear();
    expansions_ = 0;
    depthMax_ = 0;
    copies_ = 0;
    degenerate_ = false;
    MctsNode root;
    root.name = "R";
    root.t.pose = start;
    root.t.budgetRemaining = budget;
    root.t.edge.q0 = start;
    root.t.edge.rho = s_.turnRadius;
    root.information = 0;
    root.nVisits = rootVisits_;       // tree_root is a member: its statistics carry over
    root.futureValue = rootFuture_;
    root.edgeValue = rootEdgeValue_;
    {
        // auto info_map = dynamic_cast<InfoMapSearch &>(info_map_): a copy, then another copy
        const auto local = copyMap(master_);
        root.map = copyMap(master_);
    }
    nodes_.push_back(std::move(root));
    expand(0);
    best_ = 0;
    return true;
}

bool MctsSearchPlanner::loopOnce() {
    (void)copyMap(master_);  // auto info_map = dynamic_cast<InfoMapSearch &>(info_map_): a copy per call
    int cur = 0;
    while (!nodes_[static_cast<std::size_t>(cur)].children.empty()) {
        const int c = selectChildHighestUcb1(cur);
        if (c < 0) {  // every score NaN: ROS 1 dereferences nullptr
            degenerate_ = true;
            return false;
        }
        cur = c;
    }
    if (nodes_[static_cast<std::size_t>(cur)].nVisits > 0) {
        expand(cur);
        if (nodes_[static_cast<std::size_t>(cur)].children.empty()) {  // children.at(0) throws in ROS 1
            degenerate_ = true;
            return false;
        }
        cur = nodes_[static_cast<std::size_t>(cur)].children.at(0);
    }
    rolloutAndBackpropagate(cur);
    if (best_ == 0 || nodes_[static_cast<std::size_t>(cur)].information > nodes_[static_cast<std::size_t>(best_)].information) {
        best_ = cur;
    }
    return true;
}

void MctsSearchPlanner::teardown() {
    if (nodes_.empty()) return;
    rootVisits_ = nodes_[0].nVisits;
    rootFuture_ = nodes_[0].futureValue;
    rootEdgeValue_ = nodes_[0].edgeValue;
}

int MctsSearchPlanner::bestLeaf() const {
    int cur = 0;
    while (!nodes_[static_cast<std::size_t>(cur)].children.empty()) {
        double bestV = -std::numeric_limits<double>::infinity();
        int next = cur;
        for (const int c : nodes_[static_cast<std::size_t>(cur)].children) {
            const double v = nodes_[static_cast<std::size_t>(c)].value();
            if (v > bestV) {
                next = c;
                bestV = v;
            }
        }
        if (next == cur) break;  // every child NaN: ROS 1 loops forever here
        cur = next;
    }
    return cur;
}

std::vector<TreeNode> MctsSearchPlanner::bestPath() {
    std::vector<TreeNode> out;
    if (nodes_.empty()) return out;
    std::vector<int> chain;
    for (int q = bestLeaf(); q >= 0; q = nodes_[static_cast<std::size_t>(q)].t.parent) chain.push_back(q);
    std::reverse(chain.begin(), chain.end());
    for (std::size_t i = 0; i < chain.size(); ++i) {
        TreeNode t = nodes_[static_cast<std::size_t>(chain[i])].t;
        t.parent = static_cast<int>(i) - 1;
        t.info = nodes_[static_cast<std::size_t>(chain[i])].information;
        out.push_back(t);
    }
    return out;
}

void MctsSearchPlanner::extras(std::map<std::string, double>& x) const {
    x["expansions"] = static_cast<double>(expansions_);
    x["max_depth"] = static_cast<double>(depthMax_);
    x["root_visits"] = nodes_.empty() ? 0.0 : nodes_[0].nVisits;
    x["info_map_copies"] = static_cast<double>(copies_);
    x["logged_best_information"] = nodes_.empty() ? 0.0 : nodes_[static_cast<std::size_t>(best_)].information;
    x["plan_depth"] = static_cast<double>(nodes_.empty() ? 0 : nodes_[static_cast<std::size_t>(bestLeaf())].t.depth);
    x["degenerate_stop"] = degenerate_ ? 1.0 : 0.0;
}

}  // namespace ipp_baselines_search
