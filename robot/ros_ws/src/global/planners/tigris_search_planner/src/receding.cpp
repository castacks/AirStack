// =============================================================================
//  receding.cpp — receding-horizon TIGRIS and the run-folder JSON writers.
// =============================================================================
#include "tigris_search_planner/receding.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace J = tigris_json;

namespace tigris_search {
namespace {

constexpr double kPi = 3.14159265358979323846;

J::Array numbersOf(const std::vector<double>& v, int ndigits) {
    const double scale = std::pow(10.0, ndigits);
    J::Array out;
    out.reserve(v.size());
    for (const double d : v) out.push_back(J::Value(std::round(d * scale) / scale));
    return out;
}

J::Array indicesOf(const std::vector<int>& v) {
    J::Array out;
    for (const int i : v) out.push_back(J::Value(i));
    return out;
}

J::Value vec2(double a, double b) { return J::Value(J::Array{J::Value(a), J::Value(b)}); }
J::Value vec3(double a, double b, double c) {
    return J::Value(J::Array{J::Value(a), J::Value(b), J::Value(c)});
}

J::Value rewardJson(const PathReward& r) {
    J::Object o;
    o["original"] = J::Value(r.original);
    o["matched"] = J::Value(r.matched);
    return J::Value(std::move(o));
}

}  // namespace

Pose2 defaultStartPose(const Scenario& sc, int agentIndex, const HorizonParams& hp) {
    const AgentSpec& a = sc.agents.at(static_cast<std::size_t>(agentIndex));
    Pose2 p;
    p.x = a.startE;
    p.y = a.startN;
    p.yaw = hp.startYaw < 1e8 ? wrapPi(hp.startYaw) : std::atan2(sc.centerY() - p.y, sc.centerX() - p.x);
    return p;
}

RecedingHorizon::RecedingHorizon(const Scenario& sc, int agentIndex, const Camera& cam, const TigrisParams& tp,
                                 const HorizonParams& hp)
    : sc_(sc), agent_(agentIndex), tp_(tp), hp_(hp),
      grid_(sc, hp.gridRes, tp.reward.initialConfidence) {
    if (agentIndex < 0 || static_cast<std::size_t>(agentIndex) >= sc.agents.size()) {
        throw std::out_of_range("agent index not in the scenario team");
    }
    setup_.camera = cam;
    setup_.det = sc.det;
    setup_.speed = sc.speed;
    setup_.turnRadius = sc.minTurnRadius;
    setup_.dubinsStep = sc.dubinsStep;
    setup_.altitude = sc.altitude + sc.agents[static_cast<std::size_t>(agentIndex)].altitudeOffset;
    setup_.xMin = sc.xMin;
    setup_.xMax = sc.xMax;
    setup_.yMin = sc.yMin;
    setup_.yMax = sc.yMax;
    planner_ = std::make_unique<TigrisPlanner>(grid_, setup_, tp_);
    eval_ = std::make_unique<RewardEvaluator>(grid_, setup_.det, tp_.reward);
    belief_ = BeliefState::fromGrid(grid_);
    budget_ = hp.budgetOverride > 0.0 ? hp.budgetOverride : sc.budgetDistance();
    if (!std::isfinite(budget_) || !(budget_ > 0.0)) {
        throw std::runtime_error("the budget is not finite and positive: set team.max_flight_time_s or "
                                 "team.max_flight_distance_m in mission.yaml (or budget_m)");
    }
}

double RecedingHorizon::commitDistance() const {
    return hp_.lookahead + setup_.speed * (hp_.replanPlanningTime + hp_.commitMarginTime);
}

std::size_t RecedingHorizon::indexAtArc(double arc) const {
    const auto it = std::lower_bound(track_.begin(), track_.end(), arc,
                                     [](const TrackSample& s, double a) { return s.arc < a; });
    if (it == track_.end()) return track_.empty() ? 0 : track_.size() - 1;
    return static_cast<std::size_t>(it - track_.begin());
}

std::vector<TrackSample> RecedingHorizon::sampleSegment(const std::vector<TreeNode>& path,
                                                        const TrackSample& from) const {
    std::vector<TrackSample> out;
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) total += path[i].edgeLen;
    if (!(total > 0.0)) return out;
    const double step = std::max(setup_.speed * sc_.dt, 1e-3);
    const double h = setup_.altitude;
    const Camera& cam = setup_.camera;
    std::size_t e = 1;
    double cum = 0.0;  // arc at the start of edge e
    for (int k = 1;; ++k) {
        double u = k * step;
        const bool last = u >= total - 1e-6;
        if (last) u = total;
        while (e + 1 < path.size() && u > cum + path[e].edgeLen + 1e-12) {
            cum += path[e].edgeLen;
            ++e;
        }
        const Pose2 q = path[e].edge.sample(std::min(u - cum, path[e].edgeLen));
        TrackSample s;
        s.arc = from.arc + u;
        s.t = from.t + u / setup_.speed;
        s.x = q.x;
        s.y = q.y;
        s.z = h;
        s.yaw = q.yaw;
        s.phi = cam.phiAt(s.t);
        boresightGround(q.x, q.y, h, q.yaw, s.phi, cam.tilt, &s.bx, &s.by);
        s.bz = 0.0;
        out.push_back(s);
        if (last) break;
    }
    return out;
}

Look RecedingHorizon::sampleLook(std::size_t k) const {
    const TrackSample& s = track_.at(k);
    const double dt = k > 0 ? s.t - track_[k - 1].t : sc_.dt;
    return lookFromPose(s.x, s.y, s.z, s.yaw, setup_.camera, setup_.det, std::max(dt, 0.0) / setup_.det.dtRef,
                        s.phi);
}

std::vector<Look> RecedingHorizon::plannedLooks(double a0, double a1) const {
    std::vector<Look> out;
    for (std::size_t k = 0; k < track_.size(); ++k) {
        if (track_[k].arc > a0 && track_[k].arc <= a1) out.push_back(sampleLook(k));
    }
    return out;
}

void RecedingHorizon::record(ReplanRecord r, const PlanResult& res) {
    r.index = static_cast<int>(records_.size());
    r.seconds = res.seconds;
    r.iterations = res.iterations;
    r.treeSize = res.treeSize;
    r.pruned = res.nodesPruned;
    r.nodes = static_cast<int>(res.path.size());
    r.improved = res.improved();
    r.segmentLength = res.cost;
    r.segmentReward = res.reward;
    r.residualMassFlown = belief_.residualMass();
    r.trackLength = totalArc();
    records_.push_back(r);
}

bool RecedingHorizon::start(const Pose2& start) {
    const PlanResult res = planner_->plan(start, budget_, hp_.initialPlanningTime, belief_, 0, 0.0);
    ReplanRecord rec;
    rec.trigger = "initial";
    rec.budgetLeft = budget_;
    rec.timeLimit = hp_.initialPlanningTime;
    track_.clear();
    if (res.improved()) {
        TrackSample s0;
        s0.x = start.x;
        s0.y = start.y;
        s0.z = setup_.altitude;
        s0.yaw = start.yaw;
        s0.phi = setup_.camera.phiAt(0.0);
        boresightGround(start.x, start.y, setup_.altitude, start.yaw, s0.phi, setup_.camera.tilt, &s0.bx, &s0.by);
        track_.push_back(s0);
        const auto seg = sampleSegment(res.path, s0);
        track_.insert(track_.end(), seg.begin(), seg.end());
        revision_ = 0;
        plan_ = res.path;
        planArc0_ = 0.0;
    }
    if (!hp_.receding) done_ = true;
    record(rec, res);
    return res.improved();
}

std::vector<TreeNode> RecedingHorizon::tailFrom(const TrackSample& c) const {
    std::vector<TreeNode> out;
    if (plan_.size() < 2) return out;
    const double rel = c.arc - planArc0_;  // commit arc along plan_
    std::size_t i = 1;
    while (i < plan_.size() && plan_[i].arc <= rel + 1e-6) ++i;
    if (i >= plan_.size()) return out;  // committed to the end of the plan: nothing left
    TreeNode root;
    root.pose.x = c.x;
    root.pose.y = c.y;
    root.pose.yaw = c.yaw;
    out.push_back(root);
    // node i, reached from the commit pose along the rest of its edge
    const TreeNode& ni = plan_[i];
    const double edgeStartArc = ni.arc - ni.edgeLen;      // arc of plan_[i-1]
    const double cut = std::max(0.0, rel - edgeStartArc);  // part of edge i already committed
    TreeNode first = ni;
    first.parent = 0;
    first.edge.q0 = root.pose;
    first.edgeLen = std::max(0.0, ni.edgeLen - cut);
    first.arc = first.edgeLen;
    first.cost = first.edgeLen;
    // the straight part of the edge that is still ahead (Motion::start_edge / end_edge)
    first.hasEdge = ni.hasEdge && ni.edgeEndS > cut;
    if (first.hasEdge) {
        first.edgeStartS = std::max(ni.edgeStartS, cut) - cut;
        first.edgeEndS = ni.edgeEndS - cut;
        first.edgeStart = ni.edge.sample(first.edgeStartS + cut);
        first.edgeEnd = ni.edgeEnd;
    }
    // re-express the remaining edge as a path starting at the commit pose
    DubinsPath rest = ni.edge;
    {
        // walk the segments of the original word forward by `cut`
        double t = cut / rest.rho;
        for (int k = 0; k < 3; ++k) {
            const double d = std::min(t, rest.seg[k]);
            rest.seg[k] -= d;
            t -= d;
        }
        rest.q0 = root.pose;
    }
    first.edge = rest;
    out.push_back(first);
    for (std::size_t j = i + 1; j < plan_.size(); ++j) {
        TreeNode n = plan_[j];
        n.parent = static_cast<int>(out.size()) - 1;
        n.arc -= rel;
        n.cost = out.back().cost + n.edgeLen;
        out.push_back(n);
    }
    return out;
}

bool RecedingHorizon::due(double progressArc, double secondsSinceLastReplan) const {
    if (!hp_.receding || done_ || track_.empty()) return false;
    if (secondsSinceLastReplan >= hp_.replanPeriod) return true;
    return secondsSinceLastReplan >= hp_.minReplanSpacing &&
           totalArc() - progressArc <= commitDistance() + hp_.endTriggerMargin;
}

void RecedingHorizon::absorb(const std::vector<Look>& flownLooks) {
    if (flownLooks.empty()) return;
    eval_->begin(belief_);
    eval_->pass(flownLooks, true, true);
    eval_->commit(belief_);
}

bool RecedingHorizon::replan(double progressArc, const std::vector<Look>& flownLooks, const std::string& trigger) {
    if (track_.empty() || done_) return false;
    // 1. the flown looks become evidence (all misses); ORIGINAL: one pass
    absorb(flownLooks);
    // 2. commit point
    const double sc = std::min(progressArc + commitDistance(), totalArc());
    const std::size_t kc = indexAtArc(sc);
    // 3. planning copy with the committed-but-not-flown looks
    BeliefState planB = belief_;
    const std::vector<Look> committed = plannedLooks(progressArc, track_[kc].arc);
    if (!committed.empty()) {
        eval_->begin(planB);
        eval_->pass(committed, true, true);
        eval_->commit(planB);
    }
    ReplanRecord rec;
    rec.trigger = trigger;
    rec.progressArc = progressArc;
    rec.commitArc = track_[kc].arc;
    rec.budgetLeft = budget_ - track_[kc].arc;
    rec.timeLimit = hp_.replanPlanningTime;
    rec.flownLooks = static_cast<int>(flownLooks.size());
    // 4. remaining budget
    if (rec.budgetLeft < hp_.minReplanBudget) {
        done_ = true;
        record(rec, PlanResult());
        return false;
    }
    const TrackSample& c = track_[kc];
    Pose2 start;
    start.x = c.x;
    start.y = c.y;
    start.yaw = c.yaw;
    // c.t: the sweep phase at the commit point, so the new segment continues the sweep
    const PlanResult res =
        planner_->plan(start, rec.budgetLeft, hp_.replanPlanningTime, planB, ++replanCount_, c.t);
    // the rest of the plan we are flying, scored the same way on the same belief
    const std::vector<TreeNode> kept = tailFrom(c);
    if (kept.size() > 1) rec.keptReward = planner_->scorePath(kept, planB, c.t);
    const bool original = tp_.reward.mode == RewardMode::ORIGINAL;
    const double newR = original ? res.reward.original : res.reward.matched;
    const double keptR = original ? rec.keptReward.original : rec.keptReward.matched;
    bool changed = false;
    if (res.improved() && newR > keptR) {
        const TrackSample from = track_[kc];
        const auto seg = sampleSegment(res.path, from);
        track_.resize(kc + 1);
        track_.insert(track_.end(), seg.begin(), seg.end());
        ++revision_;
        changed = true;
        plan_ = res.path;
        planArc0_ = from.arc;
    } else if (kc + 1 >= track_.size()) {
        done_ = true;  // nothing reachable adds reward past the end of the track
    }
    record(rec, res);
    return changed;
}

GimbalLimits gimbalLimits(const Camera& cam, bool lockGimbal) {
    GimbalLimits g;
    if (cam.sweep) {
        g.locked = false;
        g.maxRad = std::max(kGimbalTravelRad, cam.sweepAmplitude);
        g.pitchNudgeMaxRad = lockGimbal ? kGimbalLockedRad : kPitchNudgeRad;
    } else if (!lockGimbal) {
        g.locked = false;
        g.maxRad = kGimbalTravelRad;
        g.pitchNudgeMaxRad = kPitchNudgeRad;
    }
    return g;
}

// ---- JSON --------------------------------------------------------------------
std::vector<int> servicedCells(const Scenario& sc, const RecedingHorizon& rh) {
    std::vector<char> hit(sc.cellX.size(), 0);
    for (std::size_t k = 0; k < rh.track().size(); ++k) {
        const Look l = rh.sampleLook(k);
        if (!l.valid) continue;
        const double r2 = l.radius * l.radius;
        for (std::size_t c = 0; c < sc.cellX.size(); ++c) {
            if (hit[c]) continue;
            const double dx = sc.cellX[c] - l.gx, dy = sc.cellY[c] - l.gy;
            if (dx * dx + dy * dy <= r2) hit[c] = 1;
        }
    }
    std::vector<int> out;
    for (std::size_t c = 0; c < hit.size(); ++c) if (hit[c]) out.push_back(static_cast<int>(c));
    return out;
}

namespace {

J::Object tigrisBlock(const RecedingHorizon& rh, const TrackMeta& meta) {
    const TigrisParams& tp = rh.tigrisParams();
    const HorizonParams& hp = rh.horizonParams();
    J::Object o;
    o["planner"] = J::Value("tigris");
    o["plan_id"] = J::Value(meta.planId);
    o["revision"] = J::Value(meta.revision);
    o["reward_mode"] = J::Value(meta.rewardMode);
    o["sampler"] = J::Value(meta.sampler);
    o["gimbal_locked"] = J::Value(meta.gimbalLocked);
    o["gimbal_max_rad"] = J::Value(meta.gimbalMaxRad);
    o["pitch_nudge_max_rad"] = J::Value(meta.pitchNudgeMaxRad);
    {
        const Camera& cam = rh.setup().camera;
        J::Object ga;
        ga["enabled"] = J::Value(cam.sweep);
        ga["sweep_rate_deg_s"] = J::Value(cam.sweepRate * 180.0 / kPi);
        ga["sweep_amplitude_deg"] = J::Value(cam.sweepAmplitude * 180.0 / kPi);
        o["gimbal_actuation"] = J::Value(std::move(ga));
    }
    o["receding"] = J::Value(hp.receding);
    o["extend_dist_m"] = J::Value(tp.extendDist);
    o["extend_radius_m"] = J::Value(tp.extendRadius);
    o["prune_radius_m"] = J::Value(tp.pruneRadius);
    o["reward_step_m"] = J::Value(tp.rewardStep);
    o["view_point_goal"] = J::Value(tp.viewPointGoal);
    o["bounds_margin_m"] = J::Value(tp.boundsMargin);
    o["use_entropy"] = J::Value(tp.reward.useEntropy);
    o["rs"] = J::Value(tp.reward.rs);
    o["rf"] = J::Value(tp.reward.rf);
    o["initial_confidence"] = J::Value(tp.reward.initialConfidence);
    o["tpr_beyond_beta"] = J::Value(tp.reward.tprBeyondBeta);
    o["grid_res_m"] = J::Value(hp.gridRes);
    o["initial_planning_time_s"] = J::Value(hp.initialPlanningTime);
    o["replan_planning_time_s"] = J::Value(hp.replanPlanningTime);
    o["replan_period_s"] = J::Value(hp.replanPeriod);
    o["commit_distance_m"] = J::Value(rh.commitDistance());
    o["budget_m"] = J::Value(rh.budget());
    o["camera_fov_deg"] = J::Value(rh.setup().camera.fov * 180.0 / kPi);
    o["camera_tilt_deg"] = J::Value(rh.setup().camera.tilt * 180.0 / kPi);
    o["seed"] = J::Value(static_cast<double>(tp.seed));
    return o;
}

}  // namespace

std::string agentTrackJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta) {
    const AgentSpec& a = sc.agents.at(static_cast<std::size_t>(agentIndex));
    const double hx = a.homeX(), hy = a.homeY(), hz = 0.0;
    std::vector<double> t, arc, x, y, z, yaw, bx, by, bz, phi, pitch, n, e, sn, se;
    for (const TrackSample& s : rh.track()) {
        t.push_back(s.t);
        arc.push_back(s.arc);
        x.push_back(s.x - hx);
        y.push_back(s.y - hy);
        z.push_back(s.z - hz);
        yaw.push_back(s.yaw);
        bx.push_back(s.bx - hx);
        by.push_back(s.by - hy);
        bz.push_back(s.bz - hz);
        phi.push_back(s.phi);
        pitch.push_back(0.0);
        n.push_back(s.y);
        e.push_back(s.x);
        sn.push_back(s.by);
        se.push_back(s.bx);
    }
    J::Object samples;
    samples["t"] = J::Value(numbersOf(t, 3));
    samples["arc"] = J::Value(numbersOf(arc, 3));
    samples["x_map"] = J::Value(numbersOf(x, 3));
    samples["y_map"] = J::Value(numbersOf(y, 3));
    samples["z_map"] = J::Value(numbersOf(z, 3));
    samples["yaw_enu"] = J::Value(numbersOf(yaw, 5));
    samples["bx_map"] = J::Value(numbersOf(bx, 3));
    samples["by_map"] = J::Value(numbersOf(by, 3));
    samples["bz_map"] = J::Value(numbersOf(bz, 3));
    samples["gimbal_phi"] = J::Value(numbersOf(phi, 5));
    samples["pitch"] = J::Value(numbersOf(pitch, 5));
    samples["n"] = J::Value(numbersOf(n, 3));
    samples["e"] = J::Value(numbersOf(e, 3));
    samples["sensor_n"] = J::Value(numbersOf(sn, 3));
    samples["sensor_e"] = J::Value(numbersOf(se, 3));

    const std::vector<int> cells = servicedCells(sc, rh);
    const auto& tr = rh.track();
    J::Object out;
    out["schema"] = J::Value("mtl.agent_track/1");
    out["mission"] = J::Value(sc.name);
    out["agent"] = J::Value(a.name);
    out["agent_index"] = J::Value(agentIndex);
    out["home_enu"] = vec3(hx, hy, hz);
    out["frame"] = J::Value("map = world ENU - home_enu; n/e = mission NED");
    out["single_axis"] = J::Value(true);
    out["scheduled"] = J::Value(rh.setup().camera.sweep);  // the gimbal follows a planned schedule
    out["tilt_rad"] = J::Value(rh.setup().camera.tilt);
    out["fov_rad"] = J::Value(rh.setup().camera.fov);
    out["speed_mps"] = J::Value(sc.speed);
    out["min_turn_radius_m"] = J::Value(sc.minTurnRadius);
    out["altitude_m"] = J::Value(rh.setup().altitude);
    out["dt_s"] = J::Value(sc.dt);
    out["budget_m"] = J::Value(rh.budget());
    out["flown_length_m"] = J::Value(rh.totalArc());
    out["flight_time_s"] = J::Value(tr.empty() ? 0.0 : tr.back().t);
    out["feasible"] = J::Value(rh.totalArc() <= rh.budget() + 1e-6);
    out["serviced_cells"] = J::Value(indicesOf(cells));
    out["planned_cells"] = J::Value(indicesOf(cells));
    out["route_note"] = J::Value("tigris " + meta.rewardMode + " reward, revision " + std::to_string(meta.revision));
    out["extension_note"] = J::Value("");
    out["tigris"] = J::Value(tigrisBlock(rh, meta));
    out["samples"] = J::Value(std::move(samples));
    return J::Value(std::move(out)).dump();
}

std::string planJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta) {
    const AgentSpec& a = sc.agents.at(static_cast<std::size_t>(agentIndex));
    std::vector<double> t, n, e, h, sn, se, roll, pitch, yaw, gphi;
    for (const TrackSample& s : rh.track()) {
        gphi.push_back(s.phi);
        t.push_back(s.t);
        n.push_back(s.y);
        e.push_back(s.x);
        h.push_back(s.z);
        sn.push_back(s.by);
        se.push_back(s.bx);
        roll.push_back(0.0);
        pitch.push_back(0.0);
        yaw.push_back(wrapPi(kPi / 2.0 - s.yaw));  // NED yaw (CW from North)
    }
    J::Object samples;
    samples["t"] = J::Value(numbersOf(t, 3));
    samples["n"] = J::Value(numbersOf(n, 3));
    samples["e"] = J::Value(numbersOf(e, 3));
    samples["h"] = J::Value(numbersOf(h, 3));
    samples["sensor_n"] = J::Value(numbersOf(sn, 3));
    samples["sensor_e"] = J::Value(numbersOf(se, 3));
    samples["roll"] = J::Value(numbersOf(roll, 5));
    samples["pitch"] = J::Value(numbersOf(pitch, 5));
    samples["yaw"] = J::Value(numbersOf(yaw, 5));
    samples["gimbal_phi"] = J::Value(numbersOf(gphi, 5));
    const std::vector<int> cells = servicedCells(sc, rh);
    const auto& tr = rh.track();
    J::Object diag;
    diag["feasible"] = J::Value(rh.totalArc() <= rh.budget() + 1e-6);
    diag["flown_length_m"] = J::Value(rh.totalArc());
    diag["flight_time_s"] = J::Value(tr.empty() ? 0.0 : tr.back().t);
    diag["budget_used_frac"] = J::Value(rh.budget() > 0 ? rh.totalArc() / rh.budget() : 0.0);
    diag["replans"] = J::Value(static_cast<int>(rh.records().size()));
    diag["revision"] = J::Value(meta.revision);
    J::Object agent;
    agent["name"] = J::Value(a.name);
    agent["start_ned"] = tr.empty() ? vec2(a.startN, a.startE) : vec2(tr.front().y, tr.front().x);
    agent["home_ned"] = vec2(a.homeN, a.homeE);
    agent["budget_m"] = J::Value(rh.budget());
    agent["path_length_m"] = J::Value(rh.totalArc());
    agent["duration_s"] = J::Value(tr.empty() ? 0.0 : tr.back().t);
    agent["serviced_cells"] = J::Value(indicesOf(cells));
    agent["planned_cells"] = J::Value(indicesOf(cells));
    agent["clusters"] = J::Value(J::Array{});
    agent["samples"] = J::Value(std::move(samples));
    agent["diagnostics"] = J::Value(std::move(diag));

    J::Array centers, mass;
    for (std::size_t i = 0; i < sc.cellX.size(); ++i) {
        centers.push_back(vec2(sc.cellY[i], sc.cellX[i]));
        mass.push_back(J::Value(sc.cellMass[i]));
    }
    J::Object cellsOut;
    cellsOut["centers"] = J::Value(std::move(centers));
    cellsOut["mass"] = J::Value(std::move(mass));
    J::Object gen;
    gen["name"] = J::Value("tigris_search_planner");
    gen["library"] = J::Value("TIGRIS (Moon et al. 2023), single agent, receding horizon");
    gen["mode"] = J::Value(meta.rewardMode);
    J::Object metaO;
    metaO["tigris"] = J::Value(tigrisBlock(rh, meta));
    metaO["budget_dist_m"] = J::Value(rh.budget());
    metaO["steps"] = J::Value(static_cast<double>(tr.size()));
    J::Object out;
    out["schema"] = J::Value("mtl.plan/1");
    out["generator"] = J::Value(std::move(gen));
    out["mission"] = sc.raw["mission"];
    out["dt"] = J::Value(sc.dt);
    out["agents"] = J::Value(J::Array{J::Value(std::move(agent))});
    out["cells"] = J::Value(std::move(cellsOut));
    out["meta"] = J::Value(std::move(metaO));
    return J::Value(std::move(out)).dump();
}

std::string replansJson(const Scenario& sc, int agentIndex, const RecedingHorizon& rh, const TrackMeta& meta) {
    J::Array recs;
    for (const ReplanRecord& r : rh.records()) {
        J::Object o;
        o["index"] = J::Value(r.index);
        o["trigger"] = J::Value(r.trigger);
        o["progress_arc_m"] = J::Value(r.progressArc);
        o["commit_arc_m"] = J::Value(r.commitArc);
        o["budget_left_m"] = J::Value(r.budgetLeft);
        o["time_limit_s"] = J::Value(r.timeLimit);
        o["planning_s"] = J::Value(r.seconds);
        o["iterations"] = J::Value(r.iterations);
        o["tree_size"] = J::Value(r.treeSize);
        o["pruned"] = J::Value(r.pruned);
        o["path_nodes"] = J::Value(r.nodes);
        o["improved"] = J::Value(r.improved);
        o["segment_length_m"] = J::Value(r.segmentLength);
        o["segment_reward"] = rewardJson(r.segmentReward);
        o["kept_reward"] = rewardJson(r.keptReward);
        o["residual_mass_after_flown"] = J::Value(r.residualMassFlown);
        o["track_length_m"] = J::Value(r.trackLength);
        o["flown_looks"] = J::Value(r.flownLooks);
        recs.push_back(J::Value(std::move(o)));
    }
    J::Object out;
    out["schema"] = J::Value("tigris.replans/1");
    out["mission"] = J::Value(sc.name);
    out["agent"] = J::Value(sc.agents.at(static_cast<std::size_t>(agentIndex)).name);
    out["params"] = J::Value(tigrisBlock(rh, meta));
    out["belief_residual_mass"] = J::Value(rh.belief().residualMass());
    out["revision"] = J::Value(rh.revision());
    out["track_length_m"] = J::Value(rh.totalArc());
    out["replans"] = J::Value(std::move(recs));
    return J::Value(std::move(out)).dump();
}

}  // namespace tigris_search
