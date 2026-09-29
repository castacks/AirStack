// =============================================================================
//  tigris_search_plan — ROS-free CLI over the same core the node runs.
//
//    tigris_search_plan --scenario scenario.json --out-dir DIR [--agent robot_1]
//                       [--set key=value ...] [--one-shot]
//
//  Runs the whole receding-horizon sortie KINEMATICALLY: the drone is assumed to
//  fly the committed track perfectly at the planned speed, so the "flown" looks
//  folded in at each replan are the planned ones. Writes the files the node
//  writes into runs/<run_id>/<agent>/: track.json, plan.json, tigris_replans.json.
//  scripts/tigris_offline_mission.py drives it and then flies the final track
//  through the real follower + logger code.
//
//  --set keys (same names as the ROS parameters): reward_mode, sampler,
//  planning_time_s, initial_planning_time_s, replan_period_s, commit_margin_s,
//  extend_dist_m, extend_radius_m, prune_radius_m, reward_step_m, grid_res_m,
//  view_point_goal, bounds_margin_m, use_entropy, rs, rf, initial_confidence,
//  budget_m, camera_fov_deg, camera_tilt_deg, max_iterations, seed, lookahead_m
// =============================================================================
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "tigris_search_planner/receding.hpp"

using namespace tigris_search;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

void usage() {
    std::cout << "tigris_search_plan --scenario FILE --out-dir DIR [--agent NAME] [--set key=value ...] "
                 "[--one-shot]\n";
}

void writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << text << "\n";
}

bool toBool(const std::string& v) { return v == "1" || v == "true" || v == "True" || v == "yes"; }

void applySet(const std::string& kv, TigrisParams& tp, HorizonParams& hp, Camera& cam, bool& camSet) {
    const auto eq = kv.find('=');
    if (eq == std::string::npos) throw std::runtime_error("--set needs key=value (got '" + kv + "')");
    const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
    auto d = [&]() { return std::stod(v); };
    if (k == "reward_mode") tp.reward.mode = rewardModeFromString(v);
    else if (k == "sampler") tp.informedSampler = (v != "uniform" && v != "random");
    else if (k == "planning_time_s") hp.replanPlanningTime = d();
    else if (k == "initial_planning_time_s") hp.initialPlanningTime = d();
    else if (k == "replan_period_s") hp.replanPeriod = d();
    else if (k == "commit_margin_s") hp.commitMarginTime = d();
    else if (k == "lookahead_m") hp.lookahead = d();
    else if (k == "extend_dist_m") tp.extendDist = d();
    else if (k == "extend_radius_m") tp.extendRadius = d();
    else if (k == "prune_radius_m") tp.pruneRadius = d();
    else if (k == "reward_step_m") tp.rewardStep = d();
    else if (k == "grid_res_m") hp.gridRes = d();
    else if (k == "view_point_goal") tp.viewPointGoal = d();
    else if (k == "bounds_margin_m") tp.boundsMargin = d();
    else if (k == "use_entropy") tp.reward.useEntropy = toBool(v);
    else if (k == "rs") tp.reward.rs = d();
    else if (k == "rf") tp.reward.rf = d();
    else if (k == "initial_confidence") tp.reward.initialConfidence = d();
    else if (k == "budget_m") hp.budgetOverride = d();
    else if (k == "camera_fov_deg") { cam.fov = d() * kDeg; camSet = true; }
    else if (k == "camera_tilt_deg") { cam.tilt = d() * kDeg; camSet = true; }
    else if (k == "max_iterations") tp.maxIterations = static_cast<int>(d());
    else if (k == "seed") tp.seed = static_cast<std::uint64_t>(d());
    else throw std::runtime_error("unknown --set key '" + k + "'");
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenarioPath, outDir, agent = "robot_1";
    std::vector<std::string> sets;
    bool oneShot = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scenario" && i + 1 < argc) scenarioPath = argv[++i];
        else if (a == "--out-dir" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--agent" && i + 1 < argc) agent = argv[++i];
        else if (a == "--set" && i + 1 < argc) sets.emplace_back(argv[++i]);
        else if (a == "--one-shot") oneShot = true;
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else { std::cerr << "unknown argument: " << a << "\n"; usage(); return 2; }
    }
    if (scenarioPath.empty()) { usage(); return 2; }
    try {
        const Scenario sc = loadScenario(scenarioPath);
        const int idx = sc.agentIndex(agent);
        if (idx < 0) throw std::runtime_error("agent '" + agent + "' is not in the scenario team");
        TigrisParams tp;
        tp.seed = sc.seed;
        HorizonParams hp;
        hp.lookahead = 1.2 * sc.minTurnRadius;
        Camera cam = cameraFromScenario(sc);  // fov, tilt and the gimbal sweep (mission.yaml)
        bool camSet = false;
        for (const auto& s : sets) applySet(s, tp, hp, cam, camSet);
        if (oneShot) hp.receding = false;
        if (camSet && std::fabs(cam.fov - sc.fovRad) > 1e-9) {
            std::cerr << "warning: camera_fov_deg differs from the scenario's sensor.fov_deg - the logger scores "
                         "the scenario FOV; change it in mission.yaml instead\n";
        }

        if (cam.sweep) {
            const SweepKinematics k = sweepKinematics(cam);
            std::cout << std::fixed << std::setprecision(1) << "tigris_search_plan: gimbal actuation ON - "
                      << "cross-track sweep +-" << cam.sweepAmplitude / kDeg << " deg at "
                      << cam.sweepRate / kDeg << " deg/s (period " << 4.0 * cam.sweepAmplitude / cam.sweepRate
                      << " s), peak gimbal axis rate " << k.peakAxisRate / kDeg << " deg/s\n";
            if (k.peakAxisRate > sc.gimbalSlewRate + 1e-9) {
                std::cerr << "warning: the sweep needs " << k.peakAxisRate / kDeg << " deg/s on an earth-frame "
                          << "gimbal axis, above the gimbal slew rate " << sc.gimbalSlewRate / kDeg
                          << " deg/s: the flown sweep will lag the plan\n";
            }
            if (k.maxAbsRoll > sc.gimbalRollLimit + 1e-9) {
                std::cerr << "warning: the sweep reaches roll " << k.maxAbsRoll / kDeg << " deg, beyond the "
                          << "gimbal roll limit " << sc.gimbalRollLimit / kDeg << " deg\n";
            }
        }
        RecedingHorizon rh(sc, idx, cam, tp, hp);
        const Pose2 start = defaultStartPose(sc, idx, hp);
        const auto t0 = std::chrono::steady_clock::now();
        if (!rh.start(start)) throw std::runtime_error("TIGRIS found no path from the start pose");

        // kinematic sortie: progress = speed * t along the committed track
        const double dt = 0.1;
        double t = 0.0, sinceReplan = 0.0, lastProgress = 0.0;
        while (true) {
            const double progress = std::min(sc.speed * t, rh.totalArc());
            const bool atEnd = progress >= rh.totalArc() - 1e-6;
            if (rh.due(progress, sinceReplan) || (atEnd && !rh.exhausted())) {
                const auto looks = rh.plannedLooks(lastProgress, progress);
                rh.replan(progress, looks, atEnd ? "end" : (sinceReplan >= hp.replanPeriod ? "period" : "horizon"));
                lastProgress = progress;
                sinceReplan = 0.0;
                if (atEnd && progress >= rh.totalArc() - 1e-6 && rh.exhausted()) break;
                continue;
            }
            if (atEnd && (rh.exhausted() || !hp.receding)) break;
            t += dt;
            sinceReplan += dt;
            if (t > 36000.0) break;
        }
        rh.absorb(rh.plannedLooks(lastProgress, rh.totalArc()));  // the last stretch is flown too
        const double wall = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        TrackMeta meta;
        meta.planId = sc.name + "/" + agent + "/offline";
        meta.rewardMode = toString(tp.reward.mode);
        meta.sampler = tp.informedSampler ? "informed" : "uniform";
        meta.budget = rh.budget();
        meta.revision = rh.revision();
        meta.setGimbal(gimbalLimits(cam, true));
        std::cout << std::fixed << std::setprecision(1) << "tigris_search_plan: " << sc.name << " / " << agent
                  << ": " << rh.records().size() << " solves in " << wall << " s wall, track "
                  << rh.totalArc() << " m of budget " << rh.budget() << " m (" << rh.track().size()
                  << " samples, revision " << rh.revision() << "), reward mode " << meta.rewardMode << "\n";
        for (const auto& r : rh.records()) {
            std::cout << std::setprecision(2) << "  solve " << r.index << " [" << r.trigger << "] at s="
                      << r.progressArc << " commit " << r.commitArc << " left " << r.budgetLeft << " m: "
                      << r.iterations << " it, tree " << r.treeSize << ", " << (r.improved ? "+" : "no ")
                      << r.segmentLength << " m, reward orig " << std::setprecision(4) << r.segmentReward.original
                      << " / matched " << r.segmentReward.matched << "\n";
        }
        if (!outDir.empty()) {
            writeText(outDir + "/track.json", agentTrackJson(sc, idx, rh, meta));
            writeText(outDir + "/plan.json", planJson(sc, idx, rh, meta));
            writeText(outDir + "/tigris_replans.json", replansJson(sc, idx, rh, meta));
        }
    } catch (const std::exception& e) {
        std::cerr << "tigris_search_plan error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
