// =============================================================================
//  ipp_baselines_search_plan — ROS-free CLI over the same core the node runs.
//
//    ipp_baselines_search_plan --scenario scenario.json --out-dir DIR [--agent robot_1]
//                              [--set planner=greedy|rect_coverage|mcts_search]
//                              [--set key=value ...] [--one-shot]
//
//  Runs the whole receding-horizon sortie KINEMATICALLY: the drone flies the committed
//  track perfectly at the planned speed, so the "flown" looks folded in at each replan are
//  the planned ones. Writes the files the node writes into runs/<run_id>/<agent>/:
//  track.json, plan.json, ipp_replans.json. scripts/ipp_baselines_offline_mission.py drives it.
//  --set keys: the node's parameter names (params.cpp paramKeys()). Reward default: matched.
// =============================================================================
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "ipp_baselines_search_planner/params.hpp"
#include "ipp_baselines_search_planner/receding.hpp"

using namespace ipp_baselines_search;

namespace {

constexpr double kDeg = 3.14159265358979323846 / 180.0;

void usage() {
    std::cout << "ipp_baselines_search_plan --scenario FILE --out-dir DIR [--agent NAME] [--set key=value ...] "
                 "[--one-shot]\n";
}

void writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << text << "\n";
}

void applySet(const std::string& kv, BaselineParams& bp, HorizonParams& hp, Camera& cam, bool& camSet) {
    const auto eq = kv.find('=');
    if (eq == std::string::npos) throw std::runtime_error("--set needs key=value (got '" + kv + "')");
    const std::string k = kv.substr(0, eq), v = kv.substr(eq + 1);
    if (!applyParam(k, v, bp, hp, cam, camSet)) throw std::runtime_error("unknown --set key '" + k + "'");
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
        BaselineParams bp;
        bp.seed = sc.seed;
        bp.reward.mode = RewardMode::MATCHED;  // default: the residual-belief reward
        HorizonParams hp;
        hp.lookahead = 1.2 * sc.minTurnRadius;
        Camera cam = cameraFromScenario(sc);  // fov, tilt and the gimbal sweep (mission.yaml)
        bool camSet = false;
        for (const auto& s : sets) applySet(s, bp, hp, cam, camSet);
        if (oneShot) hp.receding = false;
        if (camSet && std::fabs(cam.fov - sc.fovRad) > 1e-9) {
            std::cerr << "warning: camera_fov_deg differs from the scenario's sensor.fov_deg - the logger scores "
                         "the scenario FOV; change it in mission.yaml instead\n";
        }

        if (cam.sweep) {
            const SweepKinematics k = sweepKinematics(cam);
            std::cout << std::fixed << std::setprecision(1) << "ipp_baselines_search_plan: gimbal actuation ON - "
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
        RecedingHorizon rh(sc, idx, cam, bp, hp);
        const Pose2 start = defaultStartPose(sc, idx, hp);
        const auto t0 = std::chrono::steady_clock::now();
        if (!rh.start(start)) throw std::runtime_error(bp.planner + " found no path from the start pose");

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
        meta.rewardMode = toString(bp.reward.mode);
        meta.budget = rh.budget();
        meta.revision = rh.revision();
        meta.setGimbal(gimbalLimits(cam, true));
        std::cout << std::fixed << std::setprecision(1) << "ipp_baselines_search_plan [" << bp.planner << "]: " << sc.name << " / " << agent
                  << ": " << rh.records().size() << " solves in " << wall << " s wall, track "
                  << rh.totalArc() << " m of budget " << rh.budget() << " m (" << rh.track().size()
                  << " samples, revision " << rh.revision() << "), reward mode " << meta.rewardMode << "\n";
        for (const auto& r : rh.records()) {
            std::cout << std::setprecision(2) << "  solve " << r.index << " [" << r.trigger << "] at s="
                      << r.progressArc << " commit " << r.commitArc << " left " << r.budgetLeft << " m: "
                      << r.iterations << " it, tree " << r.treeSize << ", " << (r.improved ? "+" : "no ")
                      << r.segmentLength << " m (cut " << r.cutAtBudget << "), reward orig " << std::setprecision(4)
                      << r.segmentReward.original << " / matched " << r.segmentReward.matched << ", "
                      << std::setprecision(2) << r.seconds << " s\n";
        }
        if (!outDir.empty()) {
            writeText(outDir + "/track.json", agentTrackJson(sc, idx, rh, meta));
            writeText(outDir + "/plan.json", planJson(sc, idx, rh, meta));
            writeText(outDir + "/ipp_replans.json", replansJson(sc, idx, rh, meta));
        }
    } catch (const std::exception& e) {
        std::cerr << "ipp_baselines_search_plan error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
