// =============================================================================
//  apps/demo_curve_pipeline.cpp   ->   the `mtlc_demo` tool
//
//  The C++ equivalent of main_curve_planner.m: generate a scenario, plan the
//  team with continuous curves, fly the plan, score it.  Mirrors cpp_planner's
//  mtl_demo: the PLANNER is three calls (construct, plan, read the
//  trajectories); everything else is scenario and scoring, in the two
//  libraries a host would replace with its own.
//
//    ./mtlc_demo [--agents N] [--budget-seconds S] [--tilt DEG] [--cell-size M]
//                [--seed N] [--targets N] [--bspline] [--endpoint MODE]
//                [--stagger M] [--no-realloc] [--no-explore] [--greedy-only]
//                [--quiet] [--csv DIR] [--no-residual] [--residual-block N]
//                [--quad] [--quad-speed V] [--quad-no-yaw] [--quad-max-tilt DEG]
//                [--quad-mount matched|fixed|nadir]
//
//  --quad also re-plans every agent for a quadrotor with a BODY-FIXED camera
//  (quadrotor::generateQuadTrajectories, the C++ twin of main_curve_planner.m
//  step 6b) and scores it with the same evaluator.
//  MODE is open | return_home | fixed_dest.  The mapgen library is a copy of
//  cpp_planner's, so a given --seed/--cell-size is the SAME scenario mtl_demo
//  plans, and the residual belief printed at the end is directly comparable
//  with mtl_demo's.
// =============================================================================
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "mtl_curve/curve_planning/swath_polygon.hpp"
#include "mtl_curve/eval/detection.hpp"
#include "mtl_curve/eval/report.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/planner.hpp"
#include "mtl_curve/quadrotor/quad_trajectory.hpp"

namespace mc = mtl::curve;

namespace {

void writeResidualCsv(const std::string& dir, const mc::BeliefField& belief, const mc::eval::ResidualBelief& rb,
                      int block) {
    const mc::Index b = std::max(1, block);
    const mc::Index ny = belief.rows(), nx = belief.cols();
    const double total = belief.values.sum();
    std::ofstream f(dir + "/residual_belief.csv");
    f << "x,y,prior,residual\n";
    for (mc::Index c0 = 0; c0 < nx; c0 += b) {
        const mc::Index nc = std::min(b, nx - c0);
        for (mc::Index r0 = 0; r0 < ny; r0 += b) {
            const mc::Index nr = std::min(b, ny - r0);
            const double prior = belief.values.block(r0, c0, nr, nc).sum() / total;
            const double resid = rb.residual.block(r0, c0, nr, nc).sum();
            const double x = 0.5 * (belief.x(c0) + belief.x(c0 + nc - 1));
            const double y = 0.5 * (belief.y(r0) + belief.y(r0 + nr - 1));
            f << x << ',' << y << ',' << prior << ',' << resid << '\n';
        }
    }
}

void writeCsv(const std::string& dir, const mc::PlanningResult& r, const std::vector<mc::Target>& targets) {
    std::filesystem::create_directories(dir);
    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const mc::AgentTrajectory& t = r.trajectories[a];
        std::ofstream f(dir + "/agent" + std::to_string(a + 1) + "_trajectory.csv");
        f << "t,x,y,h,sensor_x,sensor_y,roll,pitch,yaw,gimbal,gimbal_cmd\n";
        for (mc::Index k = 0; k < t.drone.rows(); ++k) {
            f << r.timeVec(k) << ',' << t.drone(k, 0) << ',' << t.drone(k, 1) << ',' << t.drone(k, 2) << ','
              << t.sensor(k, 0) << ',' << t.sensor(k, 1) << ',' << t.rpy(k, 0) << ',' << t.rpy(k, 1) << ','
              << t.rpy(k, 2) << ',' << t.gimbalAngle(k) << ',' << t.gimbalCmd(k) << '\n';
        }
        // the swath boundary the plan's figure draws
        const mc::curve_planning::SwathPolygon S =
            mc::curve_planning::computeSwathPolygon(r.plans[a].v, r.plans[a].rep, r.plans[a].swathHalfWidth);
        std::ofstream g(dir + "/agent" + std::to_string(a + 1) + "_swath.csv");
        g << "x,y,left_x,left_y,right_x,right_y\n";
        for (mc::Index k = 0; k < S.center.rows(); ++k)
            g << S.center(k, 0) << ',' << S.center(k, 1) << ',' << S.left(k, 0) << ',' << S.left(k, 1) << ','
              << S.right(k, 0) << ',' << S.right(k, 1) << '\n';
    }
    {
        std::ofstream f(dir + "/clusters.csv");
        f << "x,y,reward,agent,unserviced\n";
        std::vector<char> un(static_cast<std::size_t>(r.clusters.size()), 0);
        for (const mc::Index k : r.team.audit.unserviced) un[static_cast<std::size_t>(k)] = 1;
        for (mc::Index k = 0; k < r.clusters.size(); ++k)
            f << r.clusters.centroids(k, 0) << ',' << r.clusters.centroids(k, 1) << ',' << r.clusters.reward(k)
              << ',' << r.agentOfCluster[static_cast<std::size_t>(k)] << ','
              << static_cast<int>(un[static_cast<std::size_t>(k)]) << '\n';
    }
    {
        std::ofstream f(dir + "/targets.csv");
        f << "x,y,detection_prob,detection_time\n";
        for (const mc::Target& t : targets)
            f << t.pose.x() << ',' << t.pose.y() << ',' << t.detectionProb << ','
              << (t.detected() ? std::to_string(t.detectionTime) : std::string("nan")) << '\n';
    }
    for (std::size_t a = 0; a < r.quad.size(); ++a) {
        const mc::QuadTrajectory& Q = r.quad[a];
        std::ofstream f(dir + "/agent" + std::to_string(a + 1) + "_quad.csv");
        f << "t,x,y,h,bore_x,bore_y,roll,pitch,yaw,vx,vy,ax,ay,p,q,r,thrust,tilt,look,"
             "bore_ref_x,bore_ref_y,path_ref_x,path_ref_y,bore_err,cross_err,along_err\n";
        for (mc::Index k = 0; k < Q.t.size(); ++k) {
            f << Q.t(k) << ',' << Q.traj.drone(k, 0) << ',' << Q.traj.drone(k, 1) << ',' << Q.traj.drone(k, 2) << ','
              << Q.traj.sensor(k, 0) << ',' << Q.traj.sensor(k, 1) << ',' << Q.traj.rpy(k, 0) << ','
              << Q.traj.rpy(k, 1) << ',' << Q.traj.rpy(k, 2) << ',' << Q.vel(k, 0) << ',' << Q.vel(k, 1) << ','
              << Q.acc(k, 0) << ',' << Q.acc(k, 1) << ',' << Q.rates(k, 0) << ',' << Q.rates(k, 1) << ','
              << Q.rates(k, 2) << ',' << Q.thrust(k) << ',' << Q.tilt(k) << ',' << Q.look(k) << ','
              << Q.boreRef(k, 0) << ',' << Q.boreRef(k, 1) << ',' << Q.pathRef(k, 0) << ',' << Q.pathRef(k, 1)
              << ',' << Q.boreErr(k) << ',' << Q.crossErr(k) << ',' << Q.alongErr(k) << '\n';
        }
    }
    std::cout << "CSV written to " << dir << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    mc::PlannerParams params;
    int numTargets = 50;
    std::string csvDir;
    bool residual = true;
    int residualBlock = 10;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](double def) { return (i + 1 < argc) ? std::stod(argv[++i]) : def; };
        if (arg == "--agents") {
            params.numAgents = static_cast<int>(next(3));
        } else if (arg == "--budget-seconds") {
            params.maxFlightTime = next(250.0);
        } else if (arg == "--tilt") {
            params.sensorTiltAngle = mc::deg2rad(next(50.0));
        } else if (arg == "--cell-size") {
            params.cellSize = next(1.0);
        } else if (arg == "--targets") {
            numTargets = static_cast<int>(next(50));
        } else if (arg == "--seed") {
            params.rngSeed = static_cast<std::uint64_t>(next(21));
        } else if (arg == "--bspline") {
            params.curve.representation = mc::CurveRepresentation::BSpline;
        } else if (arg == "--endpoint" && i + 1 < argc) {
            const std::string m = argv[++i];
            if (m == "open") params.curve.endpointModes = {mc::EndpointMode::Open};
            else if (m == "return_home") params.curve.endpointModes = {mc::EndpointMode::ReturnHome};
            else if (m == "fixed_dest") params.curve.endpointModes = {mc::EndpointMode::FixedDest};
            else { std::cerr << "unknown endpoint mode: " << m << "\n"; return 2; }
        } else if (arg == "--stagger") {
            params.team.altitudeStagger = next(25.0);
        } else if (arg == "--no-realloc") {
            params.team.reallocate = false;
        } else if (arg == "--no-explore") {
            params.optimizer.exploreIter = 0;
            params.optimizer.exploreIterWarm = 0;
        } else if (arg == "--realloc" && i + 1 < argc) {
            // comma-separated reallocation seeds, e.g. path,direct | insert
            params.team.reallocInitStrategies.clear();
            std::string list = argv[++i];
            std::size_t p0 = 0;
            while (p0 <= list.size()) {
                const std::size_t p1 = std::min(list.find(',', p0), list.size());
                const std::string t = list.substr(p0, p1 - p0);
                if (t == "path") params.team.reallocInitStrategies.push_back(mc::ReallocStrategy::Path);
                else if (t == "insert") params.team.reallocInitStrategies.push_back(mc::ReallocStrategy::Insert);
                else if (t == "direct") params.team.reallocInitStrategies.push_back(mc::ReallocStrategy::Direct);
                else { std::cerr << "unknown realloc strategy: " << t << "\n"; return 2; }
                p0 = p1 + 1;
            }
        } else if (arg == "--realloc-cold") {
            params.team.reallocPathWarm = false;
        } else if (arg == "--realloc-path-iter") {
            params.team.reallocPathMaxIter = static_cast<int>(next(75));
        } else if (arg == "--greedy-only") {
            params.team.initStrategies = {mc::InitStrategy::Greedy};
        } else if (arg == "--no-residual") {
            residual = false;
        } else if (arg == "--residual-block") {
            residualBlock = static_cast<int>(next(10));
        } else if (arg == "--quad") {
            params.quad.enabled = true;
        } else if (arg == "--quad-speed") {
            params.quad.enabled = true;
            params.quad.speed = next(6.0);
        } else if (arg == "--quad-no-yaw") {
            params.quad.enabled = true;
            params.quad.useYaw = false;
        } else if (arg == "--quad-max-tilt") {
            params.quad.enabled = true;
            params.quad.maxTilt = mc::deg2rad(next(30.0));
        } else if (arg == "--quad-mount" && i + 1 < argc) {
            params.quad.enabled = true;
            const std::string m = argv[++i];
            if (m == "matched") params.quad.cameraMount = mc::CameraMount::Matched;
            else if (m == "fixed") params.quad.cameraMount = mc::CameraMount::Fixed;
            else if (m == "nadir") params.quad.cameraMount = mc::CameraMount::Nadir;
            else { std::cerr << "unknown camera mount: " << m << "\n"; return 2; }
        } else if (arg == "--quiet") {
            params.verbose = false;
        } else if (arg == "--csv") {
            csvDir = (i + 1 < argc) ? argv[++i] : "out";
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: mtlc_demo [--agents N] [--budget-seconds S] [--tilt DEG] [--cell-size M]\n"
                         "                 [--seed N] [--targets N] [--bspline] [--endpoint open|return_home|fixed_dest]\n"
                         "                 [--stagger M] [--no-realloc] [--no-explore] [--greedy-only]\n"
                         "                 [--realloc path,direct|insert,direct|...] [--realloc-cold]\n"
                         "                 [--realloc-path-iter N]\n"
                         "                 [--quiet] [--csv DIR] [--no-residual] [--residual-block N]\n"
                         "                 [--quad] [--quad-speed V] [--quad-no-yaw] [--quad-max-tilt DEG]\n"
                         "                 [--quad-mount matched|fixed|nadir]\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            return 2;
        }
    }

    // Launch points, deliberately near-coincident (as mtl_demo and init_params).
    std::vector<mc::Vec2> starts;
    for (int a = 0; a < params.numAgents; ++a) starts.emplace_back(2000.0 + a, 2000.0 + a);

    try {
        // ---- 1. scenario (mtl_curve_mapgen - a host would supply its own) ----
        const mc::BeliefField belief =
            mc::mapgen::generateBeliefMap(params.mapSize, params.cellSize, mc::BeliefMapParams{}, params.rngSeed);
        std::vector<mc::Target> targets = mc::mapgen::generateTargetPoses(belief, numTargets, params.rngSeed + 1);

        // ---- 2. plan (mtl_curve_planner - this is the package) ---------------
        const auto t0 = std::chrono::steady_clock::now();
        mc::Planner planner(params);
        const mc::PlanningResult result = planner.plan(belief, starts);
        const double planSec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

        // ---- 3. score (mtl_curve_eval - a host would use its own model) -------
        mc::eval::updateTargetDetectionProbs(result.trajectories, result.timeVec, targets, params.fov,
                                             params.sensor, params.detectionThreshold);
        const bool ok = mc::eval::reportCurveSummary(std::cout, result, planner.params());
        mc::eval::reportDetectionSummary(std::cout, targets, params);
        std::cout << "\nTeam solve: static " << result.team.staticTime << " s, reallocation "
                  << result.team.reallocTime << " s (" << result.team.log.size() << " trial(s), "
                  << result.team.acceptedTrials() << " accepted)\n";
        std::cout << "\nPlanning time " << planSec << " s; horizon " << result.timeVec(result.numSteps() - 1)
                  << " s over " << result.numSteps() << " steps.\n";

        // ---- 4. the belief the search leaves behind ---------------------------
        mc::eval::ResidualBelief rb;
        if (residual) {
            rb = mc::eval::computeResidualBelief(belief, result.trajectories, params.fov, params.sensor);
            mc::eval::reportResidualBelief(std::cout, rb);
            if (!result.staticTrajectories.empty()) {
                const mc::eval::ResidualBelief rs =
                    mc::eval::computeResidualBelief(belief, result.staticTrajectories, params.fov, params.sensor);
                std::cout << std::setprecision(6) << "Static partition residual: " << rs.residualMass << " -> after reallocation "
                          << rb.residualMass << " (" << (rb.residualMass - rs.residualMass) << ")\n";
            }
        }
        // ---- 5. the quadrotor (body-fixed camera) version, same evaluator -------
        bool quadOK = true;
        if (!result.quad.empty()) {
            quadOK = mc::eval::reportQuadSummary(std::cout, result, planner.params());
            std::vector<mc::Target> tq = targets;
            const std::vector<mc::AgentTrajectory> qa = mc::quadrotor::quadAgentTrajectories(result.quad);
            const mc::eval::DetectionSummary dq = mc::eval::updateTargetDetectionProbs(
                qa, result.quadTimeVec, tq, params.fov, params.sensor, params.detectionThreshold);
            mc::Index nfw = 0;
            for (const mc::Target& t : targets) nfw += t.detected() ? 1 : 0;
            std::cout << std::setprecision(6) << "Targets detected: fixed wing " << nfw << ", quadrotor "
                      << dq.nDetected << " of " << targets.size() << "\n";
            if (residual) {
                const mc::eval::ResidualBelief rq =
                    mc::eval::computeResidualBelief(belief, qa, params.fov, params.sensor);
                mc::VecX tm;
                const std::vector<mc::AgentTrajectory> qm = mc::quadrotor::quadMatchedLooks(
                    result.quad, result.trajectories, params.avgDroneSpeed, params.dt,
                    planner.params().quad.speed, tm);
                const mc::eval::ResidualBelief rm =
                    mc::eval::computeResidualBelief(belief, qm, params.fov, params.sensor);
                std::cout << "Residual belief: fixed wing " << rb.residualMass << ", quadrotor " << rq.residualMass
                          << " (" << std::setprecision(0) << result.quadTimeVec(result.quadTimeVec.size() - 1)
                          << std::setprecision(6) << " s of looks), quadrotor at the fixed wing's look count "
                          << rm.residualMass
                          << " (geometry only)\n";
            }
        }
        if (!csvDir.empty()) {
            writeCsv(csvDir, result, targets);
            if (residual) writeResidualCsv(csvDir, belief, rb, residualBlock);
        }
        return (ok && quadOK) ? 0 : 3;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
