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
//
//  MODE is open | return_home | fixed_dest.  The mapgen library is a copy of
//  cpp_planner's, so a given --seed/--cell-size is the SAME scenario mtl_demo
//  plans, and the residual belief printed at the end is directly comparable
//  with mtl_demo's.
// =============================================================================
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "mtl_curve/curve_planning/swath_polygon.hpp"
#include "mtl_curve/eval/detection.hpp"
#include "mtl_curve/eval/report.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/planner.hpp"

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
        } else if (arg == "--greedy-only") {
            params.team.initStrategies = {mc::InitStrategy::Greedy};
        } else if (arg == "--no-residual") {
            residual = false;
        } else if (arg == "--residual-block") {
            residualBlock = static_cast<int>(next(10));
        } else if (arg == "--quiet") {
            params.verbose = false;
        } else if (arg == "--csv") {
            csvDir = (i + 1 < argc) ? argv[++i] : "out";
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: mtlc_demo [--agents N] [--budget-seconds S] [--tilt DEG] [--cell-size M]\n"
                         "                 [--seed N] [--targets N] [--bspline] [--endpoint open|return_home|fixed_dest]\n"
                         "                 [--stagger M] [--no-realloc] [--no-explore] [--greedy-only]\n"
                         "                 [--quiet] [--csv DIR] [--no-residual] [--residual-block N]\n";
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
                std::cout << "Static partition residual: " << rs.residualMass << " -> after reallocation "
                          << rb.residualMass << " (" << (rb.residualMass - rs.residualMass) << ")\n";
            }
        }
        if (!csvDir.empty()) {
            writeCsv(csvDir, result, targets);
            if (residual) writeResidualCsv(csvDir, belief, rb, residualBlock);
        }
        return ok ? 0 : 3;
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
}
