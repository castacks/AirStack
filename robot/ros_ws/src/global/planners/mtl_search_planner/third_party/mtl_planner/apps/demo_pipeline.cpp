// =============================================================================
//  apps/demo_pipeline.cpp
//
//  The C++ equivalent of main_new.m: generate a scenario, plan the team, fly the
//  plan, score it.  It exists to show the integration pattern and to give the
//  package something to run end to end.
//
//  Note the shape of it: the PLANNER is three calls (construct, plan, read the
//  trajectories).  Everything else here is scenario and scoring, and lives in
//  the two libraries a host simulation would replace with its own.
//
//    ./mtl_demo [--agents N] [--budget-seconds S] [--multi-axis] [--tilt DEG]
//               [--extend-dist M]
//               [--cell-size M] [--seed N] [--quiet] [--csv DIR]
// =============================================================================
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "mtl/eval/detection.hpp"
#include "mtl/eval/geometry_audit.hpp"
#include "mtl/eval/report.hpp"
#include "mtl/mapgen/scenario.hpp"
#include "mtl/planner.hpp"

namespace {

void writeCsv(const std::string& dir, const mtl::PlanningResult& result,
              const std::vector<mtl::Target>& targets) {
    std::filesystem::create_directories(dir);

    for (std::size_t a = 0; a < result.trajectories.size(); ++a) {
        const mtl::AgentTrajectory& t = result.trajectories[a];
        std::ofstream f(dir + "/agent" + std::to_string(a + 1) + "_trajectory.csv");
        f << "t,x,y,h,sensor_x,sensor_y,roll,pitch,yaw\n";
        for (mtl::Index k = 0; k < t.drone.rows(); ++k) {
            f << result.timeVec(k) << ',' << t.drone(k, 0) << ',' << t.drone(k, 1) << ','
              << t.drone(k, 2) << ',' << t.sensor(k, 0) << ',' << t.sensor(k, 1) << ','
              << t.rpy(k, 0) << ',' << t.rpy(k, 1) << ',' << t.rpy(k, 2) << '\n';
        }
    }

    {
        std::ofstream f(dir + "/cells.csv");
        f << "x,y,mass,serviced\n";
        std::vector<char> serviced(static_cast<std::size_t>(result.cells.size()), 0);
        for (const mtl::Index i : result.team.servicedCellIdx)
            serviced[static_cast<std::size_t>(i)] = 1;
        for (mtl::Index i = 0; i < result.cells.size(); ++i) {
            f << result.cells.centers(i, 0) << ',' << result.cells.centers(i, 1) << ','
              << result.cells.mass(i) << ',' << static_cast<int>(serviced[static_cast<std::size_t>(i)])
              << '\n';
        }
    }

    {
        std::ofstream f(dir + "/targets.csv");
        f << "x,y,detection_prob,detection_time\n";
        for (const mtl::Target& t : targets) {
            f << t.pose.x() << ',' << t.pose.y() << ',' << t.detectionProb << ','
              << (t.detected() ? std::to_string(t.detectionTime) : std::string("nan")) << '\n';
        }
    }

    std::cout << "CSV written to " << dir << "\n";
}

}  // namespace

int main(int argc, char** argv) {
    // ---- configuration: every tunable lives in PlannerParams --------------
    mtl::PlannerParams params;
    int         numTargets = 50;
    std::string csvDir;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto next = [&](double def) { return (i + 1 < argc) ? std::stod(argv[++i]) : def; };
        if (arg == "--agents") {
            params.numAgents = static_cast<int>(next(3));
        } else if (arg == "--budget-seconds") {
            params.maxFlightTime = next(250.0);
        } else if (arg == "--unlimited") {
            params.maxFlightTime = mtl::kInf;
        } else if (arg == "--extend-dist") {
            params.extension.extendDist = next(300.0);
        } else if (arg == "--multi-axis") {
            params.singleAxisGimbal = false;
        } else if (arg == "--tilt") {
            params.sensorTiltAngle = mtl::deg2rad(next(50.0));
        } else if (arg == "--cell-size") {
            params.cellSize = next(1.0);
        } else if (arg == "--targets") {
            numTargets = static_cast<int>(next(50));
        } else if (arg == "--seed") {
            params.rngSeed = static_cast<std::uint64_t>(next(21));
        } else if (arg == "--quiet") {
            params.verbose = false;
        } else if (arg == "--csv") {
            csvDir = (i + 1 < argc) ? argv[++i] : "out";
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "usage: mtl_demo [--agents N] [--budget-seconds S] [--unlimited]\n"
                         "                [--multi-axis] [--tilt DEG] [--cell-size M]\n"
                         "                [--targets N] [--seed N] [--quiet] [--csv DIR]\n";
            return 0;
        } else {
            std::cerr << "unknown argument: " << arg << "\n";
            return 2;
        }
    }

    // Launch points, deliberately near-coincident: all agents launch from the
    // same field.
    std::vector<mtl::Vec2> starts;
    for (int a = 0; a < params.numAgents; ++a)
        starts.emplace_back(2000.0 + a, 2000.0 + a);

    try {
        // ---- 1. scenario (mtl_mapgen - a host would supply its own) -------
        const mtl::BeliefField belief = mtl::mapgen::generateBeliefMap(
            params.mapSize, params.cellSize, mtl::BeliefMapParams{}, params.rngSeed);
        std::vector<mtl::Target> targets =
            mtl::mapgen::generateTargetPoses(belief, numTargets, params.rngSeed + 1);

        // ---- 2. plan (mtl_planner - this is the package) -------------------
        mtl::Planner planner(params);
        const mtl::PlanningResult result = planner.plan(belief, starts);

        // ---- 3. score (mtl_eval - a host would use its own sensor model) ---
        mtl::eval::updateTargetDetectionProbs(result.trajectories, result.timeVec, targets,
                                              params.fov, params.sensor,
                                              params.detectionThreshold);

        mtl::eval::reportDetectionSummary(std::cout, targets, params);
        mtl::eval::reportBudgetSummary(std::cout, result, params);
        mtl::eval::reportGimbalCoverage(std::cout, result);

        if (params.verifyGeometry && params.singleAxisGimbal) {
            for (std::size_t a = 0; a < result.trajectories.size(); ++a) {
                const mtl::AgentTrajectory& t = result.trajectories[a];
                if (!t.scheduled) continue;
                const mtl::eval::GeometryReport rep = mtl::eval::verifySensorGeometry(
                    t.drone, t.sensor, t.rpy, t.diagnostics, params.verifyGeometryVerbose);
                if (!rep.pass) {
                    std::cerr << "[demo] geometry audit FAILED for agent " << (a + 1) << "\n";
                }
            }
        }

        std::cout << "\nSimulation horizon: " << result.timeVec(result.numSteps() - 1) << " s over "
                  << result.numSteps() << " steps.\n";

        if (!csvDir.empty()) writeCsv(csvDir, result, targets);
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
