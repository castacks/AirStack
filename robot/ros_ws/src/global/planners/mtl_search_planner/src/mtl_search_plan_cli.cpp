// =============================================================================
//  mtl_search_plan — ROS-free CLI over the same adapter the node uses.
//
//      mtl_search_plan --scenario scenario.json [--out-dir DIR] [--agent robot_2]
//
//  Plans the team, prints a per-agent summary and (with --out-dir) writes
//  plan.json (team, mtl.plan/1, mission NED) and <agent>_track.json (map frame
//  + mission NED). Useful for tuning mission.yaml without Isaac, and it is what
//  scripts/mtl_offline_mission.py drives.
// =============================================================================
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "mtl_search_planner/search_problem.hpp"

namespace {
void usage() {
    std::cout << "mtl_search_plan --scenario FILE [--out-dir DIR] [--agent NAME]\n"
                 "  Plans the MTL team from an AirStack scenario (mtl.scenario/1).\n";
}
void writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << text << "\n";
}
}  // namespace

int main(int argc, char** argv) {
    std::string scenario, outDir, agent;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scenario" && i + 1 < argc) scenario = argv[++i];
        else if (a == "--out-dir" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--agent" && i + 1 < argc) agent = argv[++i];
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else { std::cerr << "unknown argument: " << a << "\n"; usage(); return 2; }
    }
    if (scenario.empty()) { usage(); return 2; }
    try {
        const mtl_search::SearchProblem problem = mtl_search::loadScenario(scenario);
        const auto t0 = std::chrono::steady_clock::now();
        const mtl::PlanningResult r = mtl_search::solve(problem);
        const double ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - t0).count();
        std::cout << std::fixed << std::setprecision(1)
                  << "mtl_search_plan: " << problem.name << ", " << problem.agents.size()
                  << " agents, " << problem.cells.size() << " cells, planned in " << ms
                  << " ms; team info " << 100.0 * r.team.infoFraction << " % ("
                  << std::setprecision(4) << r.team.info << " of " << r.team.infoTotal
                  << " belief mass)\n" << std::setprecision(1);
        for (std::size_t i = 0; i < problem.agents.size(); ++i) {
            if (!agent.empty() && problem.agents[i].name != agent) continue;
            const mtl_search::AgentTrack tr = mtl_search::buildAgentTrack(r, problem, static_cast<int>(i));
            std::cout << "  " << tr.name << ": " << tr.samples.size() << " samples, arc "
                      << tr.totalArc() << " m of budget " << tr.budget << " m, "
                      << tr.servicedCells.size() << " cells, feasible=" << (tr.feasible ? "yes" : "NO")
                      << ", home ENU (" << tr.homeEnu.x() << ", " << tr.homeEnu.y() << ")\n";
            if (!outDir.empty()) {
                writeText(outDir + "/" + tr.name + "_track.json", mtl_search::agentTrackJson(tr, problem));
            }
        }
        if (!outDir.empty()) writeText(outDir + "/plan.json", mtl_search::teamPlanJson(r, problem));
    } catch (const std::exception& e) {
        std::cerr << "mtl_search_plan error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
