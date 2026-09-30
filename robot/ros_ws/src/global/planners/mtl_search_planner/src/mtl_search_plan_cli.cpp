// =============================================================================
//  mtl_search_plan — ROS-free CLI over the same adapter the node uses.
//
//      mtl_search_plan --scenario scenario.json [--out-dir DIR] [--agent robot_2]
//
//  Plans the team, prints a per-agent summary and (with --out-dir) writes
//  plan.json (team, mtl.plan/1, mission NED) and <agent>_track.json (map frame
//  + mission NED). Useful for tuning mission.yaml without Isaac, and it is what
//  scripts/mtl_offline_mission.py drives.
//
//  The scenario's planner.type picks the planner that is flown:
//    orienteering  mtl::planner, in the mode info_aware.enabled selects; with
//                  info_aware.report_both (default) the other mode is planned
//                  too and written as plan_alt.json / <agent>_track_alt.json.
//    curve         mtl::curve::Planner; with planner.compare_orienteering
//                  (default) the orienteering planner is planned in BOTH modes
//                  and written as plan_alt_<mode>.json / <agent>_track_alt_<mode>.json.
//  Comparison plans are never flown, only reported.
// =============================================================================
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#include "mtl_search_planner/search_problem.hpp"

namespace {
void usage() {
    std::cout << "mtl_search_plan --scenario FILE [--out-dir DIR] [--agent NAME] [--no-alt]\n"
                 "  Plans the MTL team from an AirStack scenario (mtl.scenario/1).\n"
                 "  --no-alt  skip the comparison plans (info_aware.report_both /\n"
                 "            planner.compare_orienteering)\n";
}

void summarise(const char* tag, const mtl_search::SearchResult& r) {
    std::cout << std::fixed << std::setprecision(1) << "  [" << tag << ": "
              << mtl_search::plannerMode(r) << "] planned in " << 1000.0 * r.planningSeconds
              << " ms; team info " << 100.0 * r.teamInfoFraction() << " %";
    if (const mtl::PlanningResult* o = r.orienteering(); o && o->infoAware.enabled) {
        std::cout << std::setprecision(4) << "; info-aware chose " << o->infoAware.chosen
                  << " (coverage model: detected " << o->infoAware.chosenScore << " vs plain "
                  << o->infoAware.baselineScore << ", " << o->infoAware.candidates.size() << " plans)";
    }
    if (const mtl_search::CurvePlan* c = r.curve()) {
        std::cout << std::setprecision(4) << "; fast residual " << c->result.team.Jfinal << " (static "
                  << c->result.team.Jstatic << ", " << c->result.team.acceptedTrials() << "/"
                  << c->result.team.log.size() << " reallocations kept)";
    }
    std::cout << std::setprecision(1) << "\n";
}
void writeText(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write " + path);
    f << text << "\n";
}
}  // namespace

int main(int argc, char** argv) {
    std::string scenario, outDir, agent;
    bool noAlt = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--scenario" && i + 1 < argc) scenario = argv[++i];
        else if (a == "--out-dir" && i + 1 < argc) outDir = argv[++i];
        else if (a == "--agent" && i + 1 < argc) agent = argv[++i];
        else if (a == "--no-alt") noAlt = true;
        else if (a == "-h" || a == "--help") { usage(); return 0; }
        else { std::cerr << "unknown argument: " << a << "\n"; usage(); return 2; }
    }
    if (scenario.empty()) { usage(); return 2; }
    try {
        const mtl_search::SearchProblem problem = mtl_search::loadScenario(scenario);
        const mtl_search::SearchResult r = mtl_search::solveFlown(problem);
        const double ms = 1000.0 * r.planningSeconds;
        std::cout << std::fixed << std::setprecision(1)
                  << "mtl_search_plan: " << problem.name << " [" << mtl_search::toString(problem.plannerType)
                  << "], " << problem.agents.size() << " agents, " << problem.cells.size()
                  << " cells, planned in " << ms << " ms; team info " << 100.0 * r.teamInfoFraction() << " % ("
                  << std::setprecision(4) << r.teamInfo() << " of " << r.teamInfoTotal()
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
        summarise("flown", r);
        if (!outDir.empty()) writeText(outDir + "/plan.json", mtl_search::teamPlanJson(r, problem));

        if (!noAlt) {
            for (const mtl_search::SearchResult& alt : mtl_search::solveComparisons(problem)) {
                summarise("comparison, not flown", alt);
                if (outDir.empty()) continue;
                const std::string suffix = mtl_search::comparisonSuffix(problem, alt);
                writeText(outDir + "/plan" + suffix + ".json", mtl_search::teamPlanJson(alt, problem));
                for (std::size_t i = 0; i < problem.agents.size(); ++i) {
                    if (!agent.empty() && problem.agents[i].name != agent) continue;
                    const mtl_search::AgentTrack tr = mtl_search::buildAgentTrack(alt, problem, static_cast<int>(i));
                    writeText(outDir + "/" + tr.name + "_track" + suffix + ".json",
                              mtl_search::agentTrackJson(tr, problem));
                }
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "mtl_search_plan error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
