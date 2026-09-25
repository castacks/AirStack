#include "mtl/eval/report.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <numeric>

#include "mtl/planning/info_score.hpp"

namespace mtl::eval {
namespace {

std::string budgetString(const std::vector<double>& budgets, double speed) {
    if (budgets.empty()) return "none";
    const double b = budgets.front();
    const bool   uniform =
        std::all_of(budgets.begin(), budgets.end(), [b](double x) { return x == b; });
    std::ostringstream oss;
    if (uniform) {
        oss << std::fixed << std::setprecision(0) << b << " m per agent";
        if (speed > 0.0)
            oss << " (" << std::setprecision(1) << (b / speed / 60.0) << " min at "
                << std::setprecision(0) << speed << " m/s)";
    } else {
        oss << "per-agent [";
        for (std::size_t i = 0; i < budgets.size(); ++i)
            oss << (i ? " " : "") << std::fixed << std::setprecision(0) << budgets[i];
        oss << "] m";
    }
    return oss.str();
}

}  // namespace

void reportBudgetSummary(std::ostream& os, const PlanningResult& r, const PlannerParams& params) {
    const TeamInfo& team = r.team;
    const bool budgeted =
        std::any_of(team.budgets.begin(), team.budgets.end(),
                    [](double b) { return std::isfinite(b); });

    os << "\n--- Budgeted Route Summary ---\n";
    if (budgeted) {
        os << "Budget mode:      " << budgetString(team.budgets, params.avgDroneSpeed) << "\n";
    } else {
        os << "Budget mode:      unlimited (Inf) - full-coverage pipeline\n";
    }

    os << std::left << std::setw(6) << "agent" << std::right << std::setw(8) << "clust"
       << std::setw(10) << "anchors" << std::setw(10) << "cells" << std::setw(9) << "flown"
       << std::setw(10) << "budget" << std::setw(9) << "used" << std::setw(9) << "info %" << "\n";

    for (std::size_t a = 0; a < r.plans.size(); ++a) {
        const AgentPlan& p = r.plans[a];
        const double     b = team.budgets[std::min(a, team.budgets.size() - 1)];
        os << std::left << std::setw(6) << (a + 1) << std::right << std::setw(8)
           << p.selClusters.size() << std::setw(10) << p.cellAnchors.size() << std::setw(10)
           << p.servicedCellIdx.size() << std::setw(9) << std::fixed << std::setprecision(0)
           << p.flownLength;
        if (std::isfinite(b)) {
            os << std::setw(10) << std::setprecision(0) << b << std::setw(8)
               << std::setprecision(1) << (100.0 * p.budgetUsed) << "%";
        } else {
            os << std::setw(10) << "Inf" << std::setw(9) << "-";
        }
        os << std::setw(8) << std::setprecision(1) << (100.0 * p.score.infoFraction) << "%\n";
    }

    const double totalFlown =
        std::accumulate(team.flownLength.begin(), team.flownLength.end(), 0.0);
    os << std::left << std::setw(6) << "team" << std::right << std::setw(8)
       << team.reachedClusters.size() << std::setw(10) << "-" << std::setw(10)
       << team.servicedCellIdx.size() << std::setw(9) << std::setprecision(0) << totalFlown
       << std::setw(10) << "-" << std::setw(9) << "-" << std::setw(8) << std::setprecision(1)
       << (100.0 * team.infoFraction) << "%\n";

    os << std::setprecision(4);
    os << "Information:      " << team.info << " of " << team.infoTotal
       << " retained prior mass (" << std::setprecision(1) << (100.0 * team.infoFraction) << "%)\n";
    os << "Clusters:         " << team.reachedClusters.size() << " reached, "
       << team.unreachedClusters.size() << " unreachable within budget\n";
    os << "Cells:            " << team.servicedCellIdx.size() << " serviced, "
       << team.unservicedCellIdx.size() << " dropped\n";
    if (budgeted) {
        os << std::setprecision(4) << "Efficiency:       "
           << (1000.0 * team.info / std::max(totalFlown, 1e-300)) << " info per km flown\n";
        if (!team.reallocated.empty()) {
            os << "Reallocation:     " << team.reallocated.size()
               << " cluster(s) changed hands over " << team.rounds << " round(s)\n";
        }
    }

    // --- planned vs realized ------------------------------------------------
    if (!r.realizedCellIdx.empty()) {
        planning::InfoScoreOptions so;
        so.budget = std::accumulate(team.budgets.begin(), team.budgets.end(), 0.0);
        const InfoScore rs =
            planning::pathInformationScore(r.cells.mass, r.realizedCellIdx, totalFlown, so);
        os << std::setprecision(4) << "Realized:         " << rs.info << " ("
           << std::setprecision(1) << (100.0 * rs.infoFraction)
           << "%) actually observed by the gimbal - " << std::setprecision(1)
           << (100.0 * rs.info / std::max(team.info, 1e-300)) << "% of what was planned\n";
    }
    os << "------------------------------\n";
}

void reportDetectionSummary(std::ostream& os, const std::vector<Target>& targets,
                            const PlannerParams& params) {
    Index detected = 0;
    for (const Target& t : targets)
        if (t.detected()) ++detected;

    os << "\n--- Target Detection Summary ---\n";
    if (params.singleAxisGimbal) {
        os << "Sensor mount:     single-axis gimbal, tilt " << std::fixed << std::setprecision(1)
           << rad2deg(params.sensorTiltAngle) << " deg (stand-off " << std::setprecision(0)
           << params.sensorStandOff() << " m)\n";
    } else {
        os << "Sensor mount:     multi-axis gimbal (mount tilt not used)\n";
    }
    os << "Targets Detected: " << detected << "\n";
    os << "Targets Missed:   " << (static_cast<Index>(targets.size()) - detected) << "\n";
    os << "--------------------------------\n";
}

void reportGimbalCoverage(std::ostream& os, const PlanningResult& r) {
    bool any = false;
    for (const AgentTrajectory& t : r.trajectories) any = any || t.scheduled;
    if (!any) return;

    os << "\n--- Gimbal Coverage ---\n";
    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const AgentTrajectory& t = r.trajectories[a];
        if (!t.scheduled) continue;
        const GimbalDiagnostics& d = t.diagnostics;
        os << "agent " << (a + 1) << ": " << d.nTargetsHit << "/" << d.nTargets << " centres ("
           << std::fixed << std::setprecision(1) << (100.0 * d.targetCoverage) << "%)"
           << ", max err " << std::setprecision(2) << d.targetErrMax << " m"
           << ", |pitch| " << std::setprecision(2) << d.pitchMaxDeg << " deg"
           << ", gimbal " << std::setprecision(1) << d.gimbalMaxDeg << " deg"
           << ", slant max " << std::setprecision(0) << d.slantRangeMax << " m\n";
        if (!d.targetMissedIdx.empty()) {
            os << "         missed: " << d.nMissNeverAbeam << " never on the line (ground track), "
               << d.nMissOutOfReach << " never close enough (altitude/tilt), "
               << d.nMissDoubleBooked << " gimbal double-booked (scheduling)\n";
        }
    }
    os << "-----------------------\n";
}

}  // namespace mtl::eval
