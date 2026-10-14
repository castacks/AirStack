// test_common.hpp — the unit scenario and scaled parameters shared by the tests.
#ifndef IPP_BASELINES_TEST_COMMON_HPP
#define IPP_BASELINES_TEST_COMMON_HPP

#include <string>

#include "ipp_baselines_search_planner/planner_core.hpp"
#include "ipp_baselines_search_planner/receding.hpp"
#include "ipp_baselines_search_planner/scenario.hpp"

namespace ipp_test {

using namespace ipp_baselines_search;

constexpr double kPi = 3.14159265358979323846;

inline const char* const kScenario = R"({
  "schema": "mtl.scenario/1",
  "mission": {"name": "unit", "seed": 7, "area": {"size_m": 200.0, "center_ned": [0.0, 0.0], "belief_res_m": 2.0}},
  "aircraft": {"altitude_m": 30.0, "speed_mps": 6.0, "min_turn_radius_m": 12.0, "dt": 0.1, "dubins_step_m": 0.5},
  "mapping": {"target_cell_size_m": 20.0},
  "sensor": {"fov_deg": 60.0, "tilt_deg": 30.0,
             "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0, "p_out_of_range": 1e-6,
                           "threshold": 0.9, "dt_ref_s": 0.1}},
  "team": {"max_flight_time_s": 30.0, "max_flight_distance_m": null,
           "agents": [{"name": "robot_1", "start_ned": [-90.0, -90.0], "home_ned": [-90.0, -90.0]}]},
  "cells": {"centers": [[0.0, 40.0], [-40.0, -40.0]], "mass": [0.1, 0.05], "total_map_mass": 1.0},
  "airstack": {"belief": {"bumps": [{"n": 0.0, "e": 40.0, "sigma_n": 20.0, "sigma_e": 20.0, "amplitude": 0.4},
                                    {"n": -40.0, "e": -40.0, "sigma_n": 15.0, "sigma_e": 25.0, "amplitude": 0.4}],
                          "belief_cap": 0.85, "base_uncertainty": 0.0}}
})";

inline Scenario scenario() { return parseScenario(kScenario); }

/// Planner parameters scaled to the 200 m unit area (written out: no call to itself).
inline BaselineParams unitParams(const std::string& planner) {
    BaselineParams bp;
    bp.planner = planner;
    bp.mctsExtendRadius = 20.0;   // released 300 m at 5 km
    bp.mctsExtendDist = 200.0;    // released 3000 m (unused by MCTSSearch.h)
    return bp;
}

inline HorizonParams unitHorizon() {
    HorizonParams hp;
    hp.gridRes = 4.0;
    hp.initialPlanningTime = 60.0;
    hp.replanPlanningTime = 60.0;
    return hp;
}

inline PlannerSetup unitSetup(const Scenario& sc) {
    PlannerSetup su;
    su.det = sc.det;
    su.speed = sc.speed;
    su.turnRadius = sc.minTurnRadius;
    su.dubinsStep = sc.dubinsStep;
    su.altitude = sc.altitude;
    su.xMin = sc.xMin; su.xMax = sc.xMax; su.yMin = sc.yMin; su.yMax = sc.yMax;
    return su;
}

inline Scenario sweepScenarioText(bool enabled) {
    std::string text = kScenario;
    const std::string key = "\"airstack\": {";
    const auto at = text.find(key);
    text.insert(at + key.size(), std::string("\"gimbal_actuation\": {\"enabled\": ") + (enabled ? "true" : "false") +
                                     ", \"sweep_rate_deg_s\": 30.0, \"sweep_amplitude_deg\": 45.0}, ");
    return parseScenario(text);
}

}  // namespace ipp_test

#endif
