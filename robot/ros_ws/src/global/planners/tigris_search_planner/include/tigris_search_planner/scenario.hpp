// =============================================================================
//  tigris_search_planner/scenario.hpp
//
//  ROS-free reader of the AirStack search scenario (`mtl.scenario/1`, written by
//  scripts/tigris_generate_scenario.py or scripts/mtl_generate_scenario.py).
//  TIGRIS reads the SAME bundle format as the MTL planner so both planners are
//  scored against an identical prior, sensor model and ground truth, but it
//  shares no code with the MTL packages.
//
//  FRAMES
//    mission NED   the scenario file: n North, e East [m], origin = Isaac world origin
//    world ENU     x = e, y = n, z = up. TIGRIS plans in world ENU (z = height above
//                  the ground plane).
//    map           one robot's odometry frame: p_map = p_world - home_ENU
//  Yaw is ENU yaw (counter-clockwise from East) everywhere in this package.
// =============================================================================
#ifndef TIGRIS_SEARCH_PLANNER_SCENARIO_HPP
#define TIGRIS_SEARCH_PLANNER_SCENARIO_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "tigris_search_planner/json_mini.hpp"

namespace tigris_search {

/// Moon et al. (2022) detection sigmoid, exactly as mtl_metrics_logger scores it.
struct DetectionModel {
    double a = 1.10, b = 0.10, c = 61.0, beta = 61.0;
    double pOut = 1.0e-6;
    double threshold = 0.9;
    double dtRef = 0.1;
    /// Per-look detection probability at 3-D range r (p_out beyond beta).
    double prob(double r) const;
};

/// The scenario prior raster in WORLD ENU, rebuilt from `airstack.belief.bumps`
/// term for term like mtl_metrics_logger.detection.prior_from_scenario.
/// Pixel (i, j) is centred at (xs[j], ys[i]); index i * nx + j.
struct PriorRaster {
    std::vector<double> xs, ys;
    double res = 2.0;
    std::vector<double> raw;   ///< bump sum, capped and floored (NOT normalised): a presence belief in [0, cap]
    std::vector<double> norm;  ///< raw / sum(raw): the probability mass function (sums to 1)
    int nx() const { return static_cast<int>(xs.size()); }
    int ny() const { return static_cast<int>(ys.size()); }
    bool empty() const { return norm.empty(); }
};

struct AgentSpec {
    std::string name;
    double homeN = 0.0, homeE = 0.0;    ///< mission NED of the map origin (spawn)
    double startN = 0.0, startE = 0.0;  ///< mission NED of the sortie start
    double altitudeOffset = 0.0;        ///< vertical deconfliction layer [m]
    double homeX() const { return homeE; }  ///< world ENU x of the map origin
    double homeY() const { return homeN; }  ///< world ENU y of the map origin
};

struct Scenario {
    std::string   name = "search";
    std::uint64_t seed = 21;
    // search area (square), world ENU
    double size = 400.0;
    double xMin = -200.0, xMax = 200.0, yMin = -200.0, yMax = 200.0;
    // aircraft
    double altitude = 30.0, speed = 6.0, minTurnRadius = 12.0, dt = 0.1, dubinsStep = 0.5;
    // sensor (camera mount): full cone FOV and forward tilt from nadir
    double fovRad = 1.0471975511965976, tiltRad = 0.5235987755982988;
    DetectionModel det;
    // budget (the tighter of the two binds; either may be +inf)
    double maxFlightTime = 90.0, maxFlightDistance = 1e300;
    std::vector<AgentSpec> agents;
    // host-extracted valid cells (world ENU centres) and their prior mass
    std::vector<double> cellX, cellY, cellMass;
    double cellSize = 20.0;
    PriorRaster prior;
    tigris_json::Value raw;

    int agentIndex(const std::string& agentName) const;  ///< -1 when absent
    /// Path-length budget [m] = min(max_flight_distance_m, max_flight_time_s * speed).
    double budgetDistance() const;
    double centerX() const { return 0.5 * (xMin + xMax); }
    double centerY() const { return 0.5 * (yMin + yMax); }
};

/// @throws std::runtime_error with a named cause.
Scenario parseScenario(const std::string& jsonText);
Scenario loadScenario(const std::string& path);
PriorRaster priorFromScenario(const tigris_json::Value& sc);
std::string readTextFile(const std::string& path);

}  // namespace tigris_search

#endif  // TIGRIS_SEARCH_PLANNER_SCENARIO_HPP
