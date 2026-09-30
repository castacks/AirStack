// =============================================================================
//  search_problem.cpp — scenario JSON -> mtl::Planner inputs, and the planner's
//  team result -> one agent's map-frame track.
//
//  paramsFromScenario()/cellsFromScenario() follow cpp_planner/apps/
//  mtl_plan_json.cpp field for field, so a scenario plans identically through
//  the stock `mtl_plan` tool and through this ROS node.
//
//  curveParamsFromScenario()/curveCellsFromScenario() and the curve half of
//  teamPlanJson() follow cpp_curve_planner/apps/mtl_curve_plan_json.cpp
//  (`mtlc_plan`) the same way, so a planner.type = curve scenario plans
//  identically through `mtlc_plan` and through this node.
// =============================================================================
#include "mtl_search_planner/search_problem.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>
#include <stdexcept>

namespace J = jsonmini;

namespace mtl_search {
namespace {

constexpr const char* kScenarioSchema = "mtl.scenario/1";
constexpr const char* kPlanSchema     = "mtl.plan/1";

mtl::PlannerParams paramsFromScenario(const J::Value& sc, int numAgents) {
    mtl::PlannerParams p;

    const J::Value& area = sc["mission"]["area"];
    p.mapSize  = area["size_m"].num(p.mapSize);
    p.cellSize = area["belief_res_m"].num(p.cellSize);

    const J::Value& air = sc["aircraft"];
    p.droneAltitude   = air["altitude_m"].num(p.droneAltitude);
    p.avgDroneSpeed   = air["speed_mps"].num(p.avgDroneSpeed);
    p.minTurnRadius   = air["min_turn_radius_m"].num(p.minTurnRadius);
    p.dt              = air["dt"].num(p.dt);
    p.dubins.stepSize = air["dubins_step_m"].num(p.dubins.stepSize);

    const J::Value& map = sc["mapping"];
    p.targetCellSize           = map["target_cell_size_m"].num(p.targetCellSize);
    // Per-cell probability threshold (the prior is a PMF that sums to 1). As in
    // mtl_plan_json.cpp it is informational here: the host extracted the cells
    // (scenario.py extract_valid_cells) and planFromCells never re-extracts. It is
    // still validated to [0, 1). The retired mean_information_thresh is ignored.
    p.minimumBeliefMass        = map["minimum_belief_mass"].num(p.minimumBeliefMass);
    p.maxClusterRadius         = map["max_cluster_radius_m"].num(p.maxClusterRadius);
    p.cluster.kmeansReplicates = static_cast<int>(map["kmeans_replicates"].num(p.cluster.kmeansReplicates));
    p.cluster.kmeansMaxIter    = static_cast<int>(map["kmeans_max_iter"].num(p.cluster.kmeansMaxIter));

    const J::Value& sensor = sc["sensor"];
    p.fov              = mtl::deg2rad(sensor["fov_deg"].num(60.0));
    p.singleAxisGimbal = sensor["single_axis_gimbal"].flag(p.singleAxisGimbal);
    p.sensorTiltAngle  = mtl::deg2rad(sensor["tilt_deg"].num(mtl::rad2deg(p.sensorTiltAngle)));
    const double maxSlant = sensor["max_slant_range_m"].num(p.gimbal.maxSlantRange);
    p.gimbal.maxSlantRange     = maxSlant;
    p.extension.maxSensorReach = sensor["max_sensor_reach_m"].num(maxSlant);
    p.gimbal.maxSensorReach    = sensor["max_sensor_reach_m"].num(maxSlant);

    const J::Value& det = sensor["detection"];
    p.sensor.a    = det["a"].num(p.sensor.a);
    p.sensor.b    = det["b"].num(p.sensor.b);
    p.sensor.c    = det["c"].num(p.sensor.c);
    p.sensor.beta = det["beta"].num(p.sensor.beta);
    p.sensor.pOutOfRangeMulti = det["p_out_of_range"].num(p.sensor.pOutOfRangeMulti);
    p.detectionThreshold      = det["threshold"].num(p.detectionThreshold);

    const J::Value& team = sc["team"];
    p.numAgents         = numAgents;
    p.maxFlightTime     = team["max_flight_time_s"].numOrInf();
    p.maxFlightDistance = team["max_flight_distance_m"].numOrInf();

    // Altitude box of the scheduler: +/-50 % of cruise (the reference 150/450 m
    // box around 300 m is meaningless at a scaled altitude).
    p.gimbal.hMin = sc["gimbal"]["h_min_m"].num(0.5 * p.droneAltitude);
    p.gimbal.hMax = sc["gimbal"]["h_max_m"].num(1.5 * p.droneAltitude);
    p.gimbal.minTurnRadius    = p.minTurnRadius;
    p.gimbal.enableRepairLoop = sc["gimbal"]["enable_repair_loop"].flag(p.gimbal.enableRepairLoop);
    p.gimbal.targetTol = sc["gimbal"]["target_tol_m"].num(
        std::max(1.0, 0.025 * map["target_cell_size_m"].num(p.targetCellSize)));

    p.verbose               = sc["verbose"].flag(false);
    p.verifyGeometry        = sc["verify_geometry"].flag(true);
    p.verifyGeometryVerbose = sc["verify_geometry_verbose"].flag(false);
    p.budget.verbose        = p.verbose;
    p.rngSeed = static_cast<std::uint64_t>(sc["mission"]["seed"].num(21.0));

    const J::Value& ov = sc["solver"];
    p.budget.orienteering.nStarts = static_cast<int>(ov["n_starts"].num(p.budget.orienteering.nStarts));
    p.budget.maxOuterIter  = static_cast<int>(ov["max_outer_iter"].num(p.budget.maxOuterIter));
    p.budget.reserveFrac0  = ov["reserve_frac0"].num(p.budget.reserveFrac0);
    p.budget.cellRefine    = ov["cell_refine"].flag(p.budget.cellRefine);
    p.budget.reallocate    = ov["reallocate"].flag(p.budget.reallocate);
    p.budget.maxCellCandidates =
        static_cast<int>(ov["max_cell_candidates"].num(p.budget.maxCellCandidates));
    // Single-axis lateral-coverage run-out (trajectory::extendTrajForLateralCoverage).
    // AirStack addition, not read by the stock mtl_plan: the vendored default
    // (300 m) is sized for the 5 km reference and must be scaled with the
    // mission, or the run-out flies far outside a small area and the budget
    // reserve it forces shrinks the route (see mission.yaml solver.extend_dist_m).
    p.extension.extendDist   = ov["extend_dist_m"].num(p.extension.extendDist);
    p.extension.maxExtraDist = ov["max_extra_dist_m"].num(p.extension.maxExtraDist);

    // Information-aware abstraction search (PlannerParams::infoAware), off unless
    // the scenario carries info_aware.enabled = true.
    {
        const J::Value& ia = sc["info_aware"];
        p.infoAware.enabled     = ia["enabled"].flag(p.infoAware.enabled);
        if (ia["level_sets"].isArray()) {
            p.infoAware.levelSets.clear();
            for (const J::Value& ls : ia["level_sets"].array()) p.infoAware.levelSets.push_back(ls.numbers());
        }
        if (ia["reach_scales"].isArray()) p.infoAware.reachScales = ia["reach_scales"].numbers();
        p.infoAware.persistence = ia["persistence"].num(p.infoAware.persistence);
        p.infoAware.slantMargin = ia["slant_margin"].num(p.infoAware.slantMargin);
        p.infoAware.capGimbalToDetection = ia["cap_gimbal_to_detection"].flag(p.infoAware.capGimbalToDetection);
        p.infoAware.subsample   = static_cast<int>(ia["subsample"].num(p.infoAware.subsample));
        p.infoAware.lookStride  = static_cast<int>(ia["look_stride"].num(p.infoAware.lookStride));
        p.infoAware.maxMoves    = static_cast<int>(ia["max_moves"].num(p.infoAware.maxMoves));
        p.infoAware.restarts    = static_cast<int>(ia["restarts"].num(p.infoAware.restarts));
        p.infoAware.splitMerge  = ia["split_merge"].flag(p.infoAware.splitMerge);
        p.infoAware.peelKeep    = ia["peel_keep"].num(p.infoAware.peelKeep);
        p.infoAware.threads     = static_cast<int>(ia["threads"].num(p.infoAware.threads));
    }
    return p;
}

/// Vertical deconfliction offset of agent a (team.agents[a].altitude_offset_m,
/// written by the scenario generator from team.altitude_separation_m); 0 if absent.
double agentAltitudeOffset(const J::Value& sc, std::size_t a) {
    const J::Value& agents = sc["team"]["agents"];
    if (!agents.isArray() || a >= agents.array().size()) return 0.0;
    const double dz = agents.array()[a]["altitude_offset_m"].num(0.0);
    return std::isfinite(dz) ? dz : 0.0;
}

/// Host-extracted cells. `cells.mass` is each cell's belief mass, i.e. the
/// probability that the target is in it (the host prior sums to 1), and
/// `cells.total_map_mass` is 1 for such a prior (default: the sum of the masses).
/// The route optimiser only compares rewards as ratios, so any positive scale
/// plans identically; probabilities keep the printed info in the same units as
/// the residual-belief metric.
mtl::CellSet cellsFromScenario(const J::Value& sc, const Frame& frame, double cellSize,
                               double minBeliefMass) {
    const J::Value& cells = sc["cells"];
    if (!cells["centers"].isArray()) {
        throw std::runtime_error(
            "scenario has no 'cells.centers' - the host extracts the valid cells "
            "(scripts/mtl_generate_scenario.py) and passes them");
    }
    const J::Array& centers = cells["centers"].array();
    const J::Array* mass = cells["mass"].isArray() ? &cells["mass"].array() : nullptr;
    if (mass && mass->size() != centers.size()) {
        throw std::runtime_error("scenario cells: 'mass' and 'centers' have different lengths");
    }
    mtl::CellSet out;
    const auto m = static_cast<mtl::Index>(centers.size());
    out.centers.resize(m, 2);
    out.mass.resize(m);
    for (mtl::Index i = 0; i < m; ++i) {
        const J::Array& row = centers[static_cast<std::size_t>(i)].array();
        if (row.size() != 2) throw std::runtime_error("scenario cells: each centre needs [n, e]");
        const mtl::Vec2 xy = frame.toMtl(row[0].number(), row[1].number());
        out.centers(i, 0) = xy.x();
        out.centers(i, 1) = xy.y();
        out.mass(i) = mass ? (*mass)[static_cast<std::size_t>(i)].number() : 1.0;
    }
    out.cellSize     = cellSize;
    out.area         = mtl::VecX::Constant(m, cellSize * cellSize);
    out.nPix         = mtl::VecX::Constant(m, 1.0);
    out.meanBelief   = out.mass / std::max(cellSize * cellSize, 1e-9);
    out.peakBelief   = out.meanBelief;
    out.retainedMass = out.mass.sum();
    out.totalMapMass = cells["total_map_mass"].num(out.retainedMass);
    out.massNorm     = out.retainedMass > 0 ? (out.mass / out.retainedMass).eval() : out.mass;
    out.minBeliefMass = minBeliefMass;
    const double res = sc["mission"]["area"]["belief_res_m"].num(1.0);
    out.gridRes      = mtl::Vec2(res, res);
    return out;
}


// =============================================================================
//  The curve planner's inputs - mtl_curve_plan_json.cpp field for field.
// =============================================================================
namespace mc = mtl::curve;

/// Refuse keys a block does not know: a misspelt curve / planner key would
/// otherwise silently fall back to a default (mtlc_plan ignores them).
void refuseUnknownKeys(const J::Value& block, const std::set<std::string>& known, const std::string& where) {
    if (block.isNull()) return;
    if (!block.isObject()) throw std::runtime_error("scenario '" + where + "' must be an object");
    for (const auto& kv : block.object()) {
        if (known.count(kv.first)) continue;
        std::string list;
        for (const std::string& k : known) list += (list.empty() ? "" : ", ") + k;
        throw std::runtime_error("scenario '" + where + "' has an unknown key '" + kv.first + "' (known: " + list + ")");
    }
}

const std::set<std::string> kPlannerKeys = {"type", "compare_orienteering"};
const std::set<std::string> kCurveKeys = {
    "representation", "endpoint_mode", "destinations_ned", "altitude_stagger_m", "kernel",
    "sweep_freq_hz", "sweep_range_margin", "knot_spacing_m", "num_control_points", "sample_spacing_m",
    "initial_heading_deg", "fast_grid_step_m", "explore_grid_step_m", "max_iter", "explore_iter",
    "explore_iter_warm", "coordination_sweeps", "init_strategies", "reallocate", "realloc_rounds",
    "max_realloc_trials"};
const std::set<std::string> kCurveKernelKeys = {"table_step_m", "grid_step_m", "track_len_m", "edge_width_m",
                                                "edge_inset_m", "tail_sigma_m", "tail_weight"};

mc::EndpointMode curveModeFrom(const std::string& s) {
    if (s == "open") return mc::EndpointMode::Open;
    if (s == "return_home") return mc::EndpointMode::ReturnHome;
    if (s == "fixed_dest") return mc::EndpointMode::FixedDest;
    throw std::runtime_error("curve.endpoint_mode '" + s + "' is not open | return_home | fixed_dest");
}

/// = mtlc_plan paramsFromScenario(): the shared keys, as mtlc_plan reads them.
mc::PlannerParams curveBaseParamsFromScenario(const J::Value& sc, int numAgents) {
    mc::PlannerParams p;

    const J::Value& area = sc["mission"]["area"];
    p.mapSize  = area["size_m"].num(p.mapSize);
    p.cellSize = area["belief_res_m"].num(p.cellSize);

    const J::Value& air = sc["aircraft"];
    p.droneAltitude = air["altitude_m"].num(p.droneAltitude);
    p.avgDroneSpeed = air["speed_mps"].num(p.avgDroneSpeed);
    p.minTurnRadius = air["min_turn_radius_m"].num(p.minTurnRadius);
    p.dt            = air["dt"].num(p.dt);

    const J::Value& map = sc["mapping"];
    p.targetCellSize    = map["target_cell_size_m"].num(p.targetCellSize);
    p.minimumBeliefMass = map["minimum_belief_mass"].num(p.minimumBeliefMass);
    p.maxClusterRadius  = map["max_cluster_radius_m"].num(p.maxClusterRadius);
    p.cluster.kmeansReplicates = static_cast<int>(map["kmeans_replicates"].num(p.cluster.kmeansReplicates));
    p.cluster.kmeansMaxIter    = static_cast<int>(map["kmeans_max_iter"].num(p.cluster.kmeansMaxIter));

    const J::Value& sensor = sc["sensor"];
    p.fov             = mc::deg2rad(sensor["fov_deg"].num(60.0));
    p.sensorTiltAngle = mc::deg2rad(sensor["tilt_deg"].num(mc::rad2deg(p.sensorTiltAngle)));
    p.gimbal.maxSensorReach = sensor["max_sensor_reach_m"].num(p.gimbal.maxSensorReach);

    const J::Value& det = sensor["detection"];
    p.sensor.a    = det["a"].num(p.sensor.a);
    p.sensor.b    = det["b"].num(p.sensor.b);
    p.sensor.c    = det["c"].num(p.sensor.c);
    p.sensor.beta = det["beta"].num(p.sensor.beta);
    p.sensor.pOutOfRangeMulti = det["p_out_of_range"].num(p.sensor.pOutOfRangeMulti);
    p.detectionThreshold      = det["threshold"].num(p.detectionThreshold);

    const J::Value& team = sc["team"];
    p.numAgents         = numAgents;
    p.maxFlightTime     = team["max_flight_time_s"].numOrInf();
    p.maxFlightDistance = team["max_flight_distance_m"].numOrInf();

    const J::Value& gim = sc["gimbal"];
    p.gimbal.gimbalMax  = mc::deg2rad(gim["gimbal_max_deg"].num(mc::rad2deg(p.gimbal.gimbalMax)));
    p.gimbal.gimbalRate = mc::deg2rad(gim["gimbal_rate_deg_s"].num(mc::rad2deg(p.gimbal.gimbalRate)));

    p.verbose = sc["verbose"].flag(false);
    p.rngSeed = static_cast<std::uint64_t>(sc["mission"]["seed"].num(21.0));
    return p;
}

/// = mtlc_plan curveFromScenario(): the optional "curve" block, with the same
/// automatic scaling of every ABSENT key (a key the scenario sets is used as is).
void curveFromScenario(const J::Value& sc, const Frame& frame, mc::PlannerParams& p) {
    const J::Value& cv = sc["curve"];
    refuseUnknownKeys(cv, kCurveKeys, "curve");
    refuseUnknownKeys(cv["kernel"], kCurveKernelKeys, "curve.kernel");
    const std::string rep = cv["representation"].text("curvature");
    if (rep == "curvature") p.curve.representation = mc::CurveRepresentation::Curvature;
    else if (rep == "bspline") p.curve.representation = mc::CurveRepresentation::BSpline;
    else throw std::runtime_error("curve.representation '" + rep + "' is not curvature | bspline");

    const J::Value& em = cv["endpoint_mode"];
    if (em.isArray()) {
        p.curve.endpointModes.clear();
        for (const J::Value& m : em.array()) p.curve.endpointModes.push_back(curveModeFrom(m.str()));
    } else {
        p.curve.endpointModes = {curveModeFrom(em.text("open"))};
    }
    if (cv["destinations_ned"].isArray()) {
        p.curve.destinations.clear();
        for (const J::Value& d : cv["destinations_ned"].array()) {
            const std::vector<double> v = d.numbers();
            if (v.size() != 2) throw std::runtime_error("curve.destinations_ned entries need [n, e]");
            const mtl::Vec2 xy = frame.toMtl(v[0], v[1]);
            p.curve.destinations.push_back(mc::Vec2(xy.x(), xy.y()));
        }
    }
    // Kernel calibration, edge smoothing and altitude stagger: metres tuned for
    // the reference sensor (beta = 610 m), scaled by beta / 610 unless set.
    const double ks = p.sensor.beta / 610.0;
    const J::Value& kn = cv["kernel"];
    p.kernel.tableStep  = kn["table_step_m"].num(p.kernel.tableStep * ks);
    p.kernel.gridStep   = kn["grid_step_m"].num(p.kernel.gridStep * ks);
    p.kernel.trackLen   = kn["track_len_m"].num(p.kernel.trackLen * ks);
    p.kernel.edgeWidth  = kn["edge_width_m"].num(p.kernel.edgeWidth * ks);
    p.kernel.edgeInset  = kn["edge_inset_m"].num(p.kernel.edgeInset * ks);
    p.kernel.tailSigma  = kn["tail_sigma_m"].num(p.kernel.tailSigma * ks);
    p.kernel.tailWeight = kn["tail_weight"].num(p.kernel.tailWeight);
    p.team.altitudeStagger = cv["altitude_stagger_m"].num(p.team.altitudeStagger * ks);
    p.sweep.freq           = cv["sweep_freq_hz"].num(p.sweep.freq);
    p.sweep.rangeMargin    = cv["sweep_range_margin"].num(p.sweep.rangeMargin);
    p.curve.numControlPoints = static_cast<int>(cv["num_control_points"].num(p.curve.numControlPoints));
    if (cv["initial_heading_deg"].isNumber())
        p.curve.initialHeading = Frame::yawToNed(mc::deg2rad(cv["initial_heading_deg"].number()));

    // Grids and sample spacings: the reference values (25 / 50 m grids, 25 m
    // samples, 100 m knots) are for a 5 km map; absent keys are scaled by
    // mapSize / 5000 and clamped to the belief resolution.
    const double s = p.mapSize / 5000.0;
    p.team.fastGridStep    = cv["fast_grid_step_m"].num(std::max(p.cellSize, 25.0 * s));
    p.team.exploreGridStep = cv["explore_grid_step_m"].num(std::max(p.cellSize, 50.0 * s));
    p.curve.sampleSpacing  = cv["sample_spacing_m"].num(std::max(1.0, 25.0 * s));
    p.curve.curvatureKnotSpacing = cv["knot_spacing_m"].num(std::max(4.0 * p.curve.sampleSpacing, 100.0 * s));

    p.optimizer.maxIter         = static_cast<int>(cv["max_iter"].num(p.optimizer.maxIter));
    p.optimizer.exploreIter     = static_cast<int>(cv["explore_iter"].num(p.optimizer.exploreIter));
    p.optimizer.exploreIterWarm = static_cast<int>(cv["explore_iter_warm"].num(p.optimizer.exploreIterWarm));
    p.team.coordinationSweeps   = static_cast<int>(cv["coordination_sweeps"].num(p.team.coordinationSweeps));
    if (cv["init_strategies"].isArray()) {
        p.team.initStrategies.clear();
        for (const J::Value& v : cv["init_strategies"].array()) {
            const std::string t = v.str();
            if (t == "clusters") p.team.initStrategies.push_back(mc::InitStrategy::Clusters);
            else if (t == "greedy") p.team.initStrategies.push_back(mc::InitStrategy::Greedy);
            else throw std::runtime_error("curve.init_strategies: '" + t + "' is not clusters | greedy");
        }
    }
    p.team.reallocate       = cv["reallocate"].flag(p.team.reallocate);
    p.team.reallocRounds    = static_cast<int>(cv["realloc_rounds"].num(p.team.reallocRounds));
    p.team.maxReallocTrials = static_cast<int>(cv["max_realloc_trials"].num(p.team.maxReallocTrials));
}

/// = mtlc_plan cellsFromScenario().
mc::CellSet curveCellsFromScenario(const J::Value& sc, const Frame& frame, double cellSize) {
    const J::Value& cells = sc["cells"];
    const J::Array& centers = cells["centers"].array();
    const J::Array* mass = cells["mass"].isArray() ? &cells["mass"].array() : nullptr;
    mc::CellSet out;
    const auto m = static_cast<mc::Index>(centers.size());
    out.centers.resize(m, 2);
    out.mass.resize(m);
    for (mc::Index i = 0; i < m; ++i) {
        const J::Array& row = centers[static_cast<std::size_t>(i)].array();
        const mtl::Vec2 xy = frame.toMtl(row[0].number(), row[1].number());
        out.centers(i, 0) = xy.x();
        out.centers(i, 1) = xy.y();
        out.mass(i) = mass ? (*mass)[static_cast<std::size_t>(i)].number() : 1.0;
    }
    out.cellSize = cellSize;
    out.area = mc::VecX::Constant(m, cellSize * cellSize);
    out.nPix = mc::VecX::Constant(m, 1.0);
    out.meanBelief = out.mass / std::max(cellSize * cellSize, 1e-9);
    out.peakBelief = out.meanBelief;
    out.retainedMass = out.mass.sum();
    out.totalMapMass = sc["cells"]["total_map_mass"].num(out.retainedMass);
    out.massNorm = out.retainedMass > 0 ? (out.mass / out.retainedMass).eval() : out.mass;
    const double res = sc["mission"]["area"]["belief_res_m"].num(1.0);
    out.gridRes = mc::Vec2(res, res);
    return out;
}

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open scenario file: " + path);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

J::Array numbersOf(const std::vector<double>& v, int ndigits) {
    const double scale = std::pow(10.0, ndigits);
    J::Array out;
    out.reserve(v.size());
    for (const double d : v) out.push_back(J::Value(std::round(d * scale) / scale));
    return out;
}

template <typename Int>
J::Array indicesOf(const std::vector<Int>& v) {
    J::Array out;
    out.reserve(v.size());
    for (const Int i : v) out.push_back(J::Value(static_cast<double>(i)));
    return out;
}

J::Value vec2(double a, double b) { return J::Value(J::Array{J::Value(a), J::Value(b)}); }

}  // namespace

// -----------------------------------------------------------------------------
const char* toString(PlannerType type) {
    return type == PlannerType::Curve ? "curve" : "orienteering";
}

PlannerType plannerTypeFrom(const std::string& name) {
    if (name == "orienteering") return PlannerType::Orienteering;
    if (name == "curve") return PlannerType::Curve;
    throw std::runtime_error("planner.type '" + name + "' is not orienteering | curve");
}

int SearchProblem::agentIndex(const std::string& agentName) const {
    for (std::size_t i = 0; i < agents.size(); ++i) {
        if (agents[i].name == agentName) return static_cast<int>(i);
    }
    return -1;
}

SearchProblem parseScenario(const std::string& jsonText) {
    SearchProblem out;
    out.raw = J::parse(jsonText);
    const J::Value& sc = out.raw;
    const std::string schema = sc["schema"].text("");
    if (schema != kScenarioSchema) {
        throw std::runtime_error("scenario schema is '" + schema + "', expected '" +
                                 kScenarioSchema + "'");
    }
    const J::Value& area = sc["mission"]["area"];
    const double size = area["size_m"].num(0.0);
    if (!(size > 0.0)) throw std::runtime_error("scenario mission.area.size_m must be positive");
    std::vector<double> center{0.0, 0.0};
    if (area["center_ned"].isArray()) center = area["center_ned"].numbers();
    if (center.size() != 2) throw std::runtime_error("scenario mission.area.center_ned needs [n, e]");
    out.frame.nMin = center[0] - size / 2.0;
    out.frame.eMin = center[1] - size / 2.0;
    out.name = sc["mission"]["name"].text("mtl_search");
    out.seed = static_cast<std::uint64_t>(sc["mission"]["seed"].num(21.0));

    const J::Value& teamAgents = sc["team"]["agents"];
    if (!teamAgents.isArray() || teamAgents.array().empty()) {
        throw std::runtime_error("scenario needs at least one entry in team.agents");
    }
    for (const J::Value& a : teamAgents.array()) {
        AgentSpec spec;
        spec.name = a["name"].text("agent" + std::to_string(out.agents.size() + 1));
        const std::vector<double> s = a["start_ned"].numbers();
        if (s.size() != 2) throw std::runtime_error("agent '" + spec.name + "': start_ned needs [n, e]");
        spec.startNed = mtl::Vec2(s[0], s[1]);
        const std::vector<double> h = a["home_ned"].numbers();
        spec.homeNed = h.size() == 2 ? mtl::Vec2(h[0], h[1]) : spec.startNed;
        out.startsMtl.push_back(out.frame.toMtl(spec.startNed.x(), spec.startNed.y()));
        out.agents.push_back(spec);
    }
    out.params = paramsFromScenario(sc, static_cast<int>(out.agents.size()));
    out.reportAlternative = sc["info_aware"]["report_both"].flag(true);
    out.cells  = cellsFromScenario(sc, out.frame, out.params.targetCellSize,
                                   out.params.minimumBeliefMass);
    if (out.cells.empty()) throw std::runtime_error("scenario contains no cells to plan over");

    // Planner selection (absent block = orienteering, as every scenario written
    // before the curve planner existed).
    const J::Value& planner = sc["planner"];
    refuseUnknownKeys(planner, kPlannerKeys, "planner");
    out.plannerType = plannerTypeFrom(planner["type"].text("orienteering"));
    out.compareOrienteering = planner["compare_orienteering"].flag(true);
    // The follower's single-axis gimbal law (airstack.follower.gimbal_law), carried
    // by the plan (SearchPlan.gimbal_law); absent = open_loop, the MTL default.
    const J::Value& follower = sc["airstack"]["follower"];
    refuseUnknownKeys(follower, {"gimbal_law"}, "airstack.follower");
    out.gimbalLaw = follower["gimbal_law"].text("open_loop");
    if (out.gimbalLaw != "open_loop" && out.gimbalLaw != "aim_point") {
        throw std::runtime_error("airstack.follower.gimbal_law '" + out.gimbalLaw + "' is not open_loop | aim_point");
    }
    // The curve planner's inputs, exactly as mtlc_plan builds them.
    out.curveParams = curveBaseParamsFromScenario(sc, static_cast<int>(out.agents.size()));
    curveFromScenario(sc, out.frame, out.curveParams);
    out.curveCells = curveCellsFromScenario(sc, out.frame, out.curveParams.targetCellSize);
    for (const mtl::Vec2& xy : out.startsMtl) out.curveStarts.push_back(mc::Vec2(xy.x(), xy.y()));
    return out;
}

SearchProblem loadScenario(const std::string& path) { return parseScenario(readFile(path)); }

mtl::PlanningResult solve(const SearchProblem& problem) {
    mtl::Planner planner(problem.params);
    return planner.planFromCells(problem.cells, problem.startsMtl);
}

mtl::PlanningResult solveAlternative(const SearchProblem& problem) {
    return solveOrienteering(problem, !problem.params.infoAware.enabled);
}

mtl::PlanningResult solveOrienteering(const SearchProblem& problem, bool infoAware) {
    mtl::PlannerParams p = problem.params;
    p.infoAware.enabled = infoAware;
    mtl::Planner planner(p);
    return planner.planFromCells(problem.cells, problem.startsMtl);
}

CurvePlan solveCurve(const SearchProblem& problem) {
    const mc::PlannerParams& p = problem.curveParams;
    if (!std::isfinite(std::min(p.maxFlightDistance, p.maxFlightTime * p.avgDroneSpeed))) {
        throw std::runtime_error(
            "planner.type curve needs a finite budget: set team.max_flight_time_s or "
            "team.max_flight_distance_m (the curve IS the budget)");
    }
    mc::Planner planner(p);   // finalize() + validate(); throws std::invalid_argument
    CurvePlan out;
    out.result = planner.planFromCells(problem.curveCells, problem.curveStarts);
    out.params = planner.params();
    return out;
}

namespace {
template <typename F>
SearchResult timed(F&& f) {
    const auto t0 = std::chrono::steady_clock::now();
    SearchResult r{f(), 0.0};
    r.planningSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return r;
}
}  // namespace

SearchResult solveFlown(const SearchProblem& problem) {
    if (problem.plannerType == PlannerType::Curve) return timed([&] { return solveCurve(problem); });
    return timed([&] { return solve(problem); });
}

std::vector<SearchResult> solveComparisons(const SearchProblem& problem) {
    std::vector<SearchResult> out;
    if (problem.plannerType == PlannerType::Curve) {
        if (!problem.compareOrienteering) return out;
        out.push_back(timed([&] { return solveOrienteering(problem, false); }));
        out.push_back(timed([&] { return solveOrienteering(problem, true); }));
    } else if (problem.reportAlternative) {
        out.push_back(timed([&] { return solveAlternative(problem); }));
    }
    return out;
}

std::string plannerMode(const mtl::PlanningResult& result) {
    return result.infoAware.enabled ? "info_aware" : "plain";
}

std::string comparisonSuffix(const SearchProblem& problem, const SearchResult& comparison) {
    if (problem.plannerType == PlannerType::Orienteering) return "_alt";
    return "_alt_" + plannerMode(comparison);
}

std::string plannerMode(const SearchResult& result) {
    if (const mtl::PlanningResult* o = result.orienteering()) return plannerMode(*o);
    return "curve";
}

std::size_t SearchResult::numAgents() const {
    if (const mtl::PlanningResult* o = orienteering()) return o->trajectories.size();
    return curve()->result.trajectories.size();
}

mtl::Index SearchResult::numCells() const {
    if (const mtl::PlanningResult* o = orienteering()) return o->cells.size();
    return curve()->result.cells.size();
}

mtl::Vec2 SearchResult::cellCenterMtl(mtl::Index i) const {
    if (const mtl::PlanningResult* o = orienteering()) return o->cells.centers.row(i).transpose();
    return curve()->result.cells.centers.row(i).transpose();
}

const mtl::Path3& SearchResult::droneTrackMtl(std::size_t agent) const {
    if (const mtl::PlanningResult* o = orienteering()) return o->trajectories.at(agent).drone;
    return curve()->result.trajectories.at(agent).drone;
}

double SearchResult::teamInfo() const {
    if (const mtl::PlanningResult* o = orienteering()) return o->team.info;
    return curve()->result.team.info;
}

double SearchResult::teamInfoTotal() const {
    if (const mtl::PlanningResult* o = orienteering()) return o->team.infoTotal;
    return curve()->result.team.infoTotal;
}

double SearchResult::teamInfoFraction() const {
    if (const mtl::PlanningResult* o = orienteering()) return o->team.infoFraction;
    return curve()->result.team.infoFraction;
}

// -----------------------------------------------------------------------------
AgentTrack buildAgentTrack(const mtl::PlanningResult& r, const SearchProblem& problem,
                           int agentIndex, double homeUp) {
    if (agentIndex < 0 || static_cast<std::size_t>(agentIndex) >= r.trajectories.size() ||
        static_cast<std::size_t>(agentIndex) >= problem.agents.size()) {
        throw std::out_of_range("agent index " + std::to_string(agentIndex) + " not in the plan");
    }
    const auto a = static_cast<std::size_t>(agentIndex);
    const mtl::AgentTrajectory& traj = r.trajectories[a];
    const mtl::AgentPlan&       plan = r.plans[a];
    const mtl::PlannerParams&   p    = problem.params;

    AgentTrack out;
    out.name        = problem.agents[a].name;
    out.index       = agentIndex;
    out.homeEnu     = problem.agents[a].homeEnu(homeUp);
    out.budget      = plan.budget;
    out.flownLength = plan.flownLength;
    out.flightTime  = plan.flightTime;
    out.feasible    = plan.feasible;
    out.scheduled   = traj.scheduled;
    out.singleAxis  = p.singleAxisGimbal;
    out.tilt        = p.singleAxisGimbal ? p.sensorTiltAngle : 0.0;
    out.fov         = p.fov;
    out.speedMps    = p.avgDroneSpeed;
    out.minTurnRadius = p.minTurnRadius;
    // The team is planned at one cruise altitude; each agent flies it shifted by
    // its deconfliction offset. The boresight ground points do not move, and the
    // follower's gimbal law aims from the measured position, so only z changes.
    const double dz = agentAltitudeOffset(problem.raw, a);
    out.altitude    = p.droneAltitude + dz;
    out.dt          = p.dt;
    out.gimbalMax   = p.gimbal.gimbalMax;
    out.gimbalRate  = p.gimbal.gimbalRate;
    out.pitchNudgeMax = p.gimbal.pitchNudgeMax;
    out.routeNote     = plan.routeInfo.note;
    out.extensionNote = plan.extInfo.note;
    out.plannerMode   = plannerMode(r);
    out.gimbalLaw     = problem.gimbalLaw;
    const std::vector<mtl::Index>& realized =
        traj.realizedCellIdx.empty() ? plan.servicedCellIdx : traj.realizedCellIdx;
    out.servicedCells.assign(realized.begin(), realized.end());
    out.plannedCells.assign(plan.servicedCellIdx.begin(), plan.servicedCellIdx.end());

    const mtl::Index n = std::min<mtl::Index>(traj.drone.rows(), r.timeVec.size());
    if (n == 0) return out;

    // Last sample that still moves: the team timeline pads a finished agent
    // with copies of its final state, and a carrot follower would wait on
    // that tail forever for arc length that never grows.
    mtl::Index last = 0;
    for (mtl::Index k = 1; k < n; ++k) {
        const double step = std::hypot(traj.drone(k, 0) - traj.drone(k - 1, 0),
                                       traj.drone(k, 1) - traj.drone(k - 1, 1));
        if (step > 0.05) last = k;
    }
    const mtl::Index count = std::max<mtl::Index>(last + 1, std::min<mtl::Index>(n, 1));

    const bool hasPhi = traj.scheduled && traj.diagnostics.gimbalAngle.size() >= count;
    const mtl::Vec3 home = out.homeEnu;
    out.samples.reserve(static_cast<std::size_t>(count));
    double arc = 0.0;
    for (mtl::Index k = 0; k < count; ++k) {
        double nn = 0.0, ee = 0.0;
        problem.frame.fromMtl(traj.drone(k, 0), traj.drone(k, 1), nn, ee);
        const mtl::Vec3 wp = nedToEnu(nn, ee, -traj.drone(k, 2));
        double sn = 0.0, se = 0.0;
        problem.frame.fromMtl(traj.sensor(k, 0), traj.sensor(k, 1), sn, se);
        const mtl::Vec3 wb = nedToEnu(sn, se, 0.0);

        TrackSample s;
        s.t = r.timeVec(k);
        if (k > 0) {
            const TrackSample& prev = out.samples.back();
            arc += std::hypot(wp.x() - home.x() - prev.x, wp.y() - home.y() - prev.y);
        }
        s.arc = arc;
        s.x = wp.x() - home.x();
        s.y = wp.y() - home.y();
        s.z = wp.z() - home.z() + dz;
        s.yaw = traj.rpy(k, 2);
        s.roll = traj.rpy(k, 0);
        s.pitch = traj.rpy(k, 1);
        s.bx = wb.x() - home.x();
        s.by = wb.y() - home.y();
        s.bz = wb.z() - home.z();
        s.gimbalPhi = hasPhi ? traj.diagnostics.gimbalAngle(k) : 0.0;
        out.samples.push_back(s);
    }
    for (std::size_t k = 0; k < out.samples.size(); ++k) {
        const std::size_t j = std::min(k + 1, out.samples.size() - 1);
        const std::size_t i = (j == k && k > 0) ? k - 1 : k;
        const double dt = out.samples[j].t - out.samples[i].t;
        out.samples[k].speed = dt > 0.0 ? (out.samples[j].arc - out.samples[i].arc) / dt : 0.0;
    }
    return out;
}


// -----------------------------------------------------------------------------
namespace {

/// The curve tangent heading the planner built sample k's boresight on.
/// mtl::curve does not export its tangents, and rpy's yaw is a smoothed
/// finite difference that lags them, so the heading is recovered EXACTLY from
/// the boresight offset w = sensor - drone = s T + c N  (s = h tan(tau) along
/// the unit tangent T, c = h tan(alpha) / cos(tau) along the left normal
/// N = J T):  T = (s I - c J) w / (s^2 + c^2).  In the interior of a
/// Curvature-representation track this equals the central-difference heading
/// of the flown polyline to round-off (the test checks both).  Falls back to
/// that central difference when the offset vanishes (nadir mount at alpha = 0).
double curveTangentHeading(const mc::AgentTrajectory& traj, double standOff, double tilt, mtl::Index k,
                           mtl::Index count) {
    const double h = traj.drone(k, 2);
    const double c = h * std::tan(traj.gimbalAngle(k)) / std::cos(tilt);
    const double s = standOff;
    const double wx = traj.sensor(k, 0) - traj.drone(k, 0);
    const double wy = traj.sensor(k, 1) - traj.drone(k, 1);
    const double den = s * s + c * c;
    if (den > 1e-6) {
        // (s I - c J) w with J = [[0, -1], [1, 0]]:  J w = (-wy, wx)
        const double tx = (s * wx + c * wy) / den;
        const double ty = (s * wy - c * wx) / den;
        return std::atan2(ty, tx);
    }
    const mtl::Index a = std::max<mtl::Index>(k - 1, 0), b = std::min<mtl::Index>(k + 1, count - 1);
    return std::atan2(traj.drone(b, 1) - traj.drone(a, 1), traj.drone(b, 0) - traj.drone(a, 0));
}

}  // namespace

AgentTrack buildAgentTrack(const CurvePlan& cp, const SearchProblem& problem, int agentIndex, double homeUp) {
    const mc::PlanningResult& r = cp.result;
    if (agentIndex < 0 || static_cast<std::size_t>(agentIndex) >= r.trajectories.size() ||
        static_cast<std::size_t>(agentIndex) >= problem.agents.size()) {
        throw std::out_of_range("agent index " + std::to_string(agentIndex) + " not in the plan");
    }
    const auto a = static_cast<std::size_t>(agentIndex);
    const mc::AgentTrajectory& traj = r.trajectories[a];
    const mc::AgentPlan&       plan = r.plans[a];
    const mc::PlannerParams&   p    = cp.params;

    AgentTrack out;
    out.name        = problem.agents[a].name;
    out.index       = agentIndex;
    out.homeEnu     = problem.agents[a].homeEnu(homeUp);
    out.budget      = plan.budget;
    out.flownLength = plan.flownLength;
    out.flightTime  = plan.flightTime;
    out.feasible    = plan.feasible;
    out.scheduled   = false;       // the gimbal sweeps; nothing is scheduled
    out.singleAxis  = true;        // the curve planner is single-axis only
    out.tilt        = p.sensorTiltAngle;
    out.fov         = p.fov;
    out.speedMps    = p.avgDroneSpeed;
    out.minTurnRadius = p.minTurnRadius;
    // The planned altitude IS this agent's cruise altitude (droneAltitude +
    // a * curve.altitude_stagger_m): the curve planner deconflicts in its own
    // plan, so team.agents[].altitude_offset_m is NOT added on top.
    out.altitude    = plan.altitude;
    out.dt          = p.dt;
    out.gimbalMax   = p.gimbal.gimbalMax;
    out.gimbalRate  = p.gimbal.gimbalRate;
    out.pitchNudgeMax = 0.0;       // no pitch nudge: the sweep lives on the mount's tilted plane
    out.routeNote     = "continuous curve, " + std::string(mc::toString(plan.rep.type)) +
                        " representation, seed '" + plan.initStrategy + "'";
    out.extensionNote = "n/a (no run-out: the curve spends the whole budget)";
    out.plannerMode   = "curve";
    out.plannerType   = PlannerType::Curve;
    out.gimbalLaw     = problem.gimbalLaw;
    out.servicedCells.assign(plan.servicedCellIdx.begin(), plan.servicedCellIdx.end());
    out.plannedCells  = out.servicedCells;   // there is no separate route

    CurveTrackInfo& ci = out.curve;
    ci.representation = mc::toString(plan.rep.type);
    ci.endpointMode   = mc::toString(plan.rep.mode);
    ci.initStrategy   = plan.initStrategy;
    ci.sweepAmplitude = plan.sweep.alphaMax;
    ci.sweepFreq      = plan.sweep.freq;
    ci.sweepPeakRate  = plan.sweep.peakRate;
    ci.swathHalfWidth = plan.swathHalfWidth;
    ci.standOff       = plan.sweep.standOff;
    ci.maxCurvature   = plan.maxKappa;
    ci.endpointError  = plan.endpointError;
    ci.fastObjective  = plan.lastOpt.J;
    ci.optimizerIters = plan.lastOpt.iters;
    ci.optimizerExit  = plan.lastOpt.exitMsg;

    const mtl::Index n = std::min<mtl::Index>(traj.drone.rows(), r.timeVec.size());
    if (n == 0) return out;
    // Trim the team-timeline padding (only an agent with a shorter budget has any).
    mtl::Index last = 0;
    for (mtl::Index k = 1; k < n; ++k) {
        const double step = std::hypot(traj.drone(k, 0) - traj.drone(k - 1, 0),
                                       traj.drone(k, 1) - traj.drone(k - 1, 1));
        if (step > 0.05) last = k;
    }
    const mtl::Index count = std::max<mtl::Index>(last + 1, std::min<mtl::Index>(n, 1));

    const mtl::Vec3 home = out.homeEnu;
    out.samples.reserve(static_cast<std::size_t>(count));
    double arc = 0.0;
    for (mtl::Index k = 0; k < count; ++k) {
        double nn = 0.0, ee = 0.0;
        problem.frame.fromMtl(traj.drone(k, 0), traj.drone(k, 1), nn, ee);
        const mtl::Vec3 wp = nedToEnu(nn, ee, -traj.drone(k, 2));
        double sn = 0.0, se = 0.0;
        problem.frame.fromMtl(traj.sensor(k, 0), traj.sensor(k, 1), sn, se);
        const mtl::Vec3 wb = nedToEnu(sn, se, 0.0);

        TrackSample smp;
        smp.t = r.timeVec(k);
        if (k > 0) {
            const TrackSample& prev = out.samples.back();
            arc += std::hypot(wp.x() - home.x() - prev.x, wp.y() - home.y() - prev.y);
        }
        smp.arc = arc;
        smp.x = wp.x() - home.x();
        smp.y = wp.y() - home.y();
        smp.z = wp.z() - home.z();
        smp.yaw = curveTangentHeading(traj, plan.sweep.standOff, plan.sweep.tiltAngle, k, count);
        smp.speed = p.avgDroneSpeed;   // constant ground speed: arc = V t by construction
        smp.roll = traj.rpy(k, 0);
        smp.pitch = traj.rpy(k, 1);
        smp.bx = wb.x() - home.x();
        smp.by = wb.y() - home.y();
        smp.bz = wb.z() - home.z();
        // host phi (+ right, crossAngle = roll + phi) from the curve's level-frame
        // sweep alpha (+ left):  roll + phi = -alpha  ->  phi = -(alpha + roll) = -gimbalCmd
        smp.gimbalPhi = -traj.gimbalCmd(k);
        out.samples.push_back(smp);
    }
    return out;
}

AgentTrack buildAgentTrack(const SearchResult& result, const SearchProblem& problem, int agentIndex,
                           double homeUp) {
    if (const mtl::PlanningResult* o = result.orienteering()) return buildAgentTrack(*o, problem, agentIndex, homeUp);
    return buildAgentTrack(*result.curve(), problem, agentIndex, homeUp);
}

// -----------------------------------------------------------------------------
std::string teamPlanJson(const mtl::PlanningResult& r, const SearchProblem& problem) {
    const J::Value& sc = problem.raw;
    const mtl::PlannerParams& params = problem.params;
    const Frame& frame = problem.frame;
    const mtl::Index steps = r.numSteps();

    J::Array agents;
    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const mtl::AgentTrajectory& traj = r.trajectories[a];
        const mtl::AgentPlan&       plan = r.plans[a];
        std::vector<double> t, n, e, h, sn, se, roll, pitch, yaw;
        for (mtl::Index k = 0; k < steps && k < traj.drone.rows(); ++k) {
            double nn = 0.0, ee = 0.0;
            t.push_back(r.timeVec(k));
            frame.fromMtl(traj.drone(k, 0), traj.drone(k, 1), nn, ee);
            n.push_back(nn);
            e.push_back(ee);
            h.push_back(traj.drone(k, 2));
            frame.fromMtl(traj.sensor(k, 0), traj.sensor(k, 1), nn, ee);
            sn.push_back(nn);
            se.push_back(ee);
            roll.push_back(traj.rpy(k, 0));
            pitch.push_back(traj.rpy(k, 1));
            yaw.push_back(Frame::yawToNed(traj.rpy(k, 2)));
        }
        J::Object samples;
        samples["t"] = J::Value(numbersOf(t, 3));
        samples["n"] = J::Value(numbersOf(n, 3));
        samples["e"] = J::Value(numbersOf(e, 3));
        samples["h"] = J::Value(numbersOf(h, 3));
        samples["sensor_n"] = J::Value(numbersOf(sn, 3));
        samples["sensor_e"] = J::Value(numbersOf(se, 3));
        samples["roll"]  = J::Value(numbersOf(roll, 5));
        samples["pitch"] = J::Value(numbersOf(pitch, 5));
        samples["yaw"]   = J::Value(numbersOf(yaw, 5));

        const std::vector<mtl::Index>& realized =
            traj.realizedCellIdx.empty() ? plan.servicedCellIdx : traj.realizedCellIdx;
        J::Array clusters;
        for (const mtl::Index c : plan.selClusters) {
            double cn = 0.0, ce = 0.0;
            frame.fromMtl(r.clusters.centroids(c, 0), r.clusters.centroids(c, 1), cn, ce);
            clusters.push_back(vec2(cn, ce));
        }
        J::Object diag;
        diag["scheduled"]         = J::Value(traj.scheduled);
        diag["feasible"]          = J::Value(plan.feasible);
        diag["flown_length_m"]    = J::Value(plan.flownLength);
        diag["flight_time_s"]     = J::Value(plan.flightTime);
        diag["budget_used_frac"]  = J::Value(plan.budgetUsed);
        diag["info_mass"]         = J::Value(plan.score.info);
        diag["info_fraction"]     = J::Value(plan.score.infoFraction);
        diag["route_note"]        = J::Value(plan.routeInfo.note);
        diag["refine_note"]       = J::Value(plan.refineInfo.note);
        diag["extension_note"]    = J::Value(plan.extInfo.note);
        diag["clusters_selected"] = J::Value(indicesOf(plan.selClusters));
        diag["clusters_dropped"]  = J::Value(indicesOf(plan.droppedClusters));
        if (traj.scheduled) {
            const mtl::GimbalDiagnostics& d = traj.diagnostics;
            diag["gimbal_targets"]     = J::Value(static_cast<double>(d.nTargets));
            diag["gimbal_targets_hit"] = J::Value(static_cast<double>(d.nTargetsHit));
            diag["gimbal_coverage"]    = J::Value(d.targetCoverage);
            diag["miss_never_abeam"]   = J::Value(static_cast<double>(d.nMissNeverAbeam));
            diag["miss_out_of_reach"]  = J::Value(static_cast<double>(d.nMissOutOfReach));
            diag["miss_double_booked"] = J::Value(static_cast<double>(d.nMissDoubleBooked));
            diag["slant_range_max_m"]  = J::Value(d.slantRangeMax);
            diag["gimbal_max_deg"]     = J::Value(d.gimbalMaxDeg);
            diag["gimbal_rate_max_deg_s"] = J::Value(d.gimbalRateMaxDeg);
            diag["tilt_deg"]           = J::Value(d.tiltAngleDeg);
        }
        double startN = 0.0, startE = 0.0;
        if (plan.droneRoute.rows() > 0) frame.fromMtl(plan.droneRoute(0, 0), plan.droneRoute(0, 1), startN, startE);

        J::Object agent;
        agent["name"]           = J::Value(a < problem.agents.size() ? problem.agents[a].name
                                                                     : "agent" + std::to_string(a + 1));
        agent["start_ned"]      = vec2(startN, startE);
        if (a < problem.agents.size()) {
            agent["home_ned"] = vec2(problem.agents[a].homeNed.x(), problem.agents[a].homeNed.y());
        }
        agent["budget_m"]       = J::Value(plan.budget);
        agent["path_length_m"]  = J::Value(plan.flownLength);
        agent["duration_s"]     = J::Value(plan.flightTime);
        agent["serviced_cells"] = J::Value(indicesOf(realized));
        agent["planned_cells"]  = J::Value(indicesOf(plan.servicedCellIdx));
        agent["clusters"]       = J::Value(std::move(clusters));
        agent["samples"]        = J::Value(std::move(samples));
        agent["diagnostics"]    = J::Value(std::move(diag));
        agents.push_back(J::Value(std::move(agent)));
    }

    J::Array centers, mass;
    for (mtl::Index i = 0; i < r.cells.size(); ++i) {
        double cn = 0.0, ce = 0.0;
        frame.fromMtl(r.cells.centers(i, 0), r.cells.centers(i, 1), cn, ce);
        centers.push_back(vec2(cn, ce));
        mass.push_back(J::Value(r.cells.mass(i)));
    }
    J::Object cellsOut;
    cellsOut["centers"] = J::Value(std::move(centers));
    cellsOut["mass"]    = J::Value(std::move(mass));

    J::Object gen;
    gen["name"]    = J::Value("mtl_search_planner");
    gen["library"] = J::Value("mtl::planner (vendored)");
    gen["mode"]    = J::Value(params.singleAxisGimbal ? "single_axis" : "multi_axis");

    J::Object team;
    team["info_mass"]          = J::Value(r.team.info);
    team["info_total"]         = J::Value(r.team.infoTotal);
    team["info_fraction"]      = J::Value(r.team.infoFraction);
    team["clusters_reached"]   = J::Value(static_cast<double>(r.team.reachedClusters.size()));
    team["clusters_unreached"] = J::Value(static_cast<double>(r.team.unreachedClusters.size()));
    team["cells_serviced"]     = J::Value(static_cast<double>(r.team.servicedCellIdx.size()));
    team["cells_unserviced"]   = J::Value(static_cast<double>(r.team.unservicedCellIdx.size()));
    team["realloc_rounds"]     = J::Value(r.team.rounds);
    team["flown_length_m"]     = J::Value(numbersOf(r.team.flownLength, 3));
    team["budgets_m"]          = J::Value(numbersOf(r.team.budgets, 3));

    J::Object meta;
    meta["team"]              = J::Value(std::move(team));
    meta["budget_dist_m"]     = J::Value(params.budgetDist());
    meta["sensor_standoff_m"] = J::Value(params.sensorStandOff());
    meta["single_axis"]       = J::Value(params.singleAxisGimbal);
    meta["steps"]             = J::Value(static_cast<double>(steps));
    meta["planner_mode"]      = J::Value(plannerMode(r));
    if (r.infoAware.enabled) {
        // The information-aware search's audit trail: every abstraction it
        // planned, the coverage-model score it ranked them by, and the plain
        // plan's score on the same model.
        const mtl::InfoAwareReport& ia = r.infoAware;
        J::Array cands;
        for (const mtl::InfoAwareCandidate& c : ia.candidates) {
            J::Object o;
            o["label"]     = J::Value(c.label);
            o["reach_m"]   = J::Value(c.reach);
            o["detected"]  = J::Value(c.score);
            o["info_mass"] = J::Value(c.info);
            o["flown_m"]   = J::Value(c.flown);
            o["clusters"]  = J::Value(static_cast<double>(c.clusters));
            o["accepted"]  = J::Value(c.accepted);
            cands.push_back(J::Value(std::move(o)));
        }
        J::Object io;
        io["chosen"]            = J::Value(ia.chosen);
        io["chosen_detected"]   = J::Value(ia.chosenScore);
        io["baseline_detected"] = J::Value(ia.baselineScore);
        io["chosen_reach_m"]    = J::Value(ia.chosenReach);
        io["detection_reach_m"] = J::Value(ia.detectionReach);
        io["basins"]            = J::Value(static_cast<double>(ia.basins));
        io["moves_tried"]       = J::Value(ia.movesTried);
        io["moves_accepted"]    = J::Value(ia.movesAccepted);
        io["seconds"]           = J::Value(ia.seconds);
        io["candidates"]        = J::Value(std::move(cands));
        meta["info_aware"]      = J::Value(std::move(io));
    }

    J::Object out;
    out["schema"]    = J::Value(kPlanSchema);
    out["generator"] = J::Value(std::move(gen));
    out["mission"]   = sc["mission"];
    out["dt"]        = J::Value(params.dt);
    out["agents"]    = J::Value(std::move(agents));
    out["cells"]     = J::Value(std::move(cellsOut));
    out["meta"]      = J::Value(std::move(meta));
    return J::Value(std::move(out)).dump();
}

// -----------------------------------------------------------------------------
//  A curve plan as mtl.plan/1: planToJson() / diagnosticsJson() of
//  mtl_curve_plan_json.cpp, key for key.  Host differences, additions only:
//  generator.name / library (this package), generator.planner_type,
//  meta.planner_type, meta.planner_mode.
// -----------------------------------------------------------------------------
namespace {

J::Array numbersOfVec(const mc::VecX& v, int ndigits) {
    const double scale = std::pow(10.0, ndigits);
    J::Array out;
    out.reserve(static_cast<std::size_t>(v.size()));
    for (mc::Index i = 0; i < v.size(); ++i) out.push_back(J::Value(std::round(v(i) * scale) / scale));
    return out;
}

J::Array numbersRaw(const std::vector<double>& v) {
    J::Array out;
    for (const double d : v) out.push_back(J::Value(d));
    return out;
}

J::Value curveDiagnosticsJson(const mc::AgentTrajectory& traj, const mc::AgentPlan& plan,
                              const mc::PlanningResult& r) {
    J::Object out;
    double info = 0.0;
    for (const mc::Index c : plan.servicedCellIdx) info += r.cells.mass(c);
    const double total = r.cells.mass.sum();
    out["info_mass"]          = J::Value(info);
    out["info_fraction"]      = J::Value(total > 0.0 ? info / total : 0.0);
    out["outer_iters"]        = J::Value(0);
    out["reserve_m"]          = J::Value(0.0);
    out["route_note"]         = J::Value("continuous curve, " + std::string(mc::toString(plan.rep.type)) +
                                         " representation, seed '" + plan.initStrategy + "'");
    out["refine_note"]        = J::Value("n/a (no cell anchors)");
    out["extension_note"]     = J::Value("n/a (no run-out: the curve spends the whole budget)");
    out["extension_runout_m"] = J::Value(0.0);
    out["extension_extra_m"]  = J::Value(0.0);
    out["clusters_dropped"]   = J::Value(J::Array{});
    double slant = 0.0;
    for (mc::Index k = 0; k < traj.drone.rows(); ++k) {
        const double dx = traj.drone(k, 0) - traj.sensor(k, 0), dy = traj.drone(k, 1) - traj.sensor(k, 1);
        slant = std::max(slant, std::sqrt(dx * dx + dy * dy + traj.drone(k, 2) * traj.drone(k, 2)));
    }
    out["slant_range_max_m"]  = J::Value(slant);
    out["pitch_max_deg"]      = J::Value(mc::rad2deg(traj.rpy.col(1).cwiseAbs().maxCoeff()));
    out["scheduled"]          = J::Value(false);
    out["feasible"]           = J::Value(plan.feasible);
    out["flown_length_m"]     = J::Value(plan.flownLength);
    out["flight_time_s"]      = J::Value(plan.flightTime);
    out["budget_used_frac"]   = J::Value(plan.budgetUsed);
    out["representation"]     = J::Value(mc::toString(plan.rep.type));
    out["endpoint_mode"]      = J::Value(mc::toString(plan.rep.mode));
    out["endpoint_error_m"]   = J::Value(plan.endpointError);
    out["init_strategy"]      = J::Value(plan.initStrategy);
    out["max_curvature"]      = J::Value(plan.maxKappa);
    out["altitude_m"]         = J::Value(plan.altitude);
    out["swath_half_width_m"] = J::Value(plan.swathHalfWidth);
    out["sweep_amplitude_deg"] = J::Value(mc::rad2deg(plan.sweep.alphaMax));
    out["sweep_freq_hz"]      = J::Value(plan.sweep.freq);
    out["sweep_peak_rate_deg_s"] = J::Value(mc::rad2deg(plan.sweep.peakRate));
    out["tilt_deg"]           = J::Value(mc::rad2deg(plan.sweep.tiltAngle));
    out["nadir_offset_m"]     = J::Value(plan.sweep.standOff);
    out["gimbal_max_deg"]     = J::Value(traj.maxGimbalCmdDeg);
    out["roll_max_deg"]       = J::Value(traj.maxRollDeg);
    out["fast_objective"]     = J::Value(plan.lastOpt.J);
    out["optimizer_iters"]    = J::Value(plan.lastOpt.iters);
    out["optimizer_exit"]     = J::Value(plan.lastOpt.exitMsg);
    out["clusters_selected"]  = J::Value(indicesOf(plan.waypointClusters));
    return J::Value(std::move(out));
}

}  // namespace

std::string teamPlanJson(const CurvePlan& cp, const SearchProblem& problem) {
    const mc::PlanningResult& r = cp.result;
    const mc::PlannerParams& params = cp.params;
    const J::Value& sc = problem.raw;
    const Frame& frame = problem.frame;
    const mc::Index steps = r.numSteps();
    J::Array agents;
    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const mc::AgentTrajectory& traj = r.trajectories[a];
        const mc::AgentPlan& plan = r.plans[a];

        mc::VecX t(steps), n(steps), e(steps), h(steps), sn(steps), se(steps), roll(steps), pitch(steps),
            yaw(steps), gim(steps);
        for (mc::Index k = 0; k < steps; ++k) {
            t(k) = r.timeVec(k);
            double nn = 0.0, ee = 0.0;
            frame.fromMtl(traj.drone(k, 0), traj.drone(k, 1), nn, ee);
            n(k) = nn;
            e(k) = ee;
            h(k) = traj.drone(k, 2);
            frame.fromMtl(traj.sensor(k, 0), traj.sensor(k, 1), nn, ee);
            sn(k) = nn;
            se(k) = ee;
            roll(k) = traj.rpy(k, 0);
            pitch(k) = traj.rpy(k, 1);
            yaw(k) = Frame::yawToNed(traj.rpy(k, 2));
            gim(k) = traj.gimbalAngle(k);
        }
        J::Object samples;
        samples["t"] = J::Value(numbersOfVec(t, 3));
        samples["n"] = J::Value(numbersOfVec(n, 3));
        samples["e"] = J::Value(numbersOfVec(e, 3));
        samples["h"] = J::Value(numbersOfVec(h, 3));
        samples["sensor_n"] = J::Value(numbersOfVec(sn, 3));
        samples["sensor_e"] = J::Value(numbersOfVec(se, 3));
        samples["roll"] = J::Value(numbersOfVec(roll, 5));
        samples["pitch"] = J::Value(numbersOfVec(pitch, 5));
        samples["yaw"] = J::Value(numbersOfVec(yaw, 5));
        samples["gimbal"] = J::Value(numbersOfVec(gim, 5));

        J::Array clusters;
        for (const mc::Index c : plan.waypointClusters) {
            double cn = 0.0, ce = 0.0;
            frame.fromMtl(r.clusters.centroids(c, 0), r.clusters.centroids(c, 1), cn, ce);
            clusters.push_back(J::Value(J::Array{J::Value(cn), J::Value(ce)}));
        }
        double startN = 0.0, startE = 0.0;
        frame.fromMtl(plan.start.x(), plan.start.y(), startN, startE);

        J::Object agent;
        agent["name"] = J::Value(a < problem.agents.size() ? problem.agents[a].name : "agent" + std::to_string(a + 1));
        agent["start_ned"] = J::Value(J::Array{J::Value(startN), J::Value(startE)});
        const J::Value& teamAgents = sc["team"]["agents"];
        if (teamAgents.isArray() && a < teamAgents.array().size()) {
            const J::Value& home = teamAgents.array()[a]["home_ned"];
            if (home.isArray()) agent["home_ned"] = home;
        }
        agent["budget_m"] = J::Value(plan.budget);
        agent["path_length_m"] = J::Value(plan.flownLength);
        agent["duration_s"] = J::Value(plan.flightTime);
        agent["serviced_cells"] = J::Value(indicesOf(plan.servicedCellIdx));
        agent["planned_cells"] = J::Value(indicesOf(plan.servicedCellIdx));
        agent["clusters"] = J::Value(std::move(clusters));
        agent["samples"] = J::Value(std::move(samples));
        agent["diagnostics"] = curveDiagnosticsJson(traj, plan, r);
        agents.push_back(J::Value(std::move(agent)));
    }

    J::Array centers, mass;
    for (mc::Index i = 0; i < r.cells.size(); ++i) {
        double cn = 0.0, ce = 0.0;
        frame.fromMtl(r.cells.centers(i, 0), r.cells.centers(i, 1), cn, ce);
        centers.push_back(J::Value(J::Array{J::Value(cn), J::Value(ce)}));
        mass.push_back(J::Value(r.cells.mass(i)));
    }
    J::Object cellsOut;
    cellsOut["centers"] = J::Value(std::move(centers));
    cellsOut["mass"] = J::Value(std::move(mass));

    J::Object gen;
    gen["name"] = J::Value("mtl_search_planner");
    gen["library"] = J::Value("mtl_curve::planner (vendored)");
    gen["mode"] = J::Value("single_axis_sweep");
    gen["planner_type"] = J::Value("curve");

    std::vector<double> flown, budgets;
    for (const mc::AgentPlan& pl : r.plans) {
        flown.push_back(pl.flownLength);
        budgets.push_back(pl.budget);
    }
    J::Object team;
    team["info_mass"] = J::Value(r.team.info);
    team["info_total"] = J::Value(r.team.infoTotal);
    team["info_fraction"] = J::Value(r.team.infoFraction);
    team["clusters_reached"] =
        J::Value(static_cast<double>(r.clusters.size() - static_cast<mc::Index>(r.team.audit.unserviced.size())));
    team["clusters_unreached"] = J::Value(static_cast<double>(r.team.audit.unserviced.size()));
    team["cells_serviced"] = J::Value(static_cast<double>(r.team.servicedCellIdx.size()));
    team["cells_unserviced"] = J::Value(static_cast<double>(r.team.unservicedCellIdx.size()));
    team["realloc_rounds"] = J::Value(static_cast<double>(r.team.log.size()));
    team["flown_length_m"] = J::Value(numbersRaw(flown));
    team["budgets_m"] = J::Value(numbersRaw(budgets));

    J::Array hist, stages;
    for (std::size_t i = 0; i < r.team.Jhist.size(); ++i) {
        hist.push_back(J::Value(r.team.Jhist[i]));
        stages.push_back(J::Value(r.team.stageNames[i]));
    }
    J::Object curve;
    curve["representation"] = J::Value(mc::toString(params.curve.representation));
    curve["fast_residual_static"] = J::Value(r.team.Jstatic);
    curve["fast_residual"] = J::Value(r.team.Jfinal);
    curve["fast_residual_history"] = J::Value(std::move(hist));
    curve["stages"] = J::Value(std::move(stages));
    curve["realloc_trials"] = J::Value(static_cast<double>(r.team.log.size()));
    curve["realloc_accepted"] = J::Value(r.team.acceptedTrials());
    curve["fast_grid_step_m"] = J::Value(r.grid.hg);

    J::Object meta;
    meta["team"] = J::Value(std::move(team));
    meta["curve"] = J::Value(std::move(curve));
    meta["budget_dist_m"] = J::Value(params.budgetDist());
    meta["sensor_standoff_m"] = J::Value(params.sensorStandOff());
    meta["single_axis"] = J::Value(true);
    meta["steps"] = J::Value(static_cast<double>(steps));
    meta["planner_type"] = J::Value("curve");
    meta["planner_mode"] = J::Value("curve");

    J::Object out;
    out["schema"] = J::Value(kPlanSchema);
    out["generator"] = J::Value(std::move(gen));
    out["mission"] = sc["mission"];
    out["dt"] = J::Value(params.dt);
    out["agents"] = J::Value(std::move(agents));
    out["cells"] = J::Value(std::move(cellsOut));
    out["meta"] = J::Value(std::move(meta));
    return J::Value(std::move(out)).dump();
}

std::string teamPlanJson(const SearchResult& result, const SearchProblem& problem) {
    if (const mtl::PlanningResult* o = result.orienteering()) return teamPlanJson(*o, problem);
    return teamPlanJson(*result.curve(), problem);
}

std::string agentTrackJson(const AgentTrack& tr, const SearchProblem& problem) {
    std::vector<double> t, arc, x, y, z, yaw, bx, by, bz, phi, pitch, n, e, sn, se;
    for (const TrackSample& s : tr.samples) {
        t.push_back(s.t);
        arc.push_back(s.arc);
        x.push_back(s.x);
        y.push_back(s.y);
        z.push_back(s.z);
        yaw.push_back(s.yaw);
        bx.push_back(s.bx);
        by.push_back(s.by);
        bz.push_back(s.bz);
        phi.push_back(s.gimbalPhi);
        pitch.push_back(s.pitch);
        // mission NED of the same samples, for fusing agents offline
        n.push_back(s.y + tr.homeEnu.y());
        e.push_back(s.x + tr.homeEnu.x());
        sn.push_back(s.by + tr.homeEnu.y());
        se.push_back(s.bx + tr.homeEnu.x());
    }
    J::Object samples;
    samples["t"] = J::Value(numbersOf(t, 3));
    samples["arc"] = J::Value(numbersOf(arc, 3));
    samples["x_map"] = J::Value(numbersOf(x, 3));
    samples["y_map"] = J::Value(numbersOf(y, 3));
    samples["z_map"] = J::Value(numbersOf(z, 3));
    samples["yaw_enu"] = J::Value(numbersOf(yaw, 5));
    samples["bx_map"] = J::Value(numbersOf(bx, 3));
    samples["by_map"] = J::Value(numbersOf(by, 3));
    samples["bz_map"] = J::Value(numbersOf(bz, 3));
    samples["gimbal_phi"] = J::Value(numbersOf(phi, 5));
    samples["pitch"] = J::Value(numbersOf(pitch, 5));
    samples["n"] = J::Value(numbersOf(n, 3));
    samples["e"] = J::Value(numbersOf(e, 3));
    samples["sensor_n"] = J::Value(numbersOf(sn, 3));
    samples["sensor_e"] = J::Value(numbersOf(se, 3));
    // the planned airframe roll: the cross-track angle is roll + gimbal_phi, and
    // the follower's open_loop law replays exactly that
    std::vector<double> roll;
    for (const TrackSample& s : tr.samples) roll.push_back(s.roll);
    samples["roll"] = J::Value(numbersOf(roll, 5));
    const bool isCurve = tr.plannerType == PlannerType::Curve;

    J::Object out;
    out["schema"] = J::Value("mtl.agent_track/1");
    out["mission"] = J::Value(problem.name);
    out["agent"] = J::Value(tr.name);
    out["agent_index"] = J::Value(tr.index);
    out["home_enu"] = J::Value(J::Array{J::Value(tr.homeEnu.x()), J::Value(tr.homeEnu.y()),
                                        J::Value(tr.homeEnu.z())});
    out["frame"] = J::Value("map = world ENU - home_enu; n/e = mission NED");
    out["single_axis"] = J::Value(tr.singleAxis);
    out["scheduled"] = J::Value(tr.scheduled);
    out["tilt_rad"] = J::Value(tr.tilt);
    out["fov_rad"] = J::Value(tr.fov);
    out["speed_mps"] = J::Value(tr.speedMps);
    out["min_turn_radius_m"] = J::Value(tr.minTurnRadius);
    out["altitude_m"] = J::Value(tr.altitude);
    out["dt_s"] = J::Value(tr.dt);
    out["budget_m"] = J::Value(tr.budget);
    out["flown_length_m"] = J::Value(tr.flownLength);
    out["flight_time_s"] = J::Value(tr.flightTime);
    out["feasible"] = J::Value(tr.feasible);
    out["serviced_cells"] = J::Value(indicesOf(tr.servicedCells));
    out["planned_cells"] = J::Value(indicesOf(tr.plannedCells));
    out["route_note"] = J::Value(tr.routeNote);
    out["extension_note"] = J::Value(tr.extensionNote);
    out["planner_mode"] = J::Value(tr.plannerMode);
    if (isCurve) {
        const CurveTrackInfo& ci = tr.curve;
        J::Object c;
        c["representation"] = J::Value(ci.representation);
        c["endpoint_mode"] = J::Value(ci.endpointMode);
        c["init_strategy"] = J::Value(ci.initStrategy);
        c["sweep_amplitude_deg"] = J::Value(ci.sweepAmplitude * 180.0 / mtl::kPi);
        c["sweep_freq_hz"] = J::Value(ci.sweepFreq);
        c["sweep_peak_rate_deg_s"] = J::Value(ci.sweepPeakRate * 180.0 / mtl::kPi);
        c["gimbal_rate_deg_s"] = J::Value(tr.gimbalRate * 180.0 / mtl::kPi);
        c["swath_half_width_m"] = J::Value(ci.swathHalfWidth);
        c["nadir_offset_m"] = J::Value(ci.standOff);
        c["max_curvature"] = J::Value(ci.maxCurvature);
        c["min_turn_radius_m"] = J::Value(tr.minTurnRadius);
        c["endpoint_error_m"] = J::Value(ci.endpointError);
        c["fast_objective"] = J::Value(ci.fastObjective);
        c["optimizer_iters"] = J::Value(ci.optimizerIters);
        c["optimizer_exit"] = J::Value(ci.optimizerExit);
        out["planner_type"] = J::Value("curve");
        out["curve"] = J::Value(std::move(c));
    }
    out["samples"] = J::Value(std::move(samples));
    return J::Value(std::move(out)).dump();
}

}  // namespace mtl_search
