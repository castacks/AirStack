// =============================================================================
//  apps/mtl_curve_plan_json.cpp   ->   the `mtlc_plan` tool
//
//  The bridge between the curve planner and any simulator that can write a
//  file - a DROP-IN for cpp_planner's `mtl_plan`:
//
//      mtlc_plan --scenario scenario.json --out plan.json
//      mtlc_plan --scenario -  --out -            # stdin to stdout
//
//  Reads the SAME scenario (mtl.scenario/1) and writes the SAME plan schema
//  (mtl.plan/1): one dense, time-parameterised track per agent plus the
//  boresight ground point at every sample, so a host that consumes mtl_plan's
//  output consumes this one unchanged.  paramsFromScenario() / cellsFromScenario()
//  follow apps/mtl_plan_json.cpp field for field; the curve planner's own
//  options come from an OPTIONAL "curve" block (see curveFromScenario below),
//  and keys that only mean something to cpp_planner (dubins step, gimbal
//  scheduler, solver) are accepted and ignored.
//
//  WHY CELLS AND NOT A BELIEF GRID - as for mtl_plan: the host owns the
//  scenario.  The curve planner optimises against the prior itself, so
//  planFromCells rebuilds it from the cells: each cell's mass spread over its
//  target_cell_size block.
//
//  FRAMES.  Scenario and plan are in the host's mission NED (n North, e East,
//  d Down; yaw 0 = North, clockwise).  mtl_curve works in x East, y North over
//  [0, mapSize], yaw counter-clockwise from East.  The conversions live in
//  Frame and nowhere else.
//
//  PLAN DIFFERENCES from mtl_plan's output (additions only):
//    generator.name = "mtl_curve_cpp", generator.library = "mtl_curve::planner"
//    agents[].samples.gimbal       [rad] sweep angle (+ left of track)
//    agents[].serviced_cells       cells whose centre THIS agent's swath
//                                  detects with P >= 0.5 (fast model)
//    agents[].planned_cells        = serviced_cells (there is no separate route)
//    agents[].clusters             the cluster centroids the seed pursued
//    agents[].diagnostics          curve-specific (see diagnosticsJson)
//    meta.curve                    representation, residual history, reallocation
// =============================================================================
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "json_mini.hpp"
#include "mtl_curve/planner.hpp"

namespace J = jsonmini;
namespace mc = mtl::curve;

namespace {

constexpr const char* kScenarioSchema = "mtl.scenario/1";
constexpr const char* kPlanSchema     = "mtl.plan/1";
constexpr const char* kGeneratorName  = "mtl_curve_cpp";

// --------------------------------------------------------------------------- //
//  Frame conversion.  ONE place.  (identical to apps/mtl_plan_json.cpp)
// --------------------------------------------------------------------------- //
struct Frame {
    double nMin = 0.0;
    double eMin = 0.0;
    mc::Vec2 toMtl(double n, double e) const { return {e - eMin, n - nMin}; }
    void fromMtl(double x, double y, double& n, double& e) const {
        n = y + nMin;
        e = x + eMin;
    }
    static double yawToNed(double yawMtl) { return mc::kPi / 2.0 - yawMtl; }
};

std::string readAll(const std::string& path) {
    if (path == "-")
        return std::string(std::istreambuf_iterator<char>(std::cin), std::istreambuf_iterator<char>());
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open scenario file: " + path);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

void writeAll(const std::string& path, const std::string& text) {
    if (path == "-") {
        std::cout << text << "\n";
        return;
    }
    std::ofstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot write plan file: " + path);
    f << text;
}

J::Array numbersOf(const mc::VecX& v, int ndigits = 3) {
    const double scale = std::pow(10.0, ndigits);
    J::Array out;
    out.reserve(static_cast<std::size_t>(v.size()));
    for (mc::Index i = 0; i < v.size(); ++i) out.push_back(J::Value(std::round(v(i) * scale) / scale));
    return out;
}

J::Array numbersOf(const std::vector<double>& v) {
    J::Array out;
    for (const double d : v) out.push_back(J::Value(d));
    return out;
}

J::Array indicesOf(const std::vector<mc::Index>& v) {
    J::Array out;
    for (const mc::Index i : v) out.push_back(J::Value(static_cast<double>(i)));
    return out;
}

mc::EndpointMode modeFrom(const std::string& s) {
    if (s == "open") return mc::EndpointMode::Open;
    if (s == "return_home") return mc::EndpointMode::ReturnHome;
    if (s == "fixed_dest") return mc::EndpointMode::FixedDest;
    throw std::runtime_error("curve.endpoint_mode '" + s + "' is not open | return_home | fixed_dest");
}

// --------------------------------------------------------------------------- //
//  scenario -> PlannerParams  (the shared keys, as mtl_plan reads them)
// --------------------------------------------------------------------------- //
mc::PlannerParams paramsFromScenario(const J::Value& sc, int numAgents) {
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

// --------------------------------------------------------------------------- //
//  the optional "curve" block  (every key optional; defaults = PlannerParams{})
//
//    representation        "curvature" | "bspline"
//    endpoint_mode         "open" | "return_home" | "fixed_dest", or one per agent
//    destinations_ned      [[n, e], ...] per agent (fixed_dest)
//    altitude_stagger_m    h_a = altitude_m + a * stagger  (default 25 m * beta/610)
//    kernel { table_step_m, grid_step_m, track_len_m, edge_width_m,
//             edge_inset_m, tail_sigma_m, tail_weight }  (defaults scaled by beta/610)
//    sweep_freq_hz, sweep_range_margin
//    knot_spacing_m, num_control_points, sample_spacing_m, initial_heading_deg
//    fast_grid_step_m, explore_grid_step_m  (auto-scaled if absent, see below)
//    max_iter, explore_iter, explore_iter_warm
//    coordination_sweeps, init_strategies ["clusters", "greedy"]
//    reallocate, realloc_rounds, max_realloc_trials
// --------------------------------------------------------------------------- //
void curveFromScenario(const J::Value& sc, const Frame& frame, mc::PlannerParams& p) {
    const J::Value& cv = sc["curve"];
    const std::string rep = cv["representation"].text("curvature");
    if (rep == "curvature") p.curve.representation = mc::CurveRepresentation::Curvature;
    else if (rep == "bspline") p.curve.representation = mc::CurveRepresentation::BSpline;
    else throw std::runtime_error("curve.representation '" + rep + "' is not curvature | bspline");

    const J::Value& em = cv["endpoint_mode"];
    if (em.isArray()) {
        p.curve.endpointModes.clear();
        for (const J::Value& m : em.array()) p.curve.endpointModes.push_back(modeFrom(m.str()));
    } else {
        p.curve.endpointModes = {modeFrom(em.text("open"))};
    }
    if (cv["destinations_ned"].isArray()) {
        p.curve.destinations.clear();
        for (const J::Value& d : cv["destinations_ned"].array()) {
            const std::vector<double> v = d.numbers();
            if (v.size() != 2) throw std::runtime_error("curve.destinations_ned entries need [n, e]");
            p.curve.destinations.push_back(frame.toMtl(v[0], v[1]));
        }
    }
    // The kernel calibration, the edge smoothing and the altitude stagger are
    // in metres tuned for the reference sensor (beta = 610 m); a scaled
    // mission gets them scaled by beta / 610 unless it sets them.
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

    // Grids and sample spacings scale with the mission: the reference values
    // (25 m / 50 m grids, 25 m samples, 100 m knots) are for a 5 km map and a
    // 5 km sortie.  Absent keys are scaled by mapSize / 5000 and clamped to
    // the belief resolution, so a 400 m test mission is not optimised on a
    // 16 x 16 grid.
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

// --------------------------------------------------------------------------- //
mc::CellSet cellsFromScenario(const J::Value& sc, const Frame& frame, double cellSize) {
    const J::Value& cells = sc["cells"];
    if (!cells["centers"].isArray()) {
        throw std::runtime_error(
            "scenario has no 'cells.centers'. The host extracts the valid cells and passes them; see the "
            "header of apps/mtl_plan_json.cpp in cpp_planner for why.");
    }
    const J::Array& centers = cells["centers"].array();
    const J::Array* mass = cells["mass"].isArray() ? &cells["mass"].array() : nullptr;
    if (mass && mass->size() != centers.size())
        throw std::runtime_error("scenario cells: 'mass' and 'centers' have different lengths");

    mc::CellSet out;
    const auto m = static_cast<mc::Index>(centers.size());
    out.centers.resize(m, 2);
    out.mass.resize(m);
    for (mc::Index i = 0; i < m; ++i) {
        const J::Array& row = centers[static_cast<std::size_t>(i)].array();
        if (row.size() != 2) throw std::runtime_error("scenario cells: each centre needs [n, e]");
        const mc::Vec2 xy = frame.toMtl(row[0].number(), row[1].number());
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

// --------------------------------------------------------------------------- //
J::Value diagnosticsJson(const mc::AgentTrajectory& traj, const mc::AgentPlan& plan, const mc::PlanningResult& r) {
    J::Object out;
    // --- the keys mtl_plan always writes, with their curve-planner meaning ---
    double info = 0.0;
    for (const mc::Index c : plan.servicedCellIdx) info += r.cells.mass(c);
    const double total = r.cells.mass.sum();
    out["info_mass"]          = J::Value(info);
    out["info_fraction"]      = J::Value(total > 0.0 ? info / total : 0.0);
    out["outer_iters"]        = J::Value(0);      // no reserve bisection: the curve IS the budget
    out["reserve_m"]          = J::Value(0.0);
    out["route_note"]         = J::Value("continuous curve, " + std::string(toString(plan.rep.type)) +
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
    out["scheduled"]         = J::Value(false);   // the gimbal sweeps; nothing is scheduled
    out["feasible"]          = J::Value(plan.feasible);
    out["flown_length_m"]    = J::Value(plan.flownLength);
    out["flight_time_s"]     = J::Value(plan.flightTime);
    out["budget_used_frac"]  = J::Value(plan.budgetUsed);
    out["representation"]    = J::Value(toString(plan.rep.type));
    out["endpoint_mode"]     = J::Value(toString(plan.rep.mode));
    out["endpoint_error_m"]  = J::Value(plan.endpointError);
    out["init_strategy"]     = J::Value(plan.initStrategy);
    out["max_curvature"]     = J::Value(plan.maxKappa);
    out["altitude_m"]        = J::Value(plan.altitude);
    out["swath_half_width_m"] = J::Value(plan.swathHalfWidth);
    out["sweep_amplitude_deg"] = J::Value(mc::rad2deg(plan.sweep.alphaMax));
    out["sweep_freq_hz"]     = J::Value(plan.sweep.freq);
    out["sweep_peak_rate_deg_s"] = J::Value(mc::rad2deg(plan.sweep.peakRate));
    out["tilt_deg"]          = J::Value(mc::rad2deg(plan.sweep.tiltAngle));
    out["nadir_offset_m"]    = J::Value(plan.sweep.standOff);
    out["gimbal_max_deg"]    = J::Value(traj.maxGimbalCmdDeg);
    out["roll_max_deg"]      = J::Value(traj.maxRollDeg);
    out["fast_objective"]    = J::Value(plan.lastOpt.J);
    out["optimizer_iters"]   = J::Value(plan.lastOpt.iters);
    out["optimizer_exit"]    = J::Value(plan.lastOpt.exitMsg);
    out["clusters_selected"] = J::Value(indicesOf(plan.waypointClusters));
    return J::Value(std::move(out));
}

// --------------------------------------------------------------------------- //
J::Value planToJson(const mc::PlanningResult& r, const mc::PlannerParams& params, const J::Value& sc,
                    const Frame& frame, const std::vector<std::string>& agentNames) {
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
        samples["t"] = J::Value(numbersOf(t));
        samples["n"] = J::Value(numbersOf(n));
        samples["e"] = J::Value(numbersOf(e));
        samples["h"] = J::Value(numbersOf(h));
        samples["sensor_n"] = J::Value(numbersOf(sn));
        samples["sensor_e"] = J::Value(numbersOf(se));
        samples["roll"] = J::Value(numbersOf(roll, 5));
        samples["pitch"] = J::Value(numbersOf(pitch, 5));
        samples["yaw"] = J::Value(numbersOf(yaw, 5));
        samples["gimbal"] = J::Value(numbersOf(gim, 5));

        J::Array clusters;
        for (const mc::Index c : plan.waypointClusters) {
            double cn = 0.0, ce = 0.0;
            frame.fromMtl(r.clusters.centroids(c, 0), r.clusters.centroids(c, 1), cn, ce);
            clusters.push_back(J::Value(J::Array{J::Value(cn), J::Value(ce)}));
        }
        double startN = 0.0, startE = 0.0;
        frame.fromMtl(plan.start.x(), plan.start.y(), startN, startE);

        J::Object agent;
        agent["name"] = J::Value(a < agentNames.size() ? agentNames[a] : "agent" + std::to_string(a + 1));
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
        agent["diagnostics"] = diagnosticsJson(traj, plan, r);
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
    gen["name"] = J::Value(kGeneratorName);
    gen["library"] = J::Value("mtl_curve::planner");
    gen["mode"] = J::Value("single_axis_sweep");

    std::vector<double> flown, budgets;
    for (const mc::AgentPlan& p : r.plans) {
        flown.push_back(p.flownLength);
        budgets.push_back(p.budget);
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
    team["flown_length_m"] = J::Value(numbersOf(flown));
    team["budgets_m"] = J::Value(numbersOf(budgets));

    J::Array hist, stages;
    for (std::size_t i = 0; i < r.team.Jhist.size(); ++i) {
        hist.push_back(J::Value(r.team.Jhist[i]));
        stages.push_back(J::Value(r.team.stageNames[i]));
    }
    J::Object curve;
    curve["representation"] = J::Value(toString(params.curve.representation));
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

    J::Object out;
    out["schema"] = J::Value(kPlanSchema);
    out["generator"] = J::Value(std::move(gen));
    out["mission"] = sc["mission"];
    out["dt"] = J::Value(params.dt);
    out["agents"] = J::Value(std::move(agents));
    out["cells"] = J::Value(std::move(cellsOut));
    out["meta"] = J::Value(std::move(meta));
    return J::Value(std::move(out));
}

void usage() {
    std::cout << "mtlc_plan - plan a search mission with the parameterized-curve planner\n\n"
                 "  mtlc_plan --scenario SCENARIO.json [--out PLAN.json]\n\n"
                 "  --scenario PATH   mtl.scenario/1 JSON; '-' reads stdin\n"
                 "  --out PATH        mtl.plan/1 JSON;     '-' writes stdout (default)\n"
                 "  --verbose         let the planner narrate\n"
                 "  --help\n\n"
                 "Same interchange as cpp_planner's mtl_plan; curve options in an optional \"curve\" block.\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenarioPath, outPath = "-";
    bool verbose = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) scenarioPath = argv[++i];
        else if (arg == "--out" && i + 1 < argc) outPath = argv[++i];
        else if (arg == "--verbose" || arg == "-v") verbose = true;
        else if (arg == "--help" || arg == "-h") { usage(); return 0; }
        else {
            std::cerr << "unknown argument: " << arg << "\n";
            usage();
            return 2;
        }
    }
    if (scenarioPath.empty()) {
        std::cerr << "error: --scenario is required\n";
        usage();
        return 2;
    }
    try {
        const J::Value sc = J::parse(readAll(scenarioPath));
        const std::string schema = sc["schema"].text("");
        if (schema != kScenarioSchema)
            throw std::runtime_error("scenario schema is '" + schema + "', expected '" + kScenarioSchema + "'");

        const J::Value& area = sc["mission"]["area"];
        const double size = area["size_m"].num(0.0);
        if (!(size > 0.0)) throw std::runtime_error("scenario mission.area.size_m must be positive");
        std::vector<double> center{0.0, 0.0};
        if (area["center_ned"].isArray()) center = area["center_ned"].numbers();
        Frame frame;
        frame.nMin = center[0] - size / 2.0;
        frame.eMin = center[1] - size / 2.0;

        const J::Value& teamAgents = sc["team"]["agents"];
        if (!teamAgents.isArray() || teamAgents.array().empty())
            throw std::runtime_error("scenario needs at least one entry in team.agents");
        std::vector<mc::Vec2> starts;
        std::vector<std::string> names;
        for (const J::Value& a : teamAgents.array()) {
            const std::vector<double> s = a["start_ned"].numbers();
            if (s.size() != 2) throw std::runtime_error("each team.agents entry needs start_ned [n, e]");
            starts.push_back(frame.toMtl(s[0], s[1]));
            names.push_back(a["name"].text("agent" + std::to_string(names.size() + 1)));
        }

        mc::PlannerParams params = paramsFromScenario(sc, static_cast<int>(starts.size()));
        curveFromScenario(sc, frame, params);
        if (verbose) params.verbose = true;

        const mc::CellSet cells = cellsFromScenario(sc, frame, params.targetCellSize);
        if (cells.empty()) throw std::runtime_error("scenario contains no cells to plan over");

        mc::Planner planner(params);
        const mc::PlanningResult result = planner.planFromCells(cells, starts);

        writeAll(outPath, planToJson(result, planner.params(), sc, frame, names).dump());
        if (outPath != "-") {
            std::cerr << "mtlc_plan: " << result.trajectories.size() << " agent(s), " << result.numSteps()
                      << " steps, fast residual " << result.team.Jfinal << ", cells swept " << result.team.info
                      << " of " << result.team.infoTotal << " (" << 100.0 * result.team.infoFraction << "%) -> "
                      << outPath << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "mtlc_plan error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
