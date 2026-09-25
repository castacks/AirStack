// =============================================================================
//  apps/mtl_plan_json.cpp   ->   the `mtl_plan` tool
//
//  The bridge between the planner and any simulator that can write a file.
//
//      mtl_plan --scenario scenario.json --out plan.json
//      mtl_plan --scenario -  --out -            # stdin to stdout
//
//  Reads a scenario (mtl.scenario/1): the mission geometry, the platform, the
//  sensor, the team, and the valid cells the HOST extracted.  Writes a plan
//  (mtl.plan/1): one dense, time-parameterised track per agent plus the
//  boresight ground point at every sample, and the audit trail behind it.
//
//  WHY CELLS AND NOT A BELIEF GRID.  The host simulation owns the scenario -
//  it has to, because it is also the thing that renders the world and scores
//  the run.  If the planner regenerated the prior from a seed we would have two
//  beliefs that agree only as long as two random number generators agree, which
//  across MATLAB, std::mt19937_64 and numpy they never will.  Passing the
//  extracted cells makes the disagreement impossible instead of unlikely, and
//  it is the integration path mtl::Planner::planFromCells exists for.
//
//  FRAMES.  The scenario and the plan are in the host's **mission NED**
//      n North, e East, d Down, yaw 0 = North and positive clockwise.
//  mtl works in **x East, y North, z Up**, yaw positive counter-clockwise from
//  East, over a map spanning [0, mapSize].  The two conversions live in
//  toMtl() / fromMtl() below and nowhere else in this file.
// =============================================================================
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "mtl/planner.hpp"
#include "json_mini.hpp"

namespace J = jsonmini;

namespace {

constexpr const char* kScenarioSchema = "mtl.scenario/1";
constexpr const char* kPlanSchema     = "mtl.plan/1";
constexpr const char* kGeneratorName  = "mtl_cpp";

// --------------------------------------------------------------------------- //
//  Frame conversion.  ONE place.
// --------------------------------------------------------------------------- //
struct Frame {
    double nMin = 0.0;  ///< mission NED north of the map's y = 0 edge
    double eMin = 0.0;  ///< mission NED east  of the map's x = 0 edge

    mtl::Vec2 toMtl(double n, double e) const { return {e - eMin, n - nMin}; }
    void fromMtl(double x, double y, double& n, double& e) const {
        n = y + nMin;
        e = x + eMin;
    }
    /// mtl yaw is CCW from East; NED yaw is CW from North.
    static double yawToNed(double yawMtl) { return mtl::kPi / 2.0 - yawMtl; }
};

// --------------------------------------------------------------------------- //
std::string readAll(const std::string& path) {
    if (path == "-") {
        return std::string(std::istreambuf_iterator<char>(std::cin),
                           std::istreambuf_iterator<char>());
    }
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

J::Array numbersOf(const mtl::VecX& v, int ndigits = 3) {
    const double scale = std::pow(10.0, ndigits);
    J::Array out;
    out.reserve(static_cast<std::size_t>(v.size()));
    for (mtl::Index i = 0; i < v.size(); ++i) {
        out.push_back(J::Value(std::round(v(i) * scale) / scale));
    }
    return out;
}

J::Array numbersOf(const std::vector<double>& v) {
    J::Array out;
    out.reserve(v.size());
    for (const double d : v) out.push_back(J::Value(d));
    return out;
}

J::Array indicesOf(const std::vector<mtl::Index>& v) {
    J::Array out;
    out.reserve(v.size());
    for (const mtl::Index i : v) out.push_back(J::Value(static_cast<double>(i)));
    return out;
}

// --------------------------------------------------------------------------- //
//  scenario -> PlannerParams
// --------------------------------------------------------------------------- //
mtl::PlannerParams paramsFromScenario(const J::Value& sc, const Frame& frame, int numAgents) {
    mtl::PlannerParams p;

    const J::Value& area = sc["mission"]["area"];
    p.mapSize  = area["size_m"].num(p.mapSize);
    p.cellSize = area["belief_res_m"].num(p.cellSize);

    const J::Value& air = sc["aircraft"];
    p.droneAltitude = air["altitude_m"].num(p.droneAltitude);
    p.avgDroneSpeed = air["speed_mps"].num(p.avgDroneSpeed);
    p.minTurnRadius = air["min_turn_radius_m"].num(p.minTurnRadius);
    p.dt            = air["dt"].num(p.dt);
    p.dubins.stepSize = air["dubins_step_m"].num(p.dubins.stepSize);

    const J::Value& map = sc["mapping"];
    p.targetCellSize        = map["target_cell_size_m"].num(p.targetCellSize);
    // Informational here - the host passes the cells it already extracted.  The
    // retired mean_information_thresh key, if an older host still sends it, is
    // ignored.
    p.minimumBeliefMass     = map["minimum_belief_mass"].num(p.minimumBeliefMass);
    p.maxClusterRadius      = map["max_cluster_radius_m"].num(p.maxClusterRadius);
    p.cluster.kmeansReplicates = static_cast<int>(map["kmeans_replicates"].num(p.cluster.kmeansReplicates));
    p.cluster.kmeansMaxIter    = static_cast<int>(map["kmeans_max_iter"].num(p.cluster.kmeansMaxIter));

    const J::Value& sensor = sc["sensor"];
    p.fov               = mtl::deg2rad(sensor["fov_deg"].num(60.0));
    p.singleAxisGimbal  = sensor["single_axis_gimbal"].flag(p.singleAxisGimbal);
    p.sensorTiltAngle   = mtl::deg2rad(sensor["tilt_deg"].num(mtl::rad2deg(p.sensorTiltAngle)));
    const double maxSlant = sensor["max_slant_range_m"].num(p.gimbal.maxSlantRange);
    p.gimbal.maxSlantRange = maxSlant;
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

    // The altitude box the gimbal scheduler is allowed to climb inside defaults
    // to +/- 50% of cruise: the reference 150/450 around 300 m is meaningless at
    // a scaled altitude, and a box that excludes cruise makes every step
    // altitude-limited.
    p.gimbal.hMin = sc["gimbal"]["h_min_m"].num(0.5 * p.droneAltitude);
    p.gimbal.hMax = sc["gimbal"]["h_max_m"].num(1.5 * p.droneAltitude);
    p.gimbal.minTurnRadius = p.minTurnRadius;
    p.gimbal.enableRepairLoop = sc["gimbal"]["enable_repair_loop"].flag(p.gimbal.enableRepairLoop);
    p.gimbal.targetTol = sc["gimbal"]["target_tol_m"].num(
        std::max(1.0, 0.025 * map["target_cell_size_m"].num(p.targetCellSize)));

    p.verbose                = sc["verbose"].flag(false);
    p.verifyGeometry         = sc["verify_geometry"].flag(true);
    p.verifyGeometryVerbose  = sc["verify_geometry_verbose"].flag(false);
    p.budget.verbose         = p.verbose;
    p.rngSeed = static_cast<std::uint64_t>(sc["mission"]["seed"].num(21.0));

    // Optional solver overrides, for a sweep that wants to move one knob.
    const J::Value& ov = sc["solver"];
    p.budget.orienteering.nStarts = static_cast<int>(ov["n_starts"].num(p.budget.orienteering.nStarts));
    p.budget.maxOuterIter  = static_cast<int>(ov["max_outer_iter"].num(p.budget.maxOuterIter));
    p.budget.reserveFrac0  = ov["reserve_frac0"].num(p.budget.reserveFrac0);
    p.budget.cellRefine    = ov["cell_refine"].flag(p.budget.cellRefine);
    p.budget.reallocate    = ov["reallocate"].flag(p.budget.reallocate);
    p.budget.maxCellCandidates =
        static_cast<int>(ov["max_cell_candidates"].num(p.budget.maxCellCandidates));

    (void)frame;
    return p;
}

// --------------------------------------------------------------------------- //
mtl::CellSet cellsFromScenario(const J::Value& sc, const Frame& frame, double cellSize) {
    const J::Value& cells = sc["cells"];
    if (!cells["centers"].isArray()) {
        throw std::runtime_error(
            "scenario has no 'cells.centers'. The host extracts the valid cells and "
            "passes them; see the header of apps/mtl_plan_json.cpp for why.");
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
    out.cellSize = cellSize;
    out.area = mtl::VecX::Constant(m, cellSize * cellSize);
    out.nPix = mtl::VecX::Constant(m, 1.0);
    out.meanBelief = out.mass / std::max(cellSize * cellSize, 1e-9);
    out.peakBelief = out.meanBelief;
    out.retainedMass = out.mass.sum();
    out.totalMapMass = sc["cells"]["total_map_mass"].num(out.retainedMass);
    out.massNorm = out.retainedMass > 0 ? (out.mass / out.retainedMass).eval() : out.mass;
    out.gridRes = mtl::Vec2(sc["mission"]["area"]["belief_res_m"].num(1.0),
                            sc["mission"]["area"]["belief_res_m"].num(1.0));
    return out;
}

// --------------------------------------------------------------------------- //
J::Value diagnosticsJson(const mtl::AgentTrajectory& traj, const mtl::AgentPlan& plan) {
    const mtl::GimbalDiagnostics& d = traj.diagnostics;
    J::Object out;
    out["scheduled"]        = J::Value(traj.scheduled);
    out["feasible"]         = J::Value(plan.feasible);
    out["flown_length_m"]   = J::Value(plan.flownLength);
    out["flight_time_s"]    = J::Value(plan.flightTime);
    out["budget_used_frac"] = J::Value(plan.budgetUsed);
    out["outer_iters"]      = J::Value(plan.outerIters);
    out["reserve_m"]        = J::Value(plan.reserve);
    out["info_mass"]        = J::Value(plan.score.info);
    out["info_fraction"]    = J::Value(plan.score.infoFraction);
    out["route_note"]       = J::Value(plan.routeInfo.note);
    out["refine_note"]      = J::Value(plan.refineInfo.note);
    out["extension_note"]   = J::Value(plan.extInfo.note);
    out["extension_runout_m"] = J::Value(plan.extInfo.extendDist);
    out["extension_extra_m"] = J::Value(plan.extInfo.extraDist);
    out["clusters_selected"] = J::Value(indicesOf(plan.selClusters));
    out["clusters_dropped"]  = J::Value(indicesOf(plan.droppedClusters));
    if (traj.scheduled) {
        out["gimbal_targets"]         = J::Value(static_cast<double>(d.nTargets));
        out["gimbal_targets_hit"]     = J::Value(static_cast<double>(d.nTargetsHit));
        out["gimbal_coverage"]        = J::Value(d.targetCoverage);
        out["miss_never_abeam"]       = J::Value(static_cast<double>(d.nMissNeverAbeam));
        out["miss_out_of_reach"]      = J::Value(static_cast<double>(d.nMissOutOfReach));
        out["miss_double_booked"]     = J::Value(static_cast<double>(d.nMissDoubleBooked));
        out["slant_range_max_m"]      = J::Value(d.slantRangeMax);
        out["look_angle_max_deg"]     = J::Value(d.lookAngleMaxDeg);
        out["gimbal_max_deg"]         = J::Value(d.gimbalMaxDeg);
        out["gimbal_rate_max_deg_s"]  = J::Value(d.gimbalRateMaxDeg);
        out["roll_max_deg"]           = J::Value(d.rollMaxDeg);
        out["pitch_max_deg"]          = J::Value(d.pitchMaxDeg);
        out["tilt_deg"]               = J::Value(d.tiltAngleDeg);
        out["nadir_offset_m"]         = J::Value(d.nadirOffsetM);
    }
    return J::Value(std::move(out));
}

// --------------------------------------------------------------------------- //
J::Value planToJson(const mtl::PlanningResult& r, const mtl::PlannerParams& params,
                    const J::Value& sc, const Frame& frame,
                    const std::vector<std::string>& agentNames) {
    const mtl::Index steps = r.numSteps();

    J::Array agents;
    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const mtl::AgentTrajectory& traj = r.trajectories[a];
        const mtl::AgentPlan&       plan = r.plans[a];

        mtl::VecX t(steps), n(steps), e(steps), h(steps);
        mtl::VecX sn(steps), se(steps), roll(steps), pitch(steps), yaw(steps);
        for (mtl::Index k = 0; k < steps; ++k) {
            t(k) = r.timeVec(k);
            double nn = 0.0, ee = 0.0;
            frame.fromMtl(traj.drone(k, 0), traj.drone(k, 1), nn, ee);
            n(k) = nn;
            e(k) = ee;
            h(k) = traj.drone(k, 2);
            frame.fromMtl(traj.sensor(k, 0), traj.sensor(k, 1), nn, ee);
            sn(k) = nn;
            se(k) = ee;
            roll(k)  = traj.rpy(k, 0);
            pitch(k) = traj.rpy(k, 1);
            yaw(k)   = Frame::yawToNed(traj.rpy(k, 2));
        }

        J::Object samples;
        samples["t"] = J::Value(numbersOf(t));
        samples["n"] = J::Value(numbersOf(n));
        samples["e"] = J::Value(numbersOf(e));
        samples["h"] = J::Value(numbersOf(h));
        samples["sensor_n"] = J::Value(numbersOf(sn));
        samples["sensor_e"] = J::Value(numbersOf(se));
        samples["roll"]  = J::Value(numbersOf(roll, 5));
        samples["pitch"] = J::Value(numbersOf(pitch, 5));
        samples["yaw"]   = J::Value(numbersOf(yaw, 5));

        // The cells the GIMBAL actually observed, not the ones the route was
        // drawn through: a single-axis mount can be routed past a centre and
        // still never bring it abeam, and a plan that claims the optimistic
        // number is a plan that scores itself.
        const std::vector<mtl::Index>& realized =
            traj.realizedCellIdx.empty() ? plan.servicedCellIdx : traj.realizedCellIdx;

        J::Array clusters;
        for (const mtl::Index c : plan.selClusters) {
            double cn = 0.0, ce = 0.0;
            frame.fromMtl(r.clusters.centroids(c, 0), r.clusters.centroids(c, 1), cn, ce);
            clusters.push_back(J::Value(J::Array{J::Value(cn), J::Value(ce)}));
        }

        double startN = 0.0, startE = 0.0;
        if (plan.droneRoute.rows() > 0) {
            frame.fromMtl(plan.droneRoute(0, 0), plan.droneRoute(0, 1), startN, startE);
        }

        J::Object agent;
        agent["name"] = J::Value(a < agentNames.size() ? agentNames[a]
                                                       : "agent" + std::to_string(a + 1));
        agent["start_ned"] = J::Value(J::Array{J::Value(startN), J::Value(startE)});
        const J::Value& teamAgents = sc["team"]["agents"];
        if (teamAgents.isArray() && a < teamAgents.array().size()) {
            const J::Value& home = teamAgents.array()[a]["home_ned"];
            if (home.isArray()) agent["home_ned"] = home;
        }
        agent["budget_m"]       = J::Value(plan.budget);
        agent["path_length_m"]  = J::Value(plan.flownLength);
        agent["duration_s"]     = J::Value(plan.flightTime);
        agent["serviced_cells"] = J::Value(indicesOf(realized));
        agent["planned_cells"]  = J::Value(indicesOf(plan.servicedCellIdx));
        agent["clusters"]       = J::Value(std::move(clusters));
        agent["samples"]        = J::Value(std::move(samples));
        agent["diagnostics"]    = diagnosticsJson(traj, plan);
        agents.push_back(J::Value(std::move(agent)));
    }

    J::Array centers;
    J::Array mass;
    for (mtl::Index i = 0; i < r.cells.size(); ++i) {
        double cn = 0.0, ce = 0.0;
        frame.fromMtl(r.cells.centers(i, 0), r.cells.centers(i, 1), cn, ce);
        centers.push_back(J::Value(J::Array{J::Value(cn), J::Value(ce)}));
        mass.push_back(J::Value(r.cells.mass(i)));
    }
    J::Object cellsOut;
    cellsOut["centers"] = J::Value(std::move(centers));
    cellsOut["mass"]    = J::Value(std::move(mass));

    J::Object gen;
    gen["name"]    = J::Value(kGeneratorName);
    gen["library"] = J::Value("mtl::planner");
    gen["mode"]    = J::Value(params.singleAxisGimbal ? "single_axis" : "multi_axis");

    J::Object team;
    team["info_mass"]      = J::Value(r.team.info);
    team["info_total"]     = J::Value(r.team.infoTotal);
    team["info_fraction"]  = J::Value(r.team.infoFraction);
    team["clusters_reached"]   = J::Value(static_cast<double>(r.team.reachedClusters.size()));
    team["clusters_unreached"] = J::Value(static_cast<double>(r.team.unreachedClusters.size()));
    team["cells_serviced"]     = J::Value(static_cast<double>(r.team.servicedCellIdx.size()));
    team["cells_unserviced"]   = J::Value(static_cast<double>(r.team.unservicedCellIdx.size()));
    team["realloc_rounds"]     = J::Value(r.team.rounds);
    team["flown_length_m"]     = J::Value(numbersOf(r.team.flownLength));
    team["budgets_m"]          = J::Value(numbersOf(r.team.budgets));

    J::Object meta;
    meta["team"]             = J::Value(std::move(team));
    meta["budget_dist_m"]    = J::Value(params.budgetDist());
    meta["sensor_standoff_m"] = J::Value(params.sensorStandOff());
    meta["single_axis"]      = J::Value(params.singleAxisGimbal);
    meta["steps"]            = J::Value(static_cast<double>(steps));

    J::Object out;
    out["schema"]    = J::Value(kPlanSchema);
    out["generator"] = J::Value(std::move(gen));
    out["mission"]   = sc["mission"];
    out["dt"]        = J::Value(params.dt);
    out["agents"]    = J::Value(std::move(agents));
    out["cells"]     = J::Value(std::move(cellsOut));
    out["meta"]      = J::Value(std::move(meta));
    return J::Value(std::move(out));
}

void usage() {
    std::cout <<
        "mtl_plan - plan a search mission from a scenario file\n\n"
        "  mtl_plan --scenario SCENARIO.json [--out PLAN.json]\n\n"
        "  --scenario PATH   mtl.scenario/1 JSON; '-' reads stdin\n"
        "  --out PATH        mtl.plan/1 JSON;     '-' writes stdout (default)\n"
        "  --verbose         let the planner narrate\n"
        "  --help\n\n"
        "Both files are in the host's mission NED frame. See the file header.\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string scenarioPath;
    std::string outPath = "-";
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--scenario" && i + 1 < argc) {
            scenarioPath = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            outPath = argv[++i];
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
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
        if (schema != kScenarioSchema) {
            throw std::runtime_error("scenario schema is '" + schema + "', expected '" +
                                     kScenarioSchema + "'");
        }

        // The map spans [0, mapSize] in mtl coordinates; the host's area is
        // centred wherever it likes, so anchor the frame on its south-west corner.
        const J::Value& area = sc["mission"]["area"];
        const double size = area["size_m"].num(0.0);
        if (!(size > 0.0)) throw std::runtime_error("scenario mission.area.size_m must be positive");
        std::vector<double> center{0.0, 0.0};
        if (area["center_ned"].isArray()) center = area["center_ned"].numbers();
        Frame frame;
        frame.nMin = center[0] - size / 2.0;
        frame.eMin = center[1] - size / 2.0;

        const J::Value& teamAgents = sc["team"]["agents"];
        if (!teamAgents.isArray() || teamAgents.array().empty()) {
            throw std::runtime_error("scenario needs at least one entry in team.agents");
        }
        std::vector<mtl::Vec2>   starts;
        std::vector<std::string> names;
        for (const J::Value& a : teamAgents.array()) {
            const std::vector<double> s = a["start_ned"].numbers();
            if (s.size() != 2) throw std::runtime_error("each team.agents entry needs start_ned [n, e]");
            starts.push_back(frame.toMtl(s[0], s[1]));
            names.push_back(a["name"].text("agent" + std::to_string(names.size() + 1)));
        }

        mtl::PlannerParams params = paramsFromScenario(sc, frame, static_cast<int>(starts.size()));
        if (verbose) { params.verbose = true; params.budget.verbose = true; }

        const mtl::CellSet cells = cellsFromScenario(sc, frame, params.targetCellSize);
        if (cells.empty()) throw std::runtime_error("scenario contains no cells to plan over");

        mtl::Planner planner(params);
        const mtl::PlanningResult result = planner.planFromCells(cells, starts);

        writeAll(outPath, planToJson(result, params, sc, frame, names).dump());
        if (outPath != "-") {
            std::cerr << "mtl_plan: " << result.trajectories.size() << " agent(s), "
                      << result.numSteps() << " steps, info "
                      << result.team.info << " of " << result.team.infoTotal << " ("
                      << 100.0 * result.team.infoFraction << "%) -> " << outPath << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "mtl_plan error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
