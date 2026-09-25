// =============================================================================
//  search_problem.cpp — scenario JSON -> mtl::Planner inputs, and the planner's
//  team result -> one agent's map-frame track.
//
//  paramsFromScenario()/cellsFromScenario() follow cpp_planner/apps/
//  mtl_plan_json.cpp field for field, so a scenario plans identically through
//  the stock `mtl_plan` tool and through this ROS node.
// =============================================================================
#include "mtl_search_planner/search_problem.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iterator>
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
    out.cells  = cellsFromScenario(sc, out.frame, out.params.targetCellSize,
                                   out.params.minimumBeliefMass);
    if (out.cells.empty()) throw std::runtime_error("scenario contains no cells to plan over");
    return out;
}

SearchProblem loadScenario(const std::string& path) { return parseScenario(readFile(path)); }

mtl::PlanningResult solve(const SearchProblem& problem) {
    mtl::Planner planner(problem.params);
    return planner.planFromCells(problem.cells, problem.startsMtl);
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
    out["samples"] = J::Value(std::move(samples));
    return J::Value(std::move(out)).dump();
}

}  // namespace mtl_search
