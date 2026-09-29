// =============================================================================
//  scenario.cpp — `mtl.scenario/1` JSON -> tigris_search::Scenario (world ENU).
// =============================================================================
#include "tigris_search_planner/scenario.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace J = tigris_json;

namespace tigris_search {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::vector<double> axis(double lo, double hi, double step) {
    const int count = static_cast<int>(std::floor((hi - lo) / step + 1e-9)) + 1;
    std::vector<double> out(static_cast<std::size_t>(std::max(count, 0)));
    for (int k = 0; k < count; ++k) out[static_cast<std::size_t>(k)] = lo + k * step;
    return out;
}

}  // namespace

double DetectionModel::prob(double r) const {
    if (r > beta) return pOut;
    return 1.0 / (a + std::exp(b * (r - c)));
}

std::string readTextFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot open file: " + path);
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

int Scenario::agentIndex(const std::string& agentName) const {
    for (std::size_t i = 0; i < agents.size(); ++i) {
        if (agents[i].name == agentName) return static_cast<int>(i);
    }
    return -1;
}

double Scenario::budgetDistance() const {
    double b = maxFlightDistance;
    if (std::isfinite(maxFlightTime)) b = std::min(b, maxFlightTime * speed);
    return b;
}

PriorRaster priorFromScenario(const J::Value& sc) {
    PriorRaster out;
    const J::Value& area = sc["mission"]["area"];
    const J::Value& bel = sc["airstack"]["belief"];
    if (!bel["bumps"].isArray() || bel["bumps"].array().empty()) return out;
    const double half = area["size_m"].num(400.0) / 2.0;
    double cn = 0.0, ce = 0.0;
    if (area["center_ned"].isArray()) {
        const auto c = area["center_ned"].numbers();
        if (c.size() == 2) { cn = c[0]; ce = c[1]; }
    }
    out.res = area["belief_res_m"].num(2.0);
    const std::vector<double> nAxis = axis(cn - half, cn + half, out.res);
    const std::vector<double> eAxis = axis(ce - half, ce + half, out.res);
    const double cap = bel["belief_cap"].num(0.85);
    const double floorV = bel["base_uncertainty"].num(0.0);
    const std::size_t ny = nAxis.size(), nx = eAxis.size();
    std::vector<double> rows(ny * nx, 0.0);
    std::vector<double> gn(ny), ge(nx);
    for (const J::Value& b : bel["bumps"].array()) {
        const double bn = b["n"].number(), be = b["e"].number();
        const double sn = b["sigma_n"].number(), se = b["sigma_e"].number();
        const double amp = b["amplitude"].num(0.4);
        for (std::size_t i = 0; i < ny; ++i) gn[i] = std::exp(-0.5 * std::pow((nAxis[i] - bn) / sn, 2));
        for (std::size_t j = 0; j < nx; ++j) ge[j] = std::exp(-0.5 * std::pow((eAxis[j] - be) / se, 2));
        for (std::size_t i = 0; i < ny; ++i) {
            if (gn[i] < 1e-12) continue;
            const double a = amp * gn[i];
            double* row = &rows[i * nx];
            for (std::size_t j = 0; j < nx; ++j) row[j] += a * ge[j];
        }
    }
    out.raw.resize(rows.size());
    for (std::size_t k = 0; k < rows.size(); ++k) {
        double v = std::min(rows[k], cap);
        if (floorV > 0.0) v = std::max(v, floorV);
        out.raw[k] = v;
    }
    const double total = std::accumulate(out.raw.begin(), out.raw.end(), 0.0);
    if (!(total > 0.0)) {
        out.raw.clear();
        return out;
    }
    out.norm.resize(out.raw.size());
    for (std::size_t k = 0; k < out.raw.size(); ++k) out.norm[k] = out.raw[k] / total;
    out.xs = eAxis;  // world ENU x = east
    out.ys = nAxis;  // world ENU y = north
    return out;
}

Scenario parseScenario(const std::string& jsonText) {
    Scenario s;
    s.raw = J::parse(jsonText);
    const J::Value& sc = s.raw;
    const std::string schema = sc["schema"].text("");
    if (schema != "mtl.scenario/1") {
        throw std::runtime_error("scenario schema is '" + schema + "', expected 'mtl.scenario/1'");
    }
    const J::Value& area = sc["mission"]["area"];
    s.size = area["size_m"].num(0.0);
    if (!(s.size > 0.0)) throw std::runtime_error("scenario mission.area.size_m must be positive");
    double cn = 0.0, ce = 0.0;
    if (area["center_ned"].isArray()) {
        const auto c = area["center_ned"].numbers();
        if (c.size() != 2) throw std::runtime_error("scenario mission.area.center_ned needs [n, e]");
        cn = c[0];
        ce = c[1];
    }
    s.xMin = ce - s.size / 2.0;
    s.xMax = ce + s.size / 2.0;
    s.yMin = cn - s.size / 2.0;
    s.yMax = cn + s.size / 2.0;
    s.name = sc["mission"]["name"].text("search");
    s.seed = static_cast<std::uint64_t>(sc["mission"]["seed"].num(21.0));

    const J::Value& air = sc["aircraft"];
    s.altitude      = air["altitude_m"].num(s.altitude);
    s.speed         = air["speed_mps"].num(s.speed);
    s.minTurnRadius = air["min_turn_radius_m"].num(s.minTurnRadius);
    s.dt            = air["dt"].num(s.dt);
    s.dubinsStep    = air["dubins_step_m"].num(s.dubinsStep);
    if (!(s.speed > 0.0) || !(s.minTurnRadius > 0.0) || !(s.dt > 0.0)) {
        throw std::runtime_error("scenario aircraft: speed_mps, min_turn_radius_m and dt must be positive");
    }

    const J::Value& sensor = sc["sensor"];
    s.fovRad  = sensor["fov_deg"].num(60.0) * kPi / 180.0;
    s.tiltRad = sensor["tilt_deg"].num(30.0) * kPi / 180.0;
    const J::Value& det = sensor["detection"];
    s.det.a         = det["a"].num(s.det.a);
    s.det.b         = det["b"].num(s.det.b);
    s.det.c         = det["c"].num(s.det.c);
    s.det.beta      = det["beta"].num(s.det.beta);
    s.det.pOut      = det["p_out_of_range"].num(s.det.pOut);
    s.det.threshold = det["threshold"].num(s.det.threshold);
    s.det.dtRef     = det["dt_ref_s"].num(s.det.dtRef);

    // TIGRIS-only gimbal sweep (absent = body-fixed camera, the original baseline)
    const J::Value& ga = sc["airstack"]["gimbal_actuation"];
    if (ga.isObject()) {
        s.gimbal.enabled = ga["enabled"].flag(false);
        s.gimbal.rate = ga["sweep_rate_deg_s"].num(30.0) * kPi / 180.0;
        s.gimbal.amplitude = ga["sweep_amplitude_deg"].num(45.0) * kPi / 180.0;
        if (s.gimbal.enabled) {
            if (!(s.gimbal.rate > 0.0) || !std::isfinite(s.gimbal.rate)) {
                throw std::runtime_error("scenario airstack.gimbal_actuation.sweep_rate_deg_s must be > 0");
            }
            if (!(s.gimbal.amplitude > 0.0) || !(s.gimbal.amplitude < kPi / 2.0)) {
                throw std::runtime_error("scenario airstack.gimbal_actuation.sweep_amplitude_deg must be in (0, 90)");
            }
        }
    }
    const J::Value& simG = sc["airstack"]["sim_gimbal"];
    s.gimbalSlewRate = simG["slew_rate_deg_s"].num(120.0) * kPi / 180.0;
    if (simG["roll_limit_deg"].isArray()) {
        const auto rl = simG["roll_limit_deg"].numbers();
        if (rl.size() == 2) s.gimbalRollLimit = std::min(std::fabs(rl[0]), std::fabs(rl[1])) * kPi / 180.0;
    }

    const J::Value& team = sc["team"];
    s.maxFlightTime     = team["max_flight_time_s"].numOrInf();
    s.maxFlightDistance = team["max_flight_distance_m"].numOrInf();
    if (!team["agents"].isArray() || team["agents"].array().empty()) {
        throw std::runtime_error("scenario needs at least one entry in team.agents");
    }
    for (const J::Value& a : team["agents"].array()) {
        AgentSpec ag;
        ag.name = a["name"].text("agent" + std::to_string(s.agents.size() + 1));
        const auto st = a["start_ned"].numbers();
        if (st.size() != 2) throw std::runtime_error("agent '" + ag.name + "': start_ned needs [n, e]");
        ag.startN = st[0];
        ag.startE = st[1];
        const auto h = a["home_ned"].numbers();
        ag.homeN = h.size() == 2 ? h[0] : ag.startN;
        ag.homeE = h.size() == 2 ? h[1] : ag.startE;
        const double dz = a["altitude_offset_m"].num(0.0);
        ag.altitudeOffset = std::isfinite(dz) ? dz : 0.0;
        s.agents.push_back(ag);
    }

    s.cellSize = sc["mapping"]["target_cell_size_m"].num(s.cellSize);
    const J::Value& cells = sc["cells"];
    if (cells["centers"].isArray()) {
        const J::Array& centers = cells["centers"].array();
        const J::Array* mass = cells["mass"].isArray() ? &cells["mass"].array() : nullptr;
        for (std::size_t i = 0; i < centers.size(); ++i) {
            const auto c = centers[i].numbers();
            if (c.size() != 2) throw std::runtime_error("scenario cells: each centre needs [n, e]");
            s.cellX.push_back(c[1]);
            s.cellY.push_back(c[0]);
            s.cellMass.push_back(mass && i < mass->size() ? (*mass)[i].number() : 1.0);
        }
    }
    s.prior = priorFromScenario(sc);
    if (s.prior.empty()) {
        throw std::runtime_error("scenario carries no airstack.belief.bumps: TIGRIS needs the prior raster "
                                 "(regenerate the bundle with scripts/tigris_generate_scenario.py)");
    }
    return s;
}

Scenario loadScenario(const std::string& path) { return parseScenario(readTextFile(path)); }

}  // namespace tigris_search
