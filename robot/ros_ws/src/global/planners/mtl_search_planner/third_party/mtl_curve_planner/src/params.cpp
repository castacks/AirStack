#include "mtl_curve/params.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

#include "mtl_curve/quadrotor/flatness.hpp"

namespace mtl::curve {
namespace {

[[noreturn]] void fail(const std::string& what) {
    throw std::invalid_argument("mtl::curve::PlannerParams: " + what);
}

}  // namespace

void PlannerParams::finalize() {
    // One value, several readers: pushed down rather than duplicated.
    curve.maxCurvature = 1.0 / minTurnRadius;
    curve.denseStep    = avgDroneSpeed * dt;
    gimbal.dt          = dt;
    if (!curve.controlPointBox) curve.controlPointBox = Vec2(-0.2 * mapSize, 1.2 * mapSize);
    optimizer.verbose  = verbose;
    team.orienteering.verbose = false;
}

void PlannerParams::validate() const {
    if (!(mapSize > 0.0)) fail("mapSize must be positive");
    if (!(cellSize > 0.0)) fail("cellSize must be positive");
    if (numAgents < 1) fail("numAgents must be at least 1");
    if (!(droneAltitude > 0.0)) fail("droneAltitude must be positive");
    if (!(avgDroneSpeed > 0.0)) fail("avgDroneSpeed must be positive");
    if (!(dt > 0.0)) fail("dt must be positive");
    if (!(minTurnRadius > 0.0)) fail("minTurnRadius must be positive");
    if (!(targetCellSize > 0.0)) fail("targetCellSize must be positive");
    if (!(minimumBeliefMass >= 0.0) || minimumBeliefMass >= 1.0)
        fail("minimumBeliefMass is a per-cell probability and must be in [0, 1)");
    if (!(maxClusterRadius > 0.0)) fail("maxClusterRadius must be positive");
    if (!(fov > 0.0) || fov >= kPi) fail("fov must be in (0, pi)");
    if (detectionThreshold <= 0.0 || detectionThreshold > 1.0) fail("detectionThreshold must be in (0, 1]");
    // Unlike cpp_planner an infinite budget has no meaning here: the curve IS
    // the budget.
    if (!(budgetDist() > 0.0) || !std::isfinite(budgetDist()))
        fail("the curve planner needs a finite, positive budget (maxFlightTime or maxFlightDistance)");
    if (!perAgentBudgetDist.empty() && static_cast<int>(perAgentBudgetDist.size()) != numAgents)
        fail("perAgentBudgetDist must be empty or have exactly numAgents entries");
    for (const double b : perAgentBudgetDist)
        if (!(b > 0.0) || !std::isfinite(b)) fail("perAgentBudgetDist entries must be finite and positive");
    if (std::abs(sensorTiltAngle) >= deg2rad(85.0)) fail("sensorTiltAngle must be below 85 deg");
    if (quad.enabled) (void)quadrotor::resolveQuadParams(quad, sensorTiltAngle);  // throws on a bad set
    for (int a = 0; a < numAgents; ++a) {
        const double h = agentAltitude(a);
        if (!(h > 0.0)) fail("every agent altitude must be positive");
        if (h / (sweep.rangeMargin * sensor.beta * std::cos(sensorTiltAngle)) >= 1.0) {
            std::ostringstream oss;
            oss << "agent " << a + 1 << " at " << h << " m: the boresight is out of sensor range (beta "
                << sensor.beta << " m) even at zero gimbal angle - lower the altitude/stagger or the tilt";
            fail(oss.str());
        }
    }
    if (!(gimbal.gimbalRate > 0.0)) fail("gimbal.gimbalRate must be positive");
    if (!(gimbal.gimbalMax > 0.0)) fail("gimbal.gimbalMax must be positive");
    if (!(sweep.freq > 0.0)) fail("sweep.freq must be positive");
    if (!(sweep.rangeMargin > 0.0) || sweep.rangeMargin > 1.0) fail("sweep.rangeMargin must be in (0, 1]");
    if (!(sweep.amplitudeFrac > 0.0) || sweep.amplitudeFrac > 1.0) fail("sweep.amplitudeFrac must be in (0, 1]");
    if (!(kernel.tableStep > 0.0) || !(kernel.gridStep > 0.0) || kernel.nPhase < 4)
        fail("kernel.tableStep / gridStep must be positive and nPhase >= 4");
    if (!(curve.sampleSpacing > 0.0)) fail("curve.sampleSpacing must be positive");
    if (!(curve.curvatureKnotSpacing > 0.0)) fail("curve.curvatureKnotSpacing must be positive");
    if (!(curve.curvatureMargin > 0.0) || curve.curvatureMargin > 1.0) fail("curve.curvatureMargin must be in (0, 1]");
    if (curve.representation == CurveRepresentation::BSpline && curve.numControlPoints < curve.splineDegree + 1)
        fail("curve.numControlPoints must be at least splineDegree + 1");
    if (curve.endpointModes.size() != 1 && static_cast<int>(curve.endpointModes.size()) != numAgents)
        fail("curve.endpointModes must have one entry or numAgents entries");
    for (int a = 0; a < numAgents; ++a) {
        if (curve.modeOf(a) == EndpointMode::FixedDest && static_cast<int>(curve.destinations.size()) <= a)
            fail("curve.destinations needs one row per agent in fixed_dest mode");
    }
    if (!(team.fastGridStep > 0.0) || !(team.exploreGridStep > 0.0)) fail("team grid steps must be positive");
    if (team.coordinationSweeps < 1) fail("team.coordinationSweeps must be at least 1");
    if (team.initStrategies.empty()) fail("team.initStrategies must not be empty");
    if (!(team.orderBudgetFrac > 0.0)) fail("team.orderBudgetFrac must be positive");
    {
        const SectorParams& sp = team.sector;
        if (!(sp.sectorSize > 0.0)) fail("team.sector.sectorSize must be positive");
        if (!(sp.boxFrac > 0.0)) fail("team.sector.boxFrac must be positive");
        if (!(sp.budgetFrac > 0.0)) fail("team.sector.budgetFrac must be positive");
        if (!(sp.gridStep > 0.0)) fail("team.sector.gridStep must be positive");
        if (sp.nStarts < 1 || sp.rcl < 1 || sp.nPerturb < 0 || sp.maxCandidates < 1)
            fail("team.sector: nStarts, rcl, maxCandidates must be >= 1 and nPerturb >= 0");
    }
    if (team.reallocate && team.reallocInitStrategies.empty()) fail("team.reallocInitStrategies must not be empty");
    if (optimizer.maxIter < 1 || optimizer.alOuter < 1 || optimizer.lbfgsMemory < 1)
        fail("optimizer.maxIter, alOuter and lbfgsMemory must be at least 1");
    if (infoAware.enabled) {
        if (infoAware.levelSets.empty()) fail("infoAware.levelSets must not be empty");
        for (const std::vector<double>& ls : infoAware.levelSets) {
            if (ls.empty()) fail("infoAware.levelSets entries must not be empty");
            double prev = 0.0;
            for (const double f : ls) {
                if (!(f > prev) || f > 1.0) fail("infoAware.levelSets entries must be increasing fractions in (0, 1]");
                prev = f;
            }
            if (std::abs(ls.back() - 1.0) > 1e-9) fail("infoAware.levelSets entries must end at 1");
        }
        if (!(infoAware.persistence >= 0.0) || infoAware.persistence > 1.0) fail("infoAware.persistence must be in [0, 1]");
        for (const double r : infoAware.reachScales)
            if (!(r > 0.0)) fail("infoAware.reachScales must be positive");
        if (infoAware.restarts < 0) fail("infoAware.restarts must be non-negative");
        if (infoAware.maxMoves < 0) fail("infoAware.maxMoves must be non-negative");
        if (!(infoAware.peelKeep > 0.0) || infoAware.peelKeep >= 1.0) fail("infoAware.peelKeep must be in (0, 1)");
        if (infoAware.optimizeTopK < 1) fail("infoAware.optimizeTopK must be at least 1");
    }
}

}  // namespace mtl::curve
