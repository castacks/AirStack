#include "mtl/params.hpp"

#include <cmath>
#include <sstream>
#include <stdexcept>

namespace mtl {
namespace {

[[noreturn]] void fail(const std::string& what) {
    throw std::invalid_argument("mtl::PlannerParams: " + what);
}

}  // namespace

void PlannerParams::finalize() {
    // The mount tilt has one meaning; three stages need it, so it is pushed
    // rather than duplicated.
    gimbal.tiltAngle     = singleAxisGimbal ? sensorTiltAngle : 0.0;
    extension.alongOffset = singleAxisGimbal ? sensorStandOff() : 0.0;

    // dt, the turn radius and the FOV likewise: one value, several readers.
    gimbal.dt            = dt;
    gimbal.minTurnRadius = minTurnRadius;
    gimbal.fov           = fov;

    // The scheduler's range budget defaults to the detection model's hard
    // cutoff: a boresight aimed past the sensor's own range is not a service.
    if (!std::isfinite(gimbal.maxSlantRange)) gimbal.maxSlantRange = sensor.beta;


    // The cell-anchor reach is the radius clusterCells already guarantees a
    // cluster is sweepable from, so anchors and clusters stay one kind of object.
    if (!budget.cellAnchor.gimbalReach.has_value())
        budget.cellAnchor.gimbalReach = maxClusterRadius;

    // One verbosity switch for the whole pipeline.
    budget.verbose               = verbose;
    budget.orienteering.verbose  = verbose;
    budget.macroRoute.verbose    = verbose;
    budget.cellAnchor.verbose    = verbose;
    extension.verbose            = verbose;
    gimbal.verbose               = verbose;
    verifyGeometryVerbose        = verifyGeometryVerbose && verbose;
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
    if (!(dubins.stepSize > 0.0)) fail("dubins.stepSize must be positive");
    if (!(fov > 0.0) || fov >= kPi) fail("fov must be in (0, pi)");
    if (detectionThreshold <= 0.0 || detectionThreshold > 1.0)
        fail("detectionThreshold must be in (0, 1]");
    if (budgetDist() <= 0.0) fail("the endurance budget resolves to zero or less");
    if (!perAgentBudgetDist.empty() &&
        static_cast<int>(perAgentBudgetDist.size()) != numAgents)
        fail("perAgentBudgetDist must be empty or have exactly numAgents entries");

    if (std::abs(gimbal.tiltAngle) >= gimbal.maxLookAngle) {
        std::ostringstream oss;
        oss << "sensorTiltAngle (" << rad2deg(sensorTiltAngle)
            << " deg) is at or past the look-angle stop (" << rad2deg(gimbal.maxLookAngle)
            << " deg): the boresight would be at the horizon and never reach the ground";
        fail(oss.str());
    }
    if (!(gimbal.gimbalRate > 0.0)) fail("gimbal.gimbalRate must be positive");
    if (!(gimbal.slewPeak > 0.0)) fail("gimbal.slewPeak must be positive");
    if (gimbal.hMin > gimbal.hMax) fail("gimbal.hMin exceeds gimbal.hMax");
    if (!(gimbal.targetTol > 0.0)) fail("gimbal.targetTol must be positive");
    if (gimbal.targetTol < dubins.stepSize) {
        // Not fatal, but it makes hits unachievable: the ground track is sampled
        // every stepSize metres, so the boresight cannot be placed finer.
        std::fprintf(stderr,
                     "[mtl] warning: gimbal.targetTol (%.2f m) is below the ground-track sample "
                     "spacing (%.2f m); hits may be impossible to achieve.\n",
                     gimbal.targetTol, dubins.stepSize);
    }
    if (budget.reserveFrac0 < 0.0 || budget.reserveFrac0 >= 1.0)
        fail("budget.reserveFrac0 must be in [0, 1)");
    if (budget.maxOuterIter < 1) fail("budget.maxOuterIter must be at least 1");
    if (budget.orienteering.nStarts < 1) fail("budget.orienteering.nStarts must be at least 1");
    if (extension.extendDist < 0.0) fail("extension.extendDist must be non-negative");
    if (!(extension.reachMargin > 0.0) || extension.reachMargin > 1.0)
        fail("extension.reachMargin must be in (0, 1]");
    if (!(gimbal.reachMargin > 0.0) || gimbal.reachMargin > 1.0)
        fail("gimbal.reachMargin must be in (0, 1]");

    if (infoAware.enabled) {
        if (infoAware.levelSets.empty()) fail("infoAware.levelSets must not be empty");
        for (const std::vector<double>& ls : infoAware.levelSets) {
            if (ls.empty()) fail("infoAware.levelSets entries must not be empty");
            double prev = 0.0;
            for (const double f : ls) {
                if (!(f > prev) || f > 1.0)
                    fail("infoAware.levelSets entries must be increasing fractions in (0, 1]");
                prev = f;
            }
            if (std::abs(ls.back() - 1.0) > 1e-9)
                fail("infoAware.levelSets entries must end at 1");
        }
        if (!(infoAware.persistence >= 0.0) || infoAware.persistence > 1.0)
            fail("infoAware.persistence must be in [0, 1]");
        for (const double r : infoAware.reachScales)
            if (!(r > 0.0)) fail("infoAware.reachScales must be positive");
        if (!(infoAware.slantMargin > 0.0) || infoAware.slantMargin > 1.0)
            fail("infoAware.slantMargin must be in (0, 1]");
        if (infoAware.subsample < 1) fail("infoAware.subsample must be at least 1");
        if (infoAware.lookStride < 1) fail("infoAware.lookStride must be at least 1");
        if (infoAware.maxMoves < 0) fail("infoAware.maxMoves must be non-negative");
        if (infoAware.restarts < 0) fail("infoAware.restarts must be non-negative");
        if (!(infoAware.peelKeep > 0.0) || infoAware.peelKeep >= 1.0)
            fail("infoAware.peelKeep must be in (0, 1)");
    }
}

}  // namespace mtl
