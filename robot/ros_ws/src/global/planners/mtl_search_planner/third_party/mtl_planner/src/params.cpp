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
}

}  // namespace mtl
