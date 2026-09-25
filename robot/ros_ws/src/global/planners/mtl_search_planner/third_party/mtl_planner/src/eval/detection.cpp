#include "mtl/eval/detection.hpp"

#include <cmath>
#include <limits>

namespace mtl::eval {
namespace {

double pDetect(double slant, const SensorModelParams& s, double pOut) {
    if (slant > s.beta) return pOut;
    return 1.0 / (s.a + std::exp(s.b * (slant - s.c)));
}

DetectionSummary summarise(const std::vector<Target>& targets, double threshold) {
    DetectionSummary sum;
    double acc = 0.0;
    for (std::size_t i = 0; i < targets.size(); ++i) {
        acc += targets[i].detectionProb;
        if (targets[i].detected()) {
            ++sum.nDetected;
        } else {
            ++sum.nMissed;
        }
        if (targets[i].detectionProb < threshold) sum.missedIdx.push_back(static_cast<Index>(i));
    }
    if (!targets.empty()) sum.meanDetectionProb = acc / static_cast<double>(targets.size());
    return sum;
}

}  // namespace

double footprintRadius(const Vec3& dronePos, const Vec2& sensorPos, double fov) {
    // Distance from the aircraft to the ground point the boresight is on.
    const Vec3 look(sensorPos.x(), sensorPos.y(), 0.0);
    return (dronePos - look).norm() * std::tan(fov / 2.0);
}

Path2 projectedFootprint(const Vec3& dronePos, const Vec2& sensorPos, double fov, int numPoints) {
    const int  n = std::max(3, numPoints);
    const double r = footprintRadius(dronePos, sensorPos, fov);
    Path2 pts(n, 2);
    for (int i = 0; i < n; ++i) {
        const double th = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(n - 1);
        pts(i, 0) = sensorPos.x() + r * std::cos(th);
        pts(i, 1) = sensorPos.y() + r * std::sin(th);
    }
    return pts;
}

DetectionSummary updateTargetDetectionProbs(const std::vector<AgentTrajectory>& trajectories,
                                            const VecX& timeVec, std::vector<Target>& targets,
                                            double fov, const SensorModelParams& sensor,
                                            double threshold) {
    const Index nSteps = timeVec.size();
    const double tanHalf = std::tan(fov / 2.0);

    for (Target& t : targets) {
        t.pMissTotal    = 1.0;
        t.detectionProb = 0.0;
        t.detectionTime = std::numeric_limits<double>::quiet_NaN();
    }

    for (Index k = 0; k < nSteps; ++k) {
        for (Target& t : targets) {
            for (const AgentTrajectory& tr : trajectories) {
                if (k >= tr.drone.rows()) continue;
                const Vec3 drone(tr.drone(k, 0), tr.drone(k, 1), tr.drone(k, 2));
                const Vec2 look(tr.sensor(k, 0), tr.sensor(k, 1));

                // The footprint follows the SLANT RANGE to the ground point the
                // boresight is on, not the altitude.  They are equal only at
                // nadir; the moment the gimbal swings out - and always, once the
                // mount is tilted - the slant range is longer and the footprint
                // correspondingly larger.
                const double slantToLook =
                    std::sqrt((drone.x() - look.x()) * (drone.x() - look.x()) +
                              (drone.y() - look.y()) * (drone.y() - look.y()) +
                              drone.z() * drone.z());
                const double radius = slantToLook * tanHalf;

                const double dist2D = (t.pose - look).norm();
                if (dist2D > radius) continue;

                const double dist3D =
                    std::sqrt((t.pose.x() - drone.x()) * (t.pose.x() - drone.x()) +
                              (t.pose.y() - drone.y()) * (t.pose.y() - drone.y()) +
                              drone.z() * drone.z());
                const double pz = pDetect(dist3D, sensor, sensor.pOutOfRangeMulti);

                // Joint probability of MISSING the target, accumulated over
                // every look every agent takes.
                t.pMissTotal *= (1.0 - pz);
            }

            // Evaluate the threshold after all agents have been checked for this
            // time step, so two agents looking at once are credited together.
            const double prob = 1.0 - t.pMissTotal;
            if (!t.detected() && prob >= threshold) t.detectionTime = timeVec(k);
        }
    }

    for (Target& t : targets) t.detectionProb = 1.0 - t.pMissTotal;
    return summarise(targets, threshold);
}

DetectionSummary updateTargetDetectionProbs(const Path3& droneTraj, const Path2& sensorTraj,
                                            const VecX& timeVec, std::vector<Target>& targets,
                                            double fov, const SensorModelParams& sensor,
                                            double threshold) {
    AgentTrajectory tr;
    tr.drone  = droneTraj;
    tr.sensor = sensorTraj;
    return updateTargetDetectionProbs(std::vector<AgentTrajectory>{tr}, timeVec, targets, fov,
                                      sensor, threshold);
}

}  // namespace mtl::eval
