#include "mtl/eval/detection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

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

namespace {

/// log(1 - P(Z|x)) as a function of the SQUARED slant range s = d3^2, tabulated
/// on [0, beta^2] and linearly interpolated.  The residual pass evaluates this
/// for every footprint pixel of every look - billions of times on the 1 m
/// reference grid - and the table replaces a sqrt, an exp and a log1p per pixel
/// with one multiply-add.  The function is smooth in s (the sigmoid varies on a
/// 1/b = 100 m scale), so with 2^20 knots the interpolation error is far below
/// round-off in any sum it feeds.  Past beta it is the constant log(1 - pOut).
class LogMissTable {
public:
    LogMissTable(const SensorModelParams& s, double pOut)
        : sensor_(s), pOut_(pOut), logOut_(std::log1p(-pOut)) {
        if (!(s.beta > 0.0) || !std::isfinite(s.beta)) return;  // direct evaluation
        sMax_ = s.beta * s.beta;
        const std::size_t n = std::size_t{1} << 20;
        step_ = sMax_ / static_cast<double>(n);
        inv_  = 1.0 / step_;
        tab_.resize(n + 2);
        for (std::size_t i = 0; i < tab_.size(); ++i) {
            const double si = std::min(static_cast<double>(i) * step_, sMax_);
            tab_[i] = std::log1p(-pDetect(std::sqrt(si), sensor_, pOut_));
        }
    }

    double operator()(double s) const {
        if (tab_.empty()) return std::log1p(-pDetect(std::sqrt(s), sensor_, pOut_));
        if (s > sMax_) return logOut_;
        const double      u = s * inv_;
        const auto        i = static_cast<std::size_t>(u);
        const double      f = u - static_cast<double>(i);
        return tab_[i] + f * (tab_[i + 1] - tab_[i]);
    }

private:
    SensorModelParams   sensor_;
    double              pOut_;
    double              logOut_;
    double              sMax_ = 0.0, step_ = 0.0, inv_ = 0.0;
    std::vector<double> tab_;
};

}  // namespace

ResidualBelief computeResidualBelief(const BeliefField& prior,
                                     const std::vector<AgentTrajectory>& trajectories,
                                     double fov, const SensorModelParams& sensor) {
    ResidualBelief out;
    if (prior.empty()) return out;

    const Index  ny = prior.rows();
    const Index  nx = prior.cols();
    const double total = prior.values.sum();
    if (!(total > 0.0)) {
        out.residual = MatX::Zero(ny, nx);
        return out;
    }

    const double dxr     = prior.gridResX();
    const double dyr     = prior.gridResY();
    const double tanHalf = std::tan(fov / 2.0);
    const LogMissTable logMissOf(sensor, sensor.pOutOfRangeMulti);

    // Accumulated LOG miss likelihood: sum over looks of log(1 - P(Z|x)).
    MatX logMiss = MatX::Zero(ny, nx);

    for (const AgentTrajectory& tr : trajectories) {
        const Index N = std::min(tr.drone.rows(), tr.sensor.rows());
        out.nSteps += N;

        Index k = 0;
        while (k < N) {
            // Run-length of identical consecutive [drone sensor] states (the
            // hover padding of an agent that finished early, a grounded agent):
            // integrated once, weighted by the run length.
            Index run = 1;
            while (k + run < N && tr.drone.row(k + run) == tr.drone.row(k) &&
                   tr.sensor.row(k + run) == tr.sensor.row(k))
                ++run;

            const double px = tr.drone(k, 0), py = tr.drone(k, 1), h = tr.drone(k, 2);
            const double lx = tr.sensor(k, 0), ly = tr.sensor(k, 1);
            const double w  = static_cast<double>(run);
            k += run;

            // Footprint radius follows the slant range to the boresight point -
            // the same rule updateTargetDetectionProbs applies to the targets.
            const double radius =
                std::sqrt((px - lx) * (px - lx) + (py - ly) * (py - ly) + h * h) * tanHalf;
            const double r2 = radius * radius;
            const double h2 = h * h;

            const Index c1 = std::max<Index>(0, static_cast<Index>(std::ceil((lx - radius) / dxr)));
            const Index c2 = std::min<Index>(nx - 1, static_cast<Index>(std::floor((lx + radius) / dxr)));
            if (c1 > c2) continue;

            bool any = false;
#if defined(MTL_HAVE_OPENMP)
#pragma omp parallel for schedule(static) reduction(|| : any)
#endif
            for (Index c = c1; c <= c2; ++c) {  // columns are disjoint: safe to split
                const double x    = static_cast<double>(c) * dxr;
                const double dxl2 = (x - lx) * (x - lx);
                if (dxl2 > r2) continue;
                // Rows of this column inside the circle, padded by one pixel; the
                // exact test below decides the edge, as the target accumulator does.
                const double half = std::sqrt(r2 - dxl2);
                const Index  q1 = std::max<Index>(0, static_cast<Index>(std::floor((ly - half) / dyr)) - 1);
                const Index  q2 = std::min<Index>(ny - 1, static_cast<Index>(std::ceil((ly + half) / dyr)) + 1);
                if (q1 > q2) continue;

                const double dxd2 = (x - px) * (x - px) + h2;
                double*      col  = logMiss.col(c).data();
                for (Index r = q1; r <= q2; ++r) {
                    const double y = static_cast<double>(r) * dyr;
                    if ((y - ly) * (y - ly) + dxl2 > r2) continue;   // outside the footprint
                    col[r] += w * logMissOf((y - py) * (y - py) + dxd2);
                    any = true;
                }
            }
            if (any) ++out.nLooks;
        }
    }

    out.residual     = (prior.values.array() / total) * logMiss.array().exp();
    out.priorMass    = 1.0;
    out.residualMass = out.residual.sum();
    out.detectedMass = out.priorMass - out.residualMass;
    return out;
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
