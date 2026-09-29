// =============================================================================
//  tigris_search_planner/belief.hpp — sensor footprint, planning grid, belief
//  and the two TIGRIS reward models.
//
//  SENSOR (both reward modes). The single-axis mount: full cone `fov` on a bracket
//  tilted `tilt` forward of nadir, yawed with the airframe, with ONE cross-track
//  axis phi (+ right). Without gimbal actuation phi = 0 (a body-fixed camera).
//  With actuation (mission.yaml gimbal_actuation) phi sweeps -A .. +A at a
//  constant angular speed as a function of the planned track time
//  (Camera::phiAt). One look from height h, heading psi and cross-track angle phi
//  hits the ground at
//      ahead = h tan(tilt) / cos(phi),   right = h tan(phi),
//      slant = h / (cos(tilt) cos(phi))
//  (the constraint plane mtl_trajectory_follower's single_axis_command points
//  in) and sees the disc of radius slant * tan(fov / 2) about that point, and
//  nothing when the slant exceeds beta. This is exactly the gate
//  mtl_metrics_logger scores flights with.
//
//  REWARD MODES
//    ORIGINAL  TIGRIS as written (tigris/src/MapRepresentation.cpp): an independent
//              Bernoulli "target present" belief per cell, a Bayes update with
//              tpr(range) and fpr = 1 - tpr (the code assumes a detection when
//              p > 0.5 and a miss otherwise), tpr = the scenario sigmoid up to beta
//              and 0.5 beyond (TIGRIS's flatten point). A path is scored the TIGRIS
//              way, root to leaf: every NODE's footprint (estimateBeliefandReward:
//              cells lying entirely inside it, range from the node to the cell's
//              corner, reward = entropy drop x Rs if the belief rose else x Rf) and
//              the STRAIGHT part of every edge (estimateEdgeBeliefandReward: cells
//              entirely inside the swept footprint, range from the nearest viewing
//              pose on the edge, reward = |entropy change| with the edge formula).
//              Turning arcs are not rewarded, exactly as in TIGRIS. The only change
//              is the footprint shape: the user-configured cone (a disc on the
//              ground) instead of the rectangular frustum, so the edge swath is the
//              disc swept along the straight segment and the nearest viewing pose is
//              computed for that disc. With gimbal actuation the node footprint
//              is the disc at the node's swept phi, and the straight-edge swath is
//              the union of the swept discs along the segment (every reward_step_m):
//              a cell counts when it lies entirely inside one of them, ranged from
//              the nearest such viewing pose, and is updated once per edge as in
//              estimateEdgeBeliefandReward.
//    MATCHED   (not in TIGRIS; the evaluator's metric) the expected drop in
//              residual belief mass: every cell holds prior mass * P(all looks
//              missed), updated per look along the whole path with
//              (1 - P(r))^(dt / dt_ref).
// =============================================================================
#ifndef TIGRIS_SEARCH_PLANNER_BELIEF_HPP
#define TIGRIS_SEARCH_PLANNER_BELIEF_HPP

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "tigris_search_planner/scenario.hpp"

namespace tigris_search {

enum class RewardMode { ORIGINAL, MATCHED };
RewardMode rewardModeFromString(const std::string& s);  ///< "original" | "matched"
const char* toString(RewardMode m);

struct Camera {
    double fov = 1.0471975511965976;   ///< full cone [rad]
    double tilt = 0.5235987755982988;  ///< forward tilt from nadir [rad]
    // gimbal actuation: the cross-track axis sweeps -sweepAmplitude .. +sweepAmplitude
    bool   sweep = false;
    double sweepRate = 0.0;            ///< [rad/s] constant cross-track angular speed
    double sweepAmplitude = 0.0;       ///< [rad]
    /// Cross-track angle (+ right) at planned track time t [s]: a triangle wave at constant
    /// |dphi/dt| = sweepRate, phi(0) = 0 moving right, period 4 A / rate. 0 without actuation.
    double phiAt(double t) const;
};

/// The mission's camera: sensor.fov_deg, sensor.tilt_deg and airstack.gimbal_actuation.
Camera cameraFromScenario(const Scenario& sc);

/// Earth-frame kinematics of the sweep (the single_axis_command solution for theta = tilt),
/// for checking it against the gimbal: peak |d(axis)/dt| over roll/pitch/yaw [rad/s] at the
/// camera's sweepRate, the largest |roll| and the shallowest pitch reached [rad].
struct SweepKinematics {
    double peakAxisRate = 0.0;
    double maxAbsRoll = 0.0;
    double minPitch = 0.0;
    double maxSlantPerHeight = 0.0;  ///< slant / h at the sweep's ends = 1 / (cos(tilt) cos(A))
};
SweepKinematics sweepKinematics(const Camera& cam);

/// Boresight ground point of the single-axis mount at cross-track angle phi (+ right).
void boresightGround(double x, double y, double h, double yaw, double phi, double tilt, double* gx, double* gy);

/// One look: camera position (pz = height above ground), footprint disc, weight dt/dt_ref.
struct Look {
    double px = 0.0, py = 0.0, pz = 0.0;
    double gx = 0.0, gy = 0.0;
    double radius = 0.0;
    double weight = 1.0;
    bool valid = false;
};

/// Look of the single-axis camera from pose (x, y, h, yaw) at cross-track angle phi (+ right;
/// 0 = the body-fixed camera).
Look lookFromPose(double x, double y, double h, double yaw, const Camera& cam, const DetectionModel& det,
                  double weight, double phi = 0.0);
/// Look from a MEASURED earth-frame gimbal attitude (pitch > 0 looks down), as the logger scores it.
Look lookFromGimbal(double x, double y, double h, double pitch, double yaw, double fov,
                    const DetectionModel& det, double weight);

struct RewardParams {
    RewardMode mode = RewardMode::ORIGINAL;
    bool   useEntropy = true;         ///< ORIGINAL: entropy reduction (TIGRIS launch default) or |belief change|
    double rs = 2.0, rf = 1.0;        ///< ORIGINAL: MapRepresentation::Rs / Rf
    double initialConfidence = 0.01;  ///< ORIGINAL: floor of the presence belief (TIGRIS initial_confidence)
    double tprBeyondBeta = 0.5;       ///< ORIGINAL: tpr past the sensor range (TIGRIS flatten value)
};

/// TIGRIS node-footprint update of one cell (estimateBeliefandReward / updateInformedConfig);
/// returns the reward and writes the new belief.
double originalCellUpdate(double p, double range, const DetectionModel& det, const RewardParams& rp,
                          double* pNew);
/// TIGRIS straight-edge update of one cell (estimateEdgeBeliefandReward's reward formula).
double originalEdgeCellUpdate(double p, double range, const DetectionModel& det, const RewardParams& rp,
                              double* pNew);

/// Uniform planning grid over the search area (world ENU), cell (i, j) centred at
/// (xMin + (j + .5) res, yMin + (i + .5) res), index i * nx + j.
class PlanningGrid {
public:
    PlanningGrid() = default;
    PlanningGrid(const Scenario& sc, double res, double initialConfidence);

    int nx = 0, ny = 0;
    double res = 4.0, xMin = 0.0, yMin = 0.0;
    std::vector<double> mass;       ///< prior probability mass per cell (sums to 1)
    std::vector<double> presence0;  ///< ORIGINAL: initial presence belief (max raw prior in the cell, floored)

    std::size_t size() const { return mass.size(); }
    double cx(int j) const { return xMin + (j + 0.5) * res; }
    double cy(int i) const { return yMin + (i + 0.5) * res; }

    /// Calls f(k, cornerX, cornerY) for every cell whose four corners lie within `r` of the
    /// segment a-b (a disc when a == b): TIGRIS's "cell entirely inside the footprint" test.
    /// (cornerX, cornerY) is the cell's min corner, the point TIGRIS measures range to.
    template <typename F>
    void forEachCellInside(double ax, double ay, double bx, double by, double r, F&& f) const {
        if (!(r > 0.0) || nx == 0) return;
        const double x0 = std::min(ax, bx) - r, x1 = std::max(ax, bx) + r;
        const double y0 = std::min(ay, by) - r, y1 = std::max(ay, by) + r;
        const int j0 = std::max(0, static_cast<int>(std::floor((x0 - xMin) / res)));
        const int j1 = std::min(nx - 1, static_cast<int>(std::floor((x1 - xMin) / res)));
        const int i0 = std::max(0, static_cast<int>(std::floor((y0 - yMin) / res)));
        const int i1 = std::min(ny - 1, static_cast<int>(std::floor((y1 - yMin) / res)));
        const double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy, r2 = r * r;
        auto inside = [&](double px, double py) {
            double t = len2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
            t = std::max(0.0, std::min(1.0, t));
            const double qx = ax + t * dx - px, qy = ay + t * dy - py;
            return qx * qx + qy * qy <= r2;
        };
        for (int i = i0; i <= i1; ++i) {
            const double ya = yMin + i * res, yb = ya + res;
            for (int j = j0; j <= j1; ++j) {
                const double xa = xMin + j * res, xb = xa + res;
                if (inside(xa, ya) && inside(xb, ya) && inside(xa, yb) && inside(xb, yb)) {
                    f(static_cast<std::size_t>(i * nx + j), xa, ya);
                }
            }
        }
    }

    /// Calls f(k, x, y) for every cell centre inside the closed disc.
    template <typename F>
    void forEachInDisc(double gx, double gy, double r, F&& f) const {
        if (!(r > 0.0) || nx == 0) return;
        int i0 = static_cast<int>(std::ceil((gy - r - yMin) / res - 0.5 - 1e-9));
        int i1 = static_cast<int>(std::floor((gy + r - yMin) / res - 0.5 + 1e-9));
        if (i0 < 0) i0 = 0;
        if (i1 > ny - 1) i1 = ny - 1;
        const double r2 = r * r;
        for (int i = i0; i <= i1; ++i) {
            const double y = cy(i);
            const double dy = y - gy;
            const double rem = r2 - dy * dy;
            if (rem < 0.0) continue;
            const double half = std::sqrt(rem);
            int j0 = static_cast<int>(std::ceil((gx - half - xMin) / res - 0.5 - 1e-9));
            int j1 = static_cast<int>(std::floor((gx + half - xMin) / res - 0.5 + 1e-9));
            if (j0 < 0) j0 = 0;
            if (j1 > nx - 1) j1 = nx - 1;
            for (int j = j0; j <= j1; ++j) {
                const double x = cx(j);
                if ((x - gx) * (x - gx) + dy * dy > r2) continue;
                f(static_cast<std::size_t>(i * nx + j), x, y);
            }
        }
    }
};

/// Both beliefs, always kept (so either reward can be reported whatever the mode).
struct BeliefState {
    std::vector<double> residual;  ///< MATCHED: prior mass * P(missed so far)
    std::vector<double> presence;  ///< ORIGINAL: presence belief

    static BeliefState fromGrid(const PlanningGrid& g);
    double residualMass() const;
};

/// Rewards of one sequence of passes (a "pass" = the looks of one tree edge).
struct PathReward {
    double original = 0.0;  ///< ORIGINAL-mode reward
    double matched = 0.0;   ///< MATCHED-mode reward (residual mass removed)
    double of(RewardMode m) const { return m == RewardMode::ORIGINAL ? original : matched; }
};

/// Evaluates rewards on a scratch copy of a base belief without copying the grid
/// (epoch-stamped overlays). Optionally commits the result into a BeliefState.
class RewardEvaluator {
public:
    RewardEvaluator(const PlanningGrid& grid, const DetectionModel& det, const RewardParams& rp);

    /// Start a new evaluation over `base`.
    void begin(const BeliefState& base);
    /// Fold looks in. MATCHED: residual *= (1 - P(r))^w per look. ORIGINAL (`original` = true):
    /// the looks are EVIDENCE of a miss - per look, Bayes with the TIGRIS sensor model and
    /// likelihoods to the power w = dt / dt_ref (used for the flown / committed looks of the
    /// receding horizon; the tree scores ORIGINAL paths with tigrisNode / tigrisEdge).
    void pass(const std::vector<Look>& looks, bool original, bool matched);
    PathReward reward() const { return acc_; }
    /// ORIGINAL (TIGRIS) node footprint: camera at (x, y, h, yaw), cross-track angle phi.
    void tigrisNode(double x, double y, double h, double yaw, const Camera& cam, double phi = 0.0);
    /// ORIGINAL (TIGRIS) straight edge from (sx, sy) to (ex, ey) flown at heading `yaw`, height h
    /// (body-fixed camera).
    void tigrisEdge(double sx, double sy, double ex, double ey, double yaw, double h, const Camera& cam);
    /// ORIGINAL straight edge with a SWEEPING camera: the union of the footprints of `poses`
    /// (x, y, yaw, phi along the segment); each cell entirely inside one of them is updated
    /// once, with the edge formula, at its range from the nearest such pose.
    struct ViewPose { double x = 0.0, y = 0.0, yaw = 0.0, phi = 0.0; };
    void tigrisEdgeSwept(const std::vector<ViewPose>& poses, double h, const Camera& cam);
    /// Write the overlay into `dst` (normally the same object as the base).
    void commit(BeliefState& dst) const;

private:
    const PlanningGrid& g_;
    DetectionModel det_;
    RewardParams rp_;
    const BeliefState* base_ = nullptr;
    unsigned epoch_ = 0, passEpoch_ = 0;
    std::vector<unsigned> stampRes_, stampPres_, stampPass_;
    std::vector<double> res_, pres_, rmin_;
    double& presence(std::size_t k);
    std::vector<std::size_t> touched_, touchedAll_;
    PathReward acc_;
};

}  // namespace tigris_search

#endif  // TIGRIS_SEARCH_PLANNER_BELIEF_HPP
