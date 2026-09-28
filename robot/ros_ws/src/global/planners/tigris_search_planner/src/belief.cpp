// =============================================================================
//  belief.cpp — footprint, planning grid, beliefs and the two reward models.
// =============================================================================
#include "tigris_search_planner/belief.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace tigris_search {
namespace {

double entropy(double p) {
    if (p <= 0.0 || p >= 1.0) return 0.0;
    return -p * std::log2(p) - (1.0 - p) * std::log2(1.0 - p);
}

}  // namespace

RewardMode rewardModeFromString(const std::string& s) {
    if (s == "original" || s == "tigris") return RewardMode::ORIGINAL;
    if (s == "matched" || s == "residual") return RewardMode::MATCHED;
    throw std::runtime_error("reward_mode must be 'original' or 'matched' (got '" + s + "')");
}

const char* toString(RewardMode m) { return m == RewardMode::ORIGINAL ? "original" : "matched"; }

Look lookFromPose(double x, double y, double h, double yaw, const Camera& cam, const DetectionModel& det,
                  double weight) {
    Look l;
    if (!(h > 0.0)) return l;
    const double c = std::cos(cam.tilt);
    if (c <= 1e-6) return l;
    const double slant = h / c;
    if (slant > det.beta) return l;  // the look point itself is out of range (logger rule)
    const double d = h * std::tan(cam.tilt);
    l.px = x;
    l.py = y;
    l.pz = h;
    l.gx = x + d * std::cos(yaw);
    l.gy = y + d * std::sin(yaw);
    l.radius = slant * std::tan(cam.fov / 2.0);
    l.weight = weight;
    l.valid = weight > 0.0 && l.radius > 0.0;
    return l;
}

Look lookFromGimbal(double x, double y, double h, double pitch, double yaw, double fov,
                    const DetectionModel& det, double weight) {
    Look l;
    const double cp = std::cos(pitch);
    const double bz = -std::sin(pitch);
    if (bz >= -1e-6 || !(h > 0.0)) return l;
    const double s = h / -bz;
    if (s > det.beta) return l;
    l.px = x;
    l.py = y;
    l.pz = h;
    l.gx = x + s * std::cos(yaw) * cp;
    l.gy = y + s * std::sin(yaw) * cp;
    l.radius = s * std::tan(fov / 2.0);
    l.weight = weight;
    l.valid = weight > 0.0 && l.radius > 0.0;
    return l;
}

namespace {
/// MapRepresentation's Bayes update (the p > 0.5 branch assumes a detection).
double bayes(double p, double range, const DetectionModel& det, const RewardParams& rp) {
    const double tpr = range <= det.beta ? det.prob(range) : rp.tprBeyondBeta;
    const double fpr = 1.0 - tpr;
    if (p > 0.5) {
        const double den = tpr * p + fpr * (1.0 - p);
        return den > 0.0 ? tpr * p / den : p;
    }
    const double den = (1.0 - tpr) * p + (1.0 - fpr) * (1.0 - p);
    return den > 0.0 ? (1.0 - tpr) * p / den : p;
}
}  // namespace

double originalCellUpdate(double p, double range, const DetectionModel& det, const RewardParams& rp,
                          double* pNew) {
    // estimateBeliefandReward: belief_diff = new - prev; entropy_diff = H(prev) - H(new)
    const double q = bayes(p, range, det, rp);
    *pNew = q;
    const double beliefDiff = q - p;
    if (beliefDiff > 0.0) return rp.useEntropy ? (entropy(p) - entropy(q)) * rp.rs : beliefDiff * rp.rs;
    return rp.useEntropy ? (entropy(p) - entropy(q)) * rp.rf : std::fabs(beliefDiff) * rp.rf;
}

double originalEdgeCellUpdate(double p, double range, const DetectionModel& det, const RewardParams& rp,
                              double* pNew) {
    // estimateEdgeBeliefandReward: prev/present are entropies (use_entropy) or beliefs;
    // diff = present - prev; reward = diff * Rs if diff > 0 else |diff| * Rf
    const double q = bayes(p, range, det, rp);
    *pNew = q;
    const double prev = rp.useEntropy ? entropy(p) : p;
    const double present = rp.useEntropy ? entropy(q) : q;
    const double diff = present - prev;
    return diff > 0.0 ? diff * rp.rs : std::fabs(diff) * rp.rf;
}

PlanningGrid::PlanningGrid(const Scenario& sc, double resolution, double initialConfidence) {
    if (!(resolution > 0.0)) throw std::runtime_error("grid_res_m must be positive");
    res = resolution;
    xMin = sc.xMin;
    yMin = sc.yMin;
    // row_size = (X_END - X_START) / RESOLUTION, integer division as in MapRepresentation
    nx = std::max(1, static_cast<int>(std::floor((sc.xMax - sc.xMin) / res + 1e-9)));
    ny = std::max(1, static_cast<int>(std::floor((sc.yMax - sc.yMin) / res + 1e-9)));
    mass.assign(static_cast<std::size_t>(nx * ny), 0.0);
    // MapRepresentation::setup: map = initial_confidence, each prior point value -> max into its cell
    presence0.assign(mass.size(), initialConfidence);
    const PriorRaster& pr = sc.prior;
    for (int i = 0; i < pr.ny(); ++i) {
        const int ci = std::min(ny - 1, std::max(0, static_cast<int>(std::floor((pr.ys[i] - yMin) / res))));
        for (int j = 0; j < pr.nx(); ++j) {
            const int cj = std::min(nx - 1, std::max(0, static_cast<int>(std::floor((pr.xs[j] - xMin) / res))));
            const std::size_t k = static_cast<std::size_t>(ci * nx + cj);
            const std::size_t p = static_cast<std::size_t>(i * pr.nx() + j);
            mass[k] += pr.norm[p];
            presence0[k] = std::max(presence0[k], pr.raw[p]);
        }
    }
}

BeliefState BeliefState::fromGrid(const PlanningGrid& g) {
    BeliefState b;
    b.residual = g.mass;
    b.presence = g.presence0;
    return b;
}

double BeliefState::residualMass() const { return std::accumulate(residual.begin(), residual.end(), 0.0); }

RewardEvaluator::RewardEvaluator(const PlanningGrid& grid, const DetectionModel& det, const RewardParams& rp)
    : g_(grid), det_(det), rp_(rp) {
    const std::size_t n = grid.size();
    stampRes_.assign(n, 0);
    stampPres_.assign(n, 0);
    stampPass_.assign(n, 0);
    res_.assign(n, 0.0);
    pres_.assign(n, 0.0);
    rmin_.assign(n, 0.0);
}

void RewardEvaluator::begin(const BeliefState& base) {
    base_ = &base;
    ++epoch_;
    if (epoch_ == 0) {  // wrapped: clear the stamps
        std::fill(stampRes_.begin(), stampRes_.end(), 0u);
        std::fill(stampPres_.begin(), stampPres_.end(), 0u);
        epoch_ = 1;
    }
    acc_ = PathReward();
    touchedAll_.clear();
}

void RewardEvaluator::pass(const std::vector<Look>& looks, bool original, bool matched) {
    if (base_ == nullptr) return;
    ++passEpoch_;
    if (passEpoch_ == 0) {
        std::fill(stampPass_.begin(), stampPass_.end(), 0u);
        passEpoch_ = 1;
    }
    touched_.clear();
    const double logOut = std::log1p(-std::min(std::max(det_.pOut, 0.0), 1.0 - 1e-15));
    for (const Look& l : looks) {
        if (!l.valid) continue;
        const double h2 = l.pz * l.pz;
        const double fOut = std::exp(l.weight * logOut);
        g_.forEachInDisc(l.gx, l.gy, l.radius, [&](std::size_t k, double x, double y) {
            const double r = std::sqrt((x - l.px) * (x - l.px) + (y - l.py) * (y - l.py) + h2);
            if (matched) {
                if (stampRes_[k] != epoch_) {
                    stampRes_[k] = epoch_;
                    res_[k] = base_->residual[k];
                    touchedAll_.push_back(k);
                }
                const double old = res_[k];
                if (old > 0.0) {
                    double f;
                    if (r > det_.beta) {
                        f = fOut;
                    } else {
                        const double q = 1.0 - 1.0 / (det_.a + std::exp(det_.b * (r - det_.c)));
                        f = q > 0.0 ? std::pow(q, l.weight) : 0.0;
                    }
                    const double nv = old * f;
                    res_[k] = nv;
                    acc_.matched += old - nv;
                }
            }
            if (original) {
                // evidence, not a planning expectation: the look MISSED. Bayes with the TIGRIS
                // sensor model (tpr, fpr = 1 - tpr), likelihoods to the power dt / dt_ref.
                double& p = presence(k);
                const double tpr = r <= det_.beta ? det_.prob(r) : rp_.tprBeyondBeta;
                const double lMissT = std::pow(1.0 - tpr, l.weight);  // P(no detection | target)
                const double lMissN = std::pow(tpr, l.weight);        // P(no detection | none) = 1 - fpr
                const double den = lMissT * p + lMissN * (1.0 - p);
                const double q = den > 0.0 ? lMissT * p / den : p;
                acc_.original += entropy(p) - entropy(q);
                p = q;
            }
        });
    }
}

double& RewardEvaluator::presence(std::size_t k) {
    if (stampPres_[k] != epoch_) {
        stampPres_[k] = epoch_;
        pres_[k] = base_->presence[k];
        touchedAll_.push_back(k);
    }
    return pres_[k];
}

namespace {
/// Footprint of the body-fixed cone WITHOUT the logger's whole-look range gate
/// (TIGRIS has none: out-of-range cells simply get tpr = 0.5).
bool coneFootprint(double h, const Camera& cam, double* lead, double* radius) {
    const double c = std::cos(cam.tilt);
    if (!(h > 0.0) || c <= 1e-6) return false;
    *lead = h * std::tan(cam.tilt);
    *radius = h / c * std::tan(cam.fov / 2.0);
    return *radius > 0.0;
}
}  // namespace

void RewardEvaluator::tigrisNode(double x, double y, double h, double yaw, const Camera& cam) {
    double d, R;
    if (base_ == nullptr || !coneFootprint(h, cam, &d, &R)) return;
    const double gx = x + d * std::cos(yaw), gy = y + d * std::sin(yaw);
    // estimateBeliefandReward: cells entirely inside, range from the node to the cell corner
    g_.forEachCellInside(gx, gy, gx, gy, R, [&](std::size_t k, double cx, double cy) {
        const double range = std::sqrt((cx - x) * (cx - x) + (cy - y) * (cy - y) + h * h);
        double& p = presence(k);
        double q;
        acc_.original += originalCellUpdate(p, range, det_, rp_, &q);
        p = q;
    });
}

void RewardEvaluator::tigrisEdge(double sx, double sy, double ex, double ey, double yaw, double h,
                                 const Camera& cam) {
    double d, R;
    if (base_ == nullptr || !coneFootprint(h, cam, &d, &R)) return;
    const double ux = std::cos(yaw), uy = std::sin(yaw);
    const double len = std::max(0.0, (ex - sx) * ux + (ey - sy) * uy);
    // edge geometry = the footprint swept from the start to the end pose (findEdgeGeometry)
    const double gsx = sx + d * ux, gsy = sy + d * uy;
    const double gex = gsx + len * ux, gey = gsy + len * uy;
    const double eX = sx + len * ux, eY = sy + len * uy;  // end pose on the edge line
    const double r2 = R * R;
    auto inEndFootprint = [&](double cx, double cy) {  // all four corners in the end disc
        const double res = g_.res;
        const double xs[2] = {cx, cx + res}, ys[2] = {cy, cy + res};
        for (const double px : xs) {
            for (const double py : ys) {
                if ((px - gex) * (px - gex) + (py - gey) * (py - gey) > r2) return false;
            }
        }
        return true;
    };
    g_.forEachCellInside(gsx, gsy, gex, gey, R, [&](std::size_t k, double cx, double cy) {
        double range;
        if (inEndFootprint(cx, cy)) {
            // estimateEdgeBeliefandReward: cells inside the end node's footprint are ranged
            // from the end pose
            range = std::sqrt((cx - eX) * (cx - eX) + (cy - eY) * (cy - eY) + h * h);
        } else {
            // nearest viewing pose along the edge line (not clamped to the segment, as in the
            // original): overhead when the cell can be seen abeam, otherwise from where it
            // first enters the footprint - the disc analogue of TIGRIS's
            // camera_within_footprint / bottom-edge offset and delta_l terms
            const double lat = -(cx - sx) * uy + (cy - sy) * ux;
            const double w = std::sqrt(std::max(0.0, r2 - lat * lat));
            const double offset = std::max(0.0, d - w);  // along-track distance to the nearest pose
            range = std::sqrt(lat * lat + offset * offset + h * h);
        }
        double& p = presence(k);
        double q;
        acc_.original += originalEdgeCellUpdate(p, range, det_, rp_, &q);
        p = q;
    });
}

void RewardEvaluator::commit(BeliefState& dst) const {
    for (const std::size_t k : touchedAll_) {
        if (stampRes_[k] == epoch_) dst.residual[k] = res_[k];
        if (stampPres_[k] == epoch_) dst.presence[k] = pres_[k];
    }
}

}  // namespace tigris_search
