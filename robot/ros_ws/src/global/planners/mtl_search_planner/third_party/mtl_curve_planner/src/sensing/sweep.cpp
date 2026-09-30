#include "mtl_curve/sensing/sweep.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace mtl::curve::sensing {

SweepParams computeSweepParams(double h, const SweepOptions& o, const GimbalParams& gimbal,
                               const SensorModelParams& sensor, double tau) {
    const double cosLim = h / (o.rangeMargin * sensor.beta * std::cos(tau));
    if (cosLim >= 1.0) {
        throw std::invalid_argument("computeSweepParams: at " + std::to_string(h) + " m and " +
                                    std::to_string(rad2deg(tau)) +
                                    " deg tilt the boresight is out of sensor range even at alpha = 0");
    }
    const double aRange = std::acos(cosLim);
    const double aGim = gimbal.gimbalMax;
    const double aReach = std::atan(gimbal.maxSensorReach * std::cos(tau) / h);
    double alphaMax = std::min({aRange, aGim, aReach}) * o.amplitudeFrac;
    const double fMax = 0.95 * gimbal.gimbalRate / (2.0 * kPi * alphaMax);

    SweepParams s;
    s.tiltAngle = tau;
    s.alphaMax = alphaMax;
    s.freq = std::min(o.freq, fMax);
    s.standOff = h * std::tan(tau);
    s.crossMax = h * std::tan(alphaMax) / std::cos(tau);
    s.halfWidthNominal = std::min({h * std::tan(gimbal.gimbalMax),
                                   std::sqrt(std::max(sensor.beta * sensor.beta -
                                                          h * h / (std::cos(tau) * std::cos(tau)), 0.0)),
                                   gimbal.maxSensorReach});
    s.peakRate = alphaMax * 2.0 * kPi * s.freq;
    return s;
}

}  // namespace mtl::curve::sensing
