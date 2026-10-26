#include "mtl_curve/quadrotor/flatness.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace mtl::curve::quadrotor {

QuadParams resolveQuadParams(const QuadParams& in, double fwTiltAngle) {
    QuadParams q = in;
    const double g = q.gravity;
    q.dragQuadResolved = q.dragQuad ? *q.dragQuad
                                    : (g * std::tan(q.dragSpecTilt) - q.dragLin * q.dragSpecSpeed) /
                                          (q.dragSpecSpeed * q.dragSpecSpeed);
    const double aDrag = q.dragQuadResolved * q.speed * q.speed + q.dragLin * q.speed;
    q.trimPitch = std::atan(aDrag / g);
    switch (q.cameraMount) {
        case CameraMount::Matched: q.camTilt = fwTiltAngle + q.trimPitch; break;
        case CameraMount::Fixed:   q.camTilt = q.camTiltFixed ? *q.camTiltFixed : fwTiltAngle; break;
        case CameraMount::Nadir:   q.camTilt = 0.0; break;
    }
    if (!(q.speed > 0.0)) throw std::invalid_argument("QuadParams::speed must be positive");
    if (!(q.knotSpacing > 0.0) || !(q.dtOpt > 0.0)) throw std::invalid_argument("QuadParams: knotSpacing and dtOpt must be positive");
    if (q.splineDegree < 4) throw std::invalid_argument("QuadParams::splineDegree must be >= 4 (snap is penalised)");
    if (q.trimPitch >= q.limitMargin * q.maxTilt) {
        std::ostringstream oss;
        oss << "QuadParams: cruise at " << q.speed << " m/s needs " << rad2deg(q.trimPitch)
            << " deg of trim tilt, beyond the " << rad2deg(q.maxTilt) << " deg limit";
        throw std::invalid_argument(oss.str());
    }
    q.resolved = true;
    return q;
}

// -----------------------------------------------------------------------------
//  uniform B-spline
// -----------------------------------------------------------------------------
namespace {

/// Cardinal B-spline N_p(x) of degree p on [0, p+1].
double cardinal(double x, int p) {
    if (p == 0) return (x >= 0.0 && x < 1.0) ? 1.0 : 0.0;
    return (x * cardinal(x, p - 1) + (static_cast<double>(p) + 1.0 - x) * cardinal(x - 1.0, p - 1)) /
           static_cast<double>(p);
}

double binom(int n, int k) {
    double r = 1.0;
    for (int i = 1; i <= k; ++i) r = r * static_cast<double>(n - k + i) / static_cast<double>(i);
    return r;
}

Vec3 cross3(const Vec3& a, const Vec3& b) { return a.cross(b); }

}  // namespace

Index numCoefficients(double T, double delta, int p) {
    const Index n = static_cast<Index>(std::ceil(T / delta - 1e-9)) + p;
    return std::max<Index>(n, p + 1);
}

UniformBspline uniformBspline(const VecX& t, double delta, double T, int p, int maxDeriv) {
    UniformBspline B;
    B.p = p;
    B.delta = delta;
    B.T = T;
    B.nCP = numCoefficients(T, delta, p);
    const Index K = t.size();
    B.m.resize(static_cast<std::size_t>(K));
    B.b.assign(static_cast<std::size_t>(maxDeriv + 1), MatX::Zero(K, p + 1));
    for (Index k = 0; k < K; ++k) {
        Index m = static_cast<Index>(std::floor(t(k) / delta));
        m = std::min(std::max<Index>(m, 0), B.nCP - p - 1);
        const double u = t(k) / delta - static_cast<double>(m);
        B.m[static_cast<std::size_t>(k)] = m;
        for (int d = 0; d <= maxDeriv; ++d) {
            const double sc = std::pow(delta, -d);
            for (int i = 0; i <= p; ++i) {
                const double x = u + static_cast<double>(p - i);
                double acc = 0.0;
                for (int l = 0; l <= d; ++l)
                    acc += ((l % 2) ? -1.0 : 1.0) * binom(d, l) * cardinal(x - static_cast<double>(l), p - d);
                B.b[static_cast<std::size_t>(d)](k, i) = acc * sc;
            }
        }
    }
    return B;
}

double UniformBspline::eval(int d, Index k, const VecX& c) const {
    const MatX& bd = b[static_cast<std::size_t>(d)];
    const Index m0 = m[static_cast<std::size_t>(k)];
    double s = 0.0;
    for (int i = 0; i <= p; ++i) s += bd(k, i) * c(m0 + i);
    return s;
}

VecX UniformBspline::eval(int d, const VecX& c) const {
    VecX out(samples());
    for (Index k = 0; k < samples(); ++k) out(k) = eval(d, k, c);
    return out;
}

// -----------------------------------------------------------------------------
//  the flatness map
// -----------------------------------------------------------------------------
FlatState flatMap(const Vec2& v, const Vec2& a, const Vec2& j, double psi, double psiDot, double h,
                  const QuadParams& q, FlatMode mode) {
    FlatState S;
    const double g = q.gravity;
    const double cd = q.dragQuadResolved, kl = q.dragLin;
    const double sp = v.norm();
    const Vec2 D = (cd * sp + kl) * v;
    const Vec3 t(a.x() + D.x(), a.y() + D.y(), g);
    const double f = t.norm();
    const Vec3 zb = t / f;

    Vec3 zbd = Vec3::Zero();
    if (mode != FlatMode::Look) {
        const double spg = std::max(sp, 1e-9);
        const double va = v.dot(a);
        const Vec2 td2 = j + cd * (sp * a + v * (va / spg)) + kl * a;
        const Vec3 td(td2.x(), td2.y(), 0.0);
        zbd = (td - zb * zb.dot(td)) / f;
        if (mode == FlatMode::Rate) {
            S.rateXY = zbd.norm();
            return S;
        }
    }

    const double cps = std::cos(psi), sps = std::sin(psi);
    const Vec3 yc(-sps, cps, 0.0);
    const Vec3 u = cross3(yc, zb);
    const double nu = u.norm();
    const Vec3 xb = u / nu;
    const Vec3 yb = cross3(zb, xb);
    const double tau = q.camTilt;
    const Vec3 dw = std::sin(tau) * xb - std::cos(tau) * zb;
    const double cz = -dw.z();
    S.look = std::acos(std::min(std::max(cz, -1.0), 1.0));
    const double lam = h / std::max(cz, std::cos(deg2rad(85.0)));
    S.bore = lam * dw.head<2>();
    S.tilt = std::atan2(t.head<2>().norm(), g);
    if (mode == FlatMode::Look) return S;

    S.t = t;
    S.f = f;
    S.xb = xb;
    S.yb = yb;
    S.zb = zb;
    S.dw = dw;
    const double pR = -yb.dot(zbd), qR = xb.dot(zbd);
    const Vec3 xc(cps, sps, 0.0);
    const Vec3 ud = -psiDot * cross3(xc, zb) + cross3(yc, zbd);
    const double rR = ud.dot(yb) / nu;
    S.rates = Vec3(pR, qR, rR);
    S.rateXY = std::hypot(pR, qR);
    // ZYX Euler of R = [xb yb zb]: theta = asin(-R31), phi = atan2(R32, R33)
    const double thetaFLU = std::asin(std::min(std::max(-xb.z(), -1.0), 1.0));
    const double phi = std::atan2(yb.z(), zb.z());
    double yawE = std::atan2(xb.y(), xb.x());
    yawE = psi + std::atan2(std::sin(yawE - psi), std::cos(yawE - psi));
    S.rpy = Vec3(phi, -thetaFLU, yawE);
    return S;
}

}  // namespace mtl::curve::quadrotor
