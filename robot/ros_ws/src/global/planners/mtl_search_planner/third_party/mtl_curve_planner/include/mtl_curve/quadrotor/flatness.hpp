// =============================================================================
//  mtl_curve/quadrotor/flatness.hpp
//
//  Differential flatness of a quadrotor with a body-fixed camera - the port of
//  quadResolveOpts.m, uniformBsplineMatrices.m and quadFlatMap.m.
//
//  A quadrotor is flat in sigma = (x, y, z, psi): its state and inputs are
//  algebraic functions of sigma and its derivatives (Mellinger & Kumar 2011).
//  With a drag that depends on velocity only (Faessler et al. 2018) this still
//  holds:
//      m p'' = -m g e3 + T z_b - m D(p'),     D(v) = (c_d |v| + k_l) v
//    thrust vector  t   = p'' + g e3 + D(p')       (mass-normalised), f = |t|
//    body axes      z_b = t/|t|,  x_b = (y_c x z_b)/|y_c x z_b|,  y_b = z_b x x_b,
//                   y_c = [-sin psi, cos psi, 0]    (ZYX Euler yaw = psi exactly)
//    body rates     z_b' = (t' - z_b (z_b . t'))/f,  t' = p''' + dD/dv p''
//                   p = -y_b . z_b',  q = x_b . z_b',
//                   r = ((-psi' x_c x z_b + y_c x z_b') . y_b) / |y_c x z_b|
//  The altitude is held (z = h), so v, a, j are horizontal and t_z = g.
//
//  The camera is tilted forward by camTilt from body -z toward body +x (FLU);
//  its boresight d = sin(tau) x_b - cos(tau) z_b meets the ground at
//  [x y] + h/(-d_z) [d_x d_y]; the look (off-nadir) angle is acos(-d_z).
// =============================================================================
#ifndef MTLC_QUADROTOR_FLATNESS_HPP
#define MTLC_QUADROTOR_FLATNESS_HPP

#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::quadrotor {

/// Defaults already live in QuadParams; this fills the derived fields:
/// dragQuadResolved (from dragSpecSpeed / dragSpecTilt unless dragQuad is
/// set), trimPitch = atan(drag(speed)/g), camTilt per cameraMount ('Matched':
/// fwTiltAngle + trimPitch, so in steady cruise the camera looks where the
/// fixed-wing gimbal's centre does).  Throws if the cruise trim alone exceeds
/// the tilt limit.
QuadParams resolveQuadParams(const QuadParams& in, double fwTiltAngle);

/// A uniform, unclamped B-spline basis of degree p on [0, T] with knot spacing
/// delta, evaluated at sample times: f^(d)(t_k) = sum_i b[d](k, i) c[m[k] + i].
/// nCP = ceil(T/delta) + p coefficients.
struct UniformBspline {
    int    p = 5;
    double delta = 1.0, T = 0.0;
    Index  nCP = 0;
    std::vector<Index> m;   ///< first active coefficient per sample
    std::vector<MatX>  b;   ///< b[d] is K-by-(p+1), d = 0..maxDeriv

    Index samples() const { return static_cast<Index>(m.size()); }
    double eval(int d, Index k, const VecX& c) const;
    VecX   eval(int d, const VecX& c) const;
};

Index numCoefficients(double T, double delta, int p);
UniformBspline uniformBspline(const VecX& t, double delta, double T, int p, int maxDeriv);

enum class FlatMode { Full, Look, Rate };

/// Everything the flat outputs imply at one instant.
struct FlatState {
    Vec3   t = Vec3::Zero();            ///< thrust vector [m/s^2]
    double f = 0.0, tilt = 0.0;
    Vec3   xb = Vec3::Zero(), yb = Vec3::Zero(), zb = Vec3::Zero(), dw = Vec3::Zero();
    Vec3   rpy = Vec3::Zero();          ///< [roll pitch yaw], repo convention
    Vec3   rates = Vec3::Zero();        ///< [p q r] body (FLU)
    double rateXY = 0.0;                ///< |(p, q)|
    Vec2   bore = Vec2::Zero();         ///< boresight ground point minus [x y]
    double look = 0.0;                  ///< off-nadir [rad]
};

/// FlatMode::Look fills bore / look / tilt; FlatMode::Rate only rateXY.
FlatState flatMap(const Vec2& v, const Vec2& a, const Vec2& j, double psi, double psiDot, double h,
                  const QuadParams& q, FlatMode mode = FlatMode::Full);

}  // namespace mtl::curve::quadrotor

#endif  // MTLC_QUADROTOR_FLATNESS_HPP
