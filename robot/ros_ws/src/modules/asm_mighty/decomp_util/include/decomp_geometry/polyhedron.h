/**
 * @file polygon.h
 * @brief Polygon class
 */

#ifndef DECOMP_POLYGON_H
#define DECOMP_POLYGON_H

#include <decomp_basis/data_type.h>

///Hyperplane class
template <int Dim>
struct Hyperplane {
  Hyperplane() {}
  Hyperplane(const Vecf<Dim>& p, const Vecf<Dim>& n) : p_(p), n_(n) {}

  /// Calculate the signed distance from point
  decimal_t signed_dist(const Vecf<Dim>& pt) const {
    return n_.dot(pt - p_);
  }

  /// Calculate the distance from point
  decimal_t dist(const Vecf<Dim>& pt) const {
    return std::abs(signed_dist(pt));
  }

  /// Point on the plane
  Vecf<Dim> p_;
  /// Normal of the plane, directional
  Vecf<Dim> n_;
};

///Hyperplane2D: first is the point on the hyperplane, second is the normal
typedef Hyperplane<2> Hyperplane2D;
///Hyperplane3D: first is the point on the hyperplane, second is the normal
typedef Hyperplane<3> Hyperplane3D;


///Polyhedron class
template <int Dim>
struct Polyhedron {
  ///Null constructor
  Polyhedron() {}
  ///Construct from Hyperplane array
  Polyhedron(const vec_E<Hyperplane<Dim>>& vs) : vs_(vs) {}


  ///Append Hyperplane
  void add(const Hyperplane<Dim>& v) {
    vs_.push_back(v);
  }

  /// Check if the point is inside polyhedron, non-exclusive
  bool inside(const Vecf<Dim>& pt) const {
    for (const auto& v : vs_) {
      if (v.signed_dist(pt) > epsilon_) {
        //printf("rejected pt: (%f, %f), d: %f\n",pt(0), pt(1), v.signed_dist(pt));
        return false;
      }
    }
    return true;
  }

  /// Calculate points inside polyhedron, non-exclusive
  vec_Vecf<Dim> points_inside(const vec_Vecf<Dim> &O) const {
    vec_Vecf<Dim> new_O;
    for (const auto &it : O) {
      if (inside(it))
        new_O.push_back(it);
    }
    return new_O;
  }

  /// Calculate normals, used for visualization
  vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> cal_normals() const {
    vec_E<std::pair<Vecf<Dim>, Vecf<Dim>>> ns(vs_.size());
    for (size_t i = 0; i < vs_.size(); i++)
      ns[i] = std::make_pair(vs_[i].p_, vs_[i].n_); // fist is point, second is normal
    return ns;
  }

  /// Get the hyperplane array
  vec_E<Hyperplane<Dim>> hyperplanes() const {
    return vs_;
  }

  /// Hyperplane array
  vec_E<Hyperplane<Dim>> vs_; // normal must go outside

};

///Polyhedron2D, consists of 2D hyperplane
typedef Polyhedron<2> Polyhedron2D;
///Polyhedron3D, consists of 3D hyperplane
typedef Polyhedron<3> Polyhedron3D;

// Forward declaration (cal_vertices are defined in geometric_utils.h but geometric_utils.h depends on polygon.h so I cannot cyclically include - but I need to use ca_vertices in polygon.h so I here forward declare it)
inline vec_E<vec_Vec3f> cal_vertices(const Polyhedron3D &poly);

///[A, b] for \f$Ax < b\f$
template <int Dim>
struct LinearConstraint {
  ///Null constructor
  LinearConstraint() {}
  /// Construct from \f$A, b\f$ directly, s.t \f$Ax < b\f$
  LinearConstraint(const MatDNf<Dim>& A, const VecDf& b) : A_(A), b_(b) {}
  /**
   * @brief Construct from a inside point and hyperplane array
   * @param p0 point that is inside
   * @param vs hyperplane array, normal should go outside
   */
	LinearConstraint(const Vecf<Dim> p0, const vec_E<Hyperplane<Dim>>& vs) {
		const unsigned int size = vs.size();
		MatDNf<Dim> A(size, Dim);
		VecDf b(size);

		for (unsigned int i = 0; i < size; i++) {
			auto n = vs[i].n_;
			decimal_t c = vs[i].p_.dot(n);
			if (n.dot(p0) - c > 0) {
				n = -n;
				c = -c;
			}
			A.row(i) = n;
			b(i) = c;
		}

		A_ = A;
		b_ = b;
	}

  LinearConstraint(const Vecf<Dim> p0, const vec_E<Hyperplane<Dim>>& vs, const Polyhedron<Dim>& poly) {
		const unsigned int size = vs.size();
		MatDNf<Dim> A(size, Dim);
		VecDf b(size);

		for (unsigned int i = 0; i < size; i++) {
			auto n = vs[i].n_;
			decimal_t c = vs[i].p_.dot(n);
			if (n.dot(p0) - c > 0) {
				n = -n;
				c = -c;
			}
			A.row(i) = n;
			b(i) = c;
		}

		A_ = A;
		b_ = b;
    poly_ = poly;
	}

  /// Check if the point is inside polyhedron using linear constraint
  inline bool inside(const Vecf<Dim> &pt) const {
    VecDf d = A_ * pt - b_;
    for (unsigned int i = 0; i < d.rows(); i++) {
      if (d(i) > 0)
        return false;
    }
    return true;
  }

  bool getMeanPoint(Vecf<3>& mean) const {
    
    // Use cal_vertices() to get the vertices for each face of the polyhedron.
    vec_E<vec_Vec3f> polygons = cal_vertices(poly_);
    
    // Flatten all vertices into one list.
    std::vector<Vec3f> vertices;
    for (const auto &polygon : polygons) {
      for (const auto &pt : polygon) {
        vertices.push_back(pt);
      }
    }
    
    if (vertices.empty()) {
      return false;
    }
    
    // Compute the arithmetic mean of all vertices.
    // Vecf<3> mean = Vecf<3>::Zero();
    for (const auto &pt : vertices) {
      mean += pt;
    }
    mean /= static_cast<decimal_t>(vertices.size());
    
    return true;
  }

  /// Get \f$A\f$ matrix
  MatDNf<Dim> A() const { return A_; }

  /// Get \f$b\f$ matrix
  VecDf b() const { return b_; }

  MatDNf<Dim> A_;
  VecDf b_;
  Polyhedron<Dim> poly_;
};

///LinearConstraint 2D
typedef LinearConstraint<2> LinearConstraint2D;
///LinearConstraint 3D
typedef LinearConstraint<3> LinearConstraint3D;

#endif
