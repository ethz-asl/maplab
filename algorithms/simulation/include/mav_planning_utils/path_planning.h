/*

 Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef MAV_PLANNING_UTILS_PATH_PLANNING_H_
#define MAV_PLANNING_UTILS_PATH_PLANNING_H_

#include <algorithm>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <mav_planning_utils/motion4D.h>
#include <mav_planning_utils/motion_defines.h>
#include <mav_planning_utils/polynomial.h>

#include <glog/logging.h>

namespace mav_planning_utils {

namespace path_planning {

static constexpr int kDefaultN = 12;

template <int _D = 1>
class Vertex {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static const int D = _D;
  typedef Eigen::Matrix<double, D, 1> ConstraintValueT;
  typedef std::pair<int, ConstraintValueT> Constraint;
  typedef std::map<int, ConstraintValueT, std::less<int>,
                   Eigen::aligned_allocator<std::pair<int, ConstraintValueT> > >
      Constraints;

  double time_to_next;
  int derivative_to_optimize;
  Constraints constraints;

  /// constructs an empty vertex and sets time_to_next and
  /// derivative_to_optimize to zero
  Vertex() : time_to_next(0), derivative_to_optimize(0) {}

  /// constructs an empty vertex and takes arguments for time_to_next and
  /// derivative_to_optimize
  Vertex(double _time_to_next, int _derivative_to_optimze)
      : time_to_next(_time_to_next),
        derivative_to_optimize(_derivative_to_optimze) {}

  /// constructs a vertex, takes arguments for time_to_next and
  /// derivative_to_optimize; sets constraints up to derivatives to zero
  /**
   * This is useful for beginning and end vertices
   */
  Vertex(double _time_to_next, int _derivative_to_optimze, int derivatives)
      : time_to_next(_time_to_next),
        derivative_to_optimize(_derivative_to_optimze) {
    setAllZero(derivatives);
  }

  /// adds a constraint for the derivative specified in type with the given
  /// value
  /**
   * if this is a multi-dimensional vertex, all derivatives get set to the same
   * value
   */
  void addConstraint(int type, double value) {
    constraints[type] = ConstraintValueT::Constant(value);
  }

  /// adds a constraint for the derivative specified in type with the given
  /// values in c
  /**
   * the constraints for the specified derivative get set to the values in the
   * vector c. the dimension has to match the
   * dimension of the vertex
   */
  template <class Derived>
  void addConstraint(int type, const Eigen::MatrixBase<Derived>& c) {
    constraints[type] = c;
  }

  /// sets all derivatives up to (including) up_to_type to zero; useful for
  /// beginning / end vertices
  void setAllZero(int up_to_type) {
    for (int i = 0; i <= up_to_type; ++i) {
      constraints[i] = ConstraintValueT::Zero();
    }
  }

  /// returns if the vertex has a constraint specified by type
  bool hasConstraint(int type) const {
    typename Constraints::const_iterator it = constraints.find(type);
    return it != constraints.end();
  }

  /// returns the value of the constraint specified with type. If the constraint
  /// is not set, it returns zeros --> \todo
  ConstraintValueT getConstraint(int type) const {
    typename Constraints::const_iterator it = constraints.find(type);
    if (it != constraints.end())
      return it->second;
    else
      return ConstraintValueT::Zero();  // TODO(unknown): throw exception ...
  }

  /// returns a const iterator to the first constraint
  typename Constraints::const_iterator cBegin() const {
    return constraints.begin();
  }

  /// returns a const iterator to the end of the constraints, i.e. one after the
  /// last element
  typename Constraints::const_iterator cEnd() const {
    return constraints.end();
  }

  /// Checks if both lhs and rhs are equal up to tol in case of double values.
  bool isEqualTol(const Vertex<D>& rhs, double tol) const {
    if (std::abs(time_to_next - rhs.time_to_next) > tol)
      return false;
    if (derivative_to_optimize != rhs.derivative_to_optimize)
      return false;
    if (constraints.size() != rhs.constraints.size())
      return false;
    // loop through lhs constraint map
    for (typename Constraints::const_iterator it = cBegin(); it != cEnd();
         ++it) {
      // look for matching key
      typename Constraints::const_iterator rhs_it =
          rhs.constraints.find(it->first);
      if (rhs_it == rhs.constraints.end())
        return false;
      // check value
      if (!((it->second - rhs_it->second).isZero(tol)))
        return false;
    }
    return true;
  }
};

/**
 * \brief Struct that stores waypoints and maximum values for the derivatives of
 * position / yaw
 * (if applicable). 0 means no limit.
 */
template <int _D>
class Waypoints {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { D = _D };
  typedef std::vector<Vertex<D>, Eigen::aligned_allocator<Vertex<D> > >
      WaypointVector;

  double v_max;
  double a_max;
  double j_max;
  double s_max;
  double yaw_dot_max;
  WaypointVector waypoints;

  Waypoints() : v_max(0), a_max(0), j_max(0), s_max(0), yaw_dot_max(0) {}
  Waypoints(
      double _v_max, double _a_max, double _j_max, double _s_max,
      double _yaw_dot_max)
      : v_max(_v_max),
        a_max(_a_max),
        j_max(_j_max),
        s_max(_s_max),
        yaw_dot_max(_yaw_dot_max) {}

  /// Checks if both lhs and rhs are equal up to tol in case of double values.
  bool isEqualTol(const Waypoints<D>& rhs, double tol) const {
    if (std::abs(v_max - rhs.v_max) > tol)
      return false;
    if (std::abs(a_max - rhs.a_max) > tol)
      return false;
    if (std::abs(j_max - rhs.j_max) > tol)
      return false;
    if (std::abs(s_max - rhs.s_max) > tol)
      return false;
    if (std::abs(yaw_dot_max - rhs.yaw_dot_max) > tol)
      return false;

    if (waypoints.size() != rhs.waypoints.size())
      return false;

    for (size_t i = 0; i < waypoints.size(); ++i) {
      if (!waypoints[i].isEqualTol(rhs.waypoints[i], tol))
        return false;
    }
    return true;
  }
};

template <int D>
std::ostream& operator<<(std::ostream& stream, const Vertex<D>& v) {
  stream << "time_to_next: " << v.time_to_next << std::endl;
  stream << "derivative_to_optimize: ";
  stream << mav_planning_utils::DerivativesP::toString(v.derivative_to_optimize)
         << std::endl;
  stream << "constraints: " << std::endl;

  Eigen::IOFormat format(4, 0, ", ", "\n", "[", "]");
  for (typename Vertex<D>::Constraints::const_iterator it = v.cBegin();
       it != v.cEnd(); ++it) {
    stream << "  type: "
           << mav_planning_utils::DerivativesP::toString(it->first);
    stream << "  value: " << it->second.transpose().format(format) << std::endl;
  }
  return stream;
}

template <int D>
std::ostream& operator<<(std::ostream& stream, const Waypoints<D>& w) {
  stream << "v_max: " << w.v_max << std::endl;
  stream << "a_max: " << w.a_max << std::endl;
  stream << "j_max: " << w.j_max << std::endl;
  stream << "s_max: " << w.s_max << std::endl;
  stream << "yaw_dot_max: " << w.yaw_dot_max << std::endl;

  stream << "vertices: " << std::endl;

  for (typename Waypoints<D>::WaypointVector::const_iterator it =
           w.waypoints.cbegin();
       it != w.waypoints.cend(); ++it) {
    stream << "  vertex: \n" << *it << std::endl;
  }
  return stream;
}

typedef Vertex<1> Vertex1D;  // < Single axis.
typedef Vertex<3> Vertex3D;  // < 3D position.
typedef Vertex<4> Vertex4D;  // < 3D position and yaw.

typedef Waypoints<1> Waypoints1D;  // < Single axis.
typedef Waypoints<3> Waypoints3D;  // < 3D position.
typedef Waypoints<4> Waypoints4D;  // < 3D position and yaw.

template <int _N>
class Segment {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { N = _N };

  typedef Polynomial<N> Type;
  typedef std::vector<Segment<N>, Eigen::aligned_allocator<Segment<N> > >
      Vector;

  Type p;
  double t;
  Segment() : t(0) {}

  Segment(const Type& _p, double _t) : p(_p), t(_t) {}
};

template <int N>
typename Segment<N>::Vector optimizePolynomials1D(
    const std::vector<Vertex1D, Eigen::aligned_allocator<Vertex1D> >& vertices,
    int continuity, double /*tol*/) {
  typedef Polynomial<N> P;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Q_t;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> x_t;

  typedef Vertex1D::Constraints::const_iterator CIt;

  const int n_vertices = vertices.size();

  typename Segment<N>::Vector result;

  const int n_all = (n_vertices - 1) * N * 2;

  Q_t Q(n_all, n_all);
  x_t b(n_all), x(n_all);

  Q.setZero();
  b.setZero();

  int start_pos = N * (n_vertices - 1);
  for (int i = 0; i < n_vertices - 1; ++i) {
    int n_c = 0;
    const Vertex1D& cv = vertices[i];

    // fill in cost jacobian
    // equality constraints
    P::quadraticCostJacobian(
        Q.block<N, N>(i * N, i * N), cv.time_to_next,
        cv.derivative_to_optimize);

    // current vertex
    for (CIt it = cv.cBegin(); it != cv.cEnd(); ++it) {
      if (n_c > n_all / 2) {
        std::cout << "too many constraints: " << n_c << ", but only "
                  << n_all / 2 << " available\n";
        return result;
      }

      Q.block(start_pos + n_c, N * i, 1, N) =
          P::baseCoeffsWithTime(it->first, 0);
      b[start_pos + n_c] = it->second[0];
      ++n_c;
    }

    //      std::cout << n_c << "constraints begin\n";

    // next vertex
    if (i < n_vertices - 1) {
      const Vertex1D& nv = vertices[i + 1];
      for (CIt it = nv.cBegin(); it != nv.cEnd(); ++it) {
        if (n_c > n_all / 2) {
          std::cout << "too many constraints: " << n_c << ", but only "
                    << n_all / 2 << " available\n";
          return result;
        }

        Q.block(start_pos + n_c, N * i, 1, N) =
            P::baseCoeffsWithTime(it->first, cv.time_to_next);
        b[start_pos + n_c] = it->second[0];
        ++n_c;
      }
      //        std::cout << n_c << "constraints end\n";
    }
    // continuity constraints, only for segments that do not yield to the end
    // vertex
    if (i < n_vertices - 2) {
      const Vertex1D& nv = vertices[i + 1];
      for (int d = 0; d <= continuity; ++d) {
        if (!nv.hasConstraint(d)) {
          if (n_c > n_all / 2) {
            std::cout << "too many constraints: " << n_c << ", but only "
                      << n_all / 2 << " available\n";
            return result;
          }

          Q.block(start_pos + n_c, N * i, 1, N) =
              P::baseCoeffsWithTime(d, cv.time_to_next);
          Q.block(start_pos + n_c, N * i + N, 1, N) =
              P::baseCoeffsWithTime(d, 0) * (-1.0);
          b[start_pos + n_c] = 0;
          ++n_c;
        }
      }
      //        std::cout << n_c << "constraints cont\n";

      // mirror A on diagonal including continuity constraints
      Q.block(N * i, start_pos, 2 * N, n_c) =
          Q.block(start_pos, N * i, n_c, N * 2).transpose();
    } else {
      // mirror A on diagonal
      Q.block(N * i, start_pos, N, n_c) =
          Q.block(start_pos, N * i, n_c, N).transpose();
    }

    start_pos += n_c;
  }

  Q_t _Q(start_pos, start_pos);
  _Q = Q.block(0, 0, start_pos, start_pos);
  x_t _b = b.head(start_pos);
  x = pseudoInverseSolver(_Q, _b);

  result.resize(n_vertices - 1);
  for (int i = 0; i < n_vertices - 1; ++i) {
    //      result.push_back(Segment<N>(P(x.template segment<N>(i * N)),
    //      vertices[i].time_to_next));
    result[i].p = P(x.template segment<N>(i * N));
    result[i].t = vertices[i].time_to_next;
  }
  return result;
}

template <int N, class Derived>
bool samplePath(
    const Eigen::MatrixBase<Derived>& result,
    const std::vector<Segment<N>, Eigen::aligned_allocator<Segment<N> > >& path,
    double t) {
  double t_a = 0;
  typename Segment<N>::Vector::const_iterator it;

  for (it = path.begin(); it != path.end(); ++it) {
    t_a += it->t;
    if (t_a - t >= -1e-12)
      break;
  }
  if (it == path.end()) {
    std::cout << "t longer than available path time:" << t << " / " << t_a
              << std::endl;
    return false;
  }

  it->p.evaluate(result, t - (t_a - it->t));
  return true;
}

template <int _N>
class Path4D {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { N = _N };
  typedef Segment<N> SegmentType;
  typedef typename SegmentType::Vector SegmentVector;
  typedef Aligned<std::vector, mav_planning_utils::path_planning::Vertex4D>
      Vertex4DList;

 private:
  SegmentVector sx_;
  SegmentVector sy_;
  SegmentVector sz_;
  SegmentVector syaw_;
  Vertex4DList vertices_;

  /// turns 4D vertices into a 1D vertices determined by DoF for better internal
  /// handling
  std::vector<Vertex1D, Eigen::aligned_allocator<Vertex1D> > vertices4Dto1D(
      const std::vector<Vertex4D, Eigen::aligned_allocator<Vertex4D> >& v4,
      unsigned int DoF) {
    assert(DoF < 4);
    std::vector<Vertex1D, Eigen::aligned_allocator<Vertex1D> > v1;
    v1.resize(v4.size());
    for (size_t i = 0; i < v4.size(); ++i) {
      for (Vertex4D::Constraints::const_iterator it = v4[i].cBegin();
           it != v4[i].cEnd(); ++it) {
        v1[i].addConstraint(it->first, it->second[DoF]);
      }
      v1[i].derivative_to_optimize = v4[i].derivative_to_optimize;
      v1[i].time_to_next = v4[i].time_to_next;
    }

    return v1;
  }

  //    template<int N>
  //      Eigen::Matrix<double, 4, 1> sampleSegment(const Segment<N, 4> & s,
  //      double t, int derivative)
  //      {
  //        // TODO: check max derivative
  //        Eigen::Matrix<double, 4, 1> res;
  //        for (int i = 0; i < 4; ++i)
  //          res[i] = s.p[i].evaluate(t, derivative);
  //
  //        return res;
  //      }

 public:
  /// standard constructor, doesn't initialize anything
  Path4D() {}

  /// initializes the internal segments with the arguments, so that sample() can
  /// be used
  Path4D(
      const SegmentVector& sx, const SegmentVector& sy, SegmentVector& sz,
      const SegmentVector& syaw)
      : sx_(sx), sy_(sy), sz_(sz), syaw_(syaw) {
    assert(sx_.size() == sy_.size());
    assert(sx_.size() == sz_.size());
    assert(sx_.size() == syaw_.size());
  }

  /**
   * \brief plans a path through the points given in vertices.
   * For obtaining path data \sa void sample(), getSegmentsX(), getVertices()
   * \param vertices Vertices to plan the path through
   * \param continuity Defines up to which derivative the path should be
   * continuous. Use DerivativeP for that.
   * \param tol Tolerance for Pseudo Inverse
   */
  bool optimize(
      const std::vector<Vertex4D, Eigen::aligned_allocator<Vertex4D> >&
          vertices,
      int continuity, double tol = 1e12) {
    std::vector<Vertex1D, Eigen::aligned_allocator<Vertex1D> > v1;
    v1 = vertices4Dto1D(vertices, 0);
    sx_ = optimizePolynomials1D<N>(v1, continuity, tol);
    v1 = vertices4Dto1D(vertices, 1);
    sy_ = optimizePolynomials1D<N>(v1, continuity, tol);
    v1 = vertices4Dto1D(vertices, 2);
    sz_ = optimizePolynomials1D<N>(v1, continuity, tol);
    v1 = vertices4Dto1D(vertices, 3);
    syaw_ = optimizePolynomials1D<N>(v1, continuity, tol);

    return true;
  }

  /**
   * \brief computes a scaling factor by which every segment time has to be
   * scaled in order to stay within the given bounds
   * \param max_p Vertex with "constraints" for the motion derivatives. At least
   * a constraint for velocity is required.
   * \param max_yaw Vertex with "constraints" for the motion derivatives. At
   * least a constraint for angular velocity is required.
   * \return vector with time scaling factors for each segment
   */
  std::vector<double> computeTimeScales(
      const Vertex1D& max_p, const Vertex1D& max_yaw) {
    std::vector<double> ts(sx_.size());
    for (size_t is = 0; is < sx_.size(); ++is) {
      double time_scale = 0;
      for (double t = 0; t < sx_[is].t; t += 0.1) {
        for (typename Vertex1D::Constraints::const_iterator it = max_p.cBegin();
             it != max_p.cEnd(); ++it) {
          const double x = sx_[is].p.evaluate(t, it->first);
          const double y = sy_[is].p.evaluate(t, it->first);
          const double z = sz_[is].p.evaluate(t, it->first);

          double val = sqrt(x * x + y * y + z * z) / it->second[0];
          //            std::cout<<"val "<<sqrt(x * x + y * y + z * z)<<"
          //            timescale "<< time_scale<<std::endl;
          val = pow(val, 1.0 / static_cast<double>(it->first));
          if (val > time_scale)
            time_scale = val;
        }

        for (typename Vertex1D::Constraints::const_iterator it =
                 max_yaw.cBegin();
             it != max_yaw.cEnd(); ++it) {
          const double yaw = syaw_[is].p.evaluate(t, it->first);

          double val = std::abs(yaw / it->second[0]);
          val = pow(val, 1.0 / static_cast<double>(it->first));
          if (val > time_scale)
            time_scale = val;
        }
      }

      ts[is] = time_scale;
    }
    return ts;
  }

  /**
   * \brief plans a path through the points given in vertices and optimized the
   * segment times according to the given limits in max_p and max_yaw
   * The time_to_next members in vertices are ignored. In a first step, rough
   * guesses of the segment times are made
   * based on the maximum velocity and a first path is planned. Then, it gets
   * checked for limit violations, segment
   * times are scaled accordingly and the path gets re-planned based on the new
   * times.
   * For obtaining path data \sa sample(), getSegmentsX(), getVertices()
   * \param vertices Vertices to plan the path through
   * \param continuity Defines up to which derivative the path should be
   * continuous. Use DerivativeP for that.
   * \param max_p Vertex with "constraints" for the motion derivatives. At least
   * a constraint for velocity is required.
   * \param max_yaw Vertex with "constraints" for the motion derivatives. At
   * least a constraint for angular velocity is required.
   * \param time_multiplier If not 0, the optimized segment times get ceiled to
   * a multiple of time_multiplier. To ease sampling, it's useful to set this to
   * the same value as dt in sample().
   * \param tol Tolerance for Pseudo Inverse
   */
  bool optimizeWithTime(
      const std::vector<Vertex4D, Eigen::aligned_allocator<Vertex4D> >&
          vertices,
      int continuity, const Vertex1D& max_p, const Vertex1D& max_yaw,
      double time_multiplier = 0.1, double tol = 1e12) {
    if (!(max_p.hasConstraint(DerivativesP::v) &&
          max_yaw.hasConstraint(DerivativesO::w))) {
      std::cout << "need velocity constraints for this to work\n";
      return false;
    }

    const double v_max_p = max_p.getConstraint(DerivativesP::v)[0];
    const double v_max_yaw = max_yaw.getConstraint(DerivativesO::w)[0];

    if (!(v_max_p > 0) && (v_max_yaw > 0)) {
      std::cout << "maximum velocities have to be > 0, got " << v_max_p << " "
                << v_max_yaw << std::endl;
    }

    vertices_ = vertices;

    // rough guess of time first:
    for (std::vector<Vertex4D, Eigen::aligned_allocator<Vertex4D> >::iterator
             it = vertices_.begin();
         it != (vertices_.end() - 1); ++it) {
      if (!it->hasConstraint(DerivativesP::p)) {
        std::cout << "need position / yaw constraints for this to work\n";
        return false;
      }

      std::vector<Vertex4D, Eigen::aligned_allocator<Vertex4D> >::const_iterator
          it_next = it + 1;
      const Vertex4D::ConstraintValueT cc = it->getConstraint(DerivativesP::p);
      const Vertex4D::ConstraintValueT nc =
          it_next->getConstraint(DerivativesP::p);

      const double tp = (cc.head<3>() - nc.head<3>()).norm() / v_max_p;
      const double ty =
          std::abs((cc.tail<1>() - nc.tail<1>())[0]) /
          v_max_yaw;  // no pi check here, we might want to do multiple turns
      it->time_to_next = std::max(tp, ty) * 2;
      //        std::cout<<"first guess: "<<it->time_to_next<<std::endl;
    }

    bool ret = optimize(vertices_, continuity, tol);

    if (!ret)
      return false;

    std::vector<double> time_scales = computeTimeScales(max_p, max_yaw);
    for (size_t is = 0; is < sx_.size(); ++is) {
      vertices_[is].time_to_next *= time_scales[is];

      if (time_multiplier > 0) {
        vertices_[is].time_to_next =
            ceil(vertices_[is].time_to_next / time_multiplier) *
            time_multiplier;
      }

      //        std::cout<<"old time "<<
      //        vertices_[is].time_to_next/time_scale<<"new time "<<
      //        vertices_[is].time_to_next<<std::endl;
    }

    return optimize(vertices_, continuity, tol);
  }

  /**
   * \brief samples the whole path obtained by optimize() or optimizeWithTime()
   * in steps of dt
   * \tparam n_p number of derivatives to sample for position. Implicitly
   * obtained by result
   * \tparam n_y number of derivatives to sample for position. Implicitly
   * obtained by result
   * \param[out] result vector with a Motion4D element for each sample
   * \param[in] dt timestep to sample
   */
  template <int n_p, int n_y>
  void sample(
      typename Motion4D<n_p, n_y>::Vector& result, double dt,
      Eigen::VectorXd* timestamps_seconds) const {
    CHECK_NOTNULL(timestamps_seconds);
    double path_time = 0;

    for (typename SegmentVector::const_iterator it = sx_.begin();
         it != sx_.end(); ++it) {
      path_time += it->t;
    }

    VLOG(3) << "path time: " << path_time;
    int n_samples = (path_time + dt * 0.5) / dt + 1;

    result.resize(n_samples);
    timestamps_seconds->resize(n_samples);

    VLOG(3) << "num samples: " << n_samples;

    double t = 0;
    for (int i = 0; i < n_samples; ++i) {
      Motion4D<n_p, n_y>& sample = result[i];
      samplePath(sample.x, sx_, t);
      samplePath(sample.y, sy_, t);
      samplePath(sample.z, sz_, t);
      samplePath(sample.yaw, syaw_, t);
      (*timestamps_seconds)(i) = t;
      t += dt;
    }
  }

  /// returns the time-optimized vertices after a optimizeWithTime() call
  inline const Vertex4DList& getVertices() const {
    return vertices_;
  }

  inline Vertex4DList* getVerticesPtr() {
    return &vertices_;
  }

  /// returns a vector of polynomial segments for the x-axis
  inline const SegmentVector& getSegmentsX() const {
    return sx_;
  }

  inline SegmentVector* getSegmentsXPtr() {
    return &sx_;
  }

  /// returns a vector of polynomial segments for the y-axis
  inline const SegmentVector& getSegmentsY() const {
    return sy_;
  }

  inline SegmentVector* getSegmentsYPtr() {
    return &sy_;
  }

  /// returns a vector of polynomial segments for the z-axis
  inline const SegmentVector& getSegmentsZ() const {
    return sz_;
  }

  inline SegmentVector* getSegmentsZPtr() {
    return &sz_;
  }

  /// returns a vector of polynomial segments for yaw
  inline const SegmentVector& getSegmentsYaw() const {
    return syaw_;
  }

  inline SegmentVector* getSegmentsYawPtr() {
    return &syaw_;
  }
};

}  // end namespace path_planning

}  // namespace mav_planning_utils

#endif  // MAV_PLANNING_UTILS_PATH_PLANNING_H_
