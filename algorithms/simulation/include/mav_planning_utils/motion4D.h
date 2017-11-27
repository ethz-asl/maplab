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

#ifndef MOTION4D_H_
#define MOTION4D_H_

#include <Eigen/Dense>
#include <vector>
#include <sstream>

#include <mav_planning_utils/polynomial.h>
#include <mav_planning_utils/motion_defines.h>

namespace mav_planning_utils
{

/// defines a container for 4D motion (x, y, z, yaw) and their respective derivatives of degree n_*
template<int n_pos = 5, int n_yaw = 3, class T = double>
  class Motion4D
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// defines a vector type for this class with the necessary eigen aligned allocator
    typedef std::vector<Motion4D<n_pos, n_yaw, T>, Eigen::aligned_allocator<Motion4D<n_pos, n_yaw, T> > > Vector;

    /// Position type definition
    typedef Eigen::Matrix<T, n_pos, 1> PositionType;

    /// Yaw type definition
    typedef Eigen::Matrix<T, n_yaw, 1> YawType;

    static const int N_POS = n_pos;
    static const int N_YAW = n_yaw;

    PositionType x; ///< stores the position derivatives in the x axis
    PositionType y; ///< stores the position derivatives in the y axis
    PositionType z; ///< stores the position derivatives in the z axis
    YawType yaw; ///< stores the yaw derivatives

    Motion4D()
    {
    }

    Motion4D(int size)
    {
      if (n_yaw == Eigen::Dynamic)
        yaw.resize(size, Eigen::NoChange);
      if (n_pos == Eigen::Dynamic)
      {
        x.resize(size, Eigen::NoChange);
        y.resize(size, Eigen::NoChange);
        z.resize(size, Eigen::NoChange);
      }
    }

    Motion4D(int size_pos, int size_yaw)
    {
      yaw.resize(size_yaw, Eigen::NoChange);
      x.resize(size_pos, Eigen::NoChange);
      y.resize(size_pos, Eigen::NoChange);
      z.resize(size_pos, Eigen::NoChange);
    }

    /// sets all vectors to zero
    void reset()
    {
      x.setZero();
      y.setZero();
      z.setZero();
      yaw.setZero();
    }

    /**
     * \brief Returns the given derivative of the position states
     */
    Eigen::Matrix<T, 3, 1> getStateP(unsigned int derivative) const
    {
      if (derivative >= x.size())
        return Eigen::Matrix<T, 3, 1>::Zero();
      else
        return Eigen::Matrix<T, 3, 1>(x[derivative], y[derivative], z[derivative]);
    }

    /**
     * \brief Sets the given derivative of the position states
     */
    template<class Derived>
      void setStateP(unsigned int derivative, const Eigen::MatrixBase<Derived> & dp)
      {
        EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived, 3);
        if (derivative < x.size())
        {
          x[derivative] = dp[0];
          y[derivative] = dp[1];
          z[derivative] = dp[2];
        }
      }

    /// returns a string of the vectors
    std::string toString() const
    {
      std::stringstream ss;
      ss << "x: " << x.transpose() << std::endl;
      ss << "y: " << y.transpose() << std::endl;
      ss << "z: " << z.transpose() << std::endl;
      ss << "yaw: " << yaw.transpose() << std::endl;
      return ss.str();
    }
  };

/// defines a container for 4D motion (x, y, z, yaw) and their respective derivatives of degree n_*
template<int n_pos = 12, int n_yaw = 8, class T = double>
  class Motion4DPolynomial
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// defines a vector type for this class with the necessary eigen aligned allocator
    typedef std::vector<Motion4DPolynomial<n_pos, n_yaw, T>, Eigen::aligned_allocator<Motion4DPolynomial<n_pos, n_yaw, T> > > Vector;

    /// Position type definition
    typedef Polynomial<n_pos, T> PositionType;

    /// Yaw type definition
    typedef Polynomial<n_yaw, T> YawType;

    enum {N_POS = n_pos, N_YAW = n_yaw};

    PositionType x; ///< stores the x axis polynomial
    PositionType y; ///< stores the y axis polynomial
    PositionType z; ///< stores the z axis polynomial
    YawType yaw; ///< stores the yaw polynomial

    /// sets all vectors to zero
    void reset()
    {
      x.setCoefficients(Eigen::Matrix<T, 1, n_pos>::Zero());
      y.setCoefficients(Eigen::Matrix<T, 1, n_pos>::Zero());
      z.setCoefficients(Eigen::Matrix<T, 1, n_pos>::Zero());
      yaw.setCoefficients(Eigen::Matrix<T, 1, n_yaw>::Zero());
    }

    /**
     * \brief Returns the given derivative of the position states
     */
    Eigen::Matrix<T, 3, 1> getStateP(T t, unsigned int derivative) const
    {
      if (derivative >= n_pos)
        return Eigen::Matrix<T, 3, 1>::Zero();
      else
        return Eigen::Matrix<T, 3, 1>(x.evaluate(t, derivative), y.evaluate(t, derivative), z.evaluate(t, derivative));
    }


    /// returns a string of the vectors
    std::string toString() const
    {
      std::stringstream ss;
      ss << "x: " << x.transpose() << std::endl;
      ss << "y: " << y.transpose() << std::endl;
      ss << "z: " << z.transpose() << std::endl;
      ss << "yaw: " << yaw.transpose() << std::endl;
      return ss.str();
    }
  };
}// end namespace

#endif /* MOTION4D_H_ */
