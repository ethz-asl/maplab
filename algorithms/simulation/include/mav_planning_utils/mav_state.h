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

#ifndef MAV_STATE_H_
#define MAV_STATE_H_

#include <Eigen/Dense>
#include <vector>

namespace mav_planning_utils
{

class MavState
{
  typedef std::vector<MavState, Eigen::aligned_allocator<MavState> >Vector;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void reset()
  {
    p.setZero();
    v.setZero();
    a.setZero();
    a_b.setZero();
    j.setZero();
    s.setZero();
    w_b.setZero();
    q.setIdentity();
  }

  std::string toString(){
    std::stringstream ss;
    ss << "p: " << p.transpose() << std::endl
        << "v: " << v.transpose() << std::endl
        << "a: " << a.transpose() << std::endl
        << "a_b: " << a_b.transpose() << std::endl
        << "j: " << j.transpose() << std::endl
        << "s: " << s.transpose() << std::endl
        << "w_b: " << w_b.transpose() << std::endl
        << "q (xyzw): " << q.coeffs().transpose() << std::endl;

    return ss.str();
  }

  Eigen::Quaternion<double> q;
  Eigen::Vector3d w_b;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d a_b;
  Eigen::Vector3d j;
  Eigen::Vector3d s;
};


}

#endif /* MAV_STATE_H_ */
