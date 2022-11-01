/*
 * Based on:
 *
 * Copyright (c) 2014, 2015, 2016, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef LOCALIZATION_FUSION_LOCALIZATION_FILTER_H_
#define LOCALIZATION_FUSION_LOCALIZATION_FILTER_H_

#include "localization-fusion/filter-base.h"
#include "localization-fusion/filter-common.h"

#include <fstream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <XmlRpcException.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <vio-common/pose-lookup-buffer.h>

namespace maplab {

template <class T>
class LocalizationFilter {
 public:
  //! @brief Constructor
  //!
  //! The RosFilter constructor makes sure that anyone using
  //! this template is doing so with the correct object type
  //!
  LocalizationFilter();

  //! @brief Destructor
  //!
  //! Clears out the message filters and topic subscribers.
  //!
  ~LocalizationFilter();

  //! @brief Resets the filter to its initial state
  //!
  void reset();

  void initialize(
      const common::LocalizationResult& T_G_M_init,
      const aslam::Transformation& T_M_B);

  //! @brief Callback method for receiving all localization messages
  //! @param[in] localization - The Localization Result from world - base_link
  //! @param[in] T_M_B_buffered - The odometry estimate from mission - base_link
  //!
  bool localizationCallback(
      const common::LocalizationResult& localization,
      const vio::ViNodeState& T_M_B_buffered);

  int64_t getLastMessageTimeNs() {
    return last_message_time_;
  }

  void getFusedLocalization(
      common::LocalizationResult* fused_localization_result) const;

  inline bool isInitialized() const {
    return filter_.getInitializedStatus();
  }

 protected:
  //! @brief Converts a localization result for integration into the filter
  //! @param[in] msg - The localization message to prepare
  //! @param[out] localization_G_B - The localization in the desired format
  //! (base_link - global)
  void localizationResultToMeasurement(
      const common::LocalizationResult& msg,
      LocalizationFilterMeasurement& localization_G_B);  // NOLINT

  void aslamTransformationToStateVector(
      const aslam::Transformation& transformation,
      Eigen::VectorXd& state_vector) const;  // NOLINT

  std::atomic<int64_t> last_message_time_;

  common::LocalizationMode localization_mode_;

 private:
  //! @brief inverse of the most recent odometry estimate from the buffer
  aslam::Transformation T_B_M_;

  //! @brief the most recent state estimate
  aslam::Transformation T_G_B_fused_;

 protected:
  //! @brief Our filter (EKF, UKF, etc.)
  T filter_;

  //! @brief Which entries of the filter state are being updated by a
  //! measurement.
  //! Currently set to true for all of them, but might be useful for extensions
  //! of the filter state later on.
  std::vector<size_t> update_vector_;
};

}  // namespace maplab

#endif  // LOCALIZATION_FUSION_LOCALIZATION_FILTER_H_
