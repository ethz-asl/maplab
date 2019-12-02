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

#include <algorithm>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "localization-fusion/filter-utilities.h"
#include "localization-fusion/localization-filter.h"
#include "localization-fusion/ukf.h"

DEFINE_bool(
    loc_filter_reset_on_time_jump, false,
    "Whether to reset the localization filter if a localization arrives out of "
    "order.");

DEFINE_double(
    loc_filter_ukf_alpha, 0.001,
    "The alpha and kappa variables control the spread of the sigma points. "
    "Unless you are familiar with UKFs, it's probably a good idea to leave "
    "these alone.");
DEFINE_double(
    loc_filter_ukf_kappa, 2,
    "The alpha and kappa variables control the spread of the sigma points. "
    "Unless you are familiar with UKFs, it's probably a good idea to leave "
    "these alone.");
DEFINE_double(
    loc_filter_ukf_beta, 2,
    "The beta variable relates to the distribution of the state vector. Again, "
    "it's probably best to leave this alone if you're uncertain. Defaults to 2 "
    "if unspecified.");

namespace maplab {
template <typename T>
LocalizationFilter<T>::LocalizationFilter()
    : last_message_time_(aslam::time::getInvalidTime()),
      localization_mode_(common::LocalizationMode::kUnknown),
      filter_(
          FLAGS_loc_filter_ukf_alpha, FLAGS_loc_filter_ukf_kappa,
          FLAGS_loc_filter_ukf_beta),
      update_vector_(localization_fusion::STATE_SIZE, 0u) {
  // set update vector values to 6DoF pose for all inputs
  update_vector_[localization_fusion::StateMemberX] = 1u;
  update_vector_[localization_fusion::StateMemberY] = 1u;
  update_vector_[localization_fusion::StateMemberZ] = 1u;
  update_vector_[localization_fusion::StateMemberRoll] = 1u;
  update_vector_[localization_fusion::StateMemberPitch] = 1u;
  update_vector_[localization_fusion::StateMemberYaw] = 1u;
}

template <typename T>
LocalizationFilter<T>::~LocalizationFilter() {}

template <typename T>
void LocalizationFilter<T>::reset() {
  // reset filter to uninitialized state
  last_message_time_ = aslam::time::getInvalidTime();
  filter_.reset();
}

template <typename T>
bool LocalizationFilter<T>::localizationCallback(
    const common::LocalizationResult& localization,
    const vio::ViNodeState& T_M_B_buffered) {
  bool success = false;

  // Make sure this message is newer than the last one
  if (localization.timestamp_ns >= last_message_time_) {
    if (filter_.getInitializedStatus()) {
      // we can proceed safely
      last_message_time_ = localization.timestamp_ns;
      localization_mode_ = localization.localization_mode;

      // Convert the pose measurement
      LocalizationFilterMeasurement pose_measurement_G_B;
      localizationResultToMeasurement(localization, pose_measurement_G_B);

      // Convert the odometry to a LocalizationFilterPrediction
      aslam::Transformation T_G_B_estimated =
          T_G_B_fused_ * T_B_M_ * T_M_B_buffered.get_T_M_I();
      LocalizationFilterPrediction odom_as_prediction_G_B;
      aslamTransformationToStateVector(
          T_G_B_estimated, odom_as_prediction_G_B.predicted_state_);
      odom_as_prediction_G_B.covariance_ = T_M_B_buffered.getPoseCovariance();

      if (filter_.processMeasurement(
              pose_measurement_G_B, odom_as_prediction_G_B, update_vector_)) {
        // now that the update has been completed, we can save
        // the most recent odometry estimate and state estimates for the next
        // iteration
        T_B_M_ = T_M_B_buffered.get_T_M_I().inverse();

        Eigen::VectorXd fused_state = filter_.getState();
        Eigen::Quaterniond q_G_B;
        RPYtoQuaternion(
            fused_state(localization_fusion::StateMemberRoll),
            fused_state(localization_fusion::StateMemberPitch),
            fused_state(localization_fusion::StateMemberYaw), q_G_B);
        T_G_B_fused_ = aslam::Transformation(q_G_B, fused_state.head(3));
        success = true;
      }
    } else {
      VLOG(1) << "Initializing localization filter directly from a "
              << "measurement. T_G_M:\n"
              << localization.T_G_M;
      initialize(localization, T_M_B_buffered.get_T_M_I());
      success = true;
    }
  } else if (FLAGS_loc_filter_reset_on_time_jump) {
    VLOG(1) << "Time jump detected. Resetting filter. (message time: "
            << localization.timestamp_ns << ")";
    reset();
  } else {
    LOG(WARNING) << "The message has a timestamp before that of the previous "
                    "message received,"
                 << " this message will be ignored. This may indicate a bad "
                    "timestamp. (message time: "
                 << localization.timestamp_ns << ")";
  }
  return success;
}

template <typename T>
void LocalizationFilter<T>::aslamTransformationToStateVector(
    const aslam::Transformation& transformation,
    Eigen::VectorXd& state_vector) const {  // NOLINT
  state_vector = Eigen::VectorXd::Zero(localization_fusion::STATE_SIZE);
  state_vector(localization_fusion::StateMemberX) =
      transformation.getPosition()[0];
  state_vector(localization_fusion::StateMemberY) =
      transformation.getPosition()[1];
  state_vector(localization_fusion::StateMemberZ) =
      transformation.getPosition()[2];

  // The filter needs roll, pitch, and yaw values instead of quaternions
  double roll, pitch, yaw;
  matrixToRPY(transformation.getRotationMatrix(), yaw, pitch, roll);
  state_vector(localization_fusion::StateMemberRoll) = roll;
  state_vector(localization_fusion::StateMemberPitch) = pitch;
  state_vector(localization_fusion::StateMemberYaw) = yaw;
}

template <typename T>
void LocalizationFilter<T>::localizationResultToMeasurement(
    const common::LocalizationResult& msg,
    LocalizationFilterMeasurement& localization_G_B) {  // NOLINT

  localization_G_B.covariance_ = msg.T_G_B_covariance;

  // copy the other fields over as well
  localization_G_B.time_ = msg.timestamp_ns;
  aslamTransformationToStateVector(msg.T_G_B, localization_G_B.measurement_);
}

template <class T>
void LocalizationFilter<T>::getFusedLocalization(
    common::LocalizationResult* fused_localization_result) const {
  CHECK_NOTNULL(fused_localization_result);
  fused_localization_result->timestamp_ns = last_message_time_;
  fused_localization_result->localization_mode = localization_mode_;

  fused_localization_result->T_G_B = T_G_B_fused_;
  fused_localization_result->is_T_G_B_set = true;

  // T_B_M_previous is updated along with the T_G_B_fused
  fused_localization_result->T_G_M = T_G_B_fused_ * T_B_M_;
  fused_localization_result->is_T_G_M_set = true;
}

template <class T>
void LocalizationFilter<T>::initialize(
    const common::LocalizationResult& localization_result,
    const aslam::Transformation& T_M_B) {
  reset();

  T_G_B_fused_ = localization_result.T_G_M * T_M_B;
  T_B_M_ = T_M_B.inverse();

  LocalizationFilterMeasurement initialMeasurement;
  aslamTransformationToStateVector(
      T_G_B_fused_, initialMeasurement.measurement_);
  initialMeasurement.time_ = localization_result.timestamp_ns;
  initialMeasurement.covariance_ = localization_result.T_G_B_covariance;

  filter_.processMeasurement(
      initialMeasurement, LocalizationFilterPrediction(), update_vector_);
  last_message_time_ = localization_result.timestamp_ns;
  localization_mode_ = localization_result.localization_mode;
}

}  // namespace maplab

// Instantiations of classes is required when template class code
// is placed in a .cpp file.
template class maplab::LocalizationFilter<maplab::Ukf>;
