/*
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

#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include "localization-fusion/filter-base.h"
#include "localization-fusion/filter-common.h"

DEFINE_double(
    loc_filter_malahanobis_threshold, 5.0,
    "The threshold for malahanobis distance based outlier rejection. [number "
    "of sigmas]");

namespace maplab {
FilterBase::FilterBase()
    : initialized_(false),
      last_measurement_time_ns_(0.0),
      state_(localization_fusion::STATE_SIZE),
      covariance_epsilon_(
          localization_fusion::STATE_SIZE, localization_fusion::STATE_SIZE),
      estimate_error_covariance_(
          localization_fusion::STATE_SIZE, localization_fusion::STATE_SIZE),
      mahalanobis_distance_threshold_(FLAGS_loc_filter_malahanobis_threshold) {
  reset();
}

FilterBase::~FilterBase() {}

void FilterBase::reset() {
  initialized_ = false;

  // Clear the state and predicted state
  state_.setZero();

  // Set the estimate error covariance. We want our measurements
  // to be accepted rapidly when the filter starts, so we should
  // initialize the state's covariance with large values.
  estimate_error_covariance_.setIdentity();
  estimate_error_covariance_ *= 1e-9;

  // Set the epsilon matrix to be a matrix with small values on the diagonal
  // It is used to maintain the positive-definite property of the covariance
  covariance_epsilon_.setIdentity();
  covariance_epsilon_ *= 0.001;

  // Initialize our measurement time
  last_measurement_time_ns_ = aslam::time::getInvalidTime();
}

const Eigen::MatrixXd& FilterBase::getEstimateErrorCovariance() const {
  return estimate_error_covariance_;
}

bool FilterBase::getInitializedStatus() const {
  return initialized_;
}

int64_t FilterBase::getLastMeasurementTime() const {
  return last_measurement_time_ns_;
}

const Eigen::VectorXd& FilterBase::getState() const {
  return state_;
}

bool FilterBase::processMeasurement(
    const LocalizationFilterMeasurement& measurement,
    const LocalizationFilterPrediction& odom_prediction,
    const std::vector<size_t>& updateVector) {
  bool success = false;
  // If we've had a previous reading, then go through the predict/update
  // cycle. Otherwise, set our state and covariance to whatever we get
  // from this measurement.
  if (initialized_) {
    // Determine how much time has passed since our last measurement
    double delta =
        aslam::time::to_seconds(measurement.time_ - last_measurement_time_ns_);

    if (delta >= 0) {
      validateDelta(delta);
      predict(odom_prediction);
      success = correct(measurement, updateVector);
    } else {
      LOG(WARNING) << "Jump back in time detected. Ignoring measurement.";
    }
  } else {
    VLOG(1) << "First measurement. Initializing filter.";

    // Initialize the filter, but only with the values we're using
    size_t measurementLength = updateVector.size();
    for (size_t i = 0; i < measurementLength; ++i) {
      state_[i] = (updateVector[i] ? measurement.measurement_[i] : state_[i]);
    }

    // Same for covariance
    for (size_t i = 0; i < measurementLength; ++i) {
      for (size_t j = 0; j < measurementLength; ++j) {
        estimate_error_covariance_(i, j) =
            (updateVector[i] && updateVector[j]
                 ? measurement.covariance_(i, j)
                 : estimate_error_covariance_(i, j));
      }
    }
    initialized_ = true;
    success = true;
  }
  if (success) {
    last_measurement_time_ns_ = measurement.time_;
  }
  return success;
}

void FilterBase::setEstimateErrorCovariance(
    const Eigen::MatrixXd& estimate_error_covariance) {
  estimate_error_covariance_ = estimate_error_covariance;
}

void FilterBase::setLastMeasurementTime(const int64_t last_measurement_time) {
  last_measurement_time_ns_ = last_measurement_time;
}

void FilterBase::setMahalanobisThreshold(const double mahalanobis_threshold) {
  mahalanobis_distance_threshold_ = mahalanobis_threshold;
}

void FilterBase::setState(const Eigen::VectorXd& state) {
  CHECK_EQ(state.size(), localization_fusion::STATE_SIZE);
  state_ = state;
}

void FilterBase::validateDelta(double& delta) {
  // This handles issues with ROS time when use_sim_time is on and we're playing
  // from bags.
  if (delta > 100000.0) {
    LOG(WARNING) << "Localization filter time step was very large: " << delta
                 << "s. This might be ok for infrequent localizations."
                 << "Otherwise please verify sensor timings.";
  }
}

void FilterBase::wrapStateAngles() {
  state_(localization_fusion::StateMemberRoll) =
      clampRotation(state_(localization_fusion::StateMemberRoll));
  state_(localization_fusion::StateMemberPitch) =
      clampRotation(state_(localization_fusion::StateMemberPitch));
  state_(localization_fusion::StateMemberYaw) =
      clampRotation(state_(localization_fusion::StateMemberYaw));
}

bool FilterBase::checkMahalanobisThreshold(
    const Eigen::VectorXd& innovation,
    const Eigen::MatrixXd& innovation_covariance, const double n_sigmas) {
  double sqMahalanobis = innovation.dot(innovation_covariance * innovation);
  double threshold = n_sigmas * n_sigmas;

  if (sqMahalanobis >= threshold) {
    LOG(WARNING) << "Innovation mahalanobis distance test failed. Squared "
                 << "Mahalanobis is: " << sqMahalanobis
                 << " Threshold is: " << threshold;

    return false;
  }

  return true;
}
}  // namespace maplab
