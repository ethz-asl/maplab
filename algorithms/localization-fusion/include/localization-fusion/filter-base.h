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

#ifndef LOCALIZATION_FUSION_FILTER_BASE_H_
#define LOCALIZATION_FUSION_FILTER_BASE_H_

#include "localization-fusion/filter-common.h"
#include "localization-fusion/filter-utilities.h"

#include <algorithm>
#include <limits>
#include <map>
#include <ostream>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <aslam/common/time.h>

namespace maplab {

//! @brief Structure used for storing measurements
//!
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in nanoseconds.
//!
struct LocalizationFilterMeasurement {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The real-valued time, in nanoseconds, since some epoch
  // (presumably the start of execution, but any will do)
  int64_t time_;

  // The measurement and its associated covariance
  Eigen::VectorXd measurement_;
  Eigen::MatrixXd covariance_;

  LocalizationFilterMeasurement() : time_(aslam::time::getInvalidTime()) {}
};

//! @brief Structure used for storing predicted states
//!
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in nanoseconds.
//!
struct LocalizationFilterPrediction {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // The real-valued time, in nanoseconds, since some epoch
  // (presumably the start of execution, but any will do)
  int64_t time_;

  // The prediction and its associated covariance
  Eigen::VectorXd predicted_state_;
  Eigen::MatrixXd covariance_;

  LocalizationFilterPrediction() : time_(aslam::time::getInvalidTime()) {}
};

//! @brief Structure used for storing and comparing filter states
//!
//! This structure is useful when higher-level classes need to remember filter
//! history.
//! Measurement units are assumed to be in meters and radians.
//! Times are real-valued and measured in seconds.
//!
struct FilterState {
  // The time stamp of the most recent measurement for the filter
  int64_t lastMeasurementTime_;

  // The filter state vector
  Eigen::VectorXd state_;

  // The filter error covariance matrix
  Eigen::MatrixXd estimateErrorCovariance_;

  FilterState() : lastMeasurementTime_(aslam::time::getInvalidTime()) {}
};

class FilterBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  //! @brief Constructor for the FilterBase class
  //!
  FilterBase();

  //! @brief Destructor for the FilterBase class
  //!
  virtual ~FilterBase();

  //! @brief Resets filter to its unintialized state
  //!
  void reset();

  //! @brief Carries out the correct step in the predict/update cycle. This
  //! method
  //! must be implemented by subclasses.
  //!
  //! @param[in] measurement - The measurement to fuse with the state estimate
  //!
  virtual bool correct(
      const LocalizationFilterMeasurement& measurement,
      const std::vector<size_t>& updateVector) = 0;

  //! @brief Gets the estimate error covariance
  //!
  //! @return A copy of the estimate error covariance matrix
  //!
  const Eigen::MatrixXd& getEstimateErrorCovariance() const;

  //! @brief Gets the filter's initialized status
  //!
  //! @return True if we've received our first measurement, false otherwise
  //!
  bool getInitializedStatus() const;

  //! @brief Gets the most recent measurement time
  //!
  //! @return The time at which we last received a measurement
  //!
  int64_t getLastMeasurementTime() const;

  //! @brief Gets the filter state
  //!
  //! @return A constant reference to the current state
  //!
  const Eigen::VectorXd& getState() const;

  //! @brief Carries out the predict step in the predict/update cycle.
  //! Projects the state and error matrices forward using a model of
  //! the vehicle's motion. This method must be implemented by subclasses.
  //!
  //! @param[in] delta_time_s - The time step [in seconds] over which to
  //! @param[in] odom_prediction - The predicted state from the odometry
  //!
  virtual void predict(
      const LocalizationFilterPrediction& odom_prediction =
          LocalizationFilterPrediction()) = 0;

  //! @brief Does some final preprocessing, carries out the predict/update cycle
  //!
  //! @param[in] measurement - The measurement object to fuse into the filter
  //!
  virtual bool processMeasurement(
      const LocalizationFilterMeasurement& measurement,
      const LocalizationFilterPrediction& odom_prediction,
      const std::vector<size_t>& updateVector);

  //! @brief Manually sets the filter's estimate error covariance
  //!
  //! @param[in] estimate_error_covariance - The state to set as the filter's
  //! current state
  //!
  void setEstimateErrorCovariance(
      const Eigen::MatrixXd& estimate_error_covariance);

  //! @brief Sets the filter's last measurement time.
  //!
  //! @param[in] last_measurement_time - The last measurement time of the filter
  //!
  void setLastMeasurementTime(const int64_t last_measurement_time);

  void setMahalanobisThreshold(const double mahalanobis_threshold);

  //! @brief Manually sets the filter's state
  //!
  //! @param[in] state - The state to set as the filter's current state
  //!
  void setState(const Eigen::VectorXd& state);

  //! @brief Ensures a given time delta is valid (helps with bag file playback
  //! issues)
  //!
  //! @param[in] delta - The time delta, in seconds, to validate
  //!
  void validateDelta(double& delta);  // NOLINT

 protected:
  //! @brief Keeps the state Euler angles in the range [-pi, pi]
  //!
  virtual void wrapStateAngles();

  //! @brief Tests if innovation is within N-sigmas of covariance. Returns true
  //! if passed the test.
  //! @param[in] innovation - The difference between the measurement and the
  //! state
  //! @param[in] innovation_covariance - The innovation error
  //! @param[in] n_sigmas - Number of standard deviations that are considered
  //! acceptable
  //!
  virtual bool checkMahalanobisThreshold(
      const Eigen::VectorXd& innovation,
      const Eigen::MatrixXd& innovation_covariance, const double n_sigmas);

  //! @brief Whether or not we've received any measurements
  //!
  bool initialized_;

  //! @brief Tracks the time the filter was last updated using a measurement.
  //!
  //! This value is used to monitor sensor readings with respect to the
  //! sensorTimeout_.
  //! We also use it to compute the time delta values for our prediction step.
  //!
  int64_t last_measurement_time_ns_;

  //! @brief This is the robot's state vector, which is what we are trying to
  //! filter. The values in this vector are what get reported by the node.
  //!
  Eigen::VectorXd state_;

  //! @brief Covariance matrices can be incredibly unstable. We can
  //! add a small value to it at each iteration to help maintain its
  //! positive-definite property.
  //!
  Eigen::MatrixXd covariance_epsilon_;

  //! @brief This matrix stores the total error in our position
  //! estimate (the state_ variable).
  //!
  Eigen::MatrixXd estimate_error_covariance_;

  double mahalanobis_distance_threshold_;
};

}  // namespace maplab

#endif  // LOCALIZATION_FUSION_FILTER_BASE_H_
