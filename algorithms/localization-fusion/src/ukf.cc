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

#include <assert.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <vector>

#include <Eigen/Cholesky>
#include <XmlRpcException.h>

#include "localization-fusion/filter-common.h"
#include "localization-fusion/ukf.h"

namespace maplab {
Ukf::Ukf(const double alpha, const double kappa, const double beta)
    : FilterBase() {
  // Must initialize filter base!

  size_t sigmaCount = (localization_fusion::STATE_SIZE << 1) + 1;
  sigma_points_.resize(
      sigmaCount, Eigen::VectorXd(localization_fusion::STATE_SIZE));

  // Prepare constants
  lambda_ = alpha * alpha * kappa;

  sigma_state_weights_.resize(sigmaCount);
  sigma_covariance_weights_.resize(sigmaCount);

  // initialize sigma weights
  // set sigma points to zero
  sigma_state_weights_[0] =
      (lambda_ - localization_fusion::STATE_SIZE) / lambda_;
  sigma_covariance_weights_[0] =
      sigma_state_weights_[0] + (1 - (alpha * alpha) + beta);
  sigma_points_[0].setZero();
  for (size_t i = 1; i < sigmaCount; ++i) {
    sigma_points_[i].setZero();
    sigma_state_weights_[i] = 1 / (2 * lambda_);
    sigma_covariance_weights_[i] = sigma_state_weights_[i];
  }
}

Ukf::~Ukf() {}

bool Ukf::correct(
    const LocalizationFilterMeasurement& measurement,
    const std::vector<size_t>& update_vector) {
  // We don't want to update everything, so we need to build matrices that only
  // update
  // the measured parts of our state vector

  // First, determine how many state vector values we're updating
  std::vector<size_t> updateIndices;
  for (size_t i = 0; i < update_vector.size(); ++i) {
    if (update_vector[i]) {
      // Handle nan and inf values in measurements
      if (std::isnan(measurement.measurement_(i))) {
        LOG(WARNING) << "Measurement value at index " << i
                     << " was nan. Excluding from update.";
      } else if (std::isinf(measurement.measurement_(i))) {
        LOG(WARNING) << "Measurement value at index " << i
                     << " was inf. Excluding from update.";
      } else {
        updateIndices.push_back(i);
      }
    }
  }

  size_t updateSize = updateIndices.size();
  if (updateSize < 1) {
    LOG(ERROR) << "No state variables can be updated. Please verify that the "
                  "incoming measurement is valid.";
    return false;
  }

  // Now set up the relevant matrices
  Eigen::VectorXd stateSubset(updateSize);        // x (in most literature)
  Eigen::VectorXd measurementSubset(updateSize);  // z
  Eigen::MatrixXd measurementCovarianceSubset(updateSize, updateSize);  // R
  Eigen::MatrixXd stateToMeasurementSubset(
      updateSize, localization_fusion::STATE_SIZE);  // H
  Eigen::MatrixXd kalmanGainSubset(
      localization_fusion::STATE_SIZE, updateSize);  // K
  Eigen::VectorXd innovationSubset(updateSize);  // z - Hx
  Eigen::VectorXd predictedMeasurement(updateSize);
  Eigen::VectorXd sigmaDiff(updateSize);
  Eigen::MatrixXd predictedMeasCovar(updateSize, updateSize);
  Eigen::MatrixXd crossCovar(localization_fusion::STATE_SIZE, updateSize);

  std::vector<Eigen::VectorXd> sigmaPointMeasurements(
      sigma_points_.size(), Eigen::VectorXd(updateSize));

  stateSubset.setZero();
  measurementSubset.setZero();
  measurementCovarianceSubset.setZero();
  stateToMeasurementSubset.setZero();
  kalmanGainSubset.setZero();
  innovationSubset.setZero();
  predictedMeasurement.setZero();
  predictedMeasCovar.setZero();
  crossCovar.setZero();

  // Now build the sub-matrices from the full-sized matrices
  for (size_t i = 0; i < updateSize; ++i) {
    measurementSubset(i) = measurement.measurement_(updateIndices[i]);
    stateSubset(i) = state_(updateIndices[i]);

    for (size_t j = 0; j < updateSize; ++j) {
      measurementCovarianceSubset(i, j) =
          measurement.covariance_(updateIndices[i], updateIndices[j]);
    }

    // Handle negative (read: bad) covariances in the measurement. Rather
    // than exclude the measurement or make up a covariance, just take
    // the absolute value.
    if (measurementCovarianceSubset(i, i) < 0) {
      LOG(WARNING) << "Negative covariance for index " << i
                   << " of measurement (value is"
                   << measurementCovarianceSubset(i, i)
                   << "). Using absolute value...";

      measurementCovarianceSubset(i, i) =
          ::fabs(measurementCovarianceSubset(i, i));
    }

    // If the measurement variance for a given variable is very
    // near 0 (as in e-50 or so) and the variance for that
    // variable in the covariance matrix is also near zero, then
    // the Kalman gain computation will blow up. Really, no
    // measurement can be completely without error, so add a small
    // amount in that case.
    if (measurementCovarianceSubset(i, i) < 1e-9) {
      measurementCovarianceSubset(i, i) = 1e-9;
      LOG(WARNING)
          << "The measurement had very small error covariance for index "
          << updateIndices[i]
          << ". Adding some noise to maintain filter stability.";
    }
  }

  // The state-to-measurement function, h, will now be a measurement_size x
  // full_state_size
  // matrix, with ones in the (i, i) locations of the values to be updated
  for (size_t i = 0; i < updateSize; ++i) {
    stateToMeasurementSubset(i, updateIndices[i]) = 1;
  }

  // (1) Generate sigma points, use them to generate a predicted measurement
  for (size_t sigmaInd = 0; sigmaInd < sigma_points_.size(); ++sigmaInd) {
    sigmaPointMeasurements[sigmaInd] =
        stateToMeasurementSubset * sigma_points_[sigmaInd];
    predictedMeasurement.noalias() +=
        sigma_state_weights_[sigmaInd] * sigmaPointMeasurements[sigmaInd];
  }

  // (2) Use the sigma point measurements and predicted measurement to compute a
  // predicted
  // measurement covariance matrix P_zz and a state/measurement cross-covariance
  // matrix P_xz.
  for (size_t sigmaInd = 0; sigmaInd < sigma_points_.size(); ++sigmaInd) {
    sigmaDiff = sigmaPointMeasurements[sigmaInd] - predictedMeasurement;
    predictedMeasCovar.noalias() += sigma_covariance_weights_[sigmaInd] *
                                    (sigmaDiff * sigmaDiff.transpose());
    crossCovar.noalias() +=
        sigma_covariance_weights_[sigmaInd] *
        ((sigma_points_[sigmaInd] - state_) * sigmaDiff.transpose());
  }

  // (3) Compute the Kalman gain, making sure to use the actual measurement
  // covariance: K = P_xz * (P_zz + R)^-1
  Eigen::MatrixXd invInnovCov =
      (predictedMeasCovar + measurementCovarianceSubset).inverse();
  kalmanGainSubset = crossCovar * invInnovCov;

  // (4) Apply the gain to the difference between the actual and predicted
  // measurements: x = x + K(z - z_hat)
  innovationSubset = (measurementSubset - predictedMeasurement);

  // Wrap angles in the innovation
  for (size_t i = 0; i < updateSize; ++i) {
    if (updateIndices[i] == localization_fusion::StateMemberRoll ||
        updateIndices[i] == localization_fusion::StateMemberPitch ||
        updateIndices[i] == localization_fusion::StateMemberYaw) {
      innovationSubset(i) = clampRotation(innovationSubset(i));
    }
  }

  // (5) Check Mahalanobis distance of innovation
  if (checkMahalanobisThreshold(
          innovationSubset, invInnovCov, mahalanobis_distance_threshold_)) {
    state_.noalias() += kalmanGainSubset * innovationSubset;

    // (6) Compute the new estimate error covariance P = P - (K * P_zz * K')
    estimate_error_covariance_.noalias() -=
        (kalmanGainSubset * predictedMeasCovar * kalmanGainSubset.transpose());

    wrapStateAngles();
    return true;
  }
  return false;
}

void Ukf::predict(
    const LocalizationFilterPrediction& odom_prediction) {
  // (1) Take the square root of a small fraction of the
  // estimateErrorCovariance_ using cholesky for LL' decomposition
  // utilize the fact that: cholesky(n * M) = sqrt(n) * cholesky(M), for scalar
  // n, matrix M
  weighted_covariance_sqrt_ =
      (lambda_ * estimate_error_covariance_).llt().matrixL();

  // (2) Compute sigma points based on the odometry prediction.
  // Note, we don't use a transfer function (as it is typically done) but the
  // odometry estimate instead

  // First sigma point is the predicted odom state
  sigma_points_[0] = odom_prediction.predicted_state_;

  // Next localization_fusion::STATE_SIZE sigma points are state +
  // weightedCovarSqrt_[ith column]
  // localization_fusion::STATE_SIZE sigma points after that are state -
  // weightedCovarSqrt_[ith
  // column]
  for (size_t sigmaInd = 0; sigmaInd < localization_fusion::STATE_SIZE;
       ++sigmaInd) {
    sigma_points_[sigmaInd + 1] =
        (odom_prediction.predicted_state_ +
         weighted_covariance_sqrt_.col(sigmaInd));
    sigma_points_[sigmaInd + 1 + localization_fusion::STATE_SIZE] =
        (odom_prediction.predicted_state_ -
         weighted_covariance_sqrt_.col(sigmaInd));
  }

  // (3) Sum the weighted sigma points to generate a new state prediction
  state_.setZero();
  for (size_t sigmaInd = 0; sigmaInd < sigma_points_.size(); ++sigmaInd) {
    state_.noalias() +=
        sigma_state_weights_[sigmaInd] * sigma_points_[sigmaInd];
  }

  // (4) covariance prediction
  estimate_error_covariance_ = odom_prediction.covariance_;

  // Keep the angles bounded
  wrapStateAngles();
}

}  // namespace maplab
