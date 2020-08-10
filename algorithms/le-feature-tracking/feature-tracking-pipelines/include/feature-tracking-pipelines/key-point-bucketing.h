/*
 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef FEATURE_TRACKING_PIPELINES_KEY_POINT_BUCKETING_H_
#define FEATURE_TRACKING_PIPELINES_KEY_POINT_BUCKETING_H_

#include <vector>

#include <glog/logging.h>

namespace feature_tracking_pipelines {

class KeypointBucketFiler {
 public:
  typedef std::vector<unsigned int> OccupancyColumn;
  typedef std::vector<OccupancyColumn> OccupancyMatrix;

  KeypointBucketFiler(
      size_t numBucketsU, size_t numBucketsV, size_t numImgCols,
      size_t numImgRows, size_t maxNumKeyPoints)
      : _needsReset(false),
        _numBucketsU(numBucketsU),
        _numBucketsV(numBucketsV),
        _maxNumKeyPointsTotal(maxNumKeyPoints),
        _numKeyPointsInBuckets(numBucketsU, OccupancyColumn(numBucketsV, 0u)) {
    CHECK_GT(maxNumKeyPoints, 0u);
    CHECK_GT(numBucketsU, 0u);
    CHECK_GT(numBucketsV, 0u);
    CHECK_GT(numImgRows, 0u);
    CHECK_GT(numImgCols, 0u);

    _maxNumKeyPointsPerBucket = maxNumKeyPoints / (numBucketsU * numBucketsV);
    _stepSizeU = 2u + ((numImgCols - 1u) / numBucketsU);
    _stepSizeV = 2u + ((numImgRows - 1u) / numBucketsV);
  }

  template <typename POINT_WITH_SCORE, typename SCORE_COMPARE>
  inline void filterKeyPoints(std::vector<POINT_WITH_SCORE>* keyPoints);

  void resetBucketCounters() {
    for (size_t i = 0; i < _numKeyPointsInBuckets.size(); ++i) {
      for (size_t j = 0; j < _numKeyPointsInBuckets[i].size(); ++j) {
        _numKeyPointsInBuckets[i][j] = 0u;
      }
    }
    _needsReset = false;
  }

 private:
  bool _needsReset;

  // Parameters.
  size_t _numBucketsU;
  size_t _numBucketsV;
  size_t _maxNumKeyPointsTotal;
  OccupancyMatrix _numKeyPointsInBuckets;

  unsigned int _stepSizeU;
  unsigned int _stepSizeV;
  unsigned int _maxNumKeyPointsPerBucket;
};

template <typename POINT_WITH_SCORE>
void KeyPointBucketing(
    size_t numImgRows, size_t numImgCols, size_t maxNumKeyPoints,
    size_t numBucketsU, size_t numBucketsV,
    std::vector<POINT_WITH_SCORE>* keyPoints);
}  // namespace feature_tracking_pipelines

#include "feature-tracking-pipelines/key-point-bucketing-inl.h"

#endif  // FEATURE_TRACKING_PIPELINES_KEY_POINT_BUCKETING_H_
