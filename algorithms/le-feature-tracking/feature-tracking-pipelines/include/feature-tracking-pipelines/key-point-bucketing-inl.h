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

#ifndef FEATURE_TRACKING_PIPELINES_KEY_POINT_BUCKETING_INL_H_
#define FEATURE_TRACKING_PIPELINES_KEY_POINT_BUCKETING_INL_H_

#include <algorithm>
#include <vector>

#include <glog/logging.h>

namespace feature_tracking_pipelines {

template <typename POINT_WITH_SCORE, typename SCORE_COMPARE>
inline void KeypointBucketFiler::filterKeyPoints(
    std::vector<POINT_WITH_SCORE>* keyPoints) {
  CHECK_NOTNULL(keyPoints);
  if (!keyPoints->empty()) {
    if (_needsReset) {
      resetBucketCounters();
    }

    std::sort(keyPoints->begin(), keyPoints->end(), SCORE_COMPARE());

    std::vector<POINT_WITH_SCORE> filteredKeyPoints;
    filteredKeyPoints.reserve(_maxNumKeyPointsTotal);
    for (const POINT_WITH_SCORE& key_point : *keyPoints) {
      unsigned int coord_u = key_point.pt.x / _stepSizeU;
      unsigned int coord_v = key_point.pt.y / _stepSizeV;
      CHECK_LT(coord_u, _numBucketsU);
      CHECK_LT(coord_v, _numBucketsV);
      unsigned int* bucket_counter_ptr =
          &(_numKeyPointsInBuckets[coord_u][coord_v]);
      if (*bucket_counter_ptr < _maxNumKeyPointsPerBucket) {
        ++(*bucket_counter_ptr);
        filteredKeyPoints.emplace_back(key_point);
      }
    }
    CHECK_LE(filteredKeyPoints.size(), keyPoints->size());

    keyPoints->swap(filteredKeyPoints);
    _needsReset = true;
  }
}

template <typename POINT_WITH_SCORE, typename SCORE_COMPARE>
void KeyPointBucketing(
    size_t numImgRows, size_t numImgCols, size_t maxNumKeyPoints,
    size_t numBucketsU, size_t numBucketsV,
    std::vector<POINT_WITH_SCORE>* keyPoints) {
  CHECK_NOTNULL(keyPoints);
  CHECK_GT(numImgCols, 0u);
  CHECK_GT(numImgRows, 0u);
  CHECK_GT(numBucketsU, 0u);
  CHECK_GT(numBucketsV, 0u);
  CHECK_LT(numBucketsU, numImgCols);
  CHECK_LT(numBucketsV, numImgRows);
  CHECK_GT(maxNumKeyPoints, 0u);

  if (numBucketsU == 1u || numBucketsV == 1u) {
    if (keyPoints->size() > maxNumKeyPoints) {
      std::partial_sort(
          keyPoints->begin(), keyPoints->begin() + maxNumKeyPoints,
          keyPoints->end(), SCORE_COMPARE());
      keyPoints->resize(maxNumKeyPoints);
    }
  } else {
    KeypointBucketFiler buckets(
        numBucketsU, numBucketsV, numImgCols, numImgRows, maxNumKeyPoints);
    buckets.filterKeyPoints<POINT_WITH_SCORE, SCORE_COMPARE>(keyPoints);
  }
}

}  // namespace feature_tracking_pipelines

#endif  // FEATURE_TRACKING_PIPELINES_KEY_POINT_BUCKETING_INL_H_
