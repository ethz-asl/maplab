#ifndef LIDAR_FEATURE_EXTRACTION_OUSTER_CONFIGURATION_H_
#define LIDAR_FEATURE_EXTRACTION_OUSTER_CONFIGURATION_H_

#include <cmath>

namespace LidarFeatureExtraction {
  static const std::size_t beam_size_ = 64;
  static const std::size_t ring_size_ = 1024;
  static const std::size_t grid_ = 6;
  static const float ang_res_x_ = 360.0/float(ring_size_);
  static const float ang_res_y_ = 33.2/float(beam_size_-1);
  static const float ang_bottom_ = 16.7;
}

#endif
