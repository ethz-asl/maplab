#ifndef VI_MAP_LANDMARK_QUALITY_METRICS_H_
#define VI_MAP_LANDMARK_QUALITY_METRICS_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <vi-map/vi-map.h>

namespace vi_map {
class VIMap;
class Landmark;
}

namespace vi_map {

double computeSquaredReprojectionError(
    const vi_map::Vertex& vertex, const int frame_idx, const int keypoint_idx,
    const Eigen::Vector3d& landmark_p_C);

bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark);

// Re-evaluates the quality even if the landmark quality is known if the
// parameter re_evaluate_quality is set to true.
bool isLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark,
    bool re_evaluate_quality);

bool isVisualLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark);

bool isLiDARLandmarkWellConstrained(
    const vi_map::VIMap& map, const vi_map::Landmark& landmark);

}  // namespace vi_map

#endif  // VI_MAP_LANDMARK_QUALITY_METRICS_H_
