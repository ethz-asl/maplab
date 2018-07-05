#ifndef VI_MAP_HELPERS_TEST_VI_MAP_LANDMARK_QUALITY_CHECK_H_
#define VI_MAP_HELPERS_TEST_VI_MAP_LANDMARK_QUALITY_CHECK_H_

#include <glog/logging.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

void checkLandmarkQualityInView(
    const vi_map::VIMap& map, int expected_num_unknown_quality,
    int expected_num_good_quality, int expected_num_bad_quality) {
  pose_graph::VertexIdList all_vertices;
  map.getAllVertexIds(&all_vertices);

  int num_unknown_quality = 0;
  int num_good_quality = 0;
  int num_bad_quality = 0;
  for (const pose_graph::VertexId& vertex_id : all_vertices) {
    const vi_map::Vertex& vertex = map.getVertex(vertex_id);
    for (const vi_map::Landmark& landmark : vertex.getLandmarks()) {
      switch (landmark.getQuality()) {
        case vi_map::Landmark::Quality::kUnknown: {
          ++num_unknown_quality;
          break;
        }
        case vi_map::Landmark::Quality::kBad: {
          ++num_bad_quality;
          break;
        }
        case vi_map::Landmark::Quality::kGood: {
          ++num_good_quality;
          break;
        }
        default: {
          // Fall through intended.
        }
      }
    }
  }

  EXPECT_EQ(num_unknown_quality, expected_num_unknown_quality);
  EXPECT_EQ(num_good_quality, expected_num_good_quality);
  EXPECT_EQ(num_bad_quality, expected_num_bad_quality);
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_TEST_VI_MAP_LANDMARK_QUALITY_CHECK_H_
