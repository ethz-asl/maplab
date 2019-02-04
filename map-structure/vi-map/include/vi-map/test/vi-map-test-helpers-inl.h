#ifndef VI_MAP_TEST_VI_MAP_TEST_HELPERS_INL_H_
#define VI_MAP_TEST_VI_MAP_TEST_HELPERS_INL_H_

#include <glog/logging.h>

#include "vi-map/test/vi-map-generator.h"

namespace vi_map {
namespace test {

template <typename EdgeType>
void generateMap(const size_t number_of_vertices, vi_map::VIMap* map) {
  CHECK_NOTNULL(map);

  // Generate a map.
  constexpr int kMapGeneratorSeed = 10;
  vi_map::VIMapGenerator map_generator(*map, kMapGeneratorSeed);
  const vi_map::MissionId mission_id = map_generator.createMission();

  pose::Transformation T_G_I;
  const pose_graph::VertexId vertex_id_1 =
      map_generator.createVertex(mission_id, T_G_I);
  T_G_I.getPosition() << 1, 0, 0;
  const pose_graph::VertexId vertex_id_2 =
      map_generator.createVertex(mission_id, T_G_I);
  T_G_I.getPosition() << 2, 0, 0;
  const pose_graph::VertexId vertex_id_3 =
      map_generator.createVertex(mission_id, T_G_I);

  // Create additional vertices.
  for (size_t i = 3u; i < number_of_vertices; ++i) {
    T_G_I.getPosition() << i, 0, 0;
    const pose_graph::VertexId vertex_id =
        map_generator.createVertex(mission_id, T_G_I);

    // Create a landmark for each vertex.
    Eigen::Vector3d map_landmark_p_G_fi;
    map_landmark_p_G_fi << i, 2, 3;
    map_generator.createLandmark(
        map_landmark_p_G_fi, vertex_id,
        {vertex_id_1, vertex_id_2, vertex_id_3});
  }

  Eigen::Vector3d map_landmark_p_G_fi;
  map_landmark_p_G_fi << 1, 2, 3;
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_1, {vertex_id_2, vertex_id_3});
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_2, {vertex_id_1, vertex_id_3});
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_3, {vertex_id_1, vertex_id_2});
  map_landmark_p_G_fi << 4, 2, 3;
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_2, {vertex_id_1, vertex_id_3});
  map_generator.createLandmark(
      map_landmark_p_G_fi, vertex_id_3, {vertex_id_1, vertex_id_2});

  map_generator.generateMap<EdgeType>();
}

}  // namespace test
}  // namespace vi_map

#endif  // VI_MAP_TEST_VI_MAP_TEST_HELPERS_INL_H_
