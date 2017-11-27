#include "map-sparsification/visualization/map-sparsification-visualization.h"

#include <glog/logging.h>
#include <visualization/color-palette.h>
#include <visualization/common-rviz-visualization.h>
#include <visualization/viz-primitives.h>

namespace map_sparsification_visualization {

MapSparsificationVisualizer::~MapSparsificationVisualizer() {}

void MapSparsificationVisualizer::plotSegment(
    const vi_map::VIMap& map,
    const std::vector<pose_graph::VertexIdList>& posegraph_partitioning,
    int segment_index) const {
  const size_t num_partitions = posegraph_partitioning.size();
  CHECK_LT(segment_index, static_cast<int>(num_partitions));
  CHECK_GT(posegraph_partitioning.size(), 0u);

  visualization::SphereVector spheres;

  const size_t num_vertices = map.numVertices();
  spheres.resize(num_vertices);
  size_t sphere_index = 0u;

  visualization::Palette palette = visualization::GetPalette(
      visualization::Palette::PaletteTypes::kFalseColor1);
  double colour_value =
      static_cast<double>(segment_index) / posegraph_partitioning.size();
  visualization::Color color = getPaletteColor(colour_value, palette);

  const double kRadius = 0.1;
  for (size_t partition_index = 0u; partition_index < num_partitions;
       ++partition_index) {
    visualization::Color segment_color;
    double alpha;
    if (partition_index == static_cast<size_t>(segment_index)) {
      // The relevant segment is selected.
      segment_color = color;
      alpha = 1.0;
    } else if (segment_index == -1) {
      // Global optimization. All vertices are selected and colored according
      // to the segment.
      double color_value = static_cast<double>(partition_index) /
                           static_cast<double>(num_partitions);
      visualization::Color global_color = getPaletteColor(color_value, palette);

      segment_color = global_color;
      alpha = 1.0;
    } else {
      // Deselected vertex case, make it dark grey.
      segment_color.red = 49;
      segment_color.green = 49;
      segment_color.blue = 49;
      alpha = 0.5;
    }

    // Loop over all the vertices in the i-th segment.
    for (const pose_graph::VertexId& vertex_id :
         posegraph_partitioning[partition_index]) {
      const Eigen::Vector3d p_G_I = map.getVertex_G_p_I(vertex_id);

      visualization::Sphere sphere;
      sphere.position = p_G_I;
      sphere.radius = kRadius;

      sphere.color.red = segment_color.red;
      sphere.color.green = segment_color.green;
      sphere.color.blue = segment_color.blue;
      sphere.alpha = alpha;

      CHECK_LT(sphere_index, spheres.size());
      spheres[sphere_index++] = sphere;
    }
  }
  const size_t kMarkerIndex = 0u;
  visualization::publishSpheres(
      spheres, kMarkerIndex, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, segment_vertices_topic_);
}

// Plots landmarks to visualize the partitioning and summarization process.
//   * map - VI map object
//   * segment_index - the segment the landmarks belong to, pass -1 for
//                     landmarks that don't belong to any partition (either
//                     in case of final global optimization or when plotting
//                     the initial state)
//   * selected_landmark_ids - list of selected store landmark ids
//   * are_landmarks_globally_selected - pass 'true' if the result of the final
//                                       global optimization should be plotted
void MapSparsificationVisualizer::plotLandmarks(
    const vi_map::VIMap& map, int segment_index,
    const vi_map::LandmarkIdSet& selected_landmark_ids,
    const std::vector<pose_graph::VertexIdList>& posegraph_partitioning,
    const std::vector<vi_map::LandmarkIdSet>& partition_landmarks,
    bool are_landmarks_globally_selected) const {
  if (are_landmarks_globally_selected) {
    CHECK_EQ(-1, segment_index) << "Segment index must be equal to -1 for "
                                << "the globally selected landmarks case.";
  }
  const size_t num_partitions = posegraph_partitioning.size();
  CHECK_GT(num_partitions, 0u);
  CHECK_LT(segment_index, static_cast<int>(num_partitions));

  visualization::SphereVector unselected_spheres;
  visualization::SphereVector spheres;

  visualization::Color global_color;
  global_color.red = 80;
  global_color.green = 80;
  global_color.blue = 80;

  visualization::Color unselected_color;
  unselected_color.red = 250;
  unselected_color.green = 237;
  unselected_color.blue = 187;

  visualization::Palette palette = visualization::GetPalette(
      visualization::Palette::PaletteTypes::kFalseColor1);

  vi_map::LandmarkIdSet all_landmark_ids;
  map.getAllLandmarkIds(&all_landmark_ids);
  const size_t num_partition_landmarks = partition_landmarks.size();
  for (const vi_map::LandmarkId& landmark_id : all_landmark_ids) {
    const Eigen::Vector3d p_G_fi = map.getLandmark_G_p_fi(landmark_id);

    visualization::Sphere sphere;
    sphere.position = p_G_fi;
    sphere.radius = 0.1;

    visualization::Color color;
    double alpha = 1.0;

    bool is_selected = false;
    bool should_add = false;

    for (size_t idx = 0u; idx < num_partition_landmarks; ++idx) {
      double color_value =
          static_cast<double>(idx) / static_cast<double>(num_partitions);

      visualization::Color local_color = getPaletteColor(color_value, palette);

      if (partition_landmarks[idx].count(landmark_id) == 0) {
        // The landmark is not selected from the i-th partition.
        color = unselected_color;
        alpha = 0.3;
        sphere.radius = 0.03;
        if (!are_landmarks_globally_selected) {
          // If it's not the global case, we should add the landmark with
          // a color of the deselected set.
          should_add = true;
        }
      } else if (!are_landmarks_globally_selected) {
        // Landmark is selected from the i-th partition and it's not a global
        // case. Color it according to the segment index.
        color = local_color;
        alpha = 1.0;
        sphere.radius = 0.1;
        is_selected = true;
        should_add = true;
        // No need to iterate through the other partitions.
        break;
      } else {
        // This is a final, global summarization.
        if (selected_landmark_ids.count(landmark_id) > 0) {
          // Landmark got selected in the global optimization.
          color = global_color;
          alpha = 1.0;
          sphere.radius = 0.1;
          is_selected = true;
          should_add = true;
        } else {
          // Landmark got deselected in the global optimization, but was
          // selected in the previous, local partitioning. Draw it with a small
          // alpha value, but with the original segment color.
          color = local_color;
          alpha = 0.12;
          sphere.radius = 0.08;
          is_selected = false;
          should_add = true;
        }
        break;
      }
    }

    sphere.color.red = color.red;
    sphere.color.green = color.green;
    sphere.color.blue = color.blue;
    sphere.alpha = alpha;

    if (should_add) {
      if (!is_selected) {
        unselected_spheres.push_back(sphere);
      } else {
        spheres.push_back(sphere);
      }
    }
  }
  const size_t kMarkerId = 1u;
  visualization::publishSpheres(
      spheres, kMarkerId, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, landmarks_topic_);
  // Separate marker index for smaller spheres of deselected landmarks (it's
  // not possible to mix sphere size in a single Spheres ROS message).
  const size_t kSmallSphereMarkerId = 2u;
  visualization::publishSpheres(
      unselected_spheres, kSmallSphereMarkerId, visualization::kDefaultMapFrame,
      visualization::kDefaultNamespace, landmarks_topic_);
}
}  // namespace map_sparsification_visualization
