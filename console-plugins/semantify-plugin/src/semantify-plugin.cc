#include "semantify-plugin/icp-covariance.h"
#include "semantify-plugin/k-medoids-clustering-manager.h"
#include "semantify-plugin/semantify-common.h"
#include "semantify-plugin/semantify-plugin.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <queue>
#include <set>
#include <thread>
#include <unordered_set>
#include <vector>

#include <aslam/triangulation/triangulation.h>
#include <console-common/console.h>
#include <map-resources/resource-common.h>
#include <maplab-common/progress-bar.h>
#include <semantify-plugin/semantify-common.h>
#include <visualization/common-rviz-visualization.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD>

// semantic landmark filter by class
DEFINE_string(
    semantify_semantic_landmark_class_filter, "1,73,61",
    "The class ids of the semantic landmark in this filter WILL NOT be "
    "used in the matching process of anchoring or loop closure"
    "Separate the ids with commas and no spacing, i.e.  "
    "Default: 1,73,61");  // person, refrigerator, table
// descriptor generation
DEFINE_bool(
    semantify_generate_descriptor_filter_by_mask, true,
    "If true, only send the masked part of the bounding box of the rgb images "
    "to the descriptor generator. Also, fills the non-masked region with white."
    "If false, the entire bounding box of the detection to the descriptor "
    "generator."
    "Default: false");
// evaluation/debugging
DECLARE_string(map_mission);
DEFINE_int32(
    semantify_semantic_landmark_track_id, -1,
    "The track id of the semantic landmark. Use the along with the "
    "evaluate_semantic_landmark_with_track_id command. "
    "Default: -1");
DEFINE_string(
    semantify_descriptor_comparison_mission_id_0, "",
    "The mission id for track ids group 0."
    "Default: empty string");
DEFINE_string(
    semantify_descriptor_comparison_mission_id_1, "",
    "The mission id for track ids group 1."
    "Default: empty string");
DEFINE_string(
    semantify_descriptor_comparison_semantic_landmark_track_ids_0, "",
    "The track ids of the semantic landmark in group 0 that we want to "
    "compare the descriptor clusters with group 1."
    "Separate the ids with commas and no spacing, i.e. 1,2,3 "
    "Default: empty string");
DEFINE_string(
    semantify_descriptor_comparison_semantic_landmark_track_ids_1, "",
    "The track ids of the semantic landmark in group 1 that we want to "
    "compare the descriptor clusters with group 0."
    "Separate the ids with commas and no spacing, i.e. 1,2,3 "
    "Default: empty string");
DEFINE_int32(
    semantify_visualize_match_candidate_index, 0,
    "The index of the match candidate from the hypothesis generator."
    "Default: 0");
DEFINE_bool(
    semantify_visualize_observing_verticies, false,
    "If true, the pipeline publishes lines between each semantic landmark "
    "and its observing verticies under the topic: "
    "semantic_landmark_observing_verticies"
    "Default: false");
DEFINE_bool(
    semantify_visualize_covisible_objects, false,
    "If true, the pipeline publishes lines between each semantic landmark "
    "and its covisible objects under the topic: "
    "semantic_landmark_covisible_track_ids"
    "Default: false");
DEFINE_bool(
    semantify_visualize_valid_match_candidates, false,
    "If true, the pipeline publishes lines for each valid match candidate "
    "and its supporting match candidates under the topic: "
    "semantic_landmark_valid_match_candidates"
    "Default: false");
DEFINE_bool(
    semantify_visualize_accepted_loop_closure_edge, true,
    "If true, the pipeline publishes lines for the accepted loop closure "
    "edge between the vertices and the vertices under the topic: "
    "semantic_landmark_lc_edge and lc_edge_vertex"
    "Default: true");
DEFINE_bool(
    semantify_visualize_semantic_landmark_covariance, true,
    "If true, the pipeline publishes ellipsoids that represent the covariance "
    "of the triangulated semantic landmarks under the topic: "
    "semantic_landmark_covariance"
    "Default: true");
DEFINE_bool(
    semantify_visualize_semantic_loop_closure_topological_vertex_candidates,
    false,
    "If true, the pipeline publishes topological vertex candidates of the loop "
    "closure edge under the topic : lc_edge_vertex"
    "Default: false");
// semantic landmarks matching
DEFINE_bool(
    semantify_semantic_landmark_matching_filter_by_class, true,
    "If true, only match the semantic landmarks of the same class."
    "If false, do not use class information when matching semantic landmarks "
    "Default: true");
DEFINE_double(
    semantify_semantic_landmark_matching_filter_by_max_descriptor_difference,
    1.2,
    "Only match the semantic landmarks with descriptor difference below this"
    "threshold."
    "Default: 1.2");
DEFINE_uint32(
    semantify_semantic_landmark_matching_candidates_num, 5,
    "The n number closest matches for each landmark."
    "Default: 3");
DEFINE_bool(
    semantify_semantic_landmark_matching_filter_by_unique_visibility, true,
    "If true, apply unique visibility filter to improve the quality of the raw "
    " semantic landmark matches from the k nearest descriptor space search."
    "If false, do nothing."
    "Default: true");
DEFINE_bool(
    semantify_semantic_landmark_matching_filter_by_match_candidate_distance,
    true,
    "If true, apply max distance filter to improve the quality of the raw "
    " semantic landmark matches from the k nearest descriptor space search."
    "If false, do nothing."
    "Default: true");
DEFINE_double(
    semantify_semantic_landmark_max_match_candidate_distance, 3.0,
    "The maximum distance in global frame allowed for a match candidate to be "
    "valid. "
    "Note: this value does not take into account offset set by the commmand "
    "--spatially_distribute_missions_meters. "
    "Default: 3 meters");
// ransac result
DEFINE_bool(
    semantify_ransac_optimizes_transformation_by_all_inliers, true,
    "If true, ICP points to points transformation from RANSAC is recalculated "
    "using"
    "all the inliers. "
    "Default: true");
// semantic landmarks anchoring
DEFINE_int32(
    semantify_semantic_landmark_anchoring_ransac_max_iteration, 2000,
    "The max ransac iteration for finding the map anchoring transformation "
    "using semantic landmarks command. "
    "Default: 2000");
DEFINE_double(
    semantify_semantic_landmark_anchoring_ransac_min_inlier_ratio, 0.5,
    "The minimum threshold of the inlier ratio of proposed transformation to "
    "be accepted."
    "Default: 0.5");
DEFINE_double(
    semantify_semantic_landmark_anchoring_ransac_min_inlier_num, 4,
    "The minimum threshold of the inlier ratio of proposed transformation to "
    "be accepted."
    "Default: 0.2");
DEFINE_double(
    semantify_semantic_landmark_anchoring_ransac_max_inlier_distance, 0.10,
    "The maximum distance threshold for considering the transformed landmark "
    "centroids to "
    "be an inlier."
    "Default: 0.1");
DEFINE_bool(
    semantify_semantic_landmark_anchoring_merge_landmarks, true,
    "If true, the command also merges the matched semantic landmarks used in "
    "anchoring."
    "If false, the command does not use merge the semantic landmarks used in "
    "anchoring. "
    "Default: true");
// semantic landmarks loop closure
DEFINE_bool(
    semantify_semantic_landmark_lc_extend_visible_verticies, true,
    "If true, extends the observing verticies by number specificed by "
    "--semantify_semantic_landmark_lc_extend_visible_verticies_num along and "
    "on "
    "both sides of the pose graph."
    "Default: true");
DEFINE_int32(
    semantify_semantic_landmark_lc_extend_visible_verticies_num, 10,
    "The number of verticies to add on the outer most observing verticies. "
    "using semantic landmarks command. "
    "Default: 10");
DEFINE_double(
    semantify_semantic_landmark_lc_max_covisible_object_candidate_distance_difference,
    0.5,
    "The maximum distance allowed for a match candidate to be a valid "
    "covisible "
    "object candidate. for example, the difference of the distance between the "
    "cup and the tv (source) and the distance between the cup and the tv (ref)"
    "should be relatively close."
    "Default: 0.5 meters");
DEFINE_bool(
    semantify_semantic_landmark_lc_add_edge_between_topological_center_vertices,
    true,
    "If true, the pipeline adds the sematnic loop closure edge between "
    "vertices that are "
    "the topological centers of the two object clusters described by a group "
    "of match "
    "candidates."
    "If false, the pipeline add the loop closure edge between the nearest "
    "vertices of the "
    "main candidate of the group of match candidates."
    "Default: false");
DEFINE_bool(
    semantify_semantic_landmark_lc_merge_matched_landmarks, false,
    "If true, the command also merges the matched semantic landmarks used in "
    "anchoring."
    "If false, the command does not use merge the semantic landmarks used in "
    "anchoring. "
    "Default: false");
DEFINE_bool(
    semantify_semantic_landmark_lc_remove_old_edges, false,
    "If true, the loop_close_missions_with_semantic_landmarks command removes "
    "previously "
    "added semantic loop closure edges from the posegraph when the command is "
    "executed."
    "If false, the command does not remove previously added semantic lc edges."
    "Default: false");
DEFINE_string(
    semantify_loop_closure_mission_id_ref, "",
    "The mission id for the reference map."
    "Default: empty string");
DEFINE_string(
    semantify_loop_closure_mission_id_source, "",
    "The mission id for the source map."
    "Default: empty string");

namespace semantify {
SemantifyPlugin::SemantifyPlugin(
    common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter)
    : common::ConsolePluginBaseWithPlotter(console, plotter) {
  addCommand(
      {"semantify"}, [this]() -> int { return semantify(); },
      "This command sends RGB images from optional resources attached in map "
      "to a object detection network through ROS service. On service response "
      "it stores the bounding boxes and masks into the map as optional "
      "resources.",
      common::Processing::Sync);

  addCommand(
      {"generate_and_save_semantic_object_measurements"},
      [this]() -> int { return generateAndSaveSemanticObjectMeasurements(); },
      "This command sends the detected objects from the semantify command to a "
      "description generator. Then, the command saves the bounding boxes, "
      "confidence scores, class ids, and the descriptors into the visual "
      "frame of the vertex in the map.",
      common::Processing::Sync);

  addCommand(
      {"generate_and_save_semantic_object_track_ids_from_deepsort"},
      [this]() -> int {
        return generateAndSaveSemanticObjectTrackIdsFromDeepsort();
      },
      "This command sends the detected bounding boxes and descriptors to "
      "the deepSORT algorithm to generate semantic track ids. The track  "
      "ids are stored back into the visual frame. This command will also "
      "populate the tracked object list.",
      common::Processing::Sync);

  addCommand(
      {"visualize_semantic_object_channels_in_visual_frame"},
      [this]() -> int {
        return visualizeSemanticObjectChannelsInVisualFrame();
      },
      "This command creates a window to visualize the semantic measurements "
      "inside the visual frames of the selected map. Semantic object track "
      "ids are not required and will be assigned to -1 if not present.",
      common::Processing::Sync);

  addCommand(
      {"visualize_optional_resources_bounding_boxes"},
      [this]() -> int { return visualizeOptionalResourcesBoundingboxes(); },
      "This command creates a window to visualize the optional bounding box "
      "resources inside the selected map. Semantic object track Ids are -1.",
      common::Processing::Sync);

  addCommand(
      {"visualize_semantic_landmarks_and_generate_track_id_to_semantic_"
       "landmark_map"},
      [this]() -> int {
        return visualizeSemanticLandmarksAndGenerateTrackIdToSemanticLandmarkMap();
      },
      "This command publish rviz sphere markers with class id, class names and "
      "track id for the semantic landmarks in the map. The scale of the sphere "
      "is calculated from the height and width of the projected bounding "
      "boxes.",
      common::Processing::Sync);

  addCommand(
      {"update_semantic_landmarks_class_ids"},
      [this]() -> int {
        // Select map.
        std::string selected_map_key;
        if (!getSelectedMapKeyIfSet(&selected_map_key)) {
          return common::kStupidUserError;
        }
        vi_map::VIMapManager map_manager;
        vi_map::VIMapManager::MapWriteAccess map =
            map_manager.getMapWriteAccess(selected_map_key);

        if (map->numSemanticLandmarksInIndex() == 0u) {
          LOG(INFO) << "There are no semantic landmarks in the map";
          return common::kStupidUserError;
        }

        // Updates the class ids of the semantic landmarks
        map->updateAllSemanticLandmarksClassId();
      },
      "This command updates the class ids of the semantic landmarks based on "
      "the frequency of the class ids in their measurements. The class id with"
      "the most count is assigned to the semantic landmark.",
      common::Processing::Sync);

  addCommand(
      {"evaluate_semantic_landmark_with_track_id"},
      [this]() -> int { return evaluateSemanticLandmarkWithTrackId(); },
      "This command updates the class ids of the semantic landmarks based on "
      "the frequency of the class ids in their measurements. The class id with"
      "the most count is assigned to the semantic landmark.",
      common::Processing::Sync);

  addCommand(
      {"generate_descriptor_clusters"},
      [this]() -> int { return generateDescriptorClusters(); },
      "This command generates semantic landmark descriptors clusters using "
      "K-Medoids clusting algorithms. It only clusters for semantic landmarks "
      "with all qualities except bad.",
      common::Processing::Sync);

  addCommand(
      {"display_descriptor_clusters_scores"},
      [this]() -> int { return displayDescriptorClustersScores(); },
      "This command displays the silhouette_scores of the clusters of the "
      "semantic landmarks."
      "It only shows scores for semantic landmarks with all qualities except "
      "bad.",
      common::Processing::Sync);

  addCommand(
      {"compare_descriptor_clusters_scores"},
      [this]() -> int { return compareDescriptorClustersScores(); },
      "This command displays the silhouette_scores of the clusters of the "
      "semantic landmarks."
      "It only shows scores for semantic landmarks with all qualities except "
      "bad.",
      common::Processing::Sync);

  addCommand(
      {"match_semantic_landmarks_in_one_mission"},
      [this]() -> int { return matchSemanticLandmarksInOneMission(); },
      "This command displays the silhouette_scores of the clusters of the "
      "semantic landmarks."
      "It only shows scores for semantic landmarks with all qualities except "
      "bad.",
      common::Processing::Sync);

  addCommand(
      {"anchor_mission_with_semantic_landmarks"},
      [this]() -> int { return anchorMissionWithSemanticLandmarks(); },
      "This command anchors the selected missions with unknown T_G_M to "
      "missions with known T_G_M.",
      common::Processing::Sync);

  addCommand(
      {"loop_close_missions_with_semantic_landmarks"},
      [this]() -> int { return loopCloseMissionsWithSemanticLandmarks(); },
      "This command loop close the selected mission with missions in the "
      "database.",
      common::Processing::Sync);

  addCommand(
      {"calculate_semantic_landmark_covariances"},
      [this]() -> int { return calculateSemanticLandmarkCovariances(); },
      "This command calculates and visualizes the covariances of the semantic "
      "landmarks",
      common::Processing::Sync);

  addCommand(
      {"visualize_semantic_loop_closure_edge_covariances"},
      [this]() -> int { return visualizeSemanticLoopClosureEdgeCovariances(); },
      "This command visualizes the covariances of the semantic loop closure "
      "edges.",
      common::Processing::Sync);
}  // namespace semantify

// Commands
int SemantifyPlugin::semantify() {  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIds(&vertex_ids);
  common::ProgressBar progress_bar(vertex_ids.size());
  vi_map::MissionVertexIdList mission_to_vertex_ids;
  map->getVertexIdsByMission(&mission_to_vertex_ids);
  if (mission_to_vertex_ids.empty()) {
    VLOG(1) << "No missions found!";
    return true;
  }

  int mission_num = 0;
  for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
       mission_to_vertex_ids) {
    VLOG(1) << "Mission " << (mission_num + 1) << " with "
            << mission_vertex_id_pair.second.size() << " vertices.";
    for (const pose_graph::VertexId& vertex_id :
         mission_vertex_id_pair.second) {
      vi_map::Vertex* vertex = map->getVertexPtr(vertex_id);
      const aslam::VisualNFrame& n_frame = vertex->getVisualNFrame();
      for (uint idx = 0u; idx < n_frame.getNumFrames(); ++idx) {
        cv::Mat image_resource;
        if (map->getFrameResource(
                *vertex, idx, backend::ResourceType::kRawColorImage,
                &image_resource)) {
          // Setup service request for mask_rcnn_ros node.
          // Service handler expects images in bgr8 encoding.
          mask_rcnn_ros::mrcnn_result maskrcnn_srv;
          cv_bridge::CvImage cv_image;
          cv_image.encoding =
              "bgr8";  // TODO(floriantschopp): make this generic.
          cv_image.image = image_resource;
          cv_image.toImageMsg(maskrcnn_srv.request.image);
          if (maskrcnn_client_.call(maskrcnn_srv)) {
            // Do something with service result.
            VLOG(5) << "RGB image at " << vertex_id << " has "
                    << maskrcnn_srv.response.boxes.size() << " boxes";
            if (maskrcnn_srv.response.boxes.size() > 0) {
              resources::ObjectInstanceBoundingBoxes boxes;
              cv::Mat masks;
              convertBoundingBoxesAndMasks(
                  &masks, &boxes, maskrcnn_srv.response.boxes,
                  maskrcnn_srv.response.class_ids,
                  maskrcnn_srv.response.class_names,
                  maskrcnn_srv.response.scores, maskrcnn_srv.response.masks);
              map->storeFrameResource(
                  boxes, idx,
                  backend::ResourceType::kObjectInstanceBoundingBoxes, vertex);
              map->storeFrameResource(
                  masks, idx, backend::ResourceType::kObjectInstanceMasks,
                  vertex);
            }
          } else {
            LOG(ERROR) << "RGB image at " << vertex_id
                       << ": Failed to call service infer_image at mask "
                          "rcnn ros node.";
            break;
          }

        } else {
          VLOG(2) << idx << "th camera of NFrame " << n_frame.getId()
                  << " does not have associated color image. Skipping...";
        }
      }
      progress_bar.increment();
    }
    ++mission_num;
  }
  return common::kSuccess;
}

int SemantifyPlugin::generateAndSaveSemanticObjectMeasurements() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    VLOG(1) << "No missions found!";
    return common::kStupidUserError;
  }

  // Loop through missions in a map.
  vi_map::MissionVertexIdList mission_to_vertex_ids;
  map->getVertexIdsByMission(&mission_to_vertex_ids);
  VLOG(5) << "mission_to_vertex_ids size : " << mission_to_vertex_ids.size();
  for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
       mission_to_vertex_ids) {
    VLOG(5) << "# of vertices in this mission: "
            << mission_vertex_id_pair.second.size();
    // Go through all vertices and fill in the semantic data into the visual
    // frames of RGB cameras.
    for (const pose_graph::VertexId& vertex_id :
         mission_vertex_id_pair.second) {
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      // Loop through all cameras in the vertex.
      for (unsigned int idx = 0; idx < vertex.numFrames(); ++idx) {
        // Skip if the camera does not have color image, bounding box or
        // masks.
        cv::Mat image;
        cv::Mat image_masks;
        resources::ObjectInstanceBoundingBoxes boxes;
        if (!semantify::getSemanticResources(
                *map, vertex, idx, &image, &image_masks, &boxes)) {
          VLOG(5) << "Frame " << idx << "of vertex " << vertex.id()
                  << "does not have semantic resources.";
          continue;
        }

        if (boxes.size() == 0u) {
          LOG(INFO) << "Skipping frame with no detections.";
          continue;
        }

        // Setup service request for netvlad ros node.
        // Service handler expects images in bgr8 encoding.
        netvlad_tf::netvlad_result netvlad_srv;
        netvlad_srv.request.use_mask =
            FLAGS_semantify_generate_descriptor_filter_by_mask;
        cv_bridge::CvImage cv_image, cv_image_mask;
        cv_image.encoding = "bgr8";
        cv_image_mask.encoding = "bgr8";
        // Create request for netvlad descriptors.
        cv_image.image = image;
        cv_image.toImageMsg(netvlad_srv.request.image);
        if (FLAGS_semantify_generate_descriptor_filter_by_mask) {
          cv_image_mask.image = image_masks;
          cv_image_mask.toImageMsg(netvlad_srv.request.mask);
        }

        convertResourceBoundingBoxesToDescriptorGenerationMessage(
            boxes, &netvlad_srv.request.boxes,
            &netvlad_srv.request.class_numbers,
            &netvlad_srv.request.instance_numbers);
        if (netvlad_client_.call(netvlad_srv)) {
          CHECK_EQ(
              netvlad_srv.request.boxes.size(),
              netvlad_srv.response.descriptors.size());
        } else {
          LOG(ERROR) << "Failed to call service generate_descriptor at "
                        "netvlad_tf node";
          return common::kUnknownError;
        }
        aslam::VisualFrame::Ptr visual_frame = vertex.getVisualFrameShared(idx);
        saveSemanticObjectMeasurementsToVisualFrame(
            visual_frame, boxes, netvlad_srv.response.descriptors);
        // populate the observed_semantic_landmark_ids_ with invalid for the
        // vertex to avoid access error, the variable are assumed to be
        // present when measurements are created. The ids are invalid because
        // they are not associated with any semantic landmarks yet. the
        // invalid ids will be replaced once the init_track_semantic_landmarks
        // command is called.
        vi_map::SemanticLandmarkId invalid_id;
        invalid_id.setInvalid();
        vi_map::SemanticLandmarkIdList invalid_observed_semantic_landmark_ids;
        std::vector<int> measurement_indicies;
        for (size_t j = 0; j < visual_frame->getNumSemanticObjectMeasurements();
             j++) {
          measurement_indicies.push_back(j);
          invalid_observed_semantic_landmark_ids.push_back(invalid_id);
        }
        vertex.setObservedSemanticLandmarkIds(
            idx, measurement_indicies, invalid_observed_semantic_landmark_ids);
      }
    }
  }
  return common::kSuccess;
}

int SemantifyPlugin::generateAndSaveSemanticObjectTrackIdsFromDeepsort() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    VLOG(1) << "No missions found!";
    return common::kStupidUserError;
  }

  // Gets the list of vertecies in this mission.
  vi_map::MissionVertexIdList mission_to_vertex_ids;
  map->getVertexIdsByMission(&mission_to_vertex_ids);
  LOG(INFO) << "mission_to_vertex_ids size : " << mission_to_vertex_ids.size();
  for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
       mission_to_vertex_ids) {
    LOG(INFO) << "# of vertices in this mission: "
              << mission_vertex_id_pair.second.size();
    // go through all vertex and fill in the semantic data into thevisual
    // frames of RGB cameras
    for (const pose_graph::VertexId& vertex_id :
         mission_vertex_id_pair.second) {
      vi_map::Vertex& v = map->getVertex(vertex_id);
      // TODO(jkuo): currently hard coded to the first camera as
      // it is the only color camera, so we don't think about
      // how to manage the same object observed from different camera
      // scenario and how the tracker should handle this
      aslam::VisualFrame::Ptr visual_frame = v.getVisualFrameShared(0);

      // skip frames with no detections, it can happen
      if (visual_frame->getSemanticObjectMeasurements().cols() <= 0u) {
        LOG(INFO) << "Skipping frame with no detections.";
        continue;
      }

      deep_sort_ros::deep_sort_result deep_sort_srv;
      populateDeepSortServiceRequest(visual_frame, &deep_sort_srv);
      deep_sort_srv.request.min_confidence =
          FLAGS_semantify_tracking_confidence_threshold;
      if (deep_sort_client_.call(deep_sort_srv)) {
        CHECK_EQ(
            deep_sort_srv.request.boxes.size(),
            deep_sort_srv.response.track_ids.size());
        Eigen::VectorXi semantic_track_ids;
        semantic_track_ids.resize(deep_sort_srv.response.track_ids.size());
        for (size_t i = 0; i < deep_sort_srv.response.track_ids.size(); i++) {
          semantic_track_ids(i) = deep_sort_srv.response.track_ids[i];
        }
        visual_frame->swapSemanticObjectTrackIds(&semantic_track_ids);
      } else {
        LOG(ERROR) << "Failed to call service generate_semantic_track_ids at "
                      "deep_sort_ros node";
        return common::kUnknownError;
      }
    }
  }
  return common::kSuccess;
}

int SemantifyPlugin::visualizeSemanticObjectChannelsInVisualFrame() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    VLOG(1) << "No missions found!";
    return common::kStupidUserError;
  }

  // Create visualization window.
  cv::namedWindow(input_vis_window_name, cv::WINDOW_NORMAL);

  // Loop through missions in a map.
  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::VIMission& mission = map->getMission(mission_id);
    VLOG(5) << "Currently viewing mission id: " << mission_id.hexString();
    vi_map::MissionVertexIdList mission_to_vertex_ids;
    map->getVertexIdsByMission(&mission_to_vertex_ids);
    VLOG(5) << "mission_to_vertex_ids size : " << mission_to_vertex_ids.size();
    for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
         mission_to_vertex_ids) {
      VLOG(5) << "# of vertices in this mission: "
              << mission_vertex_id_pair.second.size();
      // Go through all vertex and fill in the semantic data into the visual
      // frames of RGB cameras.
      for (const pose_graph::VertexId& vertex_id :
           mission_vertex_id_pair.second) {
        vi_map::Vertex& vertex = map->getVertex(vertex_id);
        // Loop through all cameras in the vertex.
        for (unsigned int idx = 0; idx < vertex.numFrames(); ++idx) {
          // Skip if the camera does not have color image, bounding box or
          // masks.
          cv::Mat image;
          cv::Mat image_masks;
          resources::ObjectInstanceBoundingBoxes boxes;
          if (!semantify::getSemanticResources(
                  *map, vertex, idx, &image, &image_masks, &boxes)) {
            continue;
          }

          Eigen::Matrix2Xd empty_keypoints;
          empty_keypoints.resize(Eigen::NoChange, 0);

          const aslam::VisualFrame& visual_frame = vertex.getVisualFrame(idx);
          CHECK(visual_frame.hasSemanticObjectMeasurements());
          CHECK(visual_frame.hasSemanticObjectMeasurementUncertainties());
          CHECK(visual_frame.hasSemanticObjectClassIds());
          if (!visual_frame.hasSemanticObjectTrackIds()) {
            Eigen::VectorXi empty_track_ids;
            aslam::VisualFrame& visual_frame_mutable =
                vertex.getVisualFrame(idx);
            visual_frame_mutable.setSemanticObjectTrackIds(empty_track_ids);
          }

          const Eigen::Matrix4Xd& measurements =
              visual_frame.getSemanticObjectMeasurements();
          const Eigen::VectorXd& uncertainties =
              visual_frame.getSemanticObjectMeasurementUncertainties();
          const Eigen::VectorXi& class_ids =
              visual_frame.getSemanticObjectClassIds();
          resources::ObjectInstanceBoundingBoxes res_boxes;
          convertSemanticObjectMeasurementsToResourceBoundingBoxes(
              measurements, uncertainties, class_ids, &res_boxes);

          // Copy the instance number so we can recover the mask.
          if (FLAGS_semantify_generate_descriptor_filter_by_mask) {
            CHECK_EQ(res_boxes.size(), boxes.size());
            for (size_t i = 0; i < res_boxes.size(); i++) {
              res_boxes[i].instance_number = boxes[i].instance_number;
            }
          }

          Eigen::VectorXi track_ids;
          if (visual_frame.getSemanticObjectTrackIds().size() == 0u) {
            track_ids.resize(measurements.cols());
            for (int i = 0; i < track_ids.size(); i++) {
              track_ids(i) = -1;
            }
          } else {
            track_ids = visual_frame.getSemanticObjectTrackIds();
          }
          semantify::visualizeInput(
              image, res_boxes, empty_keypoints, track_ids, image_masks, true,
              FLAGS_semantify_generate_descriptor_filter_by_mask);
          cv::waitKey(1000.0 / FLAGS_semantify_visualization_frequency);
        }
      }
    }
  }
  semantify::destroyVisualizationWindow();
  return common::kSuccess;
}

int SemantifyPlugin::visualizeOptionalResourcesBoundingboxes() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    VLOG(1) << "No missions found!";
    return common::kStupidUserError;
  }

  // create visualization window
  cv::namedWindow(input_vis_window_name, cv::WINDOW_NORMAL);
  // gets the list of vertecies in this mission
  vi_map::MissionVertexIdList mission_to_vertex_ids;
  map->getVertexIdsByMission(&mission_to_vertex_ids);
  LOG(INFO) << "mission_to_vertex_ids size : " << mission_to_vertex_ids.size();
  for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
       mission_to_vertex_ids) {
    LOG(INFO) << "# of vertices in this mission: "
              << mission_vertex_id_pair.second.size();
    // go through all vertex and fill in the semantic data into thevisual
    // frames of RGB cameras
    for (const pose_graph::VertexId& vertex_id :
         mission_vertex_id_pair.second) {
      vi_map::Vertex& vertex = map->getVertex(vertex_id);
      for (unsigned int idx = 0; idx < vertex.numFrames(); ++idx) {
        // Skip if the camera does not have color image, bounding box or
        // masks.
        cv::Mat image;
        cv::Mat image_masks;
        resources::ObjectInstanceBoundingBoxes boxes;
        if (!semantify::getSemanticResources(
                *map, vertex, idx, &image, &image_masks, &boxes)) {
          continue;
        }

        Eigen::Matrix2Xd empty_keypoints;
        empty_keypoints.resize(Eigen::NoChange, 0);

        Eigen::VectorXi empty_track_ids;
        empty_track_ids.resize(boxes.size());
        for (int i = 0; i < empty_track_ids.size(); i++) {
          empty_track_ids(i) = -1;
        }

        semantify::visualizeInput(
            image, boxes, empty_keypoints, empty_track_ids, image_masks, true,
            FLAGS_semantify_generate_descriptor_filter_by_mask);
        cv::waitKey(1000.0 / FLAGS_semantify_visualization_frequency);
      }
    }
  }
  semantify::destroyVisualizationWindow();
  return common::kSuccess;
}

int SemantifyPlugin::
    visualizeSemanticLandmarksAndGenerateTrackIdToSemanticLandmarkMap() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  if (map->numSemanticLandmarksInIndex() == 0u) {
    LOG(INFO) << "There are no semantic landmarks in the map";
    return common::kStupidUserError;
  }

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    VLOG(1) << "No missions found!";
    return common::kStupidUserError;
  }

  // loop through missions in a map
  mission_id_to_semantic_landmark_id_.clear();
  semantic_id_to_track_id_map_.clear();
  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::SemanticLandmarkIdList semantic_ids;
    map->getAllSemanticLandmarkIdsInMission(mission_id, &semantic_ids);
    for (const vi_map::SemanticLandmarkId& id : semantic_ids) {
      // only show good quality semantic landmarks
      const vi_map::SemanticLandmark& semantic_landmark =
          map->getSemanticLandmark(id);
      if (semantic_landmark.getQuality() ==
          vi_map::SemanticLandmark::Quality::kBad) {
        continue;
      }
      // make sure the semantic landmark has class id assigned
      // TODO(jkuo): probably not the best way to check, semantic landmark
      // itself should have a way to indicate if the class id is set properly
      const std::unordered_map<int, size_t>& class_count_map =
          semantic_landmark.getClassIdCount();
      if (class_count_map.size() == 0u) {
        LOG(ERROR)
            << "Please run the command update_semantic_landmarks_class_ids.";
        return common::kStupidUserError;
        ;
      }
      // filters out semantic landmarks with specific id
      std::vector<std::string> class_ids_str;
      bool skip_this = false;
      if (!FLAGS_semantify_semantic_landmark_class_filter.empty()) {
        parseIdsFromString(
            FLAGS_semantify_semantic_landmark_class_filter, &class_ids_str);
        int semantic_landmark_class_id = semantic_landmark.getClassId();
        for (const std::string& class_id : class_ids_str) {
          if (semantic_landmark_class_id == std::stoi(class_id)) {
            skip_this = true;
          }
        }
        if (skip_this) {
          continue;
        }
      }

      std::string lm_quality;
      if (semantic_landmark.getQuality() ==
          vi_map::SemanticLandmark::Quality::kGood) {
        lm_quality = "Good";
      } else if (
          semantic_landmark.getQuality() ==
          vi_map::SemanticLandmark::Quality::kUnknown) {
        lm_quality = "Unknown";
      } else if (
          semantic_landmark.getQuality() ==
          vi_map::SemanticLandmark::Quality::kLocalizationSummaryLandmark) {
        lm_quality = "LocalizationSummaryLandmark";
      }
      vi_map::SemanticObjectIdentifierList object_ids;
      map->getSemanticObjectIdentifiersForSemanticLandmark(id, &object_ids);
      // some number that is not used
      int32_t track_id = -99;
      for (const vi_map::SemanticObjectIdentifier object_id : object_ids) {
        const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
        const size_t frame_index = object_id.frame_id.frame_index;
        const size_t measurement_index = object_id.measurement_index;
        const vi_map::Vertex& v = map->getVertex(vertex_id);
        const aslam::VisualFrame& vf = v.getVisualFrame(frame_index);
        // first time check
        if (track_id == -99) {
          track_id = vf.getSemanticObjectTrackId(measurement_index);
          // save for evaluation
          TrackIdsToSemanticLandmarkIdsMap& track_id_to_semantic_landmark_id =
              mission_id_to_semantic_landmark_id_[mission_id];
          track_id_to_semantic_landmark_id[track_id] = semantic_landmark.id();
          semantic_id_to_track_id_map_[semantic_landmark.id()] = track_id;
        } else {
          // should not fail because all landmarks are initialized from
          // measurements with track ids not equal to -1 and from the same id
          CHECK_NE(track_id, -1);
          CHECK_EQ(track_id, vf.getSemanticObjectTrackId(measurement_index));
        }
      }
      if (!visualizeSemanticLandmarkInfo(
              *(map.get()), mission_id, track_id, id, class_count_map)) {
        return common::kStupidUserError;
      }
    }
  }
  return common::kSuccess;
}

int SemantifyPlugin::evaluateSemanticLandmarkWithTrackId() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  if (map->numSemanticLandmarksInIndex() == 0u) {
    LOG(ERROR) << "There are no semantic landmarks in the map";
    return common::kStupidUserError;
  }

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  CHECK_GT(mission_ids.size(), 0u);

  if (mission_ids.size() > 1u && FLAGS_map_mission.empty()) {
    LOG(ERROR) << "There are more than one mission in the map. "
               << "Please specify the mission id using --map_mission";
    return common::kStupidUserError;
  }
  // check if the specified mission is valid, else assign to the only
  // mission in the map
  vi_map::MissionId mission_id;
  if (!FLAGS_map_mission.empty()) {
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    }
  } else {
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  }

  const TrackIdsToSemanticLandmarkIdsMap& track_id_to_semantic_landmark_id =
      mission_id_to_semantic_landmark_id_[mission_id];
  auto search = track_id_to_semantic_landmark_id.find(
      FLAGS_semantify_semantic_landmark_track_id);
  if (search == track_id_to_semantic_landmark_id.end()) {
    LOG(ERROR) << "There is no semantic landmark with that track id in the "
               << "mission id: " << mission_id.hexString()
               << " track id: " << FLAGS_semantify_semantic_landmark_track_id;
    return common::kStupidUserError;
  }
  const vi_map::SemanticLandmarkId id = search->second;

  // create visualization window
  cv::namedWindow(input_vis_window_name, cv::WINDOW_NORMAL);
  Eigen::Matrix2Xd empty_keypoints;
  empty_keypoints.resize(Eigen::NoChange, 0);
  Eigen::VectorXi track_ids;
  track_ids.resize(1u);
  track_ids(0) = FLAGS_semantify_semantic_landmark_track_id;
  Eigen::VectorXd uncertainty_vec;
  uncertainty_vec.resize(1u);
  Eigen::VectorXi class_id_vec;
  class_id_vec.resize(1u);

  vi_map::SemanticObjectIdentifierList object_ids;
  map->getSemanticObjectIdentifiersForSemanticLandmark(id, &object_ids);
  for (const vi_map::SemanticObjectIdentifier object_id : object_ids) {
    const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
    const size_t frame_index = object_id.frame_id.frame_index;
    const size_t measurement_index = object_id.measurement_index;
    const vi_map::Vertex& vertex = map->getVertex(vertex_id);
    const aslam::VisualFrame& visual_frame = vertex.getVisualFrame(frame_index);
    CHECK_EQ(mission_id, vertex.getMissionId());
    // Skip if the camera does not have color image, bounding box or
    // masks.
    cv::Mat image;
    cv::Mat image_masks;
    resources::ObjectInstanceBoundingBoxes boxes;
    if (!semantify::getSemanticResources(
            *map, vertex, frame_index, &image, &image_masks, &boxes)) {
      continue;
    }
    const Eigen::Vector4d measurement =
        visual_frame.getSemanticObjectMeasurement(measurement_index);
    double uncertainty =
        visual_frame.getSemanticObjectMeasurementUncertainty(measurement_index);
    int class_id = visual_frame.getSemanticObjectClassId(measurement_index);

    uncertainty_vec(0) = uncertainty;
    class_id_vec(0) = class_id;
    resources::ObjectInstanceBoundingBoxes res_boxes;
    convertSemanticObjectMeasurementsToResourceBoundingBoxes(
        measurement, uncertainty_vec, class_id_vec, &res_boxes);

    // copy the instance number so we can recover the mask
    if (FLAGS_semantify_generate_descriptor_filter_by_mask) {
      res_boxes[0].instance_number = boxes[measurement_index].instance_number;
    }

    semantify::visualizeInput(
        image, res_boxes, empty_keypoints, track_ids, image_masks, true,
        FLAGS_semantify_generate_descriptor_filter_by_mask);
    cv::waitKey(1000.0 / FLAGS_semantify_visualization_frequency);
  }
  semantify::destroyVisualizationWindow();
  return common::kSuccess;
}

int SemantifyPlugin::generateDescriptorClusters() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (map->numSemanticLandmarksInIndex() == 0u) {
    LOG(INFO) << "There are no semantic landmarks in the map";
    return common::kStupidUserError;
  }

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  for (const vi_map::MissionId& mission_id : mission_ids) {
    LOG(INFO) << "For Mission ID: " << mission_id.hexString();
    const TrackIdsToSemanticLandmarkIdsMap& track_id_to_semantic_landmark_id =
        mission_id_to_semantic_landmark_id_[mission_id];
    if (track_id_to_semantic_landmark_id.size() == 0u) {
      LOG(INFO) << "please run the command "
                   "visualize_semantic_landmarks_and_generate_track_id_to_"
                   "semantic_landmark_map first.";
      return common::kStupidUserError;
    }

    // create k_medoids_manager
    mission_to_kmedoids_manager_[mission_id] =
        std::move(std::unique_ptr<KMedoidsClusteringManager>(
            new KMedoidsClusteringManager()));

    for (const auto& pair : track_id_to_semantic_landmark_id) {
      const int track_id = pair.first;
      const vi_map::SemanticLandmarkId id = pair.second;
      vi_map::SemanticObjectIdentifierList object_ids;
      map->getSemanticObjectIdentifiersForSemanticLandmark(id, &object_ids);
      aslam::VisualFrame::SemanticObjectDescriptorsT
          semantic_landmark_descriptors;
      // TODO(jkuo): magic number alert, will need to clean this, but no elegant
      // way
      semantic_landmark_descriptors.resize(4096, object_ids.size());
      for (size_t i = 0; i < object_ids.size(); i++) {
        const vi_map::SemanticObjectIdentifier object_id = object_ids[i];
        const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
        const size_t frame_index = object_id.frame_id.frame_index;
        const size_t measurement_index = object_id.measurement_index;
        const vi_map::Vertex& v = map->getVertex(vertex_id);
        const aslam::VisualFrame& visual_frame = v.getVisualFrame(frame_index);
        const Eigen::MatrixXf::ColXpr descriptor =
            visual_frame.getSemanticObjectDescriptor(measurement_index);
        semantic_landmark_descriptors.col(i) = descriptor;
      }
      LOG(INFO) << "Creates KMedoids for track id: " << track_id;
      mission_to_kmedoids_manager_[mission_id]->createKMedoids(
          track_id, semantic_landmark_descriptors, 1);
    }
  }
  return common::kSuccess;
}

int SemantifyPlugin::displayDescriptorClustersScores() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (map->numSemanticLandmarksInIndex() == 0u) {
    LOG(INFO) << "There are no semantic landmarks in the map";
    return common::kStupidUserError;
  }

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  CHECK_GT(mission_ids.size(), 0u);

  if (mission_ids.size() > 1u && FLAGS_map_mission.empty()) {
    LOG(ERROR) << "There are more than one mission in the map. "
               << "Please specify the mission id using --map_mission";
    return common::kStupidUserError;
  }

  vi_map::MissionId mission_id;
  if (!FLAGS_map_mission.empty()) {
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    }
  } else {
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
  }

  const TrackIdsToSemanticLandmarkIdsMap& track_id_to_semantic_landmark_id =
      mission_id_to_semantic_landmark_id_[mission_id];
  if (track_id_to_semantic_landmark_id.size() == 0u) {
    LOG(INFO) << "please run the command "
                 "visualize_initialized_semantic_landmarks first.";
    return common::kStupidUserError;
  }

  if (!mission_to_kmedoids_manager_[mission_id]) {
    LOG(INFO) << "The k medoids cluster manager for mission_id has not been "
                 "initialized yet."
              << "mission_id: " << mission_id.hexString();
    return common::kStupidUserError;
  }

  // create k_medoids_manager
  mission_to_kmedoids_manager_[mission_id]->displayKMedoidsScores();
  return common::kSuccess;
}

int SemantifyPlugin::compareDescriptorClustersScores() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (map->numSemanticLandmarksInIndex() == 0u) {
    LOG(INFO) << "There are no semantic landmarks in the map";
    return common::kStupidUserError;
  }

  if (mission_to_kmedoids_manager_.size() == 0) {
    LOG(INFO) << "Please run the command generate_descriptor_clusters first.";
    return common::kStupidUserError;
  }

  vi_map::MissionId mission_id_0, mission_id_1;
  if (FLAGS_semantify_descriptor_comparison_mission_id_0.empty()) {
    LOG(ERROR) << "Please specify the mission id 0 using "
                  "--semantify_descriptor_comparison_mission_id_0";
    return common::kStupidUserError;
  } else {
    map->ensureMissionIdValid(
        FLAGS_semantify_descriptor_comparison_mission_id_0, &mission_id_0);
    if (!mission_id_0.isValid()) {
      return common::kStupidUserError;
    }
  }
  if (FLAGS_semantify_descriptor_comparison_mission_id_1.empty()) {
    LOG(ERROR) << "Please specify the mission id 1 using "
                  "--semantify_descriptor_comparison_mission_id_0";
    return common::kStupidUserError;
  } else {
    map->ensureMissionIdValid(
        FLAGS_semantify_descriptor_comparison_mission_id_1, &mission_id_1);
    if (!mission_id_1.isValid()) {
      return common::kStupidUserError;
    }
  }
  std::vector<std::string> track_ids_str_0, track_ids_str_1;
  if (FLAGS_semantify_descriptor_comparison_semantic_landmark_track_ids_0
          .empty()) {
    LOG(ERROR)
        << "Please specify the track ids for group 0 using "
           "--semantify_descriptor_comparison_semantic_landmark_track_ids_0";
    return common::kStupidUserError;
  } else {
    // gets the track_ids from flag
    parseIdsFromString(
        FLAGS_semantify_descriptor_comparison_semantic_landmark_track_ids_0,
        &track_ids_str_0);
  }
  if (FLAGS_semantify_descriptor_comparison_semantic_landmark_track_ids_1
          .empty()) {
    LOG(ERROR)
        << "Please specify the track ids for group 1 using "
           "--semantify_descriptor_comparison_semantic_landmark_track_ids_1";
    return common::kStupidUserError;
  } else {
    // gets the track_ids from flag
    parseIdsFromString(
        FLAGS_semantify_descriptor_comparison_semantic_landmark_track_ids_1,
        &track_ids_str_1);
  }

  if (mission_id_to_semantic_landmark_id_[mission_id_0].size() == 0u) {
    LOG(INFO) << "The mapping between track ids and semantic landmarks "
              << "is not initialized for mission 0";
    return common::kStupidUserError;
  }

  if (mission_id_to_semantic_landmark_id_[mission_id_1].size() == 0u) {
    LOG(INFO) << "The mapping between track ids and semantic landmarks "
              << "is not initialized for mission 1";
    return common::kStupidUserError;
  }

  if (!mission_to_kmedoids_manager_[mission_id_0]) {
    LOG(INFO) << "The k medoids cluster manager has not been initialized "
              << "for mission 0.";
    return common::kStupidUserError;
  }

  if (!mission_to_kmedoids_manager_[mission_id_1]) {
    LOG(INFO) << "The k medoids cluster manager has not been initialized "
              << "for mission 1.";
    return common::kStupidUserError;
  }

  for (size_t i = 0; i < track_ids_str_0.size(); i++) {
    for (size_t j = 0; j < track_ids_str_1.size(); j++) {
      LOG(INFO) << track_ids_str_0[i] << " vs " << track_ids_str_1[j];
      int track_id_i = std::stoi(track_ids_str_0[i]);
      int track_id_j = std::stoi(track_ids_str_1[j]);
      KMedoidsClusteringManager::KMedoidsIndexMap k_index_map_i =
          mission_to_kmedoids_manager_[mission_id_0]
              ->getKMedoidsIndiciesForTrackId(track_id_i);
      KMedoidsClusteringManager::KMedoidsIndexMap k_index_map_j =
          mission_to_kmedoids_manager_[mission_id_1]
              ->getKMedoidsIndiciesForTrackId(track_id_j);
      const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors_i =
          mission_to_kmedoids_manager_[mission_id_0]->getKMedoidsDescriptors(
              track_id_i);
      const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors_j =
          mission_to_kmedoids_manager_[mission_id_1]->getKMedoidsDescriptors(
              track_id_j);

      for (size_t k = 1; k <= k_index_map_i.size(); k++) {
        LOG(INFO) << "For k = " << k;
        double mean_distance_from_i_to_j = 0.0;
        for (size_t m = 0; m < k; m++) {
          const std::vector<clustering::object_id>& medoids_index_i =
              k_index_map_i[k];
          // LOG(INFO)<<"medoids_index_i.size(): "<<medoids_index_i.size();
          // LOG(INFO)<<"medoids_index_i[m]: "<<medoids_index_i[m];
          const Eigen::MatrixXf::ConstColXpr descriptor_i =
              descriptors_i.col(medoids_index_i[m]);
          double mean_distance_from_m_to_n = 0.0;
          for (size_t n = 0; n < k; n++) {
            const std::vector<clustering::object_id>& medoids_index_j =
                k_index_map_j[k];
            const Eigen::MatrixXf::ConstColXpr descriptor_j =
                descriptors_j.col(medoids_index_j[n]);
            double distance = (descriptor_i - descriptor_j).norm();
            LOG(INFO) << "m: " << m << " n: " << n << " distance:" << distance;
            mean_distance_from_m_to_n += distance / k;
          }
          LOG(INFO) << "mean distance from cluster_i_" << m
                    << " to cluster_j: " << mean_distance_from_m_to_n;
          mean_distance_from_i_to_j += mean_distance_from_m_to_n / k;
        }
        LOG(INFO) << "mean distance from cluster_i to cluster_j: "
                  << mean_distance_from_i_to_j;
      }
    }
  }
  return common::kSuccess;
}

int SemantifyPlugin::matchSemanticLandmarksInOneMission() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (map->numSemanticLandmarksInIndex() == 0u) {
    LOG(INFO) << "There are no semantic landmarks in the map";
    return common::kStupidUserError;
  }

  if (mission_id_to_semantic_landmark_id_.size() == 0u) {
    LOG(INFO) << "please run the command "
                 "visualize_initialized_semantic_landmarks first.";
    return common::kStupidUserError;
  }

  if (mission_to_kmedoids_manager_.size() == 0u) {
    LOG(INFO) << "The k medoids cluster manager has not been initialized yet. "
              << "please run the command generate_descriptor_clusters.";
    return common::kStupidUserError;
  }
  // check if the specified mission is valid, else assign to the only
  // mission in the map
  vi_map::MissionId mission_id;
  if (!FLAGS_map_mission.empty()) {
    map->ensureMissionIdValid(FLAGS_map_mission, &mission_id);
    if (!mission_id.isValid()) {
      return common::kStupidUserError;
    }
  } else {
    LOG(ERROR) << "Please specify the mission you want to match semantic "
                  "landmarks within.";
    return common::kStupidUserError;
  }

  const TrackIdsToSemanticLandmarkIdsMap& track_id_to_semantic_landmark_id =
      mission_id_to_semantic_landmark_id_[mission_id];

  LOG(INFO) << "track_id_to_semantic_landmark_id.size(): "
            << track_id_to_semantic_landmark_id.size();

  SourceTrackIdToMatchedRefTrackIdsMap track_id_to_matched_ids;
  constexpr bool kSetDiagonalToMaxCost = true;
  generateMatchCandidates(
      *(map.get()), mission_id, mission_id, kSetDiagonalToMaxCost,
      &track_id_to_matched_ids);

  // creates the matched track id correspondences
  visualization::Color candidate_color(255u, 255u, 255u);
  std::string topic = "semantic_landmark_match_candidates";
  visualizeMatchedTrackIds(
      *(map.get()), mission_id, mission_id, track_id_to_matched_ids,
      candidate_color, topic);

  return common::kSuccess;
}

int SemantifyPlugin::anchorMissionWithSemanticLandmarks() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  // distinguish the missions with known (ref) and unknown (source) T_G_M
  vi_map::MissionIdList mission_ids_ref;
  vi_map::MissionIdList mission_ids_source;
  if (!separateMissionsWithKnownBaseFrame(
          *(map.get()), &mission_ids_ref, &mission_ids_source)) {
    return common::kUnknownError;
  }

  if (mission_to_kmedoids_manager_.size() == 0u) {
    LOG(INFO) << "The k medoids cluster manager has not been initialized yet. "
              << "please run the command generate_descriptor_clusters.";
    return common::kStupidUserError;
  }

  // consider only two missions for now: one known T_G_M and one unknown T_G_M
  const vi_map::MissionId mission_id_ref = mission_ids_ref[0];
  const vi_map::MissionId mission_id_source = mission_ids_source[0];

  SourceTrackIdToMatchedRefTrackIdsMap
      source_track_id_to_matched_ref_ids_candidates;
  constexpr bool kSetDiagonalToMaxCost = false;
  generateMatchCandidates(
      *(map.get()), mission_id_source, mission_id_ref, kSetDiagonalToMaxCost,
      &source_track_id_to_matched_ref_ids_candidates);

  // debugging
  // LOG(INFO)<<"In source_track_id_to_matched_ref_ids_candidates: ";
  // for (const auto& pair:source_track_id_to_matched_ref_ids_candidates) {
  //   for(const auto& matched_id: pair.second) {
  //     LOG(INFO)<<std::to_string(pair.first)<<" -> "<<
  //     std::to_string(matched_id);
  //   }
  // }

  // visualization
  visualization::Color candidate_color(255u, 255u, 255u);
  std::string topic = "semantic_landmark_match_candidates";
  visualizeMatchedTrackIds(
      *(map.get()), mission_id_source, mission_id_ref,
      source_track_id_to_matched_ref_ids_candidates, candidate_color, topic);

  // sets the source T_G_M to T_G_M of ref temporarily
  const vi_map::VIMission& mission_ref = map->getMission(mission_id_ref);
  const vi_map::MissionBaseFrame& mission_baseframe_ref =
      map->getMissionBaseFrame(mission_ref.getBaseFrameId());
  const pose::Transformation& T_G_M_ref = mission_baseframe_ref.get_T_G_M();

  vi_map::VIMission& mission_source = map->getMission(mission_id_source);
  vi_map::MissionBaseFrame& mission_baseframe_source =
      map->getMissionBaseFrame(mission_source.getBaseFrameId());
  const pose::Transformation T_G_M_source_original =
      mission_baseframe_source.get_T_G_M();
  mission_baseframe_source.set_T_G_M(T_G_M_ref);

  CandidateIdToTransformationMap G_T_ref_source_ransac_candidates;
  CandidateIdToTrackIdMatchesMap
      source_track_id_to_matched_ref_ids_ransac_candidates;
  MatchCandidateScores match_candidate_scores;
  SourceTrackIdToMatchedRefTrackIdsMap empty_predefined_inliers;

  if (!generateTransformationHypothesisFromMatches(
          *(map.get()), mission_id_source, mission_id_ref,
          source_track_id_to_matched_ref_ids_candidates,
          empty_predefined_inliers, &G_T_ref_source_ransac_candidates,
          &source_track_id_to_matched_ref_ids_ransac_candidates,
          &match_candidate_scores)) {
    LOG(ERROR) << "Was unable to find a transformation between "
               << "ref mission: " << mission_id_ref.hexString()
               << " and source mission: " << mission_id_source.hexString();
    mission_baseframe_source.set_T_G_M(T_G_M_source_original);
    return common::kUnknownError;
  }

  CHECK_EQ(
      G_T_ref_source_ransac_candidates.size(),
      source_track_id_to_matched_ref_ids_ransac_candidates.size());
  CHECK_EQ(
      G_T_ref_source_ransac_candidates.size(), match_candidate_scores.size());

  // sort the match candidate scores
  std::sort(
      match_candidate_scores.begin(), match_candidate_scores.end(),
      compareMatchCandidateScores);
  // debugging for sorted match_candidate_scores
  // for (size_t i =0; i<match_candidate_scores.size(); i++){
  //   MatchCandidateScore& score = match_candidate_scores.at(i);
  //   LOG(INFO)<<"index: "<<i;
  //   LOG(INFO)<<"candidate_id: "<<score.candidate_id;
  //   LOG(INFO)<<"inlier_ratio: "<<score.inlier_ratio;
  //   LOG(INFO)<<"inlier_num: "<<score.inlier_num;
  //   LOG(INFO)<<"total_inlier_error: "<<score.total_inlier_error;
  //   LOG(INFO)<<"total_model_error: "<<score.total_model_error;
  // }
  // we only anchor with the candidate with best score
  const MatchCandidateScore& best_candidate_score = match_candidate_scores.at(
      FLAGS_semantify_visualize_match_candidate_index);
  const CandidateId best_candidate_id = best_candidate_score.candidate_id;
  const pose::Transformation& G_T_ref_source_ransac_best =
      G_T_ref_source_ransac_candidates[best_candidate_id];
  const SourceTrackIdToMatchedRefTrackIdsMap& best_candidate_id_matches =
      source_track_id_to_matched_ref_ids_ransac_candidates[best_candidate_id];
  LOG(INFO) << "Selected candidate:";
  LOG(INFO) << "candidate_id: " << best_candidate_score.candidate_id;
  LOG(INFO) << "inlier_ratio: " << best_candidate_score.inlier_ratio;
  LOG(INFO) << "inlier_num: " << best_candidate_score.inlier_num;
  LOG(INFO) << "total_inlier_error: "
            << best_candidate_score.total_inlier_error;
  LOG(INFO) << "total_model_error: " << best_candidate_score.total_model_error;
  LOG(INFO) << "Estimated G_T_ref_source based on largest inlier ratio: ";
  LOG(INFO) << (G_T_ref_source_ransac_best).getTransformationMatrix();
  LOG(INFO) << "Source mission id: " << mission_id_source.hexString();
  LOG(INFO) << "Ref mission id: " << mission_id_ref.hexString();
  LOG(INFO) << "Inlier matches: ";
  for (const auto& pair : best_candidate_id_matches) {
    LOG(INFO) << "source track id: " << pair.first;
    for (const int& matched_id : pair.second) {
      LOG(INFO) << "ref track id: " << matched_id;
    }
  }

  //// apply ransac result to all the verticies
  pose_graph::VertexIdList vertex_ids;
  map->getAllVertexIdsInMission(mission_id_source, &vertex_ids);
  const pose::Transformation T_M_G_source =
      mission_baseframe_source.get_T_G_M().inverse();
  for (const pose_graph::VertexId& vertex_id : vertex_ids) {
    vi_map::Vertex& vertex = map->getVertex(vertex_id);
    const pose::Transformation& T_M_I = vertex.get_T_M_I();
    vertex.set_T_M_I(G_T_ref_source_ransac_best * T_M_I);
  }
  constexpr bool kIsKnown = true;
  mission_baseframe_source.set_is_T_G_M_known(kIsKnown);

  // visualization
  // temporarily set to orignal T_G_M to create better visualization
  mission_baseframe_source.set_T_G_M(T_G_M_source_original);
  topic = "semantic_landmark_match_candidates";
  visualizeMatchedTrackIds(
      *(map.get()), mission_id_source, mission_id_ref,
      source_track_id_to_matched_ref_ids_candidates, candidate_color, topic);

  visualization::Color inlier_color(0u, 255u, 0u);
  topic = "semantic_landmark_match_ransac";
  visualizeMatchedTrackIds(
      *(map.get()), mission_id_source, mission_id_ref,
      best_candidate_id_matches, inlier_color, topic);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);
  for (const vi_map::MissionId& mission_id : mission_ids) {
    const TrackIdsToSemanticLandmarkIdsMap& track_id_to_semantic_landmark_id =
        mission_id_to_semantic_landmark_id_[mission_id];
    for (const auto& pair : track_id_to_semantic_landmark_id) {
      const vi_map::SemanticLandmark& semantic_landmark =
          map->getSemanticLandmark(pair.second);
      const std::unordered_map<int, size_t>& class_count_map =
          semantic_landmark.getClassIdCount();
      if (!visualizeSemanticLandmarkInfo(
              *(map.get()), mission_id, pair.first, pair.second,
              class_count_map)) {
        return common::kStupidUserError;
      }
    }
  }

  if (hasPlotter()) {
    getPlotter().visualizeMap(*map);
  }

  mission_baseframe_source.set_T_G_M(T_G_M_ref);

  return common::kSuccess;
}

int SemantifyPlugin::loopCloseMissionsWithSemanticLandmarks() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }

  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  if (mission_to_kmedoids_manager_.size() == 0u) {
    LOG(INFO) << "The k medoids cluster manager has not been initialized yet. "
              << "please run the command generate_descriptor_clusters.";
    return common::kStupidUserError;
  }

  // loop closure should work on all pairs of missions, but we now only look at
  // two different missions now. I feel like if it is the same mission,
  // the current ransac way doesnt work. maybe it should be from the candidate
  // matches which ones if i merge, it generates the least amount of "error" in
  // the new map?
  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  // const vi_map::MissionId mission_id_ref = mission_ids[0];
  // const vi_map::MissionId mission_id_source = mission_ids[0];

  vi_map::MissionId mission_id_ref, mission_id_source;
  if (FLAGS_semantify_loop_closure_mission_id_ref.empty()) {
    LOG(ERROR) << "Please specify the mission id ref using "
                  "--semantify_loop_closure_mission_id_ref";
    return common::kStupidUserError;
  } else {
    map->ensureMissionIdValid(
        FLAGS_semantify_loop_closure_mission_id_ref, &mission_id_ref);
    if (!mission_id_ref.isValid()) {
      return common::kStupidUserError;
    }
  }
  if (FLAGS_semantify_loop_closure_mission_id_source.empty()) {
    LOG(ERROR) << "Please specify the mission id source using "
                  "--semantify_loop_closure_mission_id_source";
    return common::kStupidUserError;
  } else {
    map->ensureMissionIdValid(
        FLAGS_semantify_loop_closure_mission_id_source, &mission_id_source);
    if (!mission_id_source.isValid()) {
      return common::kStupidUserError;
    }
  }

  // get the valid track ids of existing semantic landmarks
  const TrackIdsToSemanticLandmarkIdsMap&
      valid_track_ids_to_semantic_landmark_ids_source =
          mission_id_to_semantic_landmark_id_[mission_id_source];
  const TrackIdsToSemanticLandmarkIdsMap&
      valid_track_ids_to_semantic_landmark_ids_ref =
          mission_id_to_semantic_landmark_id_[mission_id_ref];

  // generate match candidates
  SourceTrackIdToMatchedRefTrackIdsMap
      source_track_id_to_matched_ref_ids_candidates;
  bool kSetDiagonalToMaxCost = (mission_id_ref == mission_id_source);
  LOG(INFO) << "kSetDiagonalToMaxCost: " << kSetDiagonalToMaxCost;
  generateMatchCandidates(
      *(map.get()), mission_id_source, mission_id_ref, kSetDiagonalToMaxCost,
      &source_track_id_to_matched_ref_ids_candidates);

  int match_count = 0;
  for (const auto& pair : source_track_id_to_matched_ref_ids_candidates) {
    for (const auto& matched_id : pair.second) {
      match_count++;
    }
  }
  VLOG(5) << "Raw match_count: " << match_count;

  // only works for match candidates in the same map, not across maps
  if (mission_id_source == mission_id_ref) {
    if (FLAGS_semantify_semantic_landmark_matching_filter_by_unique_visibility) {
      applyUniqueVisibilityFilter(
          *(map.get()), mission_id_source,
          source_track_id_to_matched_ref_ids_candidates);
    }
  }

  // visualization
  visualization::Color candidate_color(255u, 255u, 255u);
  std::string candidate_topic = "semantic_landmark_match_candidates";
  visualizeMatchedTrackIds(
      *(map.get()), mission_id_source, mission_id_ref,
      source_track_id_to_matched_ref_ids_candidates, candidate_color,
      candidate_topic);

  match_count = 0;
  for (const auto pair : source_track_id_to_matched_ref_ids_candidates) {
    for (const auto matched_id : pair.second) {
      match_count++;
    }
  }
  VLOG(5) << "after unique visibility filter match_count: " << match_count;

  // flatten match candidate maps
  std::vector<int> source_track_ids, ref_track_ids;
  std::vector<int> index_vector;
  int idx = 0;
  for (const auto& pair : source_track_id_to_matched_ref_ids_candidates) {
    if (pair.second.empty()) {
      continue;
    } else {
      for (const int& matched_id : pair.second) {
        source_track_ids.push_back(pair.first);
        ref_track_ids.push_back(matched_id);
        index_vector.push_back(idx);
        idx++;
      }
    }
  }
  CHECK_EQ(source_track_ids.size(), ref_track_ids.size());
  CHECK_EQ(source_track_ids.size(), index_vector.size());
  LOG(INFO) << "Total # of match candidate" << index_vector.size();
  for (size_t i = 0; i < index_vector.size(); i++) {
    LOG(INFO) << "index: " << i << " source_track_id: " << source_track_ids[i]
              << " ref_track_ids: " << ref_track_ids[i];
  }

  // TODO(jkuo): think about how to refactor this part into one function
  // get covisible and observing verticies for every semantic landmark
  pose_graph::VertexIdList all_verticies;
  map->getAllVertexIdsInMissionAlongGraph(mission_id_source, &all_verticies);
  // std::set<int> test_ids {954, 943, 233,120};
  SourceTrackIdToMatchedRefTrackIdsMap track_ids_to_covisible_track_ids_source;
  TrackIdToObservingVerticesMap track_ids_to_observing_verticies_source;
  for (auto& pair : valid_track_ids_to_semantic_landmark_ids_source) {
    TrackId track_id_source = pair.first;
    const vi_map::SemanticLandmarkId& id_source = pair.second;
    pose_graph::VertexIdList observing_verticies;
    std::set<TrackId> covisible_track_ids;
    getObservingVerticiesAndCovisibleTrackIds(
        *(map.get()), mission_id_source, id_source, &observing_verticies,
        &covisible_track_ids);

    if (FLAGS_semantify_semantic_landmark_lc_extend_visible_verticies) {
      extendObservingVerticies(
          *(map.get()), all_verticies, &observing_verticies,
          &covisible_track_ids);
      // TODO(jkuo): there is a problem with extending observing verticies
      // it may add itself into covisible track ids
    }
    // store covisible track ids and observing verticies
    track_ids_to_covisible_track_ids_source[track_id_source] =
        covisible_track_ids;
    track_ids_to_observing_verticies_source[track_id_source] =
        observing_verticies;
    if (FLAGS_semantify_visualize_covisible_objects) {
      visualization::Color covisible_color(0u, 0u, 255u);
      std::string covisible_topic = "semantic_landmark_covisible_track_ids";
      visualizeCovisibleTrackIds(
          *(map.get()), mission_id_source, track_id_source, covisible_track_ids,
          covisible_color, covisible_topic);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (FLAGS_semantify_visualize_observing_verticies) {
      visualization::Color observing_verticies_color(255u, 0u, 0u);
      std::string observing_verticies_topic =
          "semantic_landmark_observing_verticies";
      visualizeSemanticLandmarksObservableVerticies(
          *(map.get()), mission_id_source, track_id_source, observing_verticies,
          observing_verticies_color, observing_verticies_topic);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  all_verticies.clear();
  map->getAllVertexIdsInMissionAlongGraph(mission_id_ref, &all_verticies);
  SourceTrackIdToMatchedRefTrackIdsMap track_ids_to_covisible_track_ids_ref;
  TrackIdToObservingVerticesMap track_ids_to_observing_verticies_ref;
  for (auto& pair : valid_track_ids_to_semantic_landmark_ids_ref) {
    TrackId track_id_ref = pair.first;
    const vi_map::SemanticLandmarkId& id_ref = pair.second;
    pose_graph::VertexIdList observing_verticies;
    std::set<TrackId> covisible_track_ids;
    getObservingVerticiesAndCovisibleTrackIds(
        *(map.get()), mission_id_ref, id_ref, &observing_verticies,
        &covisible_track_ids);
    if (FLAGS_semantify_semantic_landmark_lc_extend_visible_verticies) {
      extendObservingVerticies(
          *(map.get()), all_verticies, &observing_verticies,
          &covisible_track_ids);
      // TODO(jkuo): there is a problem with extending observing verticies
      // it may add itself into covisible track ids
    }
    // store covisible track ids and observing verticies
    track_ids_to_covisible_track_ids_ref[track_id_ref] = covisible_track_ids;
    track_ids_to_observing_verticies_ref[track_id_ref] = observing_verticies;
    if (FLAGS_semantify_visualize_covisible_objects) {
      visualization::Color covisible_color(0u, 0u, 255u);
      std::string covisible_topic = "semantic_landmark_covisible_track_ids";
      visualizeCovisibleTrackIds(
          *(map.get()), mission_id_ref, track_id_ref, covisible_track_ids,
          covisible_color, covisible_topic);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    if (FLAGS_semantify_visualize_observing_verticies) {
      visualization::Color observing_verticies_color(255u, 0u, 0u);
      std::string observing_verticies_topic =
          "semantic_landmark_observing_verticies";
      visualizeSemanticLandmarksObservableVerticies(
          *(map.get()), mission_id_ref, track_id_ref, observing_verticies,
          observing_verticies_color, observing_verticies_topic);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }

  // for each match candidate, find its covisible supporting match candidate
  std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>
      match_idx_to_covisible_supporting_match_candidates_map;
  for (const auto& match_idx : index_vector) {
    TrackId track_id_source = source_track_ids[match_idx];
    TrackId track_id_ref = ref_track_ids[match_idx];
    const vi_map::SemanticLandmarkId& lm_id_source =
        valid_track_ids_to_semantic_landmark_ids_source.at(track_id_source);
    const vi_map::SemanticLandmarkId& lm_id_ref =
        valid_track_ids_to_semantic_landmark_ids_ref.at(track_id_ref);
    const Eigen::Vector3d& p_G_landmark_source =
        map->getSemanticLandmark_G_p_fi(lm_id_source);
    const Eigen::Vector3d& p_G_landmark_ref =
        map->getSemanticLandmark_G_p_fi(lm_id_ref);

    // 1. removes match candidates if it has both ends in the same cluster (for
    // within map)
    // 2. removes the match candidates whose track id source/ref is not in the
    // covisible group
    // 3. removes match candidates that exceed the max covisible object distance
    // difference Note: we include the match index as we will be using all to
    // compute the RANSAC transformation
    const std::set<int>& covisible_track_ids_source =
        track_ids_to_covisible_track_ids_source[track_id_source];
    const std::set<int>& covisible_track_ids_ref =
        track_ids_to_covisible_track_ids_ref[track_id_ref];
    SourceTrackIdToMatchedRefTrackIdsMap covisible_supporting_match_candidates;

    std::set<TrackId> unique_covisible_object_track_ids;
    for (const int& cov_idx : index_vector) {
      TrackId track_id_cov_source = source_track_ids[cov_idx];
      TrackId track_id_cov_ref = ref_track_ids[cov_idx];
      const vi_map::SemanticLandmarkId& cov_lm_id_source =
          valid_track_ids_to_semantic_landmark_ids_source.at(
              track_id_cov_source);
      const vi_map::SemanticLandmarkId& cov_lm_id_ref =
          valid_track_ids_to_semantic_landmark_ids_ref.at(track_id_cov_ref);

      // check the match candidate is not within cluster
      auto search_cov_source_in_source =
          covisible_track_ids_source.find(track_id_cov_source);
      auto search_cov_ref_in_ref =
          covisible_track_ids_ref.find(track_id_cov_ref);
      bool covisibility_check =
          (search_cov_source_in_source != covisible_track_ids_source.end() &&
           search_cov_ref_in_ref != covisible_track_ids_ref.end());

      // only works within one map, not across map
      if (mission_id_source == mission_id_ref) {
        auto search_cov_ref_in_source =
            covisible_track_ids_source.find(track_id_cov_ref);
        covisibility_check =
            covisibility_check &&
            (search_cov_ref_in_source == covisible_track_ids_source.end());
      }

      // check the match candidate satisfy distance difference constraint
      const Eigen::Vector3d& p_G_cov_landmark_source =
          map->getSemanticLandmark_G_p_fi(cov_lm_id_source);
      const Eigen::Vector3d& p_G_cov_landmark_ref =
          map->getSemanticLandmark_G_p_fi(cov_lm_id_ref);
      double inter_distance_source =
          (p_G_landmark_source - p_G_cov_landmark_source).norm();
      double inter_distance_ref =
          (p_G_landmark_ref - p_G_cov_landmark_ref).norm();
      double inter_error = fabs(inter_distance_source - inter_distance_ref);
      bool distance_diff_check =
          inter_error <=
          FLAGS_semantify_semantic_landmark_lc_max_covisible_object_candidate_distance_difference;

      if (covisibility_check && distance_diff_check) {
        covisible_supporting_match_candidates[track_id_cov_source].insert(
            track_id_cov_ref);
        unique_covisible_object_track_ids.insert(track_id_cov_ref);
      }
    }
    // need at least this many valid match candidates
    if (unique_covisible_object_track_ids.size() >=
        FLAGS_semantify_semantic_landmark_anchoring_ransac_min_inlier_num) {
      match_idx_to_covisible_supporting_match_candidates_map[match_idx] =
          covisible_supporting_match_candidates;
      if (FLAGS_semantify_visualize_valid_match_candidates) {
        visualization::Color valid_candidate_color(0u, 255u, 255u);
        std::string valid_candidate_topic =
            "semantic_landmark_valid_match_candidates";
        std::string name_space = std::to_string(track_id_source) + "_to_" +
                                 std::to_string(track_id_ref);
        visualizeMatchedTrackIds(
            *(map.get()), mission_id_source, mission_id_ref,
            match_idx_to_covisible_supporting_match_candidates_map[match_idx],
            valid_candidate_color, valid_candidate_topic, name_space);
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
    }
  }

  // debugging
  // for (const auto& pair:
  // match_idx_to_covisible_supporting_match_candidates_map) {
  //   LOG(INFO)<<"match_idx: "<<pair.first;
  //   for (const auto& pair2: pair.second){
  //     LOG(INFO)<<"supporting source: "<<pair2.first;
  //     for (const auto& ref_id: pair2.second){
  //       LOG(INFO)<<"supporting ref: "<<ref_id;
  //     }
  //   }
  // }

  // take the best ransac result for each landmark match candidate group
  std::unordered_map<int, MatchCandidateScore> accepted_candidate_scores;
  std::unordered_map<int, pose::Transformation>
      accepted_G_T_ref_source_ransac_map;
  // stores both main and all supporting match candidates
  std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>
      accepted_track_id_matches_map;
  // stores only the main match candidate
  std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>
      accepted_main_track_id_matches_map;
  std::vector<int> accepted_match_idx;
  for (const auto& pair :
       match_idx_to_covisible_supporting_match_candidates_map) {
    const int match_idx = pair.first;
    CandidateIdToTransformationMap G_T_ref_source_ransac_candidates;
    CandidateIdToTrackIdMatchesMap
        source_track_id_to_matched_ref_ids_ransac_candidates;
    MatchCandidateScores match_candidate_scores;
    // create predefine inlier for the main match candidate
    SourceTrackIdToMatchedRefTrackIdsMap predefined_inliers;
    std::set<TrackId> predefined_matched_inlier;
    predefined_matched_inlier.insert(ref_track_ids[match_idx]);
    predefined_inliers[source_track_ids[match_idx]] = predefined_matched_inlier;

    if (!generateTransformationHypothesisFromMatches(
            *(map.get()), mission_id_source, mission_id_ref, pair.second,
            predefined_inliers, &G_T_ref_source_ransac_candidates,
            &source_track_id_to_matched_ref_ids_ransac_candidates,
            &match_candidate_scores)) {
      LOG(ERROR) << "Was unable to find a good RANSAC result for match index: "
                 << std::to_string(match_idx);
      continue;
    }
    CHECK_EQ(
        G_T_ref_source_ransac_candidates.size(),
        source_track_id_to_matched_ref_ids_ransac_candidates.size());
    CHECK_EQ(
        G_T_ref_source_ransac_candidates.size(), match_candidate_scores.size());
    accepted_match_idx.push_back(match_idx);
    std::sort(
        match_candidate_scores.begin(), match_candidate_scores.end(),
        compareMatchCandidateScores);
    MatchCandidateScore best_candidate_score = match_candidate_scores.at(0);
    accepted_candidate_scores[match_idx] = best_candidate_score;
    const CandidateId best_candidate_id = best_candidate_score.candidate_id;
    LOG(INFO) << "match_candidate_index: " << match_idx
              << "track_id_source: " << source_track_ids[match_idx]
              << "track_id_ref: " << ref_track_ids[match_idx];
    LOG(INFO) << "candidate_id: " << best_candidate_score.candidate_id;
    LOG(INFO) << "inlier_ratio: " << best_candidate_score.inlier_ratio;
    LOG(INFO) << "inlier_num: " << best_candidate_score.inlier_num;
    LOG(INFO) << "total_inlier_error: "
              << best_candidate_score.total_inlier_error;
    LOG(INFO) << "total_model_error: "
              << best_candidate_score.total_model_error;
    // for(const auto& inlier_idx: best_candidate_score.initial_index) {
    //   LOG(INFO)<<"inlier_idx: "<<inlier_idx;
    // }
    accepted_G_T_ref_source_ransac_map[match_idx] =
        G_T_ref_source_ransac_candidates[best_candidate_id];
    accepted_track_id_matches_map[match_idx] =
        source_track_id_to_matched_ref_ids_ransac_candidates[best_candidate_id];
    // creates the main match candidate from matched index
    SourceTrackIdToMatchedRefTrackIdsMap main_matched_idx_track_ids_pair;
    std::set<TrackId> temp_set;
    temp_set.insert(ref_track_ids[match_idx]);
    main_matched_idx_track_ids_pair[source_track_ids[match_idx]] = temp_set;
    accepted_main_track_id_matches_map[match_idx] =
        main_matched_idx_track_ids_pair;

    LOG(INFO) << "ransac_best_covisible_track_id_matches: ";
    for (const auto& pair : source_track_id_to_matched_ref_ids_ransac_candidates
             [best_candidate_id]) {
      for (const auto& matched_id : pair.second) {
        LOG(INFO) << std::to_string(pair.first) + " -> " +
                         std::to_string(matched_id);
      }
    }
    LOG(INFO) << "accepted_track_id_matches: ";
    LOG(INFO) << std::to_string(source_track_ids[match_idx]) + " -> " +
                     std::to_string(ref_track_ids[match_idx]);
    visualization::Color accepted_candidate_color(0u, 255u, 0u);
    std::string accepted_candidate_topic =
        "semantic_landmark_accepted_match_candidates";
    visualizeMatchedTrackIds(
        *(map.get()), mission_id_source, mission_id_ref,
        source_track_id_to_matched_ref_ids_ransac_candidates[best_candidate_id],
        accepted_candidate_color, accepted_candidate_topic,
        std::to_string(match_idx));
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  LOG(INFO) << "match_idx_to_covisible_supporting_match_candidates_map.size: "
            << match_idx_to_covisible_supporting_match_candidates_map.size();
  LOG(INFO) << "accepted_track_id_matches_map.size: "
            << accepted_track_id_matches_map.size();

  // remove previously added semantic lc
  if (FLAGS_semantify_semantic_landmark_lc_remove_old_edges &&
      !semantic_lc_edge_ids_.empty()) {
    for (const pose_graph::EdgeId& edge_id : semantic_lc_edge_ids_) {
      map->removeEdge(edge_id);
    }
    semantic_lc_edge_ids_.clear();
  }

  // add semantic loop closure edge to map from the accepted main track id pairs
  if (!addSemanticLoopClosureEdgeToMap(
          map.get(), mission_id_source, mission_id_ref, accepted_match_idx,
          accepted_G_T_ref_source_ransac_map, accepted_track_id_matches_map,
          accepted_main_track_id_matches_map,
          track_ids_to_observing_verticies_source,
          track_ids_to_observing_verticies_ref)) {
    LOG(ERROR) << "Was Failed to add semantic loop closure edge to map.";
    return common::kUnknownError;
  }

  // merge the matched semantic landmarks
  if (FLAGS_semantify_semantic_landmark_lc_merge_matched_landmarks) {
    mergeSemanticLandmarks(
        map.get(), mission_id_source, mission_id_ref, accepted_match_idx,
        accepted_main_track_id_matches_map);
  }
  return common::kSuccess;
}

int SemantifyPlugin::calculateSemanticLandmarkCovariances() {
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapWriteAccess map =
      map_manager.getMapWriteAccess(selected_map_key);

  vi_map::MissionIdList mission_ids;
  map->getAllMissionIds(&mission_ids);

  if (mission_ids.empty()) {
    VLOG(1) << "No missions found!";
    return common::kStupidUserError;
  }

  vi_map::SemanticLandmarkIdList vis_semantic_ids;
  // compute covariance for good quality landmarks
  for (const vi_map::MissionId& mission_id : mission_ids) {
    vi_map::SemanticLandmarkIdList semantic_ids;
    map->getAllSemanticLandmarkIdsInMission(mission_id, &semantic_ids);
    for (const vi_map::SemanticLandmarkId& id : semantic_ids) {
      if (map->hasSemanticLandmark(id)) {
        vi_map::SemanticLandmark& landmark = map->getSemanticLandmark(id);
        if (landmark.getQuality() == vi_map::SemanticLandmark::Quality::kGood) {
          calculateSemanticLandmarkCovarianceFromTriangulationResidual(
              map.get(), id);
          vis_semantic_ids.push_back(id);
          // Eigen::Matrix3d covariance;
          // LOG(INFO)<<"semandic_id: "<<id.hexString();
          // LOG(INFO)<<landmark.get_p_B_Covariance(&covariance);
          // LOG(INFO)<<"covariance: "<<covariance;
        }
      }
    }
  }

  // visualization
  // TODO(jkuo): should merge this type into common-rviz-visualization
  if (FLAGS_semantify_visualize_semantic_landmark_covariance) {
    visualization_msgs::MarkerArray visual_variances;
    visual_variances.markers.resize(vis_semantic_ids.size());
    for (size_t i = 0; i < vis_semantic_ids.size(); ++i) {
      const vi_map::SemanticLandmarkId& id = vis_semantic_ids[i];
      const vi_map::SemanticLandmark& landmark = map->getSemanticLandmark(id);
      const Eigen::Vector3d G_p_landmark_pos =
          map->getSemanticLandmark_G_p_fi(id);
      Eigen::Matrix3d covariance;
      CHECK(landmark.get_p_B_Covariance(&covariance));
      visual_variances.markers[i].header.frame_id = FLAGS_tf_map_frame;
      visual_variances.markers[i].header.stamp = ros::Time::now();
      visual_variances.markers[i].ns = id.hexString();
      visual_variances.markers[i].id = 0;
      visual_variances.markers[i].action = visualization_msgs::Marker::ADD;
      visual_variances.markers[i].type = visualization_msgs::Marker::SPHERE;
      visual_variances.markers[i].pose.position.x = G_p_landmark_pos[0];
      visual_variances.markers[i].pose.position.y = G_p_landmark_pos[1];
      visual_variances.markers[i].pose.position.z = G_p_landmark_pos[2];
      visual_variances.markers[i].pose.orientation.x = 0.0;
      visual_variances.markers[i].pose.orientation.y = 0.0;
      visual_variances.markers[i].pose.orientation.z = 0.0;
      visual_variances.markers[i].pose.orientation.w = 1.0;
      CHECK_GT(covariance(0, 0), 0.0);
      CHECK_GT(covariance(1, 1), 0.0);
      CHECK_GT(covariance(2, 2), 0.0);
      visual_variances.markers[i].scale.x = sqrt(covariance(0, 0)) * 2;
      visual_variances.markers[i].scale.y = sqrt(covariance(1, 1)) * 2;
      visual_variances.markers[i].scale.z = sqrt(covariance(2, 2)) * 2;
      visual_variances.markers[i].color.a = 0.4;
      visual_variances.markers[i].color.r = 1.0;
      visual_variances.markers[i].color.g = 0.0;
      visual_variances.markers[i].color.b = 0.0;
    }
    semantic_landmark_covariance_pub_.publish(visual_variances);
  }

  return common::kSuccess;
}

int SemantifyPlugin::visualizeSemanticLoopClosureEdgeCovariances() {
  // Select map.
  std::string selected_map_key;
  if (!getSelectedMapKeyIfSet(&selected_map_key)) {
    return common::kStupidUserError;
  }
  vi_map::VIMapManager map_manager;
  vi_map::VIMapManager::MapReadAccess map =
      map_manager.getMapReadAccess(selected_map_key);

  vi_map::MissionIdList missions;
  map->getAllMissionIds(&missions);

  visualization_msgs::MarkerArray vis_lc_transformation_variances;
  for (const vi_map::MissionId& mission_id : missions) {
    // const pose_graph::EdgeIdList& edges =
    // map->getGraphTraversalEdgeType(mission_id);
    pose_graph::EdgeIdList edges;
    map->getAllEdgeIdsInMissionAlongGraph(mission_id, &edges);
    for (const pose_graph::EdgeId& edge_id : edges) {
      const pose_graph::Edge& edge = map->getEdgeAs<pose_graph::Edge>(edge_id);
      if (edge.getType() == pose_graph::Edge::EdgeType::kLoopClosure) {
        const vi_map::LoopClosureEdge& lc_edge =
            map->getEdgeAs<vi_map::LoopClosureEdge>(edge_id);
        const aslam::Transformation& from_T_G_I =
            map->getVertex_T_G_I(edge.from());
        const aslam::Transformation from_T_G_lc =
            from_T_G_I * lc_edge.get_T_A_B();
        const Eigen::Matrix<double, 6, 6>& lc_covariance =
            lc_edge.get_T_A_B_Covariance();
        const Eigen::Vector3d& G_edge_end_pos = from_T_G_lc.getPosition();
        const Eigen::Quaternion<double>& G_edge_end_ori =
            from_T_G_lc.getEigenQuaternion();
        // LOG(INFO) <<"lc_covariance: \n"
        //           <<lc_covariance;
        visualization_msgs::Marker vis_lc_T_A_B_cov;
        vis_lc_T_A_B_cov.header.frame_id = FLAGS_tf_map_frame;
        vis_lc_T_A_B_cov.header.stamp = ros::Time::now();
        vis_lc_T_A_B_cov.ns = edge.id().hexString();
        vis_lc_T_A_B_cov.id = 0;
        vis_lc_T_A_B_cov.action = visualization_msgs::Marker::ADD;
        vis_lc_T_A_B_cov.type = visualization_msgs::Marker::SPHERE;
        vis_lc_T_A_B_cov.pose.position.x = G_edge_end_pos[0];
        vis_lc_T_A_B_cov.pose.position.y = G_edge_end_pos[1];
        vis_lc_T_A_B_cov.pose.position.z = G_edge_end_pos[2];
        vis_lc_T_A_B_cov.pose.orientation.x = G_edge_end_ori.x();
        vis_lc_T_A_B_cov.pose.orientation.y = G_edge_end_ori.y();
        vis_lc_T_A_B_cov.pose.orientation.z = G_edge_end_ori.z();
        vis_lc_T_A_B_cov.pose.orientation.w = G_edge_end_ori.w();
        vis_lc_T_A_B_cov.scale.x = sqrt(lc_covariance(0, 0)) * 2;
        vis_lc_T_A_B_cov.scale.y = sqrt(lc_covariance(1, 1)) * 2;
        vis_lc_T_A_B_cov.scale.z = sqrt(lc_covariance(2, 2)) * 2;
        vis_lc_T_A_B_cov.color.a = 0.4;
        vis_lc_T_A_B_cov.color.r = 0.8;
        vis_lc_T_A_B_cov.color.g = 0.1;
        vis_lc_T_A_B_cov.color.b = 0.8;
        vis_lc_transformation_variances.markers.push_back(vis_lc_T_A_B_cov);
      }
    }
  }
  semantic_landmark_lc_covariance_pub_.publish(vis_lc_transformation_variances);
  LOG(INFO) << "Published # of loop closure edge covariances: "
            << vis_lc_transformation_variances.markers.size();
  return common::kSuccess;
}

// functions
void SemantifyPlugin::convertBoundingBoxesAndMasks(
    cv::Mat* masks, resources::ObjectInstanceBoundingBoxes* boxes,
    const std::vector<sensor_msgs::RegionOfInterest>& input_boxes,
    const std::vector<int>& input_ids,
    const std::vector<std::string>& input_names,
    const std::vector<float>& input_score,
    const std::vector<sensor_msgs::Image> input_masks) {
  // check the input data has the same length
  CHECK_EQ(input_boxes.size(), input_ids.size());
  CHECK_EQ(input_boxes.size(), input_names.size());
  CHECK_EQ(input_boxes.size(), input_score.size());
  CHECK_EQ(input_boxes.size(), input_masks.size());
  // create a map for instance counting <id, count> because detector does
  // not give instance label
  std::unordered_map<int, int> instance_map;
  // stores the score of the box of the index
  std::vector<std::pair<unsigned int, float>> idx_score_pair_list;

  // loop through boxes and append them
  resources::ObjectInstanceBoundingBox box;
  for (unsigned int i = 0; i < input_boxes.size(); ++i) {
    unsigned int x = input_boxes[i].x_offset;
    unsigned int y = input_boxes[i].y_offset;
    unsigned int dx = input_boxes[i].width;
    unsigned int dy = input_boxes[i].height;
    int inst_num = 0;
    int id = input_ids[i];
    std::string class_name = input_names[i];
    auto search = instance_map.find(id);
    if (search != instance_map.end()) {
      inst_num = instance_map[id];
      inst_num += 1;
      instance_map[id] = inst_num;
    } else {
      instance_map.insert({id, 1});
      inst_num = 1;
    }
    std::unique_ptr<resources::ObjectInstanceBoundingBox> box(
        new resources::ObjectInstanceBoundingBox());
    box->bounding_box = cv::Rect(x, y, dx, dy);
    box->class_number = id;
    box->class_name = class_name;
    box->instance_number = inst_num;
    box->confidence = input_score[i];
    boxes->push_back(*box);

    // stores the confidence score and its index
    std::pair<unsigned int, float> cur_pair(i, input_score[i]);
    idx_score_pair_list.push_back(cur_pair);
  }
  VLOG(5) << "# of converted boxes: " << boxes->size();
  // Order the masks by probability first, so overwriting is easier.
  std::sort(
      idx_score_pair_list.begin(), idx_score_pair_list.end(),
      semantify::sortBySec);
  // loop through masks, convert from ros message to cv::Mat, and then overlay
  // them based on probability score. the first channel is class id,
  // second is instance, third is unfilled
  // TODO(jkuo): maybe fill the thrid with score but need to
  // convert 0-1 to uint
  int row = input_masks[0].height;
  int col = input_masks[0].width;
  // Initialize the masks channels to be stored and temp for the ros image.
  cv::Mat mask_class(row, col, CV_8UC1, cv::Scalar(0));
  cv::Mat mask_instance(row, col, CV_8UC1, cv::Scalar(0));
  cv::Mat mask_score(row, col, CV_8UC1, cv::Scalar(0));
  cv_bridge::CvImageConstPtr mask_ptr;
  for (const std::pair<unsigned int, float>& iter : idx_score_pair_list) {
    sensor_msgs::Image test_image = input_masks[iter.first];
    try {
      sensor_msgs::ImageConstPtr mask_const_ptr(
          new sensor_msgs::Image(test_image));
      mask_ptr = cv_bridge::toCvCopy(
          mask_const_ptr, sensor_msgs::image_encodings::TYPE_8UC1);
    } catch (const cv_bridge::Exception& e) {  // NOLINT
      LOG(FATAL) << "cv_bridge exception: " << e.what();
    }
    CHECK(mask_ptr);
    cv::Mat mask = (mask_ptr->image) / 255;

    cv::Mat mask_inverse = (255 - mask_ptr->image) / 255;
    mask_class = mask_class.mul(mask_inverse) +
                 mask * static_cast<uint8_t>((*boxes)[iter.first].class_number);
    mask_instance =
        mask_instance.mul(mask_inverse) +
        mask * static_cast<uint8_t>((*boxes)[iter.first].instance_number);
  }
  cv::Mat matArray[] = {mask_class, mask_instance, mask_score};
  cv::merge(matArray, 3, *masks);
}

void SemantifyPlugin::convertResourceBoundingBoxesToDescriptorGenerationMessage(
    const resources::ObjectInstanceBoundingBoxes& res_boxes,
    std::vector<sensor_msgs::RegionOfInterest>* sensor_boxes,
    std::vector<int>* class_numbers, std::vector<int>* instance_numbers) {
  CHECK(sensor_boxes);
  CHECK(class_numbers);
  CHECK(instance_numbers);

  for (const resources::ObjectInstanceBoundingBox& res_box : res_boxes) {
    sensor_msgs::RegionOfInterest sensor_box;
    sensor_box.x_offset = res_box.bounding_box.x;
    sensor_box.y_offset = res_box.bounding_box.y;
    sensor_box.width = res_box.bounding_box.width;
    sensor_box.height = res_box.bounding_box.height;
    sensor_boxes->push_back(sensor_box);
    class_numbers->push_back(res_box.class_number);
    instance_numbers->push_back(res_box.instance_number);
  }
}

void SemantifyPlugin::convertSemanticObjectMeasurementsToResourceBoundingBoxes(
    const Eigen::Matrix4Xd measurements, const Eigen::VectorXd confidences,
    const Eigen::VectorXi class_ids,
    resources::ObjectInstanceBoundingBoxes* res_boxes) {
  CHECK(res_boxes);
  CHECK_GT(measurements.cols(), 0u);
  CHECK_EQ(measurements.cols(), confidences.size());
  CHECK_EQ(measurements.cols(), class_ids.size());
  for (int i = 0; i < measurements.cols(); i++) {
    // convert centroid to top left corner point
    double dx = static_cast<double>(measurements(2, i));
    double dy = static_cast<double>(measurements(3, i));
    double x = static_cast<double>(measurements(0, i) - measurements(2, i) / 2);
    double y = static_cast<double>(measurements(1, i) - measurements(3, i) / 2);

    resources::ObjectInstanceBoundingBox box;
    box.bounding_box = cv::Rect(x, y, dx, dy);
    box.confidence = confidences(i);
    box.class_number = class_ids(i);
    res_boxes->push_back(box);
  }
  CHECK_EQ(measurements.cols(), res_boxes->size());
}

void SemantifyPlugin::populateDeepSortServiceRequest(
    aslam::VisualFrame::Ptr visual_frame,
    deep_sort_ros::deep_sort_result* deep_sort_srv) {
  CHECK(visual_frame);
  CHECK(deep_sort_srv);
  CHECK_GT(visual_frame->getSemanticObjectMeasurements().cols(), 0u);
  CHECK_EQ(
      visual_frame->getSemanticObjectMeasurements().cols(),
      visual_frame->getSemanticObjectMeasurementUncertainties().size());
  CHECK_EQ(
      visual_frame->getSemanticObjectMeasurements().cols(),
      visual_frame->getSemanticObjectClassIds().size());
  CHECK_EQ(
      visual_frame->getSemanticObjectMeasurements().cols(),
      visual_frame->getSemanticObjectDescriptors().cols());

  for (int i = 0; i < visual_frame->getSemanticObjectMeasurements().cols();
       i++) {
    sensor_msgs::RegionOfInterest sensor_box;
    sensor_box.width =
        static_cast<uint32_t>(visual_frame->getSemanticObjectMeasurement(i)(2));
    sensor_box.height =
        static_cast<uint32_t>(visual_frame->getSemanticObjectMeasurement(i)(3));
    sensor_box.x_offset = static_cast<uint32_t>(
        visual_frame->getSemanticObjectMeasurement(i)(0) -
        sensor_box.width / 2);
    sensor_box.y_offset = static_cast<uint32_t>(
        visual_frame->getSemanticObjectMeasurement(i)(1) -
        sensor_box.height / 2);
    deep_sort_srv->request.boxes.push_back(sensor_box);
    deep_sort_srv->request.confidences.push_back(
        visual_frame->getSemanticObjectMeasurementUncertainty(i));
    // TODO(jkuo): could use std::vector allocator if this is too slow
    netvlad_tf::Descriptor netvlad_descriptor;
    const Eigen::VectorXf& sem_descriptor =
        visual_frame->getSemanticObjectDescriptor(i);
    for (int j = 0; j < sem_descriptor.rows(); j++) {
      netvlad_descriptor.descriptor.push_back(sem_descriptor(j));
    }
    deep_sort_srv->request.descriptors.push_back(netvlad_descriptor);
  }
}

void SemantifyPlugin::saveSemanticObjectMeasurementsToVisualFrame(
    aslam::VisualFrame::Ptr visual_frame,
    const resources::ObjectInstanceBoundingBoxes& boxes,
    std::vector<::netvlad_tf::Descriptor>& descriptors) {
  CHECK_NOTNULL(visual_frame);
  CHECK_EQ(boxes.size(), descriptors.size());
  CHECK_GT(boxes.size(), 0u);
  CHECK(visual_frame->hasSemanticObjectMeasurements());
  CHECK(visual_frame->hasSemanticObjectMeasurementUncertainties());
  CHECK(visual_frame->hasSemanticObjectDescriptors());
  CHECK(visual_frame->hasSemanticObjectClassIds());
  // instantiate containter
  Eigen::Matrix4Xd measurements(4, boxes.size());
  Eigen::VectorXd uncertainties(boxes.size());
  Eigen::VectorXi class_ids(boxes.size());
  aslam::VisualFrame::SemanticObjectDescriptorsT temp_descriptors(
      descriptors[0].descriptor.size(), boxes.size());
  // the boxes and descriptors are aligned by index
  for (size_t i = 0; i < boxes.size(); i++) {
    // visual frame stores centroid_col, centroid_row, width, and height,
    // but boxes are (x,y,dx,dy) x and y are corners of top left corner
    const resources::ObjectInstanceBoundingBox& box = boxes[i];
    measurements(0, i) = static_cast<double>(box.bounding_box.x) +
                         static_cast<double>(box.bounding_box.width) / 2;
    measurements(1, i) = static_cast<double>(box.bounding_box.y) +
                         static_cast<double>(box.bounding_box.height) / 2;
    measurements(2, i) = static_cast<double>(box.bounding_box.width);
    measurements(3, i) = static_cast<double>(box.bounding_box.height);
    uncertainties(i) = box.confidence;
    class_ids(i) = box.class_number;
    float* vec_ptr = &((descriptors[i].descriptor)[0]);
    Eigen::Map<Eigen::VectorXf> descriptor(
        vec_ptr, descriptors[i].descriptor.size());
    temp_descriptors.col(i) = descriptor;
  }
  visual_frame->swapSemanticObjectMeasurements(&measurements);
  visual_frame->swapSemanticObjectMeasurementUncertainties(&uncertainties);
  visual_frame->swapSemanticObjectClassIds(&class_ids);
  visual_frame->swapSemanticObjectDescriptors(&temp_descriptors);
}

void SemantifyPlugin::insertSemanticObjectTrackIds(
    aslam::VisualFrame::Ptr const visual_frame,
    Eigen::VectorXi& track_ids) const {
  visual_frame->swapSemanticObjectTrackIds(&track_ids);
}
void SemantifyPlugin::parseIdsFromString(
    const std::string input, std::vector<std::string>* parsed_ids) {
  auto it = std::begin(input);
  while (true) {
    auto commaPosition = std::find(it, std::end(input), ',');
    parsed_ids->emplace_back(it, commaPosition);
    if (commaPosition == std::end(input)) {
      break;
    }
    it = std::next(commaPosition);
  }
}

void SemantifyPlugin::getMedoidsAndClassIdsOfTrackIdsFromMissionId(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    std::map<int, Eigen::VectorXf>* track_id_to_medoid_descriptor,
    std::map<int, int>* track_id_to_class_id) {
  CHECK(mission_id.isValid());
  CHECK(track_id_to_medoid_descriptor);
  CHECK(track_id_to_class_id);
  const TrackIdsToSemanticLandmarkIdsMap& track_ids_to_semantic_landmark_ids =
      mission_id_to_semantic_landmark_id_[mission_id];
  for (const auto& pair : track_ids_to_semantic_landmark_ids) {
    int track_id = pair.first;
    // get medoids
    KMedoidsClusteringManager::KMedoidsIndexMap k_index_map =
        mission_to_kmedoids_manager_[mission_id]->getKMedoidsIndiciesForTrackId(
            track_id);
    const std::vector<clustering::object_id>& medoids_indicies = k_index_map[1];
    size_t medoid_index = medoids_indicies[0];
    const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors =
        mission_to_kmedoids_manager_[mission_id]->getKMedoidsDescriptors(
            track_id);
    Eigen::VectorXf medoid_descriptor = descriptors.col(medoid_index);
    (*track_id_to_medoid_descriptor)[track_id] = medoid_descriptor;
    // get class ids
    const vi_map::SemanticLandmark& semantic_landmark =
        map.getSemanticLandmark(pair.second);
    (*track_id_to_class_id)[track_id] = semantic_landmark.getClassId();
  }
}

void SemantifyPlugin::buildCostMatrix(
    const std::map<int, Eigen::VectorXf>& source_track_id_to_medoid_descriptor,
    const std::map<int, Eigen::VectorXf>& ref_track_id_to_medoid_descriptor,
    const std::map<int, int>& source_track_id_to_class_id,
    const std::map<int, int>& ref_track_id_to_class_id,
    const bool set_diagonal_to_max_cost,
    std::vector<std::vector<float>>* cost_matrix) {
  CHECK_EQ(
      source_track_id_to_medoid_descriptor.size(),
      source_track_id_to_class_id.size());
  CHECK_EQ(
      ref_track_id_to_medoid_descriptor.size(),
      ref_track_id_to_class_id.size());
  CHECK(cost_matrix);
  if (set_diagonal_to_max_cost) {
    CHECK_EQ(
        source_track_id_to_medoid_descriptor.size(),
        ref_track_id_to_medoid_descriptor.size())
        << "the flag to set diagonal elements is true, but the matrix "
           "dimension is not square.";
  }

  for (const auto& pair_i : source_track_id_to_medoid_descriptor) {
    std::vector<float> i_to_j_cost;
    for (const auto& pair_j : ref_track_id_to_medoid_descriptor) {
      // same landmark
      if (set_diagonal_to_max_cost && pair_i.first == pair_j.first) {
        i_to_j_cost.push_back(std::numeric_limits<float>::max());
      } else {
        // different landmarks, but different class
        if (FLAGS_semantify_semantic_landmark_matching_filter_by_class) {
          const int class_i = source_track_id_to_class_id.at(pair_i.first);
          const int class_j = ref_track_id_to_class_id.at(pair_j.first);
          if (class_i != class_j) {
            i_to_j_cost.push_back(std::numeric_limits<float>::max());
          } else {
            i_to_j_cost.push_back((pair_i.second - pair_j.second).norm());
          }
        } else {
          i_to_j_cost.push_back((pair_i.second - pair_j.second).norm());
        }
      }
    }
    cost_matrix->emplace_back(i_to_j_cost);
  }
}

void SemantifyPlugin::getKNearestAssignments(
    const std::vector<std::vector<float>>& cost_matrix,
    const std::vector<int>& index_to_source_track_id,
    const std::vector<int>& index_to_ref_track_id,
    SourceTrackIdToMatchedRefTrackIdsMap* source_track_id_to_matched_ref_ids) {
  CHECK_GT(cost_matrix.size(), 0u);
  CHECK_EQ(cost_matrix.size(), index_to_source_track_id.size());
  CHECK_EQ(cost_matrix[0].size(), index_to_ref_track_id.size());
  CHECK(source_track_id_to_matched_ref_ids);
  for (size_t i = 0; i < cost_matrix.size(); ++i) {
    std::priority_queue<
        std::pair<float, int>, std::vector<std::pair<float, int>>,
        std::greater<std::pair<float, int>>>
        q;
    const std::vector<float>& cost_row_i = cost_matrix[i];
    for (int j = 0; j < cost_row_i.size(); ++j) {
      q.push(std::pair<float, int>(cost_row_i[j], j));
    }
    // saves the track ids of the k nearest match
    int current_id = index_to_source_track_id[i];
    // LOG(INFO)<<"current_id: "<<current_id;
    // TODO(jkuo): meant for vector, remove if we end up sticking with set;
    // (*source_track_id_to_matched_ref_ids)[current_id].reserve(q.size());
    for (size_t j = 0;
         j < FLAGS_semantify_semantic_landmark_matching_candidates_num; ++j) {
      int matched_id = index_to_ref_track_id[q.top().second];
      if (q.top().first <
          FLAGS_semantify_semantic_landmark_matching_filter_by_max_descriptor_difference) {
        // TODO(jkuo): originally had the matched ids pushed back in a vector,
        // so that the order in priority queue is kept, but it seems like the
        // specific order isn't really used. I am changing it to set for ease of
        // removing in the filter stage. if in the future, you wish to keep
        // order, maybe create a stuct and specific comparison function for the
        // set.
        (*source_track_id_to_matched_ref_ids)[current_id].insert(matched_id);
        // LOG(INFO)<<"cost: "<<q.top().first <<" matched_id: "<<matched_id;
      }
      q.pop();
    }
  }
}

void SemantifyPlugin::generateMatchCandidates(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
    const vi_map::MissionId& mission_id_ref,
    const bool set_diagonal_to_max_cost,
    SourceTrackIdToMatchedRefTrackIdsMap*
        source_track_id_to_matched_ref_ids_candidates) {
  CHECK(source_track_id_to_matched_ref_ids_candidates);
  CHECK(source_track_id_to_matched_ref_ids_candidates->empty());
  // gets the medoids and class ids of the track_ids
  std::map<int, Eigen::VectorXf> source_track_id_to_medoid_descriptor;
  std::map<int, int> source_track_id_to_class_id;
  getMedoidsAndClassIdsOfTrackIdsFromMissionId(
      map, mission_id_source, &source_track_id_to_medoid_descriptor,
      &source_track_id_to_class_id);

  std::map<int, Eigen::VectorXf> ref_track_id_to_medoid_descriptor;
  std::map<int, int> ref_track_id_to_class_id;
  getMedoidsAndClassIdsOfTrackIdsFromMissionId(
      map, mission_id_ref, &ref_track_id_to_medoid_descriptor,
      &ref_track_id_to_class_id);
  // compute cost matrix
  std::vector<std::vector<float>> cost_matrix;
  buildCostMatrix(
      source_track_id_to_medoid_descriptor, ref_track_id_to_medoid_descriptor,
      source_track_id_to_class_id, ref_track_id_to_class_id,
      set_diagonal_to_max_cost, &cost_matrix);

  // creates the index and track id correspondences
  std::vector<int> index_to_ref_track_id, index_to_source_track_id;
  index_to_ref_track_id.reserve(ref_track_id_to_medoid_descriptor.size());
  index_to_source_track_id.reserve(source_track_id_to_medoid_descriptor.size());
  for (const auto& pair : ref_track_id_to_medoid_descriptor) {
    index_to_ref_track_id.push_back(pair.first);
  }
  for (const auto& pair : source_track_id_to_medoid_descriptor) {
    index_to_source_track_id.push_back(pair.first);
  }

  // finds k nearest assignments
  getKNearestAssignments(
      cost_matrix, index_to_source_track_id, index_to_ref_track_id,
      source_track_id_to_matched_ref_ids_candidates);

  // removes match candidates that are too far away
  if (FLAGS_semantify_semantic_landmark_matching_filter_by_match_candidate_distance) {
    const TrackIdsToSemanticLandmarkIdsMap&
        track_ids_to_semantic_landmark_ids_source =
            mission_id_to_semantic_landmark_id_[mission_id_source];
    const TrackIdsToSemanticLandmarkIdsMap&
        track_ids_to_semantic_landmark_ids_ref =
            mission_id_to_semantic_landmark_id_[mission_id_ref];
    for (auto it = source_track_id_to_matched_ref_ids_candidates->begin();
         it != source_track_id_to_matched_ref_ids_candidates->end();) {
      // need to copy the values and delete from original data else it may
      // result in garbage value in the data structure
      std::set<int> matched_ids_ref = it->second;
      for (auto matched_id : matched_ids_ref) {
        const vi_map::SemanticLandmarkId& lm_id_source =
            track_ids_to_semantic_landmark_ids_source.at(it->first);
        const vi_map::SemanticLandmarkId& lm_id_ref =
            track_ids_to_semantic_landmark_ids_ref.at(matched_id);
        const Eigen::Vector3d& p_G_landmark_source =
            map.getSemanticLandmark_G_p_fi(lm_id_source);
        const Eigen::Vector3d& p_G_landmark_ref =
            map.getSemanticLandmark_G_p_fi(lm_id_ref);
        double match_distance = (p_G_landmark_source - p_G_landmark_ref).norm();

        if (match_distance >=
            FLAGS_semantify_semantic_landmark_max_match_candidate_distance) {
          it->second.erase(matched_id);
        }
      }
      // remove if match is empty
      if (it->second.empty()) {
        it = source_track_id_to_matched_ref_ids_candidates->erase(it);
      } else {
        ++it;
      }
    }
  }
}

void SemantifyPlugin::visualizeMatchedTrackIds(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
    const vi_map::MissionId& mission_id_ref,
    const SourceTrackIdToMatchedRefTrackIdsMap& track_id_to_matched_ids,
    const visualization::Color& color, const std::string& topic,
    const std::string& name_space) {
  // TODO(jkuo):
  // Can't publish individual lines as multiple landmark_pairs because the time
  // would be the same and cause markers to be overwritten, even with differen
  // namespace... take a look at this later if have time.
  visualization::LineSegmentVector landmark_pairs;
  for (const auto& pair : track_id_to_matched_ids) {
    for (const int& matched_id : pair.second) {
      const vi_map::SemanticLandmarkId& id_1 =
          mission_id_to_semantic_landmark_id_[mission_id_source].at(pair.first);
      const vi_map::SemanticLandmarkId& id_2 =
          mission_id_to_semantic_landmark_id_[mission_id_ref].at(matched_id);
      const Eigen::Vector3d& p_G_landmark_1 =
          map.getSemanticLandmark_G_p_fi(id_1);
      const Eigen::Vector3d& p_G_landmark_2 =
          map.getSemanticLandmark_G_p_fi(id_2);

      visualization::LineSegment landmark_connection;
      landmark_connection.alpha = 1.0;
      landmark_connection.color = color;
      landmark_connection.scale = 0.02;
      landmark_connection.from = p_G_landmark_1;
      landmark_connection.to = p_G_landmark_2;

      landmark_pairs.push_back(landmark_connection);
    }
  }
  std::string kSemanticLCMarkerNameSpace;
  if (name_space == "default") {
    kSemanticLCMarkerNameSpace =
        mission_id_source.hexString() + "_to_" + mission_id_ref.hexString();
  } else {
    kSemanticLCMarkerNameSpace = name_space;
  }
  visualization::publishLines(
      landmark_pairs, 0, FLAGS_tf_map_frame, kSemanticLCMarkerNameSpace, topic);
}

void SemantifyPlugin::visualizeCovisibleTrackIds(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const int& track_id_source,
    const std::set<int>& track_id_covisible_track_ids,
    const visualization::Color& color, const std::string& topic) {
  visualization::LineSegmentVector landmark_pairs;
  for (const int& covisible_id : track_id_covisible_track_ids) {
    const vi_map::SemanticLandmarkId& id_1 =
        mission_id_to_semantic_landmark_id_[mission_id].at(track_id_source);
    const vi_map::SemanticLandmarkId& id_2 =
        mission_id_to_semantic_landmark_id_[mission_id].at(covisible_id);
    const Eigen::Vector3d& p_G_landmark_1 =
        map.getSemanticLandmark_G_p_fi(id_1);
    const Eigen::Vector3d& p_G_landmark_2 =
        map.getSemanticLandmark_G_p_fi(id_2);

    visualization::LineSegment landmark_connection;
    landmark_connection.alpha = 1.0;
    landmark_connection.color = color;
    landmark_connection.scale = 0.02;
    landmark_connection.from = p_G_landmark_1;
    landmark_connection.to = p_G_landmark_2;

    landmark_pairs.push_back(landmark_connection);
  }

  std::string kSemanticLCMarkerNameSpace = std::to_string(track_id_source);
  visualization::publishLines(
      landmark_pairs, 0, FLAGS_tf_map_frame, kSemanticLCMarkerNameSpace, topic);
}

void SemantifyPlugin::visualizeSemanticLandmarksObservableVerticies(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const int& track_id, const pose_graph::VertexIdList& observable_verticies,
    const visualization::Color& color, const std::string& topic) {
  visualization::LineSegmentVector landmark_pairs;
  const vi_map::SemanticLandmarkId& landmark_id =
      mission_id_to_semantic_landmark_id_[mission_id].at(track_id);
  for (const auto& vertex_id : observable_verticies) {
    const Eigen::Vector3d& p_G_landmark =
        map.getSemanticLandmark_G_p_fi(landmark_id);
    const Eigen::Vector3d& p_G_vertex = map.getVertex_G_p_I(vertex_id);
    visualization::LineSegment landmark_connection;
    landmark_connection.alpha = 1.0;
    landmark_connection.color = color;
    landmark_connection.scale = 0.02;
    landmark_connection.from = p_G_landmark;
    landmark_connection.to = p_G_vertex;

    landmark_pairs.push_back(landmark_connection);
  }
  std::string kSemanticLCMarkerNameSpace =
      mission_id.hexString() + "/" + std::to_string(track_id);
  visualization::publishLines(
      landmark_pairs, 0, FLAGS_tf_map_frame, kSemanticLCMarkerNameSpace, topic);
}

void SemantifyPlugin::visualizeSemanticLoopClosureEdge(
    const vi_map::VIMap& map, const pose_graph::VertexId& from_vertex,
    const pose_graph::VertexId& to_vertex, const visualization::Color& color,
    const std::string& topic, const std::string& name_space) {
  visualization::LineSegmentVector vertex_pair;
  const Eigen::Vector3d& p_G_from_vertex =
      map.getVertex_T_G_I(from_vertex).getPosition();
  const Eigen::Vector3d& p_G_to_vertex =
      map.getVertex_T_G_I(to_vertex).getPosition();
  visualization::LineSegment vertex_connection;
  vertex_connection.alpha = 1.0;
  vertex_connection.color = color;
  vertex_connection.scale = 0.02;
  vertex_connection.from = p_G_from_vertex;
  vertex_connection.to = p_G_to_vertex;
  vertex_pair.push_back(vertex_connection);
  std::string kSemanticLCEdgeMarkerNameSpace = name_space;
  visualization::publishLines(
      vertex_pair, 0, FLAGS_tf_map_frame, kSemanticLCEdgeMarkerNameSpace,
      topic);
  // publish the vertices
  Eigen::Matrix3Xd points;
  points.resize(Eigen::NoChange, 2u);
  points.col(0) = map.getVertex_T_G_I(from_vertex).getPosition();
  points.col(1) = map.getVertex_T_G_I(to_vertex).getPosition();
  std::string lc_edge_vertices_topic = "lc_edge_vertex";
  publish3DPointsAsSpheres(
      points, color, 1.0, 0.1, 0, FLAGS_tf_map_frame, name_space,
      lc_edge_vertices_topic);
}

bool SemantifyPlugin::visualizeSemanticLandmarkInfo(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const int& track_id, const vi_map::SemanticLandmarkId& id,
    const std::unordered_map<int, size_t>& class_count_map) {
  const vi_map::SemanticLandmark& semantic_landmark =
      map.getSemanticLandmark(id);
  if (class_count_map.size() == 0u) {
    LOG(ERROR) << "Please run the command update_semantic_landmarks_class_ids.";
    return false;
  }
  Eigen::Vector3d G_p_landmark_pos = map.getSemanticLandmark_G_p_fi(id);
  std::string lm_quality;
  if (semantic_landmark.getQuality() ==
      vi_map::SemanticLandmark::Quality::kGood) {
    lm_quality = "Good";
  }
  int class_id = semantic_landmark.getClassId();
  // create text marker
  visualization_msgs::Marker text_marker;
  text_marker.header.frame_id = "map";
  text_marker.header.stamp = ros::Time();
  text_marker.ns = mission_id.hexString();
  // TODO(jkuo): figure out a way to assign id for text markers
  text_marker.id = track_id;
  text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  text_marker.action = visualization_msgs::Marker::ADD;
  text_marker.pose.position.x = G_p_landmark_pos(0);
  text_marker.pose.position.y = G_p_landmark_pos(1);
  text_marker.pose.position.z =
      G_p_landmark_pos(2) + 0.2;  //+ sphere_marker.scale.z/2.0 + 0.3;
  text_marker.pose.orientation.w = 1;
  text_marker.scale.x = 0.1;
  text_marker.scale.y = 0.1;
  text_marker.scale.z = 0.1;
  if (class_id == 73) {
    // refrigerator
    text_marker.color.r = 1;
    text_marker.color.g = 0;
    text_marker.color.b = 0;
  } else if (class_id == 57) {
    // chair
    text_marker.color.r = 1;
    text_marker.color.g = 1;
    text_marker.color.b = 0;
  } else if (class_id == 61) {
    // table
    text_marker.color.r = 0;
    text_marker.color.g = 1;
    text_marker.color.b = 0;
  } else {
    // others
    text_marker.color.r = 1;
    text_marker.color.g = 1;
    text_marker.color.b = 1;
  }
  text_marker.color.a = 1;
  // fills the text
  text_marker.text += "Track ID: " + std::to_string(track_id) +
                      " Class name: " + mask_rcnn_class_names[class_id];
  text_marker.text += "\n Quality: " + lm_quality;
  // LOG(INFO)<<"Mission ID: "<<mission_id.hexString();
  // LOG(INFO)<<"Semantic Landmark ID: "<<semantic_landmark.id().hexString();
  // LOG(INFO)<<"Track ID: "<< track_id
  //          <<" Class name: " << mask_rcnn_class_names[class_id]
  //          <<" Quality: " << lm_quality;
  // LOG(INFO)<<"Contains:";
  for (auto const& pair : class_count_map) {
    text_marker.text += "\nClass name: " + mask_rcnn_class_names[pair.first] +
                        " occurence: " + std::to_string(pair.second);
    // LOG(INFO)<<"Class name: "<<mask_rcnn_class_names[pair.first]
    //          <<" occurence: "<<std::to_string(pair.second);
  }
  semantic_landmark_pub_.publish(text_marker);
  return true;
}

bool SemantifyPlugin::separateMissionsWithKnownBaseFrame(
    const vi_map::VIMap& map, vi_map::MissionIdList* ref_mission_ids,
    vi_map::MissionIdList* source_mission_ids) {
  CHECK(ref_mission_ids);
  CHECK(source_mission_ids);
  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);
  for (const vi_map::MissionId& map_mission_id : all_mission_ids) {
    const vi_map::VIMission& map_mission = map.getMission(map_mission_id);
    if (map.getMissionBaseFrame(map_mission.getBaseFrameId())
            .is_T_G_M_known()) {
      ref_mission_ids->emplace_back(map_mission_id);
    } else {
      source_mission_ids->emplace_back(map_mission_id);
    }
  }

  if (ref_mission_ids->empty()) {
    LOG(ERROR) << "At least one mission should have a known T_G_M baseframe "
               << "transformation.";
    return false;
  }

  if (source_mission_ids->empty()) {
    LOG(ERROR) << "All missions have known T_G_M. There is nothing to anchor";
    return false;
  }

  return true;
}

void SemantifyPlugin::computeICPPointsToPointsTransformation(
    const Eigen::Matrix3Xd& X, const Eigen::Matrix3Xd& Y,
    Eigen::Matrix4d* T_result) {
  // follows the notation of https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
  CHECK_EQ(X.cols(), Y.cols());
  Eigen::Vector3d X_mean = X.rowwise().mean();
  Eigen::Vector3d Y_mean = Y.rowwise().mean();
  Eigen::Matrix3Xd X_centered = X;
  Eigen::Matrix3Xd Y_centered = Y;
  X_centered.colwise() -= X_mean;
  Y_centered.colwise() -= Y_mean;

  // we set W to identity so we omit it
  Eigen::Matrix3d S = X_centered * Y_centered.transpose();
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      S, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d reflection_handler = Eigen::Matrix3d::Identity();
  reflection_handler(2, 2) =
      (svd.matrixV() * svd.matrixU().transpose()).determinant();
  Eigen::Matrix4d T_ref_source_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix3d R =
      svd.matrixV() * reflection_handler * svd.matrixU().transpose();
  T_ref_source_matrix.block<3, 3>(0, 0) = R;
  T_ref_source_matrix.block<3, 1>(0, 3) = Y_mean - R * X_mean;

  *T_result = T_ref_source_matrix;
}

bool SemantifyPlugin::generateTransformationHypothesisFromMatches(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
    const vi_map::MissionId& mission_id_ref,
    const SourceTrackIdToMatchedRefTrackIdsMap&
        source_track_id_to_matched_ref_ids,
    const SourceTrackIdToMatchedRefTrackIdsMap& predefined_inliers,
    CandidateIdToTransformationMap* G_T_ref_source_ransac_candidates,
    CandidateIdToTrackIdMatchesMap*
        source_track_id_to_matched_ref_ids_ransac_candidates,
    MatchCandidateScores* match_candidate_scores) {
  CHECK(G_T_ref_source_ransac_candidates);
  CHECK(source_track_id_to_matched_ref_ids_ransac_candidates);
  CHECK(match_candidate_scores);
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_source =
          mission_id_to_semantic_landmark_id_[mission_id_source];
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_ref =
          mission_id_to_semantic_landmark_id_[mission_id_ref];

  if (source_track_id_to_matched_ref_ids.size() < 4) {
    LOG(ERROR) << "the number of source_track_id_to_matched_ref_ids "
               << "is not sufficient for transformation calculation: "
               << source_track_id_to_matched_ref_ids.size();
    return false;
  }
  // TODO(jkuo): should refactor this into a flatten function
  // flatten the map into two vectors that are aligned by index
  std::vector<int> source_track_ids, ref_track_ids;
  std::vector<int> index_vector;
  int i = 0;
  for (const auto& pair : source_track_id_to_matched_ref_ids) {
    if (pair.second.empty()) {
      continue;
    } else {
      for (const int& matched_id : pair.second) {
        source_track_ids.push_back(pair.first);
        ref_track_ids.push_back(matched_id);
        index_vector.push_back(i);
        // debugging
        // LOG(INFO)<<"index: "<<i
        //          <<" source: "<<pair.first
        //          <<" ref: "<<matched_id;
        i++;
      }
    }
  }
  CHECK_EQ(source_track_ids.size(), ref_track_ids.size());
  CHECK_EQ(source_track_ids.size(), index_vector.size());
  // LOG(INFO)<<"index_vector.size(): "<<index_vector.size();

  // finds the predefined inlier index and add to inlier collection
  std::vector<int> predefined_inlier_source_track_ids,
      predefined_inlier_ref_track_ids;
  for (const auto& pair : predefined_inliers) {
    if (pair.second.empty()) {
      continue;
    } else {
      for (const int& matched_id : pair.second) {
        predefined_inlier_source_track_ids.push_back(pair.first);
        predefined_inlier_ref_track_ids.push_back(matched_id);
      }
    }
  }
  CHECK_EQ(
      predefined_inlier_source_track_ids.size(),
      predefined_inlier_ref_track_ids.size());
  // LOG(INFO) <<"predefined_inlier_source_track_ids.size():
  // "<<predefined_inlier_source_track_ids.size(); LOG(INFO)
  // <<"predefined_inlier_source_track_id:
  // "<<predefined_inlier_source_track_ids[0]; LOG(INFO)
  // <<"predefined_inlier_ref_track_id: "<<predefined_inlier_ref_track_ids[0];

  // TODO(jkuo): May not be the optmial way to search, but JUST DO IT first
  // searches the mapping index of the predefined inlier
  std::set<int> predefined_inlier_index;
  for (size_t in_idx = 0; in_idx < predefined_inlier_source_track_ids.size();
       in_idx++) {
    int predef_source_track_id = predefined_inlier_source_track_ids[in_idx];
    int predef_ref_track_id = predefined_inlier_ref_track_ids[in_idx];
    for (const auto& map_idx : index_vector) {
      if (predef_source_track_id == source_track_ids[map_idx] &&
          predef_ref_track_id == ref_track_ids[map_idx]) {
        predefined_inlier_index.insert(map_idx);
      }
    }
  }

  // using set to filter match candidates because random can generate duplicate.
  // since we flatten the SourceTrackIdToMatchedRefTrackIdsMap and they are
  // labelled by a unique index number, we can maintain uniqueness of the
  // inlier set of correspondences using set
  std::set<std::set<int>> filtered_candidate_matches;

  // RANSAC for generating transformation hypothesis
  random_selector<> selector{};
  size_t sample_num = index_vector.size();
  size_t iter_num =
      FLAGS_semantify_semantic_landmark_anchoring_ransac_max_iteration;
  // size_t iter_num = 1;
  size_t best_inlier_num = 0;
  double best_inlier_ratio = 0.0;
  double best_total_inlier_error = 0.0;
  CandidateId candidate_id = 0;
  for (size_t j = 0; j < iter_num; j++) {
    std::vector<int> index_vector_temp = index_vector;
    SourceTrackIdToMatchedRefTrackIdsMap inlier_id_pairs;
    // tracks the indicies of the inlier correspondence
    std::set<int> inlier_index;
    std::set<int> initial_index;
    if (predefined_inlier_index.size() > 0u) {
      inlier_index = predefined_inlier_index;
      initial_index = predefined_inlier_index;
    }

    // selected initial 3 points for the model
    while (inlier_index.size() < 3) {
      auto iter_selected =
          selector(index_vector_temp.begin(), index_vector_temp.end());
      auto search = inlier_index.find(*iter_selected);
      // does not find in set
      if (search == inlier_index.end()) {
        inlier_index.insert(*iter_selected);
        initial_index.insert(*iter_selected);
      }
    }

    // col is aligned with vector index
    Eigen::Matrix3d X, Y;
    int k = 0;
    for (const auto& index : inlier_index) {
      const int source_track_id = source_track_ids[index];
      const int ref_track_id = ref_track_ids[index];
      const vi_map::SemanticLandmarkId& source_id =
          track_ids_to_semantic_landmark_ids_source.at(source_track_id);
      const vi_map::SemanticLandmarkId& ref_id =
          track_ids_to_semantic_landmark_ids_ref.at(ref_track_id);
      X.col(k) = map.getSemanticLandmark_G_p_fi(source_id);
      Y.col(k) = map.getSemanticLandmark_G_p_fi(ref_id);
      inlier_id_pairs[source_track_id].insert(ref_track_id);
      k++;
    }

    Eigen::Matrix4d G_T_ref_source_matrix;
    computeICPPointsToPointsTransformation(X, Y, &G_T_ref_source_matrix);
    Eigen::Matrix<double, 6, 6> ICP_Covariance;
    pose::Transformation G_T_ref_source_tmp(G_T_ref_source_matrix);

    size_t inlier_num = inlier_index.size();
    double total_inlier_error = 0.0;
    double total_model_error = 0.0;
    for (const auto& index : index_vector_temp) {
      auto search = inlier_index.find(index);
      if (search != inlier_index.end()) {
        continue;
      }
      const int source_track_id = source_track_ids[index];
      const int ref_track_id = ref_track_ids[index];
      const vi_map::SemanticLandmarkId& source_id =
          track_ids_to_semantic_landmark_ids_source.at(source_track_id);
      const vi_map::SemanticLandmarkId& ref_id =
          track_ids_to_semantic_landmark_ids_ref.at(ref_track_id);
      Eigen::Vector3d x = map.getSemanticLandmark_G_p_fi(source_id);
      Eigen::Vector3d y = map.getSemanticLandmark_G_p_fi(ref_id);
      double error = (y - G_T_ref_source_tmp * x).norm();
      // for debugging
      // LOG(INFO)<<"index: "<<index <<" calculated error: "<<error;
      total_model_error += error;
      if (error <
          FLAGS_semantify_semantic_landmark_anchoring_ransac_max_inlier_distance) {
        inlier_id_pairs[source_track_id].insert(ref_track_id);
        total_inlier_error += error;
        inlier_num++;
        inlier_index.insert(index);
      }
    }
    CHECK_EQ(inlier_index.size(), inlier_num);

    auto result = filtered_candidate_matches.insert(inlier_index);
    double inlier_ratio =
        static_cast<double>(inlier_num) / static_cast<double>(sample_num);
    // saves the candidates if pass criteria and no duplicate
    if (inlier_num >=
            FLAGS_semantify_semantic_landmark_anchoring_ransac_min_inlier_num &&
        result.second &&
        inlier_ratio >=
            FLAGS_semantify_semantic_landmark_anchoring_ransac_min_inlier_ratio) {
      MatchCandidateScore candidate_score;
      candidate_score.candidate_id = candidate_id;
      candidate_score.inlier_num = inlier_num;
      candidate_score.inlier_ratio = inlier_ratio;
      candidate_score.total_inlier_error = total_inlier_error;
      candidate_score.total_model_error = total_model_error;
      candidate_score.inlier_index = inlier_index;
      candidate_score.initial_index = initial_index;
      if (FLAGS_semantify_ransac_optimizes_transformation_by_all_inliers) {
        Eigen::Matrix3Xd X_with_inlier, Y_with_inlier;
        X_with_inlier.resize(Eigen::NoChange, inlier_num);
        Y_with_inlier.resize(Eigen::NoChange, inlier_num);
        int col_idx = 0;
        for (const auto& idx : inlier_index) {
          const int source_track_id = source_track_ids[idx];
          const int ref_track_id = ref_track_ids[idx];
          const vi_map::SemanticLandmarkId& source_id =
              track_ids_to_semantic_landmark_ids_source.at(source_track_id);
          const vi_map::SemanticLandmarkId& ref_id =
              track_ids_to_semantic_landmark_ids_ref.at(ref_track_id);
          X_with_inlier.col(col_idx) =
              map.getSemanticLandmark_G_p_fi(source_id);
          Y_with_inlier.col(col_idx) = map.getSemanticLandmark_G_p_fi(ref_id);
          ++col_idx;
        }
        Eigen::Matrix4d T_G_G_with_inlier;
        computeICPPointsToPointsTransformation(
            X_with_inlier, Y_with_inlier, &T_G_G_with_inlier);

        pose::Transformation G_T_ref_source_tmp_with_inlier(
            G_T_ref_source_matrix);
        G_T_ref_source_tmp = G_T_ref_source_tmp_with_inlier;

        // recompute total_inlier_error and total_model_error
        double total_inlier_error_new = 0;
        double total_model_error_new = 0;
        for (const auto& idx : index_vector) {
          const int source_track_id = source_track_ids[idx];
          const int ref_track_id = ref_track_ids[idx];
          const vi_map::SemanticLandmarkId& source_id =
              track_ids_to_semantic_landmark_ids_source.at(source_track_id);
          const vi_map::SemanticLandmarkId& ref_id =
              track_ids_to_semantic_landmark_ids_ref.at(ref_track_id);
          Eigen::Vector3d x = map.getSemanticLandmark_G_p_fi(source_id);
          Eigen::Vector3d y = map.getSemanticLandmark_G_p_fi(ref_id);
          double error = (y - G_T_ref_source_tmp * x).norm();
          total_model_error_new += error;
          auto search = inlier_index.find(idx);
          if (search != inlier_index.end()) {
            total_inlier_error_new += error;
          }
        }
        candidate_score.total_inlier_error = total_inlier_error_new;
        candidate_score.total_model_error = total_model_error_new;
      }

      (*G_T_ref_source_ransac_candidates)[candidate_id] = G_T_ref_source_tmp;
      (*source_track_id_to_matched_ref_ids_ransac_candidates)[candidate_id] =
          inlier_id_pairs;
      match_candidate_scores->push_back(candidate_score);
      candidate_id++;
    }
    // selects the most inlier set.
    // if equal number, then choose the set with lower error
    // TODO(jkuo): inlier_ratio is not part of scoring for now
    if (inlier_num >= best_inlier_num) {
      if (inlier_num > best_inlier_num) {
        best_inlier_num = inlier_num;
        best_total_inlier_error = total_inlier_error;
        best_inlier_ratio = inlier_ratio;
      } else if (
          inlier_num == best_inlier_num &&
          best_total_inlier_error > total_inlier_error) {
        best_inlier_num = inlier_num;
        best_total_inlier_error = total_inlier_error;
        best_inlier_ratio = inlier_ratio;
      }
    }
  }  // end of ransac iteration

  if (match_candidate_scores->empty()) {
    LOG(ERROR) << "Was not able to find enough inlier sample points. "
               << "The best_inlier_num: " << best_inlier_num
               << "The best_inlier_ratio: " << best_inlier_ratio;
    return false;
  }

  return true;
}

void SemantifyPlugin::applyUniqueVisibilityFilter(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
    SourceTrackIdToMatchedRefTrackIdsMap& source_track_id_to_matched_ref_ids) {
  // since we use instance aware object detector, it is not possible to have
  // matches between semantic landmarks that both exists in the same frame
  // because they should've been recognized as one object in the detection
  // stage. Hence, we can use this to remove spurious matches. This is only
  // works within the same map because track ids are map specific.

  // build the covisibility map[source_id] = {track ids that are present in the
  // same frame}
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_source =
          mission_id_to_semantic_landmark_id_[mission_id_source];

  for (auto& pair : source_track_id_to_matched_ref_ids) {
    const vi_map::SemanticLandmarkId& id =
        track_ids_to_semantic_landmark_ids_source.at(pair.first);
    vi_map::SemanticObjectIdentifierList object_ids;
    map.getSemanticObjectIdentifiersForSemanticLandmark(id, &object_ids);
    std::set<int> covisible_track_ids_source;
    for (const vi_map::SemanticObjectIdentifier object_id : object_ids) {
      const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
      const size_t frame_index = object_id.frame_id.frame_index;
      const size_t measurement_index = object_id.measurement_index;
      const vi_map::Vertex& v = map.getVertex(vertex_id);
      const aslam::VisualFrame& vf = v.getVisualFrame(frame_index);
      const Eigen::VectorXi& track_ids = vf.getSemanticObjectTrackIds();
      for (int i = 0; i < track_ids.size(); i++) {
        covisible_track_ids_source.insert(track_ids(i));
      }
    }
    // filter the matched ref track ids, they should not be in the covisible
    // track ids set
    for (int ref_id : pair.second) {
      auto search = covisible_track_ids_source.find(ref_id);
      if (search != covisible_track_ids_source.end()) {
        pair.second.erase(ref_id);
      }
    }
  }
}

void SemantifyPlugin::getObservingVerticiesAndCovisibleTrackIds(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const vi_map::SemanticLandmarkId& landmark_id,
    pose_graph::VertexIdList* observing_verticies,
    std::set<int>* covisible_track_ids) {
  CHECK(observing_verticies);
  CHECK(covisible_track_ids);
  const TrackIdsToSemanticLandmarkIdsMap&
      valid_track_ids_to_semantic_landmark_ids_map =
          mission_id_to_semantic_landmark_id_[mission_id];
  vi_map::SemanticObjectIdentifierList object_ids;
  map.getSemanticObjectIdentifiersForSemanticLandmark(landmark_id, &object_ids);
  for (const vi_map::SemanticObjectIdentifier object_id : object_ids) {
    const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
    observing_verticies->push_back(vertex_id);
    const size_t frame_index = object_id.frame_id.frame_index;
    const vi_map::Vertex& v = map.getVertex(vertex_id);
    const aslam::VisualFrame& vf = v.getVisualFrame(frame_index);
    const Eigen::VectorXi& track_ids = vf.getSemanticObjectTrackIds();
    for (int i = 0; i < track_ids.size(); i++) {
      // only insert valid track ids
      if (valid_track_ids_to_semantic_landmark_ids_map.find(track_ids(i)) !=
          valid_track_ids_to_semantic_landmark_ids_map.end()) {
        covisible_track_ids->insert(track_ids(i));
      }
    }
  }
}

void SemantifyPlugin::extendObservingVerticies(
    const vi_map::VIMap& map, const pose_graph::VertexIdList& all_verticies,
    pose_graph::VertexIdList* observing_verticies,
    std::set<int>* covisible_track_ids) {
  CHECK(observing_verticies);
  CHECK(covisible_track_ids);
  // jkuo(TODO): need to ensure the assumption is always true some how
  // Assumes the observing_verticies is sorted by timestamp
  std::map<int64_t, pose_graph::VertexId> timestamp_to_vertex_id;

  for (const auto& vertex_id : *observing_verticies) {
    timestamp_to_vertex_id.emplace(
        map.getVertex(vertex_id).getMinTimestampNanoseconds(), vertex_id);
  }
  const pose_graph::VertexId first_id = *(all_verticies.begin());
  const pose_graph::VertexId last_id = *(--all_verticies.end());

  const pose_graph::VertexId first_observed_id =
      timestamp_to_vertex_id.begin()->second;
  const pose_graph::VertexId last_observed_id =
      (--timestamp_to_vertex_id.end())->second;

  auto search_first =
      std::find(all_verticies.begin(), all_verticies.end(), first_observed_id);
  auto search_last =
      std::find(all_verticies.begin(), all_verticies.end(), last_observed_id);
  int count = FLAGS_semantify_semantic_landmark_lc_extend_visible_verticies_num;

  const vi_map::MissionId& mission_id =
      map.getVertex(*search_first).getMissionId();
  const TrackIdsToSemanticLandmarkIdsMap&
      valid_track_ids_to_semantic_landmark_ids_map =
          mission_id_to_semantic_landmark_id_[mission_id];
  // if not already at the end of pose graph
  if (*search_first != first_id) {
    while (*(--search_first) != first_id && count != 0) {
      observing_verticies->push_back(*search_first);
      const vi_map::Vertex& v = map.getVertex(*search_first);
      // TODO(jkuo): only support one cam now
      const aslam::VisualFrame& vf = v.getVisualFrame(0);
      if (vf.hasSemanticObjectTrackIds()) {
        const Eigen::VectorXi& track_ids = vf.getSemanticObjectTrackIds();
        for (int i = 0; i < track_ids.size(); i++) {
          // only insert valid track ids
          if (valid_track_ids_to_semantic_landmark_ids_map.find(track_ids(i)) !=
              valid_track_ids_to_semantic_landmark_ids_map.end()) {
            covisible_track_ids->insert(track_ids(i));
          }
        }
      }
      count--;
    }
  }

  count = FLAGS_semantify_semantic_landmark_lc_extend_visible_verticies_num;
  if (*search_last != last_id) {
    while (*(++search_last) != last_id && count != 0) {
      observing_verticies->push_back(*search_last);
      const vi_map::Vertex& v = map.getVertex(*search_last);
      // TODO(jkuo): only support one cam now
      const aslam::VisualFrame& vf = v.getVisualFrame(0);
      if (vf.hasSemanticObjectTrackIds()) {
        const Eigen::VectorXi& track_ids = vf.getSemanticObjectTrackIds();
        for (int i = 0; i < track_ids.size(); i++) {
          // only insert valid track ids
          if (valid_track_ids_to_semantic_landmark_ids_map.find(track_ids(i)) !=
              valid_track_ids_to_semantic_landmark_ids_map.end()) {
            covisible_track_ids->insert(track_ids(i));
          }
        }
      }
      count--;
    }
  }
}

void SemantifyPlugin::getNearestObservingVertex(
    const vi_map::VIMap& map,
    const vi_map::SemanticLandmarkId& landmark_id_qurey,
    pose_graph::VertexId* nearest_vertex_id) {
  CHECK(nearest_vertex_id);
  const Eigen::Vector3d& p_G_landmark_query =
      map.getSemanticLandmark_G_p_fi(landmark_id_qurey);
  vi_map::SemanticObjectIdentifierList object_ids;
  map.getSemanticObjectIdentifiersForSemanticLandmark(
      landmark_id_qurey, &object_ids);
  // sorted in asceding order by default
  std::map<double, pose_graph::VertexId> verticies_ordered_by_distance_map;
  for (const vi_map::SemanticObjectIdentifier object_id : object_ids) {
    const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
    const Eigen::Vector3d vertex_G_p_I = map.getVertex_G_p_I(vertex_id);
    verticies_ordered_by_distance_map.emplace(
        (p_G_landmark_query - vertex_G_p_I).norm(), vertex_id);
  }

  *nearest_vertex_id = verticies_ordered_by_distance_map.begin()->second;
}

void SemantifyPlugin::getTopologicalCenterVertex(
    const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
    const std::unordered_set<TrackId>& covisible_lm_track_ids,
    const TrackIdToObservingVerticesMap& track_ids_to_observing_verticies,
    pose_graph::VertexIdList* topological_vertices_candidates,
    pose_graph::VertexId* topological_center_vertex_id) {
  CHECK(topological_center_vertex_id);
  // finds the observers that observe the largest number of semantic landmarks
  std::unordered_map<pose_graph::VertexId, size_t> observer_to_count;
  size_t max_count = 0;
  for (const auto& track_id : covisible_lm_track_ids) {
    const pose_graph::VertexIdList& observer_ids =
        track_ids_to_observing_verticies.at(track_id);
    for (const pose_graph::VertexId& observer_id : observer_ids) {
      auto search = observer_to_count.find(observer_id);
      if (search == observer_to_count.end()) {
        observer_to_count[observer_id] = 1;
      } else {
        ++search->second;
        if (search->second > max_count) {
          max_count = search->second;
        }
      }
    }
  }
  pose_graph::VertexIdList observers_with_most_landmarks;
  for (const auto& pair : observer_to_count) {
    if (pair.second == max_count) {
      observers_with_most_landmarks.push_back(pair.first);
    }
  }
  // LOG(INFO)<<"max_count: "<<max_count;
  // LOG(INFO)<<"observers_with_most_landmarks:
  // "<<observers_with_most_landmarks.size();
  *topological_vertices_candidates = observers_with_most_landmarks;

  // finds the vertex closest to the observed landmarks
  const TrackIdsToSemanticLandmarkIdsMap& track_ids_to_semantic_landmark_ids =
      mission_id_to_semantic_landmark_id_.at(mission_id);
  std::vector<Eigen::Vector3d> lm_positions;
  for (const auto& track_id : covisible_lm_track_ids) {
    const vi_map::SemanticLandmarkId& lm_id =
        track_ids_to_semantic_landmark_ids.at(track_id);
    const Eigen::Vector3d& lm_position = map.getSemanticLandmark_G_p_fi(lm_id);
    lm_positions.push_back(lm_position);
    // LOG(INFO)<<"track_id: "<<track_id <<"\n"
    //         <<lm_position;
  }
  double min_error_sum = std::numeric_limits<double>::max();
  pose_graph::VertexId current_best;
  for (const pose_graph::VertexId& id : observers_with_most_landmarks) {
    const pose::Transformation& T_G_I = map.getVertex_T_G_I(id);
    const Eigen::Vector3d& vertex_position = T_G_I.getPosition();
    double error = 0;
    for (const auto& lm_position : lm_positions) {
      error += (lm_position - vertex_position).norm();
    }
    if (error < min_error_sum) {
      min_error_sum = error;
      current_best = id;
    }
  }
  *topological_center_vertex_id = current_best;
}

bool SemantifyPlugin::addSemanticLoopClosureEdgeToMap(
    vi_map::VIMap* map, const vi_map::MissionId& mission_id_source,
    const vi_map::MissionId& mission_id_ref,
    const std::vector<int>& accepted_match_idx,
    const std::unordered_map<int, pose::Transformation>&
        accepted_G_T_ref_source_ransac_map,
    const std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>&
        accepted_track_id_matches_map,
    const std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>&
        accepted_main_track_id_matches_map,
    const TrackIdToObservingVerticesMap&
        track_ids_to_observing_verticies_source,
    const TrackIdToObservingVerticesMap& track_ids_to_observing_verticies_ref) {
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_source =
          mission_id_to_semantic_landmark_id_.at(mission_id_source);
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_ref =
          mission_id_to_semantic_landmark_id_.at(mission_id_ref);

  // since we consider for every landmark, there is possiblility that we get
  // duplicate in the accepted match from either side of the clusters, so we use
  // SetofSets to filter the match candidates
  SetOfSets accepted_track_id_pairs;
  for (int match_idx : accepted_match_idx) {
    LOG(INFO) << "match_idx: " << match_idx;
    const pose::Transformation& accepted_G_T_ref_source_ransac =
        accepted_G_T_ref_source_ransac_map.at(match_idx);
    const SourceTrackIdToMatchedRefTrackIdsMap& accepted_match_candidate =
        accepted_track_id_matches_map.at(match_idx);
    const SourceTrackIdToMatchedRefTrackIdsMap& accepted_main_match_candidate =
        accepted_main_track_id_matches_map.at(match_idx);
    CHECK_EQ(accepted_main_match_candidate.size(), 1u);
    for (const auto& pair : accepted_main_match_candidate) {
      const int track_id_source = pair.first;
      // TODO(jkuo): dont need to loop here anymore because we only add for main
      // match candidates
      for (const int& matched_id_ref : pair.second) {
        vi_map::SemanticLandmarkIdOrderedSet semantic_lm_id_pair;
        semantic_lm_id_pair.insert(
            track_ids_to_semantic_landmark_ids_source.at(track_id_source));
        semantic_lm_id_pair.insert(
            track_ids_to_semantic_landmark_ids_ref.at(matched_id_ref));
        CHECK_EQ(semantic_lm_id_pair.size(), 2u);
        // if we have not add an edge for this pair of candidates before
        if (accepted_track_id_pairs.insert(semantic_lm_id_pair).second) {
          LOG(INFO) << "accepted track id pair: " << track_id_source << " -> "
                    << matched_id_ref;

          pose_graph::VertexId lc_edge_vertex_id_source;
          pose_graph::VertexId lc_edge_vertex_id_ref;
          if (FLAGS_semantify_semantic_landmark_lc_add_edge_between_topological_center_vertices) {
            // create the lists of covisible landmark track ids for source and
            // ref
            std::unordered_set<TrackId> covisible_lm_track_id_source;
            std::unordered_set<TrackId> covisible_lm_track_id_ref;
            for (const auto& lm_pair : accepted_match_candidate) {
              covisible_lm_track_id_source.insert(lm_pair.first);
              for (const auto& match_lm_track_id : lm_pair.second) {
                covisible_lm_track_id_ref.insert(match_lm_track_id);
              }
            }
            // LOG(INFO)<<"Source ids:";
            // for(const auto& id: covisible_lm_track_id_source){
            //   LOG(INFO)<<"id: "<<id;
            // }
            // LOG(INFO)<<"Ref ids:";
            // for(const auto& id: covisible_lm_track_id_ref){
            //   LOG(INFO)<<"id: "<<id;
            // }
            pose_graph::VertexIdList topological_vertices_candidates_source;
            getTopologicalCenterVertex(
                *map, mission_id_source, covisible_lm_track_id_source,
                track_ids_to_observing_verticies_source,
                &topological_vertices_candidates_source,
                &lc_edge_vertex_id_source);

            pose_graph::VertexIdList topological_vertices_candidates_ref;
            getTopologicalCenterVertex(
                *map, mission_id_ref, covisible_lm_track_id_ref,
                track_ids_to_observing_verticies_ref,
                &topological_vertices_candidates_ref, &lc_edge_vertex_id_ref);
            // publish  visualization for the topological_vertices_candidates
            if (FLAGS_semantify_visualize_semantic_loop_closure_topological_vertex_candidates) {
              std::string top_vertices_topic =
                  "topological_vertices_candidates";
              visualization::Color color(255u, 255u, 0u);

              Eigen::Matrix3Xd points;
              points.resize(
                  Eigen::NoChange,
                  topological_vertices_candidates_source.size());
              for (int i = 0; i < topological_vertices_candidates_source.size();
                   i++) {
                points.col(i) =
                    map->getVertex_T_G_I(
                           topological_vertices_candidates_source[i])
                        .getPosition();
              }
              std::string name_space = std::to_string(track_id_source);
              publish3DPointsAsSpheres(
                  points, color, 1.0, 0.1, 0, FLAGS_tf_map_frame, name_space,
                  top_vertices_topic);
              std::this_thread::sleep_for(std::chrono::seconds(1));

              points.resize(
                  Eigen::NoChange, topological_vertices_candidates_ref.size());
              for (int i = 0; i < topological_vertices_candidates_ref.size();
                   i++) {
                points.col(i) =
                    map->getVertex_T_G_I(topological_vertices_candidates_ref[i])
                        .getPosition();
              }
              name_space = std::to_string(matched_id_ref);
              publish3DPointsAsSpheres(
                  points, color, 1.0, 0.1, 0, FLAGS_tf_map_frame, name_space,
                  top_vertices_topic);
              std::this_thread::sleep_for(std::chrono::seconds(1));
            }
          } else {
            const vi_map::SemanticLandmarkId& lm_id_source =
                track_ids_to_semantic_landmark_ids_source.at(track_id_source);
            const vi_map::SemanticLandmarkId& lm_id_ref =
                track_ids_to_semantic_landmark_ids_ref.at(matched_id_ref);
            getNearestObservingVertex(
                *map, lm_id_source, &lc_edge_vertex_id_source);
            getNearestObservingVertex(*map, lm_id_ref, &lc_edge_vertex_id_ref);
          }
          // visualization
          if (FLAGS_semantify_visualize_accepted_loop_closure_edge) {
            visualization::Color accepted_lc_edge_color(255u, 255u, 0u);
            std::string accepted_lc_edge_topic = "semantic_accepted_lc_edge";
            std::string name_space = std::to_string(track_id_source) + "_to_" +
                                     std::to_string(matched_id_ref);
            visualizeSemanticLoopClosureEdge(
                *map, lc_edge_vertex_id_source, lc_edge_vertex_id_ref,
                accepted_lc_edge_color, accepted_lc_edge_topic, name_space);
            std::this_thread::sleep_for(std::chrono::seconds(1));
          }

          // create vi_map::LoopClosureEdge
          const pose::Transformation& T_G_I_source =
              map->getVertex_T_G_I(lc_edge_vertex_id_source);
          const pose::Transformation& T_G_I_ref =
              map->getVertex_T_G_I(lc_edge_vertex_id_ref);
          const aslam::Transformation G_T_ref_source_lc =
              T_G_I_ref.inverse() * accepted_G_T_ref_source_ransac *
              T_G_I_source;

          // landmark covariances are computed in global frame, but the
          // transformation for the lc edge is in the vertex_ref frame. So we
          // should compute the covariance of the transformation in the
          // vertex_ref frame.
          const SourceTrackIdToMatchedRefTrackIdsMap&
              accepted_track_id_matches =
                  accepted_track_id_matches_map.at(match_idx);
          std::vector<int> source_track_ids, ref_track_ids;
          std::vector<int> index_vector;
          int i = 0;
          for (const auto& pair : accepted_track_id_matches) {
            if (pair.second.empty()) {
              continue;
            } else {
              for (const int& matched_id : pair.second) {
                source_track_ids.push_back(pair.first);
                ref_track_ids.push_back(matched_id);
                index_vector.push_back(i);
                i++;
              }
            }
          }
          CHECK_EQ(source_track_ids.size(), ref_track_ids.size());
          CHECK_EQ(source_track_ids.size(), index_vector.size());
          // const pose::Transformation T_I_G_ref = T_G_I_ref.inverse();
          // const Eigen::Matrix3d R_I_G_ref = T_I_G_ref.getRotationMatrix();
          Eigen::Matrix3Xd X, Y;
          X.resize(Eigen::NoChange, index_vector.size());
          Y.resize(Eigen::NoChange, index_vector.size());
          std::vector<Eigen::Matrix3d> X_cov, Y_cov;
          for (const auto& idx : index_vector) {
            const int source_track_id = source_track_ids[idx];
            const int ref_track_id = ref_track_ids[idx];
            const vi_map::SemanticLandmarkId& source_id =
                track_ids_to_semantic_landmark_ids_source.at(source_track_id);
            const vi_map::SemanticLandmarkId& ref_id =
                track_ids_to_semantic_landmark_ids_ref.at(ref_track_id);
            const vi_map::SemanticLandmark& lm_source =
                map->getSemanticLandmark(source_id);
            const vi_map::SemanticLandmark& lm_ref =
                map->getSemanticLandmark(ref_id);
            X.col(idx) = map->getSemanticLandmark_G_p_fi(source_id);
            Y.col(idx) = map->getSemanticLandmark_G_p_fi(ref_id);
            Eigen::Matrix3d G_lm_source_covariance;
            Eigen::Matrix3d G_lm_ref_covariance;
            if (!lm_source.get_p_B_Covariance(&G_lm_source_covariance)) {
              LOG(ERROR) << "landmark source track id: " << source_track_id
                         << " does not have its covariance set.";
              return false;
            }
            if (!lm_ref.get_p_B_Covariance(&G_lm_ref_covariance)) {
              LOG(ERROR) << "landmark ref track id: " << ref_track_id
                         << " does not have its covariance set.";
              return false;
            }
            X_cov.push_back(G_lm_source_covariance);
            Y_cov.push_back(G_lm_ref_covariance);
          }
          // LOG(INFO)<<"X: \n"<<X;
          // LOG(INFO)<<"Y: \n"<<Y;
          // for(const auto& x_cov: X_cov) {
          //   LOG(INFO)<<"x_cov: \n"<<x_cov;
          // }
          // for(const auto& y_cov: Y_cov) {
          //   LOG(INFO)<<"y_cov: \n"<<y_cov;
          // }
          // Eigen::Matrix4d G_T_ref_source;
          // computeICPPointsToPointsTransformation(X, Y,
          // &G_T_ref_source);
          // LOG(INFO)<<"G_T_ref_source: \n"
          //          <<G_T_ref_source;
          // Eigen::Matrix<double, 6, 6> G_T_ref_source_covariance;
          // semantify::calculateICPCovariance(X, Y, G_T_ref_source,
          //     X_cov, Y_cov, &G_T_ref_source_covariance);
          // LOG(INFO) <<"G_T_ref_source_covariance: \n"
          //           <<G_T_ref_source_covariance;

          // LOG(INFO)<<"accepted_G_T_ref_source_ransac: \n"
          //          <<accepted_G_T_ref_source_ransac.getTransformationMatrix();

          Eigen::Matrix<double, 6, 6> G_T_ref_source_covariance;
          semantify::calculateICPCovariance(
              X, Y, accepted_G_T_ref_source_ransac.getTransformationMatrix(),
              X_cov, Y_cov, &G_T_ref_source_covariance);
          // LOG(INFO) <<"G_T_ref_source_covariance: \n"
          //           <<G_T_ref_source_covariance;
          // LOG(INFO) <<"lc_edge_vertex_id_ref:
          // "<<lc_edge_vertex_id_ref.hexString(); LOG(INFO)
          // <<"lc_edge_vertex_id_source:
          // "<<lc_edge_vertex_id_source.hexString();

          pose_graph::EdgeId loop_closure_edge_id;
          aslam::generateId(&loop_closure_edge_id);
          CHECK(loop_closure_edge_id.isValid());
          const double kSwitchVariable = 1.0;
          vi_map::Edge::UniquePtr loop_closure_edge(new vi_map::LoopClosureEdge(
              loop_closure_edge_id, lc_edge_vertex_id_ref,
              lc_edge_vertex_id_source, kSwitchVariable, 0.000001,
              G_T_ref_source_lc, G_T_ref_source_covariance));
          map->addEdge(std::move(loop_closure_edge));
          semantic_lc_edge_ids_.push_back(loop_closure_edge_id);
        }
      }
    }
  }
  LOG(INFO) << "Added # of loop closure edge from unique match candidate pair: "
            << accepted_track_id_pairs.size();
  return true;
}

void SemantifyPlugin::mergeSemanticLandmarks(
    vi_map::VIMap* map, const vi_map::MissionId& mission_id_source,
    const vi_map::MissionId& mission_id_ref,
    const std::vector<int>& accepted_match_idx,
    const std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>&
        accepted_track_id_matches_map) {
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_source =
          mission_id_to_semantic_landmark_id_.at(mission_id_source);
  const TrackIdsToSemanticLandmarkIdsMap&
      track_ids_to_semantic_landmark_ids_ref =
          mission_id_to_semantic_landmark_id_.at(mission_id_ref);

  // since we consider for every landmark, there is possiblility that we get
  // duplicate in the accepted match from either side of the clusters
  // TODO(jkuo): ideally should use a set of sets for constant search time
  std::vector<vi_map::SemanticLandmarkIdOrderedSet> accepted_lm_pairs;
  // we also need to save the mapping of which landmark is merged into in case
  // the merged landmark is used in another match candidate later
  landmark_id_old_to_new_.clear();
  track_id_old_to_new_.clear();
  int merged_lm_num = 0;
  for (const int& match_idx : accepted_match_idx) {
    const SourceTrackIdToMatchedRefTrackIdsMap&
        source_track_id_to_matched_ref_ids =
            accepted_track_id_matches_map.at(match_idx);
    for (const auto& pair : source_track_id_to_matched_ref_ids) {
      const TrackId track_id_source = pair.first;
      for (const auto& matched_id_ref : pair.second) {
        vi_map::SemanticLandmarkId lm_id_to_be_merged =
            track_ids_to_semantic_landmark_ids_source.at(track_id_source);
        vi_map::SemanticLandmarkId lm_id_into =
            track_ids_to_semantic_landmark_ids_ref.at(matched_id_ref);
        if (lm_id_to_be_merged == lm_id_into) {
          LOG(ERROR) << "Skip trying to merge semantic landmark with itself.";
          continue;
        }
        CHECK_NE(lm_id_to_be_merged, lm_id_into);
        vi_map::SemanticLandmarkIdOrderedSet lm_pair;
        lm_pair.insert(lm_id_into);
        lm_pair.insert(lm_id_to_be_merged);
        // if the pair has not been merged before
        if (std::find(
                accepted_lm_pairs.begin(), accepted_lm_pairs.end(), lm_pair) ==
            accepted_lm_pairs.end()) {
          accepted_lm_pairs.push_back(lm_pair);
          // check if the to_be_merged landmark has already been merged into
          // other landmarks if it has already been merged, change the landmark
          // id to the one that currently exists and saves the merging
          // information
          vi_map::SemanticLandmarkId updated_lm_id_to_be_merged,
              updated_lm_id_into;
          getUpdatedLandmarkIdAfterMerges(
              lm_id_to_be_merged, lm_id_into, updated_lm_id_to_be_merged,
              updated_lm_id_into);
          landmark_id_old_to_new_[updated_lm_id_to_be_merged] =
              updated_lm_id_into;

          // The updated method above could result in merging with itself
          // eventually
          if (updated_lm_id_to_be_merged == updated_lm_id_into) {
            LOG(WARNING)
                << "Skip trying to merge semantic landmark with itself."
                << "original track ids are from " << track_id_source << "to "
                << matched_id_ref;
            continue;
          }
          // just as a fail safe
          CHECK_NE(updated_lm_id_to_be_merged, updated_lm_id_into);
          // Before adding the observations from "to be merged" to "into", we
          // first set the track ids of all observations to the "into" one to
          // keep consistency. because we currently use semantic landmark track
          // id in visualizeInitializedSemanticLandmarks() and creating
          // mission_id_to_semantic_landmark_id_
          TrackId updated_track_id =
              semantic_id_to_track_id_map_[updated_lm_id_into];
          setTrackIdForSemanticLandmark(
              map, updated_lm_id_to_be_merged, updated_track_id);
          map->mergeSemanticLandmarks(
              updated_lm_id_to_be_merged, updated_lm_id_into);
          ++merged_lm_num;
          // for debugging only
          std::unordered_map<TrackId, TrackId> lm_track_id_pair;
          lm_track_id_pair[track_id_source] = matched_id_ref;
          track_id_old_to_new_.emplace_back(lm_track_id_pair);
        }
      }
    }
  }
  LOG(INFO) << "Merged # of semantic landmarks: " << merged_lm_num;
  // print out which track id is merged with which
  for (const auto& map : track_id_old_to_new_) {
    LOG(INFO) << "From track id: " << map.begin()->first << "to "
              << map.begin()->second;
  }
}

void SemantifyPlugin::setTrackIdForSemanticLandmark(
    vi_map::VIMap* map, const vi_map::SemanticLandmarkId& landmark_id,
    TrackId new_track_id) {
  vi_map::SemanticObjectIdentifierList object_ids;
  map->getSemanticObjectIdentifiersForSemanticLandmark(
      landmark_id, &object_ids);
  for (const vi_map::SemanticObjectIdentifier object_id : object_ids) {
    const pose_graph::VertexId vertex_id = object_id.frame_id.vertex_id;
    const size_t frame_index = object_id.frame_id.frame_index;
    const size_t measurement_index = object_id.measurement_index;
    vi_map::Vertex& v = map->getVertex(vertex_id);
    aslam::VisualFrame& vf = v.getVisualFrame(frame_index);
    Eigen::VectorXi* track_ids = vf.getSemanticObjectTrackIdsMutable();
    (*track_ids)[measurement_index] = new_track_id;
  }
}

// stores the history of how landmarks were merged
void SemantifyPlugin::getUpdatedLandmarkIdAfterMerges(
    const vi_map::SemanticLandmarkId& old_lm_id_to_be_merged,
    const vi_map::SemanticLandmarkId& old_lm_id_into,
    vi_map::SemanticLandmarkId& updated_lm_id_to_be_merged,
    vi_map::SemanticLandmarkId& updated_lm_id_into) {
  updated_lm_id_to_be_merged = old_lm_id_to_be_merged;
  updated_lm_id_into = old_lm_id_into;
  // TODO(jkuo): sometimes it goes into infinite loop, so change it to for loop
  // later
  do {
    auto it_map_landmark_already_changed =
        landmark_id_old_to_new_.find(updated_lm_id_to_be_merged);
    if (it_map_landmark_already_changed != landmark_id_old_to_new_.end()) {
      // Recursively update the landmark_id according to the past merges.
      updated_lm_id_to_be_merged = it_map_landmark_already_changed->second;
    }
  } while (landmark_id_old_to_new_.count(updated_lm_id_to_be_merged) > 0);

  do {
    auto it_map_landmark_already_changed =
        landmark_id_old_to_new_.find(updated_lm_id_into);
    if (it_map_landmark_already_changed != landmark_id_old_to_new_.end()) {
      // Recursively update the landmark_id according to the past merges.
      updated_lm_id_into = it_map_landmark_already_changed->second;
    }
  } while (landmark_id_old_to_new_.count(updated_lm_id_into) > 0);
}

void SemantifyPlugin::
    calculateSemanticLandmarkCovarianceFromTriangulationResidual(
        vi_map::VIMap* map, const vi_map::SemanticLandmarkId& landmark_id) {
  // get triangulated landmark pos
  const Eigen::Vector3d& p_G_landmark =
      map->getSemanticLandmark_G_p_fi(landmark_id);
  vi_map::SemanticObjectIdentifierList observations;
  map->getSemanticObjectIdentifiersForSemanticLandmark(
      landmark_id, &observations);
  int num_measurements = 0;
  Eigen::Matrix3Xd pos_difference;
  pos_difference.resize(Eigen::NoChange, observations.size());
  for (const vi_map::SemanticObjectIdentifier observation : observations) {
    const pose_graph::VertexId& observer_id = observation.frame_id.vertex_id;
    CHECK(map->hasVertex(observer_id))
        << "Observer " << observer_id << " of store semantic landmark "
        << landmark_id.hexString() << " not in currently loaded map!";

    const vi_map::Vertex& observer =
        const_cast<const vi_map::VIMap*>(map)->getVertex(observer_id);
    const aslam::VisualFrame& visual_frame =
        observer.getVisualFrame(observation.frame_id.frame_index);
    const aslam::Transformation& T_M_I_observer = observer.get_T_M_I();
    const aslam::Transformation& T_G_M_observer =
        const_cast<const vi_map::VIMap*>(map)
            ->getMissionBaseFrameForVertex(observer_id)
            .get_T_G_M();
    aslam::Transformation T_G_I_observer = T_G_M_observer * T_M_I_observer;

    Eigen::Vector4d object_measurement =
        visual_frame.getSemanticObjectMeasurement(
            observation.measurement_index);
    Eigen::Vector2d measurement = object_measurement.block<2, 1>(0, 0);

    Eigen::Vector3d C_bearing_vector;
    bool projection_result =
        observer.getCamera(observation.frame_id.frame_index)
            ->backProject3(measurement, &C_bearing_vector);
    if (!projection_result) {
      LOG(ERROR) << "Landmark triangulation failed proj failed.";
      continue;
    }

    const aslam::CameraId& cam_id =
        observer.getCamera(observation.frame_id.frame_index)->getId();
    aslam::Transformation T_G_C =
        (T_G_I_observer * observer.getNCameras()->get_T_C_B(cam_id).inverse());
    Eigen::Vector3d p_G_C = T_G_C.getPosition();
    Eigen::Vector3d G_bearing_vector =
        T_G_C.getRotationMatrix() * C_bearing_vector;
    // compute the projection of landmark on to bearing vector
    Eigen::Vector3d C_lm_vector = p_G_landmark - p_G_C;
    Eigen::Vector3d projected_point =
        C_lm_vector.dot(G_bearing_vector) /
            G_bearing_vector.dot(G_bearing_vector) * G_bearing_vector +
        p_G_C;
    pos_difference.col(num_measurements) = p_G_landmark - projected_point;

    ++num_measurements;
  }
  pos_difference.conservativeResize(Eigen::NoChange, num_measurements);
  Eigen::Vector3d mean = pos_difference.rowwise().mean();
  Eigen::Vector3d sum_of_squared_diff;
  sum_of_squared_diff = Eigen::Vector3d::Zero();
  for (int i = 0; i < num_measurements; i++) {
    sum_of_squared_diff +=
        (pos_difference.col(i) - mean).array().square().matrix();
  }
  // sample variance
  Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
  covariance(0, 0) = sum_of_squared_diff(0) / (num_measurements - 1);
  covariance(1, 1) = sum_of_squared_diff(1) / (num_measurements - 1);
  covariance(2, 2) = sum_of_squared_diff(2) / (num_measurements - 1);
  vi_map::SemanticLandmark& landmark = map->getSemanticLandmark(landmark_id);
  landmark.set_p_B_Covariance(covariance);
}

}  // namespace semantify

MAPLAB_CREATE_CONSOLE_PLUGIN_WITH_PLOTTER(semantify::SemantifyPlugin);
