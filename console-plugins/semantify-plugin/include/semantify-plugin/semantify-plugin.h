#ifndef SEMANTIFY_PLUGIN_SEMANTIFY_PLUGIN_H_
#define SEMANTIFY_PLUGIN_SEMANTIFY_PLUGIN_H_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/frames/visual-frame.h>
#include <console-common/console-plugin-base-with-plotter.h>
#include <map-manager/map-manager.h>
#include <map-resources/resource-common.h>
#include <map-resources/resource-typedefs.h>
#include <vi-map/vi-map.h>
#include <visualization/viwls-graph-plotter.h>
#include <visualization/viz-primitives.h>

#include "semantify-plugin/hungarian-algorithm-manager.h"
#include "semantify-plugin/k-medoids-clustering-manager.h"

#include <deep_sort_ros/deep_sort_result.h>
#include <mask_rcnn_ros/mrcnn_result.h>
#include <netvlad_tf/netvlad_result.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace semantify {

typedef int TrackId;
typedef std::unordered_map<TrackId, vi_map::SemanticLandmarkId>
    TrackIdsToSemanticLandmarkIdsMap;
typedef std::unordered_map<vi_map::MissionId, TrackIdsToSemanticLandmarkIdsMap>
    MissionIdsToTrackIdsAndSemanticLandmarkIdsMap;
typedef std::unordered_map<
    vi_map::MissionId, std::unique_ptr<KMedoidsClusteringManager>>
    MissionIdsToKMedoidsClusteringManagersMap;
typedef std::unordered_map<
    vi_map::SemanticLandmarkId, vi_map::SemanticLandmarkId>
    LandmarkToLandmarkMap;
typedef std::unordered_map<TrackId, pose_graph::VertexIdList>
    TrackIdToObservingVerticesMap;

// unique id for each candidate
typedef size_t CandidateId;
// the first element contains the number of inlier and the second is the total
// error from ransac
struct MatchCandidateScore {
  CandidateId candidate_id = 0;
  int inlier_num = 0;
  double inlier_ratio = 0.0;
  double total_inlier_error = 0.0;  // total error from only the inlier set
  double total_model_error = 0.0;   // total error from all the candidates
  std::set<int> initial_index;
  std::set<int> inlier_index;
  MatchCandidateScore& operator=(const MatchCandidateScore& other) {
    candidate_id = other.candidate_id;
    inlier_num = other.inlier_num;
    inlier_ratio = other.inlier_ratio;
    total_inlier_error = other.total_inlier_error;
    total_model_error = other.total_model_error;
    inlier_index = other.inlier_index;
    initial_index = other.initial_index;

    return *this;
  }
};

typedef std::unordered_map<TrackId, std::set<TrackId>>
    SourceTrackIdToMatchedRefTrackIdsMap;
typedef std::unordered_map<CandidateId, pose::Transformation>
    CandidateIdToTransformationMap;
typedef std::unordered_map<CandidateId, SourceTrackIdToMatchedRefTrackIdsMap>
    CandidateIdToTrackIdMatchesMap;
typedef std::vector<MatchCandidateScore> MatchCandidateScores;

struct SetOfSetsHasher {
  size_t operator()(const vi_map::SemanticLandmarkIdOrderedSet& x) const {
    return x.begin()->hashToSizeT();
  }
};

typedef std::unordered_set<
    vi_map::SemanticLandmarkIdOrderedSet, SetOfSetsHasher>
    SetOfSets;

class SemantifyPlugin : public common::ConsolePluginBaseWithPlotter {
 public:
  SemantifyPlugin(
      common::Console* console, visualization::ViwlsGraphRvizPlotter* plotter);

  virtual std::string getPluginId() const {
    return "semantify_plugin";
  }

 private:
  // -------------------- Commands ---------------------- //
  int semantify();
  int generateAndSaveSemanticObjectMeasurements();
  int generateAndSaveSemanticObjectTrackIdsFromDeepsort();
  int visualizeSemanticObjectChannelsInVisualFrame();
  int visualizeOptionalResourcesBoundingboxes();
  int visualizeSemanticLandmarksAndGenerateTrackIdToSemanticLandmarkMap();
  int evaluateSemanticLandmarkWithTrackId();
  int generateDescriptorClusters();
  int displayDescriptorClustersScores();
  int compareDescriptorClustersScores();
  int matchSemanticLandmarksInOneMission();
  int anchorMissionWithSemanticLandmarks();
  int loopCloseMissionsWithSemanticLandmarks();
  int calculateSemanticLandmarkCovariances();
  int visualizeSemanticLoopClosureEdgeCovariances();

  // -------------------- Functions ---------------------- //
  // Converts the detection result into maplab resources.
  // Inputs: (all aligned by index)
  //  input_boxes: the bounding box results
  //  input_ids: the class id of the objects, specified by detector
  //  input_score: the confidence score of the detections
  //  input_masks: the object masks of the detections
  // Outputs:
  //  masks: the object masks in opencv Mat data format
  //  boxes: the bounding boxes in Maplab optional resource format
  void convertBoundingBoxesAndMasks(
      cv::Mat* masks, resources::ObjectInstanceBoundingBoxes* boxes,
      const std::vector<sensor_msgs::RegionOfInterest>& input_boxes,
      const std::vector<int>& input_ids,
      const std::vector<std::string>& input_names,
      const std::vector<float>& input_score,
      const std::vector<sensor_msgs::Image> input_masks);

  void convertResourceBoundingBoxesToDescriptorGenerationMessage(
      const resources::ObjectInstanceBoundingBoxes& res_boxes,
      std::vector<sensor_msgs::RegionOfInterest>* sensor_boxes,
      std::vector<int>* class_numbers, std::vector<int>* instance_numbers);

  void convertSemanticObjectMeasurementsToResourceBoundingBoxes(
      const Eigen::Matrix4Xd measurements, const Eigen::VectorXd confidences,
      const Eigen::VectorXi class_ids,
      resources::ObjectInstanceBoundingBoxes* res_boxes);

  void saveSemanticObjectMeasurementsToVisualFrame(
      aslam::VisualFrame::Ptr visual_frame,
      const resources::ObjectInstanceBoundingBoxes& boxes,
      std::vector<::netvlad_tf::Descriptor>& descriptors);

  void populateDeepSortServiceRequest(
      aslam::VisualFrame::Ptr visual_frame,
      deep_sort_ros::deep_sort_result* deep_sort_srv);

  void insertSemanticObjectTrackIds(
      aslam::VisualFrame::Ptr const visual_frame,
      Eigen::VectorXi& track_ids) const;
  void parseIdsFromString(
      const std::string ids, std::vector<std::string>* parsed_ids);

  void getMedoidsAndClassIdsOfTrackIdsFromMissionId(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      std::map<int, Eigen::VectorXf>* track_id_to_medoid_descriptor,
      std::map<int, int>* track_id_to_class_id);
  void buildCostMatrix(
      const std::map<int, Eigen::VectorXf>&
          track_id_to_medoid_descriptor_source,
      const std::map<int, Eigen::VectorXf>& track_id_to_medoid_descriptor_ref,
      const std::map<int, int>& source_track_id_to_class_id,
      const std::map<int, int>& ref_track_id_to_class_id,
      const bool set_diagonal_to_max_cost,
      std::vector<std::vector<float>>* cost_matrix);
  void getKNearestAssignments(
      const std::vector<std::vector<float>>& cost_matrix,
      const std::vector<int>& index_to_track_id_source,
      const std::vector<int>& index_to_track_id_ref,
      SourceTrackIdToMatchedRefTrackIdsMap* source_track_id_to_matched_ref_ids);
  void generateMatchCandidates(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
      const vi_map::MissionId& mission_id_ref,
      const bool set_diagonal_to_max_cost,
      SourceTrackIdToMatchedRefTrackIdsMap*
          source_track_id_to_matched_ref_ids_candidates);
  void visualizeMatchedTrackIds(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
      const vi_map::MissionId& mission_id_ref,
      const SourceTrackIdToMatchedRefTrackIdsMap&
          source_track_id_to_matched_ref_ids,
      const visualization::Color& color, const std::string& topic,
      const std::string& name_space = "default");
  void visualizeCovisibleTrackIds(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      const TrackId& track_id_source,
      const std::set<int>& track_id_covisible_track_ids,
      const visualization::Color& color, const std::string& topic);
  void visualizeSemanticLandmarksObservableVerticies(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      const TrackId& track_id,
      const pose_graph::VertexIdList& observable_verticies,
      const visualization::Color& color, const std::string& topic);
  void visualizeSemanticLoopClosureEdge(
      const vi_map::VIMap& map, const pose_graph::VertexId& from_vertex,
      const pose_graph::VertexId& to_vertex, const visualization::Color& color,
      const std::string& topic, const std::string& name_space = "default");
  bool visualizeSemanticLandmarkInfo(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      const int& track_id, const vi_map::SemanticLandmarkId& id,
      const std::unordered_map<int, size_t>& class_count_map);

  bool separateMissionsWithKnownBaseFrame(
      const vi_map::VIMap& map, vi_map::MissionIdList* mission_ids_source,
      vi_map::MissionIdList* mission_ids_ref);

  void computeICPPointsToPointsTransformation(
      const Eigen::Matrix3Xd& X, const Eigen::Matrix3Xd& Y,
      Eigen::Matrix4d* T_result);

  bool generateTransformationHypothesisFromMatches(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
      const vi_map::MissionId& mission_id_ref,
      const SourceTrackIdToMatchedRefTrackIdsMap&
          source_track_id_to_matched_ref_ids,
      const SourceTrackIdToMatchedRefTrackIdsMap& predefined_inliers,
      CandidateIdToTransformationMap* G_T_ref_source_ransac_candidates,
      CandidateIdToTrackIdMatchesMap*
          source_track_id_to_matched_ref_ids_ransac_candidates,
      MatchCandidateScores* match_candidate_scores);

  void applyUniqueVisibilityFilter(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
      SourceTrackIdToMatchedRefTrackIdsMap& source_track_id_to_matched_ref_ids);

  void getObservingVerticiesAndCovisibleTrackIds(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id_source,
      const vi_map::SemanticLandmarkId& landmark_id,
      pose_graph::VertexIdList* observing_verticies,
      std::set<int>* covisible_track_ids);
  void extendObservingVerticies(
      const vi_map::VIMap& map, const pose_graph::VertexIdList& all_verticies,
      pose_graph::VertexIdList* observing_verticies,
      std::set<int>* covisible_track_ids);
  void getNearestObservingVertex(
      const vi_map::VIMap& map, const vi_map::SemanticLandmarkId& landmark_id,
      pose_graph::VertexId* nearest_vertex_id);
  void getTopologicalCenterVertex(
      const vi_map::VIMap& map, const vi_map::MissionId& mission_id,
      const std::unordered_set<TrackId>& covisible_lm_track_ids,
      const TrackIdToObservingVerticesMap& track_ids_to_observing_verticies,
      pose_graph::VertexIdList* topological_vertices_candidates,
      pose_graph::VertexId* topological_center_vertex_id);
  bool addSemanticLoopClosureEdgeToMap(
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
      const TrackIdToObservingVerticesMap&
          track_ids_to_observing_verticies_ref);
  void mergeSemanticLandmarks(
      vi_map::VIMap* map, const vi_map::MissionId& mission_id_source,
      const vi_map::MissionId& mission_id_ref,
      const std::vector<int>& accepted_match_idx,
      const std::unordered_map<int, SourceTrackIdToMatchedRefTrackIdsMap>&
          accepted_track_id_matches_map);
  void setTrackIdForSemanticLandmark(
      vi_map::VIMap* map, const vi_map::SemanticLandmarkId& landmark_id,
      TrackId new_track_id);

  void getUpdatedLandmarkIdAfterMerges(
      const vi_map::SemanticLandmarkId& old_lm_id_to_be_merged,
      const vi_map::SemanticLandmarkId& old_lm_id_into,
      vi_map::SemanticLandmarkId& updated_lm_id_to_be_merged,
      vi_map::SemanticLandmarkId& updated_lm_id_into);

  void calculateSemanticLandmarkCovarianceFromTriangulationResidual(
      vi_map::VIMap* map, const vi_map::SemanticLandmarkId& landmark_id);

  // auxilary function
  static bool compareMatchCandidateScores(
      const MatchCandidateScore& lhs, const MatchCandidateScore& rhs) {
    if (lhs.inlier_num == rhs.inlier_num) {
      return lhs.total_inlier_error < rhs.total_inlier_error;
    } else {
      return lhs.inlier_num > rhs.inlier_num;
    }
  };

  ros::NodeHandle nh_;
  ros::ServiceClient maskrcnn_client_ =
      nh_.serviceClient<mask_rcnn_ros::mrcnn_result>("infer_image");
  ros::ServiceClient netvlad_client_ =
      nh_.serviceClient<netvlad_tf::netvlad_result>("generate_descriptor");
  ros::ServiceClient deep_sort_client_ =
      nh_.serviceClient<deep_sort_ros::deep_sort_result>(
          "generate_semantic_track_ids");

  // publisher for 3D semantic landmark visualization
  ros::Publisher semantic_landmark_pub_ =
      nh_.advertise<visualization_msgs::Marker>("vis_semantic_landmarks", 0);
  ros::Publisher semantic_landmark_covariance_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "semantic_landmark_covariances", 0);
  ros::Publisher semantic_landmark_lc_covariance_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>(
          "semantic_lc_covariances", 0);

  // track id and semantic landmark id map per mission
  // It contains all except bad quality landmarks.
  MissionIdsToTrackIdsAndSemanticLandmarkIdsMap
      mission_id_to_semantic_landmark_id_;
  MissionIdsToKMedoidsClusteringManagersMap mission_to_kmedoids_manager_;
  std::unique_ptr<HungaraianAlgorithmManager> hungarian_algorithm_manager_;
  LandmarkToLandmarkMap landmark_id_old_to_new_;
  // just for checking because semantic landmark id hexstring is too painful for
  // manual comparison
  std::vector<std::unordered_map<TrackId, TrackId>> track_id_old_to_new_;
  // to keep track of the semantic loop closure edges from previous iteration
  std::vector<pose_graph::EdgeId> semantic_lc_edge_ids_;
  // for landmark merging only, a map of semantic landmark id to track id
  std::unordered_map<vi_map::SemanticLandmarkId, TrackId>
      semantic_id_to_track_id_map_;
};

}  // namespace semantify
#endif  // SEMANTIFY_PLUGIN_SEMANTIFY_PLUGIN_H_
