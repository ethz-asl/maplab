#ifndef VI_MAP_VERTEX_H_
#define VI_MAP_VERTEX_H_

#include <posegraph/vertex.h>

#include <string>
#include <unordered_set>
#include <vector>

#include <gtest/gtest_prod.h>

#include <aslam/cameras/ncamera.h>
#include <aslam/common/memory.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <map-resources/resource-common.h>
#include <maplab-common/macros.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/proto-helpers.h>
#include <maplab-common/traits.h>

#include "vi-map/landmark-store.h"
#include "vi-map/landmark.h"
#include "vi-map/mission.h"
#include "vi-map/unique-id.h"
#include "vi-map/vi_map.pb.h"

namespace map_optimization_legacy {
class ViwlsGraph;  // Used for testing.
}

namespace vi_map {

class Vertex : public pose_graph::Vertex {
  friend class map_optimization_legacy::ViwlsGraph;  // Test.
  friend class MapConsistencyCheckTest;      // Test.
  friend class VertexResourcesTest;          // Test.
  FRIEND_TEST(MapConsistencyCheckTest, mapInconsistentMissingBackLink);
  friend class VIMap;

 public:
  MAPLAB_POINTER_TYPEDEFS(Vertex);

  friend void addObservedLandmarkId(
      const LandmarkId& landmark_id, vi_map::Vertex* vertex_ptr);

  Vertex(
      const pose_graph::VertexId& vertex_id,
      const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
      const Eigen::Matrix2Xd& img_points_distorted,
      const Eigen::VectorXd& uncertainties,
      const aslam::VisualFrame::DescriptorsT& descriptors,
      const Eigen::VectorXd& descriptor_scales,
      const std::vector<LandmarkId>& observed_landmark_ids,
      const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
      int64_t frame_timestamp, const aslam::NCamera::Ptr cameras);

  Vertex(
      const pose_graph::VertexId& vertex_id,
      const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
      const Eigen::Matrix2Xd& img_points_distorted,
      const Eigen::VectorXd& uncertainties,
      const aslam::VisualFrame::DescriptorsT& descriptors,
      const std::vector<LandmarkId>& observed_landmark_ids,
      const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
      int64_t frame_timestamp, const aslam::NCamera::Ptr cameras);

  Vertex(
      const pose_graph::VertexId& vertex_id,
      const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
      const aslam::VisualNFrame::Ptr visual_n_frame,
      const std::vector<std::vector<LandmarkId>>& n_frame_landmarks,
      const vi_map::MissionId& mission_id);

  Vertex(
      const pose_graph::VertexId& vertex_id,
      const aslam::VisualNFrame::Ptr visual_n_frame,
      const vi_map::MissionId& mission_id);

  explicit Vertex(const aslam::NCamera::Ptr cameras);

  // Default constructor for methods which take a Vertex* as argument and
  // populate it.
  Vertex();
  virtual ~Vertex() {}
  Vertex(const Vertex&) = default;
  Vertex& operator=(const Vertex&) = delete;

  virtual const pose_graph::VertexId& id() const;
  void setId(const pose_graph::VertexId& id);

  void set_T_M_I(const pose::Transformation& T_M_I);
  void set_p_M_I(const Eigen::Vector3d& p_M_I);
  void set_q_M_I(const Eigen::Quaterniond& q_M_I);
  void set_v_M(const Eigen::Vector3d& v_M);
  void setAccelBias(const Eigen::Vector3d& accel_bias);
  void setGyroBias(const Eigen::Vector3d& gyro_bias);

  virtual bool addIncomingEdge(const pose_graph::EdgeId& edge);
  virtual bool addOutgoingEdge(const pose_graph::EdgeId& edge);

  virtual void getOutgoingEdges(pose_graph::EdgeIdSet* edges) const;
  virtual void getIncomingEdges(pose_graph::EdgeIdSet* edges) const;
  virtual void getAllEdges(pose_graph::EdgeIdSet* edges) const;

  virtual bool hasIncomingEdges() const;
  virtual bool hasOutgoingEdges() const;
  size_t numOutgoingEdges() const;

  virtual void removeIncomingEdge(const pose_graph::EdgeId& edge_id);
  virtual void removeOutgoingEdge(const pose_graph::EdgeId& edge_id);

  // Pointers to data containers, useful for optimization purposes
  // (e.g. Google Ceres).
  double* get_q_M_I_Mutable();
  double* get_p_M_I_Mutable();
  double* get_v_M_Mutable();
  double* getAccelBiasMutable();
  double* getGyroBiasMutable();

  const pose::Transformation& get_T_M_I() const;
  const Eigen::Quaterniond& get_q_M_I() const;
  const Eigen::Vector3d& get_p_M_I() const;
  const Eigen::Vector3d& get_v_M() const;
  const Eigen::Vector3d& getAccelBias() const;
  const Eigen::Vector3d& getGyroBias() const;

  const LandmarkId& getObservedLandmarkId(
      unsigned int frame_idx, int keypoint_idx) const;
  const LandmarkId& getObservedLandmarkId(
      const KeypointIdentifier& keypoint_id) const;
  void setObservedLandmarkId(
      const KeypointIdentifier& keypoint_id, const LandmarkId& landmark_id);
  void setObservedLandmarkId(
      unsigned int frame_idx, int keypoint_idx, const LandmarkId& landmark_id);
  size_t observedLandmarkIdsSize(unsigned int frame_idx) const;
  int numValidObservedLandmarkIds(unsigned int frame_idx) const;
  int numValidObservedLandmarkIdsInAllFrames() const;
  void getFrameObservedLandmarkIds(
      unsigned int frame_idx, LandmarkIdList* landmark_ids) const;
  const LandmarkIdList& getFrameObservedLandmarkIds(
      unsigned int frame_idx) const;
  void getAllObservedLandmarkIds(LandmarkIdList* landmark_ids) const;
  void getAllObservedLandmarkIds(
      std::vector<LandmarkIdList>* landmark_ids) const;

  // Calls action on each possible keypoint identifier in this vertex.
  void forEachKeypoint(
      const std::function<void(const KeypointIdentifier&)>& action) const;
  void forEachFrame(
      const std::function<
          void(const unsigned int frame_idx, const aslam::VisualFrame& frame)>&
          action) const;

  inline const vi_map::MissionId& getMissionId() const;
  inline void setMissionId(const vi_map::MissionId& mission_id);

  inline aslam::VisualNFrame& getVisualNFrame();
  inline const aslam::VisualNFrame& getVisualNFrame() const;
  inline aslam::VisualNFrame::Ptr& getVisualNFrameShared();
  inline aslam::VisualNFrame::ConstPtr getVisualNFrameShared() const;
  inline size_t numFrames() const;
  inline bool isVisualFrameSet(unsigned int frame_idx) const;
  inline bool isVisualFrameValid(unsigned int frame_idx) const;
  inline aslam::VisualFrame& getVisualFrame(unsigned int frame_idx);
  inline aslam::VisualFrame::Ptr getVisualFrameShared(unsigned int frame_idx);
  inline aslam::VisualFrame::Ptr getVisualFrameShared(aslam::FrameId frame_id);
  inline unsigned int getVisualFrameIndex(aslam::FrameId frame_id) const;
  inline const aslam::VisualFrame& getVisualFrame(unsigned int frame_idx) const;
  inline const aslam::VisualFrame::ConstPtr getVisualFrameShared(
      unsigned int frame_idx) const;

  inline aslam::NCamera::ConstPtr getNCameras() const;
  inline aslam::Camera::ConstPtr getCamera(unsigned int frame_idx) const;
  inline aslam::Camera::Ptr getCamera(unsigned int frame_idx);
  inline void setCamera(
      unsigned int frame_idx, const aslam::Camera::Ptr& camera);
  inline void setNCameras(const aslam::NCamera::Ptr& n_cameras);

  inline LandmarkStore& getLandmarks();
  inline const LandmarkStore& getLandmarks() const;
  void getStoredLandmarkIdList(LandmarkIdList* landmark_id_list) const;
  bool hasStoredLandmark(const LandmarkId& landmark_id) const;
  pose::Position3D getLandmark_p_LM_fi(const LandmarkId& landmark_id) const;
  void setLandmark_LM_p_fi(
      const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& LM_p_fi);
  inline void setLandmarks(const LandmarkStore& landmark_store);

  void setFrameAndLandmarkObservations(
      aslam::VisualNFrame::Ptr visual_n_frame,
      const std::vector<std::vector<LandmarkId>>& img_landmarks);

  // Clears all observed landmark ids, resizes the vectors to match the
  // number of keypoints in the corresponding frame, and sets all
  // landmark ids to invalid.
  void resetObservedLandmarkIdsToInvalid();

  inline void forEachUnassociatedKeypoint(
      const unsigned int frame_idx,
      const std::function<void(const int keypoint_index)>& action) const;
  inline void getUnassociatedKeypoints(
      const unsigned int frame_idx, std::vector<int>* result) const;

  bool isFrameIndexValid(unsigned int frame_idx) const;
  bool areFrameAndKeypointIndicesValid(
      unsigned int frame_idx, int keypoint_idx) const;
  void checkConsistencyOfVisualObservationContainers() const;

  typedef std::vector<backend::ResourceTypeToIdsMap> FrameResourceMap;
  const FrameResourceMap& getFrameResourceMap() const;
  void setFrameResourceMap(const FrameResourceMap& frame_resource_map);

  bool hasFrameResourceOfType(
      const unsigned int frame_idx, const backend::ResourceType& type) const;
  bool hasFrameResourceWithId(
      const unsigned int frame_idx,
      const backend::ResourceId& resource_id) const;
  void getFrameResourceIdsOfType(
      const unsigned int frame_idx, const backend::ResourceType& type,
      backend::ResourceIdSet* resource_ids) const;
  size_t getNumFrameResourcesOfType(
      const unsigned int frame_idx, const backend::ResourceType& type) const;
  void addFrameResourceIdOfType(
      const unsigned int frame_idx, const backend::ResourceType& type,
      const backend::ResourceId& resource_id);

  void deleteFrameResourceIdsOfType(
      const unsigned int frame_idx, const backend::ResourceType& resource_type);
  void deleteAllFrameResourceInfo(const unsigned int frame_idx);
  void deleteAllFrameResourceInfo();

  void serialize(vi_map::proto::ViwlsVertex* proto) const;
  void deserialize(
      const pose_graph::VertexId& vertex_id,
      const vi_map::proto::ViwlsVertex& proto);

  inline bool operator==(const Vertex& lhs) const;
  inline bool operator!=(const Vertex& lhs) const;
  inline bool isSameApartFromOutgoingEdges(const Vertex& lhs) const;

  // Repairs the visual observation containers of this vertex after adding new
  // keypoints to one of the frames. Resizes the observed_landmark_ids_ vector
  // for every frame based on the number of keypoints the frames number of
  // keypoints.
  // Crashes hard if:
  //    The new number of keypoints is smaller than the old one.
  //    The keypoint vector sizes are not all equal (if set)
  void expandVisualObservationContainersIfNecessary();

  size_t discardUntrackedObservations();

  std::string getComparisonString(const Vertex& other) const;

  inline int64_t getMinTimestampNanoseconds() const;

  // Updates an entry in the observed landmark ids list. This is needed after a
  // landmark has been merged to get rid of the old, deleted id entry and
  // replace it with the new id.
  void updateIdInObservedLandmarkIdList(
      const LandmarkId& old_landmark_id, const LandmarkId& new_landmark_id);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // Used for testing only.
  void addObservedLandmarkId(
      unsigned int frame_idx, const LandmarkId& landmark_id);

  // Utility function to determine the new size of observed_landmark_ids_ if
  // the number of keypoints increase.
  // previous_new_size: new observed_landmark_ids size determined by a
  //                    previously checked keypoint vector size
  // current_new_size:  new observed_landmark_ids size determined by the
  //                    current keypoint vector size
  // old_size:          original observed_landmark_ids size
  // returns:           the new size of observed_landmark_ids
  int determineNewObservedLandmarkIdVectorSize(
      int previous_new_size, int current_new_size, int old_size) const;

  pose_graph::VertexId id_;
  vi_map::MissionId mission_id_;

  // Pose, velocity, IMU biases.
  aslam::Transformation T_M_I_;
  Eigen::Vector3d v_M_;
  Eigen::Vector3d accel_bias_;
  Eigen::Vector3d gyro_bias_;

  // Incoming, outgoing edges.
  pose_graph::EdgeIdSet incoming_edges_;
  pose_graph::EdgeIdSet outgoing_edges_;

  aslam::VisualNFrame::Ptr n_frame_;
  std::vector<LandmarkIdList> observed_landmark_ids_;

  // Landmark storage.
  LandmarkStore landmarks_;

  // VisualFrame resources;
  FrameResourceMap resource_map_;
};

}  // namespace vi_map

#include "./vertex-inl.h"

#endif  // VI_MAP_VERTEX_H_
