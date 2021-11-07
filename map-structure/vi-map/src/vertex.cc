#include "vi-map/vertex.h"

#include <string>
#include <unordered_set>

#include <glog/logging.h>

#include <aslam-serialization/visual-frame-serialization.h>
#include <aslam/common/hash-id.h>
#include <aslam/common/stl-helpers.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/quaternion-math.h>

#include "vi-map/vi_map.pb.h"

namespace vi_map {

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
    const Eigen::Matrix2Xd& img_points_distorted,
    const Eigen::VectorXd& uncertainties,
    const aslam::VisualFrame::DescriptorsT& descriptors,
    const std::vector<LandmarkId>& observed_landmark_ids,
    const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
    int64_t frame_timestamp, const aslam::NCamera::Ptr cameras)
    : Vertex(
          vertex_id, imu_ba_bw, img_points_distorted, uncertainties,
          descriptors, /* descriptor scales = */ Eigen::VectorXd(0, 1),
          observed_landmark_ids, mission_id, frame_id, frame_timestamp,
          cameras) {}

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
    const Eigen::Matrix2Xd& img_points_distorted,
    const Eigen::VectorXd& uncertainties,
    const aslam::VisualFrame::DescriptorsT& descriptors,
    const Eigen::VectorXd& descriptor_scales,
    const std::vector<LandmarkId>& observed_landmark_ids,
    const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
    int64_t frame_timestamp, const aslam::NCamera::Ptr cameras)
    : Vertex(
          vertex_id, imu_ba_bw, img_points_distorted, uncertainties,
          descriptors, /* descriptor scales = */ Eigen::VectorXd(0, 1),
          observed_landmark_ids,
          /*semantic_object_measurements = */ Eigen::Matrix4Xd(4, 0),
          /*semantic_object_uncertainties = */ Eigen::VectorXd(0, 1),
          /*semantic_object_class_ids = */ Eigen::VectorXi(0, 1),
          /*semantic_object_descriptors = */
          Eigen::MatrixXf(1, 0),  // we set a row size here so we don't divide
                                  // by 0 in serialization
          /*observed_semantic_landmark_ids = */ SemanticLandmarkIdList(),
          mission_id, frame_id, frame_timestamp, cameras) {}

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
    const Eigen::Matrix2Xd& img_points_distorted,
    const Eigen::VectorXd& uncertainties,
    const aslam::VisualFrame::DescriptorsT& descriptors,
    const std::vector<LandmarkId>& observed_landmark_ids,
    const Eigen::Matrix4Xd& semantic_object_measurements,
    const Eigen::VectorXd& semantic_object_uncertainties,
    const Eigen::VectorXi& semantic_object_class_ids,
    const aslam::VisualFrame::SemanticObjectDescriptorsT&
        semantic_object_descriptors,
    const std::vector<SemanticLandmarkId>& observed_semantic_landmark_ids,
    const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
    int64_t frame_timestamp, const aslam::NCamera::Ptr cameras)
    : Vertex(
          vertex_id, imu_ba_bw, img_points_distorted, uncertainties,
          descriptors, /* descriptor scales = */ Eigen::VectorXd(0, 1),
          observed_landmark_ids, semantic_object_measurements,
          semantic_object_uncertainties, semantic_object_class_ids,
          semantic_object_descriptors, observed_semantic_landmark_ids,
          mission_id, frame_id, frame_timestamp, cameras) {}

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
    const Eigen::Matrix2Xd& img_points_distorted,
    const Eigen::VectorXd& uncertainties,
    const aslam::VisualFrame::DescriptorsT& descriptors,
    const Eigen::VectorXd& descriptor_scales,
    const std::vector<LandmarkId>& observed_landmark_ids,
    const Eigen::Matrix4Xd& semantic_object_measurements,
    const Eigen::VectorXd& semantic_object_uncertainties,
    const Eigen::VectorXi& semantic_object_class_ids,
    const aslam::VisualFrame::SemanticObjectDescriptorsT&
        semantic_object_descriptors,
    const std::vector<SemanticLandmarkId>& observed_semantic_landmark_ids,
    const vi_map::MissionId& mission_id, const aslam::FrameId& frame_id,
    int64_t frame_timestamp, const aslam::NCamera::Ptr cameras)
    : id_(vertex_id), mission_id_(mission_id) {
  CHECK(cameras != nullptr);
  CHECK_EQ(1u, cameras->numCameras())
      << "This constructor supports "
      << "only a single camera in NCamera object";
  // Our assumptions in this ctor: only a single camera passed in NCamera.
  static constexpr unsigned int kNumOfFrames = 1;
  static constexpr unsigned int kFirstFrameIndex = 0;

  n_frame_.reset(new aslam::VisualNFrame(cameras));
  aslam::NFramesId n_frame_id;
  aslam::generateId(&n_frame_id);
  n_frame_->setId(n_frame_id);

  CHECK_EQ(img_points_distorted.cols(), descriptors.cols());
  CHECK_EQ(img_points_distorted.cols(), uncertainties.size());
  CHECK_EQ(
      observed_landmark_ids.size(),
      static_cast<unsigned int>(descriptors.cols()));

  CHECK_EQ(
      semantic_object_measurements.cols(),
      semantic_object_uncertainties.size());
  CHECK_EQ(
      semantic_object_measurements.cols(), semantic_object_class_ids.size());
  CHECK_EQ(
      semantic_object_measurements.cols(), semantic_object_descriptors.cols());
  CHECK_EQ(
      observed_semantic_landmark_ids.size(),
      static_cast<unsigned int>(semantic_object_descriptors.cols()));

  // Fill a single visual frame.
  aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
  CHECK(n_frame_ != nullptr);

  CHECK(frame_id.isValid());
  frame->setId(frame_id);
  frame->setTimestampNanoseconds(frame_timestamp);
  // create empty channels if necessary, so we use rows instead of col
  if (img_points_distorted.rows() != 0) {
    frame->setKeypointMeasurements(img_points_distorted);
    frame->setKeypointMeasurementUncertainties(uncertainties);
    frame->setDescriptors(descriptors);

    if (descriptor_scales.rows() != 0) {
      CHECK_EQ(descriptor_scales.rows(), descriptors.cols());
      frame->setKeypointScales(descriptor_scales);
      CHECK(frame->hasKeypointScales());
    }
  }
  if (semantic_object_measurements.rows() != 0) {
    frame->setSemanticObjectMeasurements(semantic_object_measurements);
    frame->setSemanticObjectMeasurementUncertainties(
        semantic_object_uncertainties);
    frame->setSemanticObjectClassIds(semantic_object_class_ids);
    frame->setSemanticObjectDescriptors(semantic_object_descriptors);
  }

  frame->setCameraGeometry(cameras->getCameraShared(kFirstFrameIndex));

  n_frame_->setFrame(kFirstFrameIndex, frame);

  observed_landmark_ids_.resize(kNumOfFrames);
  if (observed_landmark_ids.size() > 0u) {
    observed_landmark_ids_[kFirstFrameIndex] = observed_landmark_ids;
    CHECK_EQ(
        observed_landmark_ids_[kFirstFrameIndex].size(),
        frame->getNumKeypointMeasurements());
  }

  observed_semantic_landmark_ids_.resize(kNumOfFrames);
  if (observed_semantic_landmark_ids.size() > 0u) {
    observed_semantic_landmark_ids_[kFirstFrameIndex] =
        observed_semantic_landmark_ids;
    CHECK_EQ(
        observed_semantic_landmark_ids_[kFirstFrameIndex].size(),
        frame->getNumSemanticObjectMeasurements());
  }

  checkConsistencyOfVisualObservationContainers();

  accel_bias_ = imu_ba_bw.head<3>();
  gyro_bias_ = imu_ba_bw.tail<3>();
  v_M_.setZero();

  resource_map_.resize(kNumOfFrames);
}

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
    const aslam::VisualNFrame::Ptr visual_n_frame,
    const std::vector<std::vector<LandmarkId>>& observed_landmark_ids,
    const vi_map::MissionId& mission_id)
    : Vertex(
          vertex_id, imu_ba_bw, visual_n_frame, observed_landmark_ids,
          /* observed_semantic_landmark_ids = */
          std::vector<SemanticLandmarkIdList>(), mission_id) {}

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const Eigen::Matrix<double, 6, 1>& imu_ba_bw,
    const aslam::VisualNFrame::Ptr visual_n_frame,
    const std::vector<std::vector<LandmarkId>>& observed_landmark_ids,
    const std::vector<std::vector<SemanticLandmarkId>>&
        observed_semantic_landmark_ids,
    const vi_map::MissionId& mission_id)
    : id_(vertex_id),
      mission_id_(mission_id),
      n_frame_(visual_n_frame),
      observed_landmark_ids_(observed_landmark_ids),
      observed_semantic_landmark_ids_(observed_semantic_landmark_ids) {
  CHECK(n_frame_ != nullptr);
  // resize the container to avoid index access failure
  size_t num_frames = n_frame_->getNumFrames();
  if (observed_landmark_ids_.size() == 0) {
    observed_landmark_ids_.resize(num_frames);
  }
  if (observed_semantic_landmark_ids_.size() == 0) {
    observed_semantic_landmark_ids_.resize(num_frames);
  }
  checkConsistencyOfVisualObservationContainers();

  accel_bias_ = imu_ba_bw.head<3>();
  gyro_bias_ = imu_ba_bw.tail<3>();
  v_M_.setZero();

  resource_map_.resize(n_frame_->getNumFrames());
}

Vertex::Vertex(
    const pose_graph::VertexId& vertex_id,
    const aslam::VisualNFrame::Ptr visual_n_frame,
    const vi_map::MissionId& mission_id)
    : id_(vertex_id), mission_id_(mission_id), n_frame_(visual_n_frame) {
  CHECK(n_frame_ != nullptr);

  observed_landmark_ids_.resize(n_frame_->getNumFrames());
  for (size_t i = 0; i < observed_landmark_ids_.size(); ++i) {
    size_t num_keypoints = n_frame_->getFrame(i).getNumKeypointMeasurements();
    observed_landmark_ids_[i].resize(num_keypoints);
  }

  observed_semantic_landmark_ids_.resize(n_frame_->getNumFrames());
  for (size_t i = 0; i < observed_semantic_landmark_ids_.size(); ++i) {
    size_t num_measurements =
        n_frame_->getFrame(i).getNumSemanticObjectMeasurements();
    observed_semantic_landmark_ids_[i].resize(num_measurements);
  }

  checkConsistencyOfVisualObservationContainers();

  T_M_I_.setIdentity();
  v_M_.setZero();
  accel_bias_.setZero();
  gyro_bias_.setZero();

  resource_map_.resize(n_frame_->getNumFrames());
}

Vertex::Vertex(const aslam::NCamera::Ptr cameras) {
  aslam::generateId(&id_);

  CHECK(cameras != nullptr);
  n_frame_.reset(new aslam::VisualNFrame(cameras));
  observed_landmark_ids_.resize(n_frame_->getNumFrames());
  observed_semantic_landmark_ids_.resize(n_frame_->getNumFrames());

  aslam::NFramesId n_frame_id;
  aslam::generateId(&n_frame_id);
  n_frame_->setId(n_frame_id);

  for (unsigned int i = 0; i < cameras->numCameras(); ++i) {
    int64_t timestamp = 0u;
    aslam::FrameId frame_id;
    aslam::generateId(&frame_id);
    aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
    frame->setId(frame_id);
    frame->setTimestampNanoseconds(timestamp);

    frame->setCameraGeometry(n_frame_->getNCameraShared()->getCameraShared(i));
    n_frame_->setFrame(i, frame);
  }

  T_M_I_.setIdentity();
  v_M_.setZero();
  accel_bias_.setZero();
  gyro_bias_.setZero();

  resource_map_.resize(n_frame_->getNumFrames());
}

Vertex::Vertex() {
  T_M_I_.setIdentity();
  v_M_.setZero();
  accel_bias_.setZero();
  gyro_bias_.setZero();
}

Vertex* Vertex::cloneWithoutVisualNFrame() const {
  Vertex* cloned_vertex = new Vertex();

  // Copy non-vision related members.
  cloned_vertex->id_ = id_;
  cloned_vertex->mission_id_ = mission_id_;
  cloned_vertex->T_M_I_ = T_M_I_;
  cloned_vertex->v_M_ = v_M_;
  cloned_vertex->accel_bias_ = accel_bias_;
  cloned_vertex->gyro_bias_ = gyro_bias_;
  cloned_vertex->incoming_edges_ = incoming_edges_;
  cloned_vertex->outgoing_edges_ = outgoing_edges_;

  cloned_vertex->resource_map_ = resource_map_;
  cloned_vertex->absolute_6dof_measurements_ = absolute_6dof_measurements_;

  // Check vertex state.
  CHECK(cloned_vertex->id_.isValid());
  CHECK(cloned_vertex->mission_id_.isValid());
  return cloned_vertex;
}

Vertex* Vertex::cloneWithVisualNFrame(
    const aslam::NCamera::Ptr ncamera_for_cloned_vertex) const {
  CHECK(hasVisualNFrame())
      << "This clone function is only applicable if this vertex has a "
      << "VisualNFrame!";

  Vertex* cloned_vertex = new Vertex();

  // Copy non-vision related members.
  cloned_vertex->id_ = id_;
  cloned_vertex->mission_id_ = mission_id_;
  cloned_vertex->T_M_I_ = T_M_I_;
  cloned_vertex->v_M_ = v_M_;
  cloned_vertex->accel_bias_ = accel_bias_;
  cloned_vertex->gyro_bias_ = gyro_bias_;
  cloned_vertex->incoming_edges_ = incoming_edges_;
  cloned_vertex->outgoing_edges_ = outgoing_edges_;

  // Copy vision related members.
  cloned_vertex->observed_landmark_ids_ = observed_landmark_ids_;
  cloned_vertex->landmarks_ = landmarks_;
  cloned_vertex->resource_map_ = resource_map_;

  cloned_vertex->absolute_6dof_measurements_ = absolute_6dof_measurements_;

  // Verify that the camera of the other vertex and the new one are the same.
  const aslam::NCamera& this_ncamera = *CHECK_NOTNULL(getNCameras().get());
  CHECK(ncamera_for_cloned_vertex);
  CHECK(this_ncamera.isEqual(*ncamera_for_cloned_vertex))
      << "Cannot copy this vertex and assign a different camera model to the "
      << "new vertex, because the visual measurements might not be compatible "
      << "anymore!";

  // Create new VisualNFrame based on existing sensor and the VisualNFrame of
  // the other vertex.
  cloned_vertex->n_frame_.reset(
      new aslam::VisualNFrame(ncamera_for_cloned_vertex));
  CHECK(
      ncamera_for_cloned_vertex == cloned_vertex->n_frame_->getNCameraShared());
  cloned_vertex->n_frame_->setId(getVisualNFrame().getId());
  for (unsigned int camera_idx = 0u;
       camera_idx < ncamera_for_cloned_vertex->numCameras(); ++camera_idx) {
    const aslam::VisualFrame& this_frame = getVisualFrame(camera_idx);
    aslam::VisualFrame::Ptr cloned_frame(new aslam::VisualFrame);
    cloned_frame->setId(this_frame.getId());
    cloned_frame->setTimestampNanoseconds(this_frame.getTimestampNanoseconds());
    cloned_frame->setCameraGeometry(
        ncamera_for_cloned_vertex->getCameraShared(camera_idx));

    // Fill in measurements of other visualframe.
    if (this_frame.hasKeypointMeasurements()) {
      cloned_frame->setKeypointMeasurements(
          this_frame.getKeypointMeasurements());
    }
    if (this_frame.hasKeypointMeasurementUncertainties()) {
      cloned_frame->setKeypointMeasurementUncertainties(
          this_frame.getKeypointMeasurementUncertainties());
    }
    if (this_frame.hasKeypointOrientations()) {
      cloned_frame->setKeypointOrientations(
          this_frame.getKeypointOrientations());
    }
    if (this_frame.hasKeypointScores()) {
      cloned_frame->setKeypointScores(this_frame.getKeypointScores());
    }
    if (this_frame.hasKeypointScales()) {
      cloned_frame->setKeypointScales(this_frame.getKeypointScales());
    }
    if (this_frame.hasDescriptors()) {
      cloned_frame->setDescriptors(this_frame.getDescriptors());
    }
    if (this_frame.hasTrackIds()) {
      cloned_frame->setTrackIds(this_frame.getTrackIds());
    }
    if (this_frame.hasRawImage()) {
      cloned_frame->setRawImage(this_frame.getRawImage().clone());
    }
    if (this_frame.hasColorImage()) {
      cloned_frame->setColorImage(this_frame.getColorImage().clone());
    }


    // Verify landmark vs keypoint state.
    size_t num_keypoints = cloned_frame->getNumKeypointMeasurements();
    CHECK_EQ(
        cloned_vertex->observed_landmark_ids_[camera_idx].size(),
        num_keypoints);

    cloned_vertex->n_frame_->setFrame(camera_idx, cloned_frame);
  }

  // Check vertex state.
  CHECK(cloned_vertex->id_.isValid());
  CHECK(cloned_vertex->mission_id_.isValid());
  CHECK_EQ(
      cloned_vertex->observed_landmark_ids_.size(),
      ncamera_for_cloned_vertex->getNumCameras());
  cloned_vertex->checkConsistencyOfVisualObservationContainers();
  CHECK_EQ(
      cloned_vertex->resource_map_.size(),
      cloned_vertex->n_frame_->getNumFrames());

  return cloned_vertex;
}

const pose_graph::VertexId& Vertex::id() const {
  return id_;
}

void Vertex::setId(const pose_graph::VertexId& id) {
  id_ = id;
}

bool Vertex::addIncomingEdge(const pose_graph::EdgeId& edge) {
  incoming_edges_.insert(edge);
  return true;
}

bool Vertex::addOutgoingEdge(const pose_graph::EdgeId& edge) {
  outgoing_edges_.insert(edge);
  return true;
}

void Vertex::getOutgoingEdges(pose_graph::EdgeIdSet* edges) const {
  CHECK_NOTNULL(edges);
  *edges = outgoing_edges_;
}

void Vertex::getIncomingEdges(pose_graph::EdgeIdSet* edges) const {
  CHECK_NOTNULL(edges);
  *edges = incoming_edges_;
}
void Vertex::getAllEdges(pose_graph::EdgeIdSet* edges) const {
  CHECK_NOTNULL(edges);
  *edges = incoming_edges_;
  edges->insert(outgoing_edges_.begin(), outgoing_edges_.end());
}

bool Vertex::hasIncomingEdges() const {
  return !incoming_edges_.empty();
}

bool Vertex::hasOutgoingEdges() const {
  return !outgoing_edges_.empty();
}

size_t Vertex::numOutgoingEdges() const {
  return outgoing_edges_.size();
}

void Vertex::removeIncomingEdge(const pose_graph::EdgeId& edge_id) {
  CHECK_GT(incoming_edges_.count(edge_id), 0u);
  incoming_edges_.erase(edge_id);
}

void Vertex::removeOutgoingEdge(const pose_graph::EdgeId& edge_id) {
  CHECK_GT(outgoing_edges_.count(edge_id), 0u);
  outgoing_edges_.erase(edge_id);
}

void Vertex::serialize(vi_map::proto::ViwlsVertex* proto) const {
  CHECK_NOTNULL(proto);
  proto->Clear();

  CHECK(mission_id_.isValid());
  mission_id_.serialize(proto->mutable_mission_id());

  for (const pose_graph::EdgeId& incoming_edge_id : incoming_edges_) {
    incoming_edge_id.serialize(proto->add_incoming());
  }
  CHECK_EQ(
      incoming_edges_.size(),
      static_cast<unsigned int>(proto->incoming_size()));
  for (const pose_graph::EdgeId& outgoing_edge_id : outgoing_edges_) {
    outgoing_edge_id.serialize(proto->add_outgoing());
  }
  CHECK_EQ(
      outgoing_edges_.size(),
      static_cast<unsigned int>(proto->outgoing_size()));

  common::eigen_proto::serialize(T_M_I_, proto->mutable_t_m_i());
  common::eigen_proto::serialize(v_M_, proto->mutable_v_m());
  common::eigen_proto::serialize(accel_bias_, proto->mutable_accel_bias());
  common::eigen_proto::serialize(gyro_bias_, proto->mutable_gyro_bias());

  CHECK(n_frame_ != nullptr);
  aslam::serialization::serializeVisualNFrame(
      *n_frame_, proto->mutable_n_visual_frame());

  // Serialize observed landmark ids which are saved inside the NFrame.
  const unsigned int num_frames = n_frame_->getNumFrames();
  aslam::proto::VisualNFrame* proto_n_frame = proto->mutable_n_visual_frame();
  CHECK_EQ(static_cast<int>(num_frames), proto_n_frame->frames_size());
  for (unsigned int i = 0u; i < num_frames; ++i) {
    google::protobuf::RepeatedPtrField<aslam::proto::Id>* proto_landmark_ids =
        proto_n_frame->mutable_frames(i)->mutable_landmark_ids();
    proto_landmark_ids->Reserve(observed_landmark_ids_[i].size());
    for (const LandmarkId& landmark_id : observed_landmark_ids_[i]) {
      landmark_id.serialize(proto_landmark_ids->Add());
    }
    // semantic landmarks
    google::protobuf::RepeatedPtrField<aslam::proto::Id>*
        proto_semantic_landmark_ids =
            proto_n_frame->mutable_frames(i)->mutable_semantic_landmark_ids();
    proto_semantic_landmark_ids->Reserve(
        observed_semantic_landmark_ids_[i].size());
    for (const SemanticLandmarkId& landmark_id :
         observed_semantic_landmark_ids_[i]) {
      landmark_id.serialize(proto_semantic_landmark_ids->Add());
    }
  }

  landmarks_.serialize(proto->mutable_landmark_store());
  semantic_landmarks_.serialize(proto->mutable_semantic_landmark_store());

  // Serialize FrameResourceMap.
  for (backend::ResourceTypeToIdsMap resource_type_to_id_map : resource_map_) {
    vi_map::proto::FrameResourceMap* resource_map_proto =
        proto->add_resource_map();
    for (const backend::ResourceTypeToIdsMap::value_type& pair :
         resource_type_to_id_map) {
      vi_map::proto::ResourceTypeMap* resource_type_map_proto =
          resource_map_proto->add_resource_type_map();
      resource_type_map_proto->set_type(static_cast<int>(pair.first));
      for (const backend::ResourceId& resource_id : pair.second) {
        resource_id.serialize(resource_type_map_proto->add_resource_ids());
      }
    }
  }

  // Serialize Absolute6DoFMeasurements
  for (const Absolute6DoFMeasurement& measurement :
       absolute_6dof_measurements_) {
    vi_map::proto::Absolute6DoFMeasurement* absolute_6dof_proto_ptr =
        proto->add_absolute_6dof_measurements();
    measurement.getSensorId().serialize(
        absolute_6dof_proto_ptr->mutable_sensor_id());
    absolute_6dof_proto_ptr->set_timestamp_ns(
        measurement.getTimestampNanoseconds());
    common::eigen_proto::serialize(
        measurement.get_T_G_S(), absolute_6dof_proto_ptr->mutable_t_g_s());
    common::eigen_proto::serialize(
        measurement.get_T_G_S_covariance(),
        absolute_6dof_proto_ptr->mutable_t_g_s_covariance());
  }
}

void Vertex::deserialize(
    const pose_graph::VertexId& vertex_id,

    const vi_map::proto::ViwlsVertex& proto) {
  CHECK(vertex_id.isValid());
  id_ = vertex_id;

  CHECK(proto.has_mission_id());
  mission_id_.deserialize(proto.mission_id());

  CHECK(proto.has_n_visual_frame());
  aslam::serialization::deserializeVisualNFrame(
      proto.n_visual_frame(), &n_frame_);
  const int num_frames = proto.n_visual_frame().frames_size();
  observed_landmark_ids_.resize(num_frames);
  observed_semantic_landmark_ids_.resize(num_frames);
  for (int i = 0; i < num_frames; ++i) {
    if (n_frame_->isFrameSet(static_cast<size_t>(i))) {
      aslam::proto::VisualFrame visual_frame = proto.n_visual_frame().frames(i);
      observed_landmark_ids_[i].resize(visual_frame.landmark_ids_size());
      observed_semantic_landmark_ids_[i].resize(
          visual_frame.semantic_landmark_ids_size());
      for (int j = 0; j < visual_frame.landmark_ids_size(); ++j) {
        observed_landmark_ids_[i][j].deserialize(visual_frame.landmark_ids(j));
      }
      for (int j = 0; j < visual_frame.semantic_landmark_ids_size(); ++j) {
        observed_semantic_landmark_ids_[i][j].deserialize(
            visual_frame.semantic_landmark_ids(j));
      }
    }
  }
  // Deserialize landmark store.
  if (proto.has_landmark_store()) {
    getLandmarks().deserialize(proto.landmark_store());
  }
  if (proto.has_semantic_landmark_store()) {
    getSemanticLandmarks().deserialize(proto.semantic_landmark_store());
  }

  // Deserialize transformation.
  common::eigen_proto::deserialize(proto.t_m_i(), &T_M_I_);

  // Deserialize velocity and IMU biases.
  common::eigen_proto::deserialize(proto.v_m(), &v_M_);
  common::eigen_proto::deserialize(proto.accel_bias(), &accel_bias_);
  common::eigen_proto::deserialize(proto.gyro_bias(), &gyro_bias_);

  // Deserialize incoming edges.
  for (int i = 0; i < proto.incoming_size(); ++i) {
    incoming_edges_.insert(pose_graph::EdgeId(proto.incoming(i)));
  }

  // Deserialize outgoing edges.
  for (int i = 0; i < proto.outgoing_size(); ++i) {
    pose_graph::EdgeId outgoing_edge_id;
    outgoing_edges_.insert(pose_graph::EdgeId(proto.outgoing(i)));
  }

  // Deserialize resource map.
  resource_map_.resize(num_frames);
  for (int i = 0; i < proto.resource_map_size(); ++i) {
    const vi_map::proto::FrameResourceMap& resource_map = proto.resource_map(i);
    for (int j = 0; j < resource_map.resource_type_map_size(); ++j) {
      const vi_map::proto::ResourceTypeMap& resource_type_map =
          resource_map.resource_type_map(j);
      int type = resource_type_map.type();
      for (int k = 0; k < resource_type_map.resource_ids_size(); ++k) {
        backend::ResourceId resource_id;
        resource_id.deserialize(resource_type_map.resource_ids(k));
        resource_map_[i][static_cast<backend::ResourceType>(type)].insert(
            resource_id);
      }
    }
  }

  // Deserialize Absolute6DoFMeasurements
  const size_t num_abs_6dof_measurements =
      proto.absolute_6dof_measurements_size();
  absolute_6dof_measurements_.clear();
  absolute_6dof_measurements_.reserve(num_abs_6dof_measurements);
  for (size_t i = 0; i < num_abs_6dof_measurements; ++i) {
    const vi_map::proto::Absolute6DoFMeasurement& absolute_6dof_proto =
        proto.absolute_6dof_measurements(i);

    aslam::SensorId sensor_id;
    sensor_id.deserialize(absolute_6dof_proto.sensor_id());

    const int64_t timestamp_ns = absolute_6dof_proto.timestamp_ns();

    aslam::Transformation T_G_S;
    common::eigen_proto::deserialize(absolute_6dof_proto.t_g_s(), &T_G_S);

    aslam::TransformationCovariance T_G_S_covariance;
    common::eigen_proto::deserialize(
        absolute_6dof_proto.t_g_s_covariance(), &T_G_S_covariance);

    CHECK(sensor_id.isValid()) << "Absolute 6DoF measurement deserialized from "
                               << "proto has an invalid sensor id!";

    absolute_6dof_measurements_.emplace_back(
        sensor_id, timestamp_ns, T_G_S, T_G_S_covariance);
  }
}

double* Vertex::get_q_M_I_Mutable() {
  return T_M_I_.getRotation().toImplementation().coeffs().data();
}

double* Vertex::get_p_M_I_Mutable() {
  return T_M_I_.getPosition().data();
}

double* Vertex::get_v_M_Mutable() {
  return v_M_.data();
}

double* Vertex::getAccelBiasMutable() {
  return accel_bias_.data();
}

double* Vertex::getGyroBiasMutable() {
  return gyro_bias_.data();
}

const Eigen::Vector3d& Vertex::get_p_M_I() const {
  return T_M_I_.getPosition();
}

const Eigen::Quaterniond& Vertex::get_q_M_I() const {
  return T_M_I_.getRotation().toImplementation();
}

const pose::Transformation& Vertex::get_T_M_I() const {
  return T_M_I_;
}

const Eigen::Vector3d& Vertex::get_v_M() const {
  return v_M_;
}

const Eigen::Vector3d& Vertex::getAccelBias() const {
  return accel_bias_;
}

const Eigen::Vector3d& Vertex::getGyroBias() const {
  return gyro_bias_;
}

void Vertex::set_q_M_I(const Eigen::Quaterniond& q_M_I) {
  CHECK_DOUBLE_EQ(1.0, q_M_I.squaredNorm());
  CHECK_GE(q_M_I.w(), 0.0);
  T_M_I_.getRotation().toImplementation() = q_M_I;
}

void Vertex::set_p_M_I(const Eigen::Vector3d& p_M_I) {
  T_M_I_.getPosition() = p_M_I;
}

void Vertex::set_T_M_I(const pose::Transformation& T_M_I) {
  T_M_I_ = T_M_I;
}

void Vertex::set_v_M(const Eigen::Vector3d& v_M) {
  v_M_ = v_M;
}

void Vertex::setAccelBias(const Eigen::Vector3d& accel_bias) {
  accel_bias_ = accel_bias;
}

void Vertex::setGyroBias(const Eigen::Vector3d& gyro_bias) {
  gyro_bias_ = gyro_bias;
}

bool Vertex::isFrameIndexValid(unsigned int frame_idx) const {
  CHECK(n_frame_ != nullptr);

  bool is_valid = true;
  is_valid &= observed_landmark_ids_.size() == n_frame_->getNumFrames();
  is_valid &= frame_idx < n_frame_->getNumFrames();
  is_valid &= n_frame_->isFrameSet(frame_idx);
  return is_valid;
}

bool Vertex::areFrameAndKeypointIndicesValid(
    unsigned int frame_idx, int keypoint_idx) const {
  CHECK(n_frame_ != nullptr);

  const bool is_frame_idx_valid = isFrameIndexValid(frame_idx);
  LOG_IF(ERROR, !is_frame_idx_valid) << "Invalid frame index: " << frame_idx;
  const bool is_observed_landmarks_ids_size_correct =
      observed_landmark_ids_[frame_idx].size() ==
      static_cast<unsigned int>(
          n_frame_->getFrame(frame_idx).getNumKeypointMeasurements());
  LOG_IF(ERROR, !is_observed_landmarks_ids_size_correct)
      << "Observed landmark IDs vector size is incorrect. ("
      << observed_landmark_ids_[frame_idx].size() << " vs. "
      << n_frame_->getFrame(frame_idx).getNumKeypointMeasurements() << ").";

  const bool is_keypoint_index_valid =
      keypoint_idx <
      static_cast<int>(
          n_frame_->getFrame(frame_idx).getNumKeypointMeasurements());
  LOG_IF(ERROR, !is_keypoint_index_valid)
      << "Keypoint index out of bounds. (" << keypoint_idx << " vs. "
      << n_frame_->getFrame(frame_idx).getNumKeypointMeasurements() << ").";

  return is_frame_idx_valid && is_observed_landmarks_ids_size_correct &&
         is_keypoint_index_valid;
}

bool Vertex::areFrameAndMeasurementIndicesValid(
    unsigned int frame_idx, int measurement_idx) const {
  CHECK(n_frame_ != nullptr);

  const bool is_frame_idx_valid = isFrameIndexValid(frame_idx);
  LOG_IF(ERROR, !is_frame_idx_valid) << "Invalid frame index: " << frame_idx;
  const bool is_observed_semantic_landmarks_ids_size_correct =
      observed_semantic_landmark_ids_[frame_idx].size() ==
      static_cast<unsigned int>(
          n_frame_->getFrame(frame_idx).getNumSemanticObjectMeasurements());
  LOG_IF(ERROR, !is_observed_semantic_landmarks_ids_size_correct)
      << "Observed semantic landmark IDs vector size is incorrect. ("
      << observed_semantic_landmark_ids_[frame_idx].size() << " vs. "
      << n_frame_->getFrame(frame_idx).getNumSemanticObjectMeasurements()
      << ").";

  const bool is_measurement_index_valid =
      measurement_idx <
      static_cast<int>(
          n_frame_->getFrame(frame_idx).getNumSemanticObjectMeasurements());
  LOG_IF(ERROR, !is_measurement_index_valid)
      << "Keypoint index out of bounds. (" << measurement_idx << " vs. "
      << n_frame_->getFrame(frame_idx).getNumSemanticObjectMeasurements()
      << ").";

  return is_frame_idx_valid &&
         is_observed_semantic_landmarks_ids_size_correct &&
         is_measurement_index_valid;
}

const LandmarkId& Vertex::getObservedLandmarkId(
    unsigned int frame_idx, int keypoint_idx) const {
  return observed_landmark_ids_[frame_idx][keypoint_idx];
}

const LandmarkId& Vertex::getObservedLandmarkId(
    const KeypointIdentifier& keypoint_id) const {
  return getObservedLandmarkId(
      keypoint_id.frame_id.frame_index, keypoint_id.keypoint_index);
}

void Vertex::setObservedLandmarkId(
    const KeypointIdentifier& keypoint_id, const LandmarkId& landmark_id) {
  setObservedLandmarkId(
      keypoint_id.frame_id.frame_index, keypoint_id.keypoint_index,
      landmark_id);
}

void Vertex::setObservedLandmarkId(
    unsigned int frame_idx, int keypoint_idx, const LandmarkId& landmark_id) {
  CHECK(areFrameAndKeypointIndicesValid(frame_idx, keypoint_idx));
  observed_landmark_ids_[frame_idx][keypoint_idx] = landmark_id;
}

void Vertex::addObservedLandmarkId(
    unsigned int frame_idx, const LandmarkId& landmark_id) {
  CHECK(isFrameIndexValid(frame_idx));
  observed_landmark_ids_[frame_idx].push_back(landmark_id);
}

size_t Vertex::observedLandmarkIdsSize(unsigned int frame_idx) const {
  CHECK(isFrameIndexValid(frame_idx));
  return observed_landmark_ids_[frame_idx].size();
}

int Vertex::numValidObservedLandmarkIds(unsigned int frame_idx) const {
  CHECK(isFrameIndexValid(frame_idx));

  vi_map::LandmarkIdSet landmark_ids;
  landmark_ids.rehash(observed_landmark_ids_[frame_idx].size());
  for (unsigned int i = 0; i < observed_landmark_ids_[frame_idx].size(); ++i) {
    if (observed_landmark_ids_[frame_idx][i].isValid()) {
      landmark_ids.emplace(observed_landmark_ids_[frame_idx][i]);
    }
  }
  return landmark_ids.size();
}

const SemanticLandmarkId& Vertex::getObservedSemanticLandmarkId(
    unsigned int frame_idx, int measurement_idx) const {
  return observed_semantic_landmark_ids_[frame_idx][measurement_idx];
}

const SemanticLandmarkId& Vertex::getObservedSemanticLandmarkId(
    const SemanticObjectIdentifier& object_id) const {
  return getObservedSemanticLandmarkId(
      object_id.frame_id.frame_index, object_id.measurement_index);
}

void Vertex::setObservedSemanticLandmarkId(
    const SemanticObjectIdentifier& object_id,
    const SemanticLandmarkId& landmark_id) {
  setObservedSemanticLandmarkId(
      object_id.frame_id.frame_index, object_id.measurement_index, landmark_id);
}

void Vertex::setObservedSemanticLandmarkId(
    unsigned int frame_idx, int measurement_idx,
    const SemanticLandmarkId& landmark_id) {
  CHECK(areFrameAndMeasurementIndicesValid(frame_idx, measurement_idx));
  observed_semantic_landmark_ids_[frame_idx][measurement_idx] = landmark_id;
}

void Vertex::addObservedSemanticLandmarkId(
    unsigned int frame_idx, const SemanticLandmarkId& id) {
  CHECK(isFrameIndexValid(frame_idx));
  observed_semantic_landmark_ids_[frame_idx].push_back(id);
}

void Vertex::setObservedSemanticLandmarkIds(
    unsigned int frame_idx, const std::vector<int>& measurement_idicies,
    const SemanticLandmarkIdList& ids) {
  CHECK_EQ(measurement_idicies.size(), ids.size());
  observed_semantic_landmark_ids_[frame_idx].resize(ids.size());
  for (size_t i = 0; i < ids.size(); i++) {
    observed_semantic_landmark_ids_[frame_idx][measurement_idicies[i]] = ids[i];
  }
}

size_t Vertex::observedSemanticLandmarkIdsSize(unsigned int frame_idx) const {
  CHECK(isFrameIndexValid(frame_idx));
  return observed_semantic_landmark_ids_[frame_idx].size();
}

int Vertex::numValidObservedSemanticLandmarkIds(unsigned int frame_idx) const {
  CHECK(isFrameIndexValid(frame_idx));

  vi_map::SemanticLandmarkIdSet semantic_landmark_ids;
  semantic_landmark_ids.rehash(
      observed_semantic_landmark_ids_[frame_idx].size());
  for (unsigned int i = 0;
       i < observed_semantic_landmark_ids_[frame_idx].size(); ++i) {
    if (observed_semantic_landmark_ids_[frame_idx][i].isValid()) {
      semantic_landmark_ids.emplace(
          observed_semantic_landmark_ids_[frame_idx][i]);
    }
  }
  return semantic_landmark_ids.size();
}

int Vertex::determineNewObservedLandmarkIdVectorSize(
    int previous_new_size, int current_new_size, int old_size) const {
  // If the new size has been determined before by another keypoint vector, it
  // should have be equal to the current new size.
  CHECK_GE(current_new_size, old_size);
  CHECK(
      !((previous_new_size != old_size) &&
        (previous_new_size != current_new_size)));
  if (current_new_size > old_size) {
    return current_new_size;
  } else {
    return old_size;
  }
}

void Vertex::removeObservedLandmarkIdList(const LandmarkId& landmark_id) {
  CHECK(landmark_id.isValid());
  LandmarkId invalid_id;
  invalid_id.setInvalid();
  updateIdInObservedLandmarkIdList(landmark_id, invalid_id);
}

size_t Vertex::discardUntrackedObservations() {
  size_t num_removed = 0u;
  const size_t num_frames = numFrames();
  for (size_t i = 0u; i < num_frames; ++i) {
    const size_t original_count =
        getVisualFrame(i).getNumKeypointMeasurements();
    std::vector<size_t> discarded_indices;
    getVisualFrame(i).discardUntrackedObservations(&discarded_indices);
    aslam::common::stl_helpers::eraseIndicesFromContainer(
        discarded_indices, original_count, &observed_landmark_ids_[i]);
    num_removed += discarded_indices.size();
  }
  return num_removed;
}

void Vertex::updateIdInObservedLandmarkIdList(
    const LandmarkId& old_landmark_id, const LandmarkId& new_landmark_id) {
  CHECK(old_landmark_id.isValid());
  if (old_landmark_id == new_landmark_id) {
    return;
  }
  for (LandmarkIdList& landmark_ids : observed_landmark_ids_) {
    LandmarkIdList::iterator it_to_landmark =
        std::find(landmark_ids.begin(), landmark_ids.end(), old_landmark_id);
    while (it_to_landmark != landmark_ids.end()) {
      *it_to_landmark = new_landmark_id;
      it_to_landmark =
          std::find(++it_to_landmark, landmark_ids.end(), old_landmark_id);
    }
  }
}

void Vertex::updateIdInObservedSemanticLandmarkIdList(
    const SemanticLandmarkId& old_landmark_id,
    const SemanticLandmarkId& new_landmark_id) {
  for (SemanticLandmarkIdList& landmark_ids : observed_semantic_landmark_ids_) {
    SemanticLandmarkIdList::iterator it_to_landmark =
        std::find(landmark_ids.begin(), landmark_ids.end(), old_landmark_id);
    while (it_to_landmark != landmark_ids.end()) {
      *it_to_landmark = new_landmark_id;
      it_to_landmark =
          std::find(it_to_landmark, landmark_ids.end(), old_landmark_id);
    }
  }
}

// TODO(jkuo): not sure if it is backward compatible if add semantic related
// checks
// need to check how the size are initialized
void Vertex::checkConsistencyOfVisualObservationContainers() const {
  CHECK_EQ(n_frame_->getNumFrames(), observed_landmark_ids_.size());
  CHECK_EQ(n_frame_->getNumFrames(), n_frame_->getNumCameras());
  for (unsigned int frame_idx = 0; frame_idx < n_frame_->getNumFrames();
       ++frame_idx) {
    if (n_frame_->isFrameSet(frame_idx)) {
      const aslam::VisualFrame& frame = n_frame_->getFrame(frame_idx);
      // keypoints
      if (frame.hasKeypointMeasurements()) {
        CHECK_EQ(
            frame.getKeypointMeasurements().cols(),
            static_cast<int>(observed_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasDescriptors()) {
        CHECK_EQ(
            frame.getDescriptors().cols(),
            static_cast<int>(observed_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasKeypointMeasurementUncertainties()) {
        CHECK_EQ(
            frame.getKeypointMeasurementUncertainties().rows(),
            static_cast<int>(observed_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasKeypointOrientations()) {
        CHECK_EQ(
            frame.getKeypointOrientations().rows(),
            static_cast<int>(observed_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasKeypointScales()) {
        CHECK_EQ(
            frame.getKeypointScales().rows(),
            static_cast<int>(observed_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasKeypointScores()) {
        CHECK_EQ(
            frame.getKeypointScores().rows(),
            static_cast<int>(observed_landmark_ids_[frame_idx].size()));
      }
      // semantic measurements
      if (frame.hasSemanticObjectMeasurements()) {
        CHECK_EQ(
            frame.getSemanticObjectMeasurements().cols(),
            static_cast<int>(
                observed_semantic_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasSemanticObjectMeasurementUncertainties()) {
        CHECK_EQ(
            frame.getSemanticObjectMeasurementUncertainties().rows(),
            static_cast<int>(
                observed_semantic_landmark_ids_[frame_idx].size()));
      }
      // we check with rows instead of cols because empty vector has col of 1
      if (frame.hasSemanticObjectClassIds()) {
        CHECK_EQ(
            frame.getSemanticObjectClassIds().rows(),
            static_cast<int>(
                observed_semantic_landmark_ids_[frame_idx].size()));
      }
      if (frame.hasSemanticObjectDescriptors()) {
        CHECK_EQ(
            frame.getSemanticObjectDescriptors().cols(),
            static_cast<int>(
                observed_semantic_landmark_ids_[frame_idx].size()));
      }
    }
  }
}

void Vertex::setFrameAndLandmarkObservations(
    aslam::VisualNFrame::Ptr visual_n_frame,
    const std::vector<std::vector<LandmarkId>>& img_landmarks) {
  CHECK(visual_n_frame != nullptr);
  n_frame_ = visual_n_frame;
  for (unsigned int frame_idx = 0u; frame_idx < observed_landmark_ids_.size();
       ++frame_idx) {
    // We want to make sure nobody calls this on a populated vertex since in
    // that situation we would need to first properly clean up all the
    // references for the landmarks.
    // This method is intended to be called on an empty vertex only.
    CHECK(observed_landmark_ids_[frame_idx].empty())
        << "Tried to reset the visual observation of a vertex that already "
        << "contained data.";
  }
  observed_landmark_ids_ = img_landmarks;

  checkConsistencyOfVisualObservationContainers();
}

void Vertex::resetObservedLandmarkIdsToInvalid() {
  CHECK(n_frame_);
  CHECK_EQ(observed_landmark_ids_.size(), n_frame_->getNumFrames());
  LandmarkId invalid_landmark_id;
  invalid_landmark_id.setInvalid();
  for (unsigned int frame_idx = 0u; frame_idx < n_frame_->getNumFrames();
       ++frame_idx) {
    observed_landmark_ids_[frame_idx].resize(
        n_frame_->getFrame(frame_idx).getNumKeypointMeasurements(),
        invalid_landmark_id);
  }
  checkConsistencyOfVisualObservationContainers();
}

int Vertex::numValidObservedLandmarkIdsInAllFrames() const {
  int valid_observed_landmark_id_count = 0;
  for (unsigned int i = 0; i < numFrames(); ++i) {
    valid_observed_landmark_id_count += numValidObservedLandmarkIds(i);
  }
  return valid_observed_landmark_id_count;
}

int Vertex::numValidObservedSemanticLandmarkIdsInAllFrames() const {
  int valid_observed_landmark_id_count = 0;
  for (unsigned int i = 0; i < numFrames(); ++i) {
    valid_observed_landmark_id_count += numValidObservedSemanticLandmarkIds(i);
  }
  return valid_observed_landmark_id_count;
}

void Vertex::getFrameObservedLandmarkIds(
    unsigned int frame_idx, LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids);
  landmark_ids->clear();
  *landmark_ids = observed_landmark_ids_[frame_idx];
}

void Vertex::getFrameObservedSemanticLandmarkIds(
    unsigned int frame_idx, SemanticLandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids);
  landmark_ids->clear();
  *landmark_ids = observed_semantic_landmark_ids_[frame_idx];
}

const LandmarkIdList& Vertex::getFrameObservedLandmarkIds(
    unsigned int frame_idx) const {
  CHECK_LT(frame_idx, numFrames());
  CHECK_LT(frame_idx, observed_landmark_ids_.size());
  return observed_landmark_ids_[frame_idx];
}

const SemanticLandmarkIdList& Vertex::getFrameObservedSemanticLandmarkIds(
    unsigned int frame_idx) const {
  CHECK_LT(frame_idx, numFrames());
  CHECK_LT(frame_idx, observed_semantic_landmark_ids_.size());
  return observed_semantic_landmark_ids_[frame_idx];
}

void Vertex::getAllObservedLandmarkIds(LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  for (unsigned int frame_idx = 0u; frame_idx < numFrames(); ++frame_idx) {
    landmark_ids->insert(
        landmark_ids->begin(), observed_landmark_ids_[frame_idx].begin(),
        observed_landmark_ids_[frame_idx].end());
  }
}

void Vertex::getAllObservedSemanticLandmarkIds(
    SemanticLandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  for (unsigned int frame_idx = 0u; frame_idx < numFrames(); ++frame_idx) {
    landmark_ids->insert(
        landmark_ids->begin(),
        observed_semantic_landmark_ids_[frame_idx].begin(),
        observed_semantic_landmark_ids_[frame_idx].end());
  }
}

void Vertex::getAllObservedLandmarkIds(
    std::vector<LandmarkIdList>* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  *landmark_ids = observed_landmark_ids_;
}

size_t Vertex::getNumLandmarkObservations(const LandmarkId& landmark_id) const {
  CHECK(landmark_id.isValid());
  size_t num_observations = 0u;
  for (const LandmarkIdList& landmark_ids : observed_landmark_ids_) {
    LandmarkIdList::const_iterator it_to_landmark =
        std::find(landmark_ids.cbegin(), landmark_ids.cend(), landmark_id);
    while (it_to_landmark != landmark_ids.cend()) {
      ++num_observations;
      it_to_landmark =
          std::find(++it_to_landmark, landmark_ids.cend(), landmark_id);
    }
  }
  return num_observations;
}

void Vertex::getAllObservedSemanticLandmarkIds(
    std::vector<SemanticLandmarkIdList>* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  *landmark_ids = observed_semantic_landmark_ids_;
}

void Vertex::forEachKeypoint(
    const std::function<void(const KeypointIdentifier&)>& action) const {
  KeypointIdentifier keypoint_identifier;
  keypoint_identifier.frame_id.vertex_id = id();
  for (size_t frame_i = 0u; frame_i < numFrames(); ++frame_i) {
    keypoint_identifier.frame_id.frame_index = frame_i;
    const size_t num_keypoints =
        getVisualFrame(frame_i).getNumKeypointMeasurements();
    for (size_t keypoint_i = 0u; keypoint_i < num_keypoints; ++keypoint_i) {
      keypoint_identifier.keypoint_index = keypoint_i;
      action(keypoint_identifier);
    }
  }
}

void Vertex::forEachFrame(const std::function<void(
                              const unsigned int frame_idx,
                              const aslam::VisualFrame& frame)>& action) const {
  const size_t num_frames = numFrames();
  for (size_t frame_i = 0u; frame_i < num_frames; ++frame_i) {
    const aslam::VisualFrame& frame = getVisualFrame(frame_i);
    action(frame_i, frame);
  }
}

pose::Position3D Vertex::getLandmark_p_LM_fi(
    const LandmarkId& landmark_id) const {
  const Eigen::Vector3d& p_I_fi =
      getLandmarks().getLandmark(landmark_id).get_p_B();

  return get_T_M_I() * p_I_fi;
}

pose::Position3D Vertex::getSemanticLandmark_p_LM_fi(
    const SemanticLandmarkId& id) const {
  const Eigen::Vector3d& p_I_fi =
      getSemanticLandmarks().getSemanticLandmark(id).get_p_B();

  return get_T_M_I() * p_I_fi;
}

void Vertex::getStoredLandmarkIdList(LandmarkIdList* landmark_id_list) const {
  CHECK_NOTNULL(landmark_id_list)->clear();
  landmark_id_list->reserve(landmarks_.size());
  for (const Landmark& landmark : landmarks_) {
    landmark_id_list->emplace_back(landmark.id());
  }
}

void Vertex::getStoredSemanticLandmarkIdList(
    SemanticLandmarkIdList* semantic_landmark_id_list) const {
  CHECK_NOTNULL(semantic_landmark_id_list)->clear();
  semantic_landmark_id_list->reserve(semantic_landmarks_.size());
  for (const SemanticLandmark& landmark : semantic_landmarks_) {
    semantic_landmark_id_list->emplace_back(landmark.id());
  }
}

void Vertex::setLandmark_LM_p_fi(
    const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& LM_p_fi) {
  Eigen::Vector3d I_p_fi = get_T_M_I().inverse() * LM_p_fi;
  getLandmarks().getLandmark(landmark_id).set_p_B(I_p_fi);
}

void Vertex::setSemanticLandmark_LM_p_fi(
    const vi_map::SemanticLandmarkId& landmark_id,
    const Eigen::Vector3d& LM_p_fi) {
  Eigen::Vector3d I_p_fi = get_T_M_I().inverse() * LM_p_fi;
  getSemanticLandmarks().getSemanticLandmark(landmark_id).set_p_B(I_p_fi);
}

bool Vertex::hasStoredLandmark(const LandmarkId& landmark_id) const {
  return landmarks_.hasLandmark(landmark_id);
}

bool Vertex::hasStoredSemanticLandmark(
    const SemanticLandmarkId& semantic_landmark_id) const {
  return semantic_landmarks_.hasSemanticLandmark(semantic_landmark_id);
}

bool Vertex::hasFrameResourceOfType(
    const unsigned int frame_idx,
    const backend::ResourceType& resource_type) const {
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  const backend::ResourceTypeToIdsMap::const_iterator it =
      resource_map_.at(frame_idx).find(resource_type);
  return it != resource_map_.at(frame_idx).end() && !it->second.empty();
}

bool Vertex::hasFrameResourceWithId(
    const unsigned int frame_idx,
    const backend::ResourceId& resource_id) const {
  CHECK(resource_id.isValid());
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  for (const backend::ResourceTypeToIdsMap::value_type& resource_type_vector :
       resource_map_[frame_idx]) {
    for (const backend::ResourceId& res_id : resource_type_vector.second) {
      if (res_id == resource_id) {
        return true;
      }
    }
  }
  return false;
}

void Vertex::getFrameResourceIdsOfType(
    const unsigned int frame_idx, const backend::ResourceType& resource_type,
    backend::ResourceIdSet* resource_ids) const {
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK(n_frame_);
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  backend::ResourceTypeToIdsMap::const_iterator res_it =
      resource_map_[frame_idx].find(resource_type);
  if (res_it != resource_map_[frame_idx].end()) {
    *resource_ids = res_it->second;
  }
}

size_t Vertex::getNumFrameResourcesOfType(
    const unsigned int frame_idx,
    const backend::ResourceType& resource_type) const {
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  backend::ResourceTypeToIdsMap::const_iterator res_it =
      resource_map_[frame_idx].find(resource_type);
  if (res_it != resource_map_[frame_idx].end()) {
    return res_it->second.size();
  }
  return 0u;
}

void Vertex::addFrameResourceIdOfType(
    const unsigned int frame_idx, const backend::ResourceType& resource_type,
    const backend::ResourceId& resource_id) {
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  resource_map_[frame_idx][resource_type].insert(resource_id);
}

void Vertex::deleteAllFrameResourceInfo() {
  resource_map_.clear();
  resource_map_.resize(n_frame_->getNumFrames());
}

void Vertex::deleteAllFrameResourceInfo(const unsigned int frame_idx) {
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  resource_map_[frame_idx].clear();
}

void Vertex::deleteFrameResourceIdsOfType(
    const unsigned int frame_idx, const backend::ResourceType& resource_type) {
  CHECK_LT(frame_idx, getVisualNFrame().getNumFrames());
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  resource_map_[frame_idx][resource_type].clear();
}

const Vertex::FrameResourceMap& Vertex::getFrameResourceMap() const {
  CHECK_EQ(resource_map_.size(), n_frame_->getNumFrames());
  return resource_map_;
}

void Vertex::setFrameResourceMap(const FrameResourceMap& frame_resource_map) {
  resource_map_ = frame_resource_map;
}

size_t Vertex::getNumAbsolute6DoFMeasurements() const {
  return absolute_6dof_measurements_.size();
}

bool Vertex::hasAbsolute6DoFMeasurements() const {
  return !absolute_6dof_measurements_.empty();
}

const std::vector<Absolute6DoFMeasurement>&
Vertex::getAbsolute6DoFMeasurements() const {
  return absolute_6dof_measurements_;
}

std::vector<Absolute6DoFMeasurement>& Vertex::getAbsolute6DoFMeasurements() {
  return absolute_6dof_measurements_;
}

void Vertex::addAbsolute6DoFMeasurement(
    const Absolute6DoFMeasurement& measurement) {
  CHECK(measurement.getTimestampNanoseconds() == getMinTimestampNanoseconds())
      << "Before adding the Absolute6DoFMeasurement to a vertex, make sure "
      << "it corresponds exactly to the vertex time or use IMU "
      << "integration to adapt it accordingly!";
  CHECK(measurement.isValid());
  absolute_6dof_measurements_.push_back(measurement);
}
}  // namespace vi_map
