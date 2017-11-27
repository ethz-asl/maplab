#include "vi-map/vertex.h"

#include <string>
#include <unordered_set>

#include <glog/logging.h>

#include <aslam-serialization/visual-frame-serialization.h>
#include <aslam/common/hash-id.h>
#include <aslam/common/stl-helpers.h>
#include <maplab-common/aslam-id-proto.h>
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
  common::generateId(&n_frame_id);
  n_frame_->setId(n_frame_id);

  CHECK_EQ(img_points_distorted.cols(), descriptors.cols());
  CHECK_EQ(img_points_distorted.cols(), uncertainties.size());
  CHECK_EQ(
      observed_landmark_ids.size(),
      static_cast<unsigned int>(descriptors.cols()));

  // Fill a single visual frame.
  aslam::VisualFrame::Ptr frame(new aslam::VisualFrame);
  CHECK(n_frame_ != nullptr);

  CHECK(frame_id.isValid());
  frame->setId(frame_id);
  frame->setTimestampNanoseconds(frame_timestamp);

  if (img_points_distorted.cols() != 0) {
    frame->setKeypointMeasurements(img_points_distorted);
    frame->setKeypointMeasurementUncertainties(uncertainties);
    frame->setDescriptors(descriptors);

    if (descriptor_scales.rows() != 0) {
      CHECK_EQ(descriptor_scales.rows(), descriptors.cols());
      frame->setKeypointScales(descriptor_scales);
      CHECK(frame->hasKeypointScales());
    }
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
    : id_(vertex_id),
      mission_id_(mission_id),
      n_frame_(visual_n_frame),
      observed_landmark_ids_(observed_landmark_ids) {
  CHECK(n_frame_ != nullptr);

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

  checkConsistencyOfVisualObservationContainers();

  T_M_I_.setIdentity();
  v_M_.setZero();
  accel_bias_.setZero();
  gyro_bias_.setZero();

  resource_map_.resize(n_frame_->getNumFrames());
}

Vertex::Vertex(const aslam::NCamera::Ptr cameras) {
  CHECK(cameras != nullptr);
  n_frame_.reset(new aslam::VisualNFrame(cameras));
  observed_landmark_ids_.resize(n_frame_->getNumFrames());

  aslam::NFramesId n_frame_id;
  common::generateId(&n_frame_id);
  n_frame_->setId(n_frame_id);

  for (unsigned int i = 0; i < cameras->numCameras(); ++i) {
    int64_t timestamp = 0u;
    aslam::FrameId frame_id;
    common::generateId(&frame_id);
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
    google::protobuf::RepeatedPtrField<common::proto::Id>* proto_landmark_ids =
        proto_n_frame->mutable_frames(i)->mutable_landmark_ids();
    proto_landmark_ids->Reserve(observed_landmark_ids_[i].size());
    for (const LandmarkId& landmark_id : observed_landmark_ids_[i]) {
      landmark_id.serialize(proto_landmark_ids->Add());
    }
  }

  landmarks_.serialize(proto->mutable_landmark_store());

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
  for (int i = 0; i < num_frames; ++i) {
    if (n_frame_->isFrameSet(static_cast<size_t>(i))) {
      aslam::proto::VisualFrame visual_frame = proto.n_visual_frame().frames(i);
      observed_landmark_ids_[i].resize(visual_frame.landmark_ids_size());
      for (int j = 0; j < visual_frame.landmark_ids_size(); ++j) {
        observed_landmark_ids_[i][j].deserialize(visual_frame.landmark_ids(j));
      }
    }
  }

  // Deserialize landmark store.
  CHECK(proto.has_landmark_store());
  getLandmarks().deserialize(proto.landmark_store());

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

void Vertex::expandVisualObservationContainersIfNecessary() {
  CHECK_EQ(n_frame_->getNumFrames(), observed_landmark_ids_.size());
  CHECK_EQ(n_frame_->getNumFrames(), n_frame_->getNumCameras());

  for (unsigned int frame_idx = 0; frame_idx < n_frame_->getNumFrames();
       ++frame_idx) {
    if (n_frame_->isFrameSet(frame_idx)) {
      const aslam::VisualFrame& frame = n_frame_->getFrame(frame_idx);
      int old_size = static_cast<int>(observed_landmark_ids_[frame_idx].size());
      int new_size = old_size;
      if (frame.hasDescriptors()) {
        new_size = determineNewObservedLandmarkIdVectorSize(
            new_size, frame.getDescriptors().cols(), old_size);
      }
      if (frame.hasKeypointMeasurements()) {
        new_size = determineNewObservedLandmarkIdVectorSize(
            new_size, frame.getKeypointMeasurements().cols(), old_size);
      }
      if (frame.hasKeypointMeasurementUncertainties()) {
        new_size = determineNewObservedLandmarkIdVectorSize(
            new_size, frame.getKeypointMeasurementUncertainties().rows(),
            old_size);
      }
      if (frame.hasKeypointOrientations()) {
        new_size = determineNewObservedLandmarkIdVectorSize(
            new_size, frame.getKeypointOrientations().rows(), old_size);
      }
      if (frame.hasKeypointScales()) {
        new_size = determineNewObservedLandmarkIdVectorSize(
            new_size, frame.getKeypointScales().rows(), old_size);
      }
      if (frame.hasKeypointScores()) {
        new_size = determineNewObservedLandmarkIdVectorSize(
            new_size, frame.getKeypointScores().rows(), old_size);
      }
      if (new_size > old_size) {
        VLOG(4) << "Resizing visual observation container from " << old_size
                << " to " << new_size << " for VisualFrame " << frame_idx
                << " of Vertex " << id() << " ...";
        observed_landmark_ids_[frame_idx].resize(new_size);
      }
    }
  }
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
  for (LandmarkIdList& landmark_ids : observed_landmark_ids_) {
    LandmarkIdList::iterator it_to_landmark =
        std::find(landmark_ids.begin(), landmark_ids.end(), old_landmark_id);
    while (it_to_landmark != landmark_ids.end()) {
      *it_to_landmark = new_landmark_id;
      it_to_landmark =
          std::find(it_to_landmark, landmark_ids.end(), old_landmark_id);
    }
  }
}

std::string Vertex::getComparisonString(const Vertex& other) const {
  if (operator==(other)) {
    return "There is no difference between the given vertices!\n";
  }

  std::ostringstream ss;
  ss << "This enumeration of differences may be incomplete!\n";

  if (numFrames() != other.numFrames()) {
    ss << "The amount of frames differs.\n";
  }

  if (observed_landmark_ids_ != other.observed_landmark_ids_) {
    ss << "The observed landmark ids differ.\n";
  }

  // Important: Landmark operator == doesn't cover the observations!
  if (landmarks_ != other.landmarks_) {
    ss << "The store landmarks differ (amount, position).\n";
  }
  if (landmarks_.size() == other.landmarks_.size()) {
    bool observations_differ = false;
    for (size_t i = 0u; i < landmarks_.size(); ++i) {
      if (landmarks_[i].getObservations() !=
          other.landmarks_[i].getObservations()) {
        observations_differ = true;
        break;
      }
    }
    if (observations_differ) {
      ss << "The store landmark observations differ.\n";
    }
  }

  // Epsilon-free comparison intended.
  if (!(get_T_M_I() == other.get_T_M_I())) {
    ss << "The poses differ.\n";
  }

  return ss.str();
}

void Vertex::checkConsistencyOfVisualObservationContainers() const {
  CHECK_EQ(n_frame_->getNumFrames(), observed_landmark_ids_.size());
  CHECK_EQ(n_frame_->getNumFrames(), n_frame_->getNumCameras());
  for (unsigned int frame_idx = 0; frame_idx < n_frame_->getNumFrames();
       ++frame_idx) {
    if (n_frame_->isFrameSet(frame_idx)) {
      const aslam::VisualFrame& frame = n_frame_->getFrame(frame_idx);
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

void Vertex::getFrameObservedLandmarkIds(
    unsigned int frame_idx, LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids);
  landmark_ids->clear();
  *landmark_ids = observed_landmark_ids_[frame_idx];
}

const LandmarkIdList& Vertex::getFrameObservedLandmarkIds(
    unsigned int frame_idx) const {
  CHECK_LT(frame_idx, numFrames());
  CHECK_LT(frame_idx, observed_landmark_ids_.size());
  return observed_landmark_ids_[frame_idx];
}

void Vertex::getAllObservedLandmarkIds(LandmarkIdList* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  for (unsigned int frame_idx = 0u; frame_idx < numFrames(); ++frame_idx) {
    landmark_ids->insert(
        landmark_ids->begin(), observed_landmark_ids_[frame_idx].begin(),
        observed_landmark_ids_[frame_idx].end());
  }
}

void Vertex::getAllObservedLandmarkIds(
    std::vector<LandmarkIdList>* landmark_ids) const {
  CHECK_NOTNULL(landmark_ids)->clear();
  *landmark_ids = observed_landmark_ids_;
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

void Vertex::forEachFrame(
    const std::function<void(
        const unsigned int frame_idx, const aslam::VisualFrame& frame)>& action)
    const {
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

void Vertex::getStoredLandmarkIdList(LandmarkIdList* landmark_id_list) const {
  CHECK_NOTNULL(landmark_id_list)->clear();
  landmark_id_list->reserve(landmarks_.size());
  for (const Landmark& landmark : landmarks_) {
    landmark_id_list->emplace_back(landmark.id());
  }
}

void Vertex::setLandmark_LM_p_fi(
    const vi_map::LandmarkId& landmark_id, const Eigen::Vector3d& LM_p_fi) {
  Eigen::Vector3d I_p_fi = get_T_M_I().inverse() * LM_p_fi;
  getLandmarks().getLandmark(landmark_id).set_p_B(I_p_fi);
}

bool Vertex::hasStoredLandmark(const LandmarkId& landmark_id) const {
  return landmarks_.hasLandmark(landmark_id);
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
}  // namespace vi_map
