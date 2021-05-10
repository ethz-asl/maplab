#include "online-map-builders/stream-map-builder.h"

#include <aslam/common/stl-helpers.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <vi-map-helpers/vi-map-manipulation.h>
#include <vi-map-helpers/vi-map-queries.h>
#include <vi-map/check-map-consistency.h>
#include <vi-map/sensor-utils.h>
#include <vi-map/vi-map.h>
#include <vio-common/map-update.h>
#include <vio-common/vio-types.h>

DEFINE_bool(
    map_builder_save_image_as_resources, false,
    "Store the images associated with the visual frames to the map resource "
    "folder.");

DEFINE_bool(
    map_builder_save_point_clouds_as_resources, false,
    "Store the point clouds associated with a lidar sensor to the map resource "
    "folder.");

DEFINE_string(
    map_builder_save_point_clouds_as_range_image_camera_id, "",
    "Camera id of the depth camera used to convert lidar point clouds to range "
    "images "
    "maps.");

DEFINE_bool(
    map_builder_save_point_clouds_as_range_image_including_intensity_image,
    true,
    "If enabled, the color or intensity information in the point cloud is used "
    "to create a corresponding intensity/color for each point cloud in "
    "addition to the range image.");

DEFINE_bool(
    map_builder_save_point_cloud_maps_as_resources, true,
    "Store the point cloud (sub-)maps associated with an external source to "
    "the map resource folder.");

DEFINE_bool(
    map_builder_visualize_lidar_depth_maps_in_ocv_window, false,
    "If enabled, opencv windows with the result of the lidar scan to lidar "
    "depth map conversion will be opened.");

namespace online_map_builders {

const vi_map::VIMap* StreamMapBuilder::constMap() const {
  return map_;
}

pose_graph::VertexId StreamMapBuilder::getRootVertexId() const {
  CHECK(mission_id_.isValid());
  return map_->getMission(mission_id_).getRootVertexId();
}

pose_graph::VertexId StreamMapBuilder::getLastVertexId() const {
  CHECK(last_vertex_.isValid());
  return last_vertex_;
}

StreamMapBuilder::StreamMapBuilder(
    const vi_map::SensorManager& sensor_manager, vi_map::VIMap* map)
    : map_(CHECK_NOTNULL(map)),
      oldest_vertex_timestamp_ns_(aslam::time::getInvalidTime()),
      newest_vertex_timestamp_ns_(aslam::time::getInvalidTime()),
      queries_(*map),
      manipulation_(map),
      mission_id_(aslam::createRandomId<vi_map::MissionId>()),
      found_wheel_odometry_origin_(false),
      is_first_baseframe_estimate_processed_(false) {
  // Clone sensor manager into the new map.
  map_->getSensorManager().merge(sensor_manager);

  // Add new mission.
  map_->addNewMissionWithBaseframe(
      mission_id_, aslam::Transformation(),
      Eigen::Matrix<double, 6, 6>::Identity(),
      vi_map::Mission::BackBone::kViwls);

  // Associate sensors to the newly created mission.
  aslam::SensorIdSet sensor_ids;
  sensor_manager.getAllSensorIds(&sensor_ids);
  map_->associateMissionSensors(sensor_ids, mission_id_);

  // Retrieve depth camera id from flags to later convert lidar point clouds to
  // depth maps.
  if (FLAGS_map_builder_save_point_clouds_as_resources &&
      !FLAGS_map_builder_save_point_clouds_as_range_image_camera_id.empty()) {
    lidar_depth_camera_id_.fromHexString(
        FLAGS_map_builder_save_point_clouds_as_range_image_camera_id);
    if (!lidar_depth_camera_id_.isValid()) {
      LOG(ERROR)
          << "[StreamMapBuilder] The depth camera id ("
          << FLAGS_map_builder_save_point_clouds_as_range_image_camera_id
          << ") provided to project the lidar point clouds into depth maps is "
          << "not valid! Point clouds will not be projected into depth maps.";
    } else if (!map_->getSensorManager().hasSensor(lidar_depth_camera_id_)) {
      LOG(ERROR)
          << "[StreamMapBuilder] The depth camera id ("
          << lidar_depth_camera_id_
          << ") provided to project the lidar point clouds into depth maps is "
          << "not in the sensor manager! Point clouds will not be projected "
             "into depth maps.";
    } else {
      VLOG(1) << "[StreamMapBuilder] Using depth camera "
              << lidar_depth_camera_id_
              << " to project lidar scans into depth maps.";

      aslam::NCamera::Ptr lidar_depth_camera_sensor_ncamera_ptr =
          map_->getSensorManager().getSensorPtr<aslam::NCamera>(
              lidar_depth_camera_id_);
      CHECK(lidar_depth_camera_sensor_ncamera_ptr);
      CHECK_EQ(lidar_depth_camera_sensor_ncamera_ptr->numCameras(), 1u);
      lidar_depth_camera_sensor_ =
          lidar_depth_camera_sensor_ncamera_ptr->getCameraShared(0u);
      CHECK(lidar_depth_camera_sensor_);
      const aslam::Transformation& T_C_lidar_Cn_lidar =
          lidar_depth_camera_sensor_ncamera_ptr->get_T_C_B(0u);

      const aslam::Transformation& T_B_Cn_lidar =
          map_->getSensorManager().getSensor_T_B_S(lidar_depth_camera_id_);
      CHECK(map_->getMission(mission_id_).hasLidar())
          << "[StreamMapBuilder] Mission " << mission_id_
          << " does not have a lidar sensor and therefore cannot attach lidar "
          << "measurements!";
      const aslam::SensorId& lidar_sensor_id =
          map_->getMission(mission_id_).getLidarId();
      CHECK(map_->getSensorManager().hasSensor(lidar_sensor_id))
          << "[StreamMapBuilder] Mission " << mission_id_
          << " has a lidar id associated (" << lidar_sensor_id
          << ") but this lidar does not exist in the sensor manager!";

      const aslam::Transformation& T_B_S_lidar =
          map_->getSensorManager().getSensor_T_B_S(lidar_sensor_id);
      T_C_lidar_S_lidar_ =
          T_C_lidar_Cn_lidar * T_B_Cn_lidar.inverse() * T_B_S_lidar;
    }
  }

  // Initialize wheel odometry origin frame tracking
  T_Ow_Btm1_.setIdentity();
}

void StreamMapBuilder::updateMapDependentData() {
  // Update status of wheel odometry edge attachement we either:
  //   - reset to the initial condition OR
  //   - if by luck the last vertex we attached a relative wheel odometry
  //     message to is still in the submap we continue from there
  if (!map_->hasVertex(Btm1_vertex_id_)) {
    Btm1_vertex_id_.setInvalid();
    T_Ow_Btm1_.setIdentity();
    found_wheel_odometry_origin_ = false;

    // If there is no root vertex this will initialize to an invalid id which
    // is fine since then the whole stream map builder will go through the
    // default initialization procedure and initialize this again to the
    // actual root vertex
    vertex_processing_wheel_odometry_id_ = getRootVertexId();
    done_current_vertex_wheel_odometry_ = false;
  } else {
    vertex_processing_wheel_odometry_id_ = Btm1_vertex_id_;
    done_current_vertex_wheel_odometry_ = true;
  }

  // Update first and last vertex information
  pose_graph::VertexIdList vertex_ids;
  map_->getAllVertexIdsInMissionAlongGraph(mission_id_, &vertex_ids);

  if (vertex_ids.empty()) {
    // Nothing to do.
    return;
  }

  const vi_map::Vertex& first_vertex = map_->getVertex(vertex_ids.front());
  const vi_map::Vertex& last_vertex = map_->getVertex(vertex_ids.back());
  newest_vertex_timestamp_ns_.store(last_vertex.getMinTimestampNanoseconds());
  oldest_vertex_timestamp_ns_.store(first_vertex.getMinTimestampNanoseconds());
}

void StreamMapBuilder::apply(const vio::MapUpdate& update) {
  CHECK(update.check());
  constexpr bool kDeepCopyNFrame = true;
  apply(update, kDeepCopyNFrame);
}

void StreamMapBuilder::apply(
    const vio::MapUpdate& update, bool deep_copy_nframe) {
  CHECK(update.check());
  std::shared_ptr<aslam::VisualNFrame> nframe_to_insert;
  if (deep_copy_nframe) {
    const aslam::VisualNFrame& nframe_original = *update.keyframe->nframe;
    nframe_to_insert = aligned_shared<aslam::VisualNFrame>(
        nframe_original.getId(), nframe_original.getNumCameras());
    *nframe_to_insert = *update.keyframe->nframe;
  } else {
    nframe_to_insert = update.keyframe->nframe;
  }
  CHECK(nframe_to_insert);

  // Replace the NCamera pointer stored in the VisualNFrame with the correct
  // pointer from the sensor manager of the map. The original pointer is
  // pointing to an identical camera that is owned by the sensor manager of the
  // maplab node.
  const aslam::SensorId& ncamera_id = nframe_to_insert->getNCamera().getId();
  nframe_to_insert->setNCameras(
      map_->getSensorManager().getSensorPtr<aslam::NCamera>(ncamera_id));

  if (!last_vertex_.isValid()) {
    oldest_vertex_timestamp_ns_.store(
        nframe_to_insert->getMinTimestampNanoseconds());
    addRootViwlsVertex(nframe_to_insert, update.vinode);
    vertex_processing_wheel_odometry_id_ = getRootVertexId();
    done_current_vertex_wheel_odometry_ = false;
  } else {
    CHECK(mission_id_.isValid());
    newest_vertex_timestamp_ns_.store(
        nframe_to_insert->getMinTimestampNanoseconds());
    if (!is_first_baseframe_estimate_processed_ &&
        (update.localization_state == common::LocalizationState::kLocalized ||
         update.localization_state ==
             common::LocalizationState::kMapTracking)) {
      T_M0_G_ = update.T_G_M.inverse();
      map_->getMissionBaseFrameForMission(mission_id_).set_T_G_M(update.T_G_M);
      is_first_baseframe_estimate_processed_ = true;
    }

    addViwlsVertexAndEdge(
        nframe_to_insert, update.vinode, update.T_G_M, update.imu_timestamps,
        update.imu_measurements);
  }
  notifyBuffers();
}

void StreamMapBuilder::addRootViwlsVertex(
    const aslam::VisualNFrame::Ptr& nframe,
    const vio::ViNodeState& vinode_state) {
  CHECK(nframe);
  CHECK(!last_vertex_.isValid()) << "Root vertex has already been set!";

  const aslam::Transformation T_G_M;
  pose_graph::VertexId root_vertex_id =
      addViwlsVertex(nframe, vinode_state, T_G_M);
  CHECK(root_vertex_id.isValid());

  CHECK(!constMap()->getMission(mission_id_).getRootVertexId().isValid())
      << "Root vertex has already been set for this mission.";
  map_->getMission(mission_id_).setRootVertexId(root_vertex_id);

  last_vertex_ = root_vertex_id;
}

void StreamMapBuilder::addViwlsVertexAndEdge(
    const aslam::VisualNFrame::Ptr& nframe,
    const vio::ViNodeState& vinode_state, const aslam::Transformation& T_G_M,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_data) {
  CHECK(nframe);
  CHECK(last_vertex_.isValid())
      << "Need to have a previous vertex to connect to!";

  pose_graph::VertexId new_vertex_id =
      addViwlsVertex(nframe, vinode_state, T_G_M);

  addImuEdge(new_vertex_id, imu_timestamps, imu_data);

  if (kKeepNMostRecentImages > 0u) {
    manipulation_.releaseOldVisualFrameImages(
        last_vertex_, kKeepNMostRecentImages);
  }
}

pose_graph::VertexId StreamMapBuilder::addViwlsVertex(
    const aslam::VisualNFrame::Ptr& nframe,
    const vio::ViNodeState& vinode_state, const aslam::Transformation& T_G_M) {
  aslam::SensorId nframe_id = nframe->getNCamera().getId();
  CHECK(
      map_->getMissionNCamera(mission_id_).getId() == nframe_id ||
      map_->getAdditionalNCamera(mission_id_).getId() == nframe_id)
      << "Can only add nframes that correspond to the mission camera";

  // Initialize all keypoint <-> landmark associations to invalid.
  std::vector<vi_map::LandmarkIdList> invalid_landmark_ids;
  const std::size_t num_frames = nframe->getNumFrames();
  for (std::size_t frame_idx = 0u; frame_idx < num_frames; ++frame_idx) {
    if (nframe->isFrameSet(frame_idx)) {
      const aslam::VisualFrame& frame = nframe->getFrame(frame_idx);
      invalid_landmark_ids.emplace_back(
          // frame.getTotalNumKeypointMeasurements());
          frame.getTotalNumKeypointMeasurements());
    }
  }
  // Create and add the new map vertex.
  pose_graph::VertexId vertex_id =
      aslam::createRandomId<pose_graph::VertexId>();
  vi_map::Vertex* map_vertex = new vi_map::Vertex(
      vertex_id, vinode_state.getImuBias(), nframe, invalid_landmark_ids,
      mission_id_);

  // Set pose and velocity.
  const aslam::Transformation T_M0_M = T_M0_G_ * T_G_M;
  map_vertex->set_T_M_I(T_M0_M * vinode_state.get_T_M_I());
  map_vertex->set_v_M(T_M0_M * vinode_state.get_v_M_I());
  map_->addVertex(vi_map::Vertex::UniquePtr(map_vertex));

  // Optionally dump the image to disk.
  if (FLAGS_map_builder_save_image_as_resources) {
    CHECK(map_->hasMapFolder())
        << "Cannot store resources to a map that has no associated map folder, "
        << "please set the map folder in the VIMap constructor or by using "
        << "map.setMapFolder()!";
    map_->useMapResourceFolder();
    for (std::size_t frame_idx = 0u; frame_idx < nframe->getNumFrames();
         ++frame_idx) {
      if (nframe->isFrameSet(frame_idx)) {
        map_->storeFrameResource(
            nframe->getFrame(frame_idx).getRawImage(), frame_idx,
            backend::ResourceType::kRawImage, map_vertex);
      }
    }
  }
  return vertex_id;
}

void StreamMapBuilder::removeAllVerticesAfterVertexId(
    const pose_graph::VertexId& vertex_id_from,
    pose_graph::VertexIdList* removed_vertex_ids) {
  CHECK_NOTNULL(removed_vertex_ids);
  manipulation_.removePosegraphAfter(vertex_id_from, removed_vertex_ids);
  last_vertex_ = vertex_id_from;
}

void StreamMapBuilder::addImuEdge(
    pose_graph::VertexId target_vertex_id,
    const Eigen::Matrix<int64_t, 1, Eigen::Dynamic>& imu_timestamps,
    const Eigen::Matrix<double, 6, Eigen::Dynamic>& imu_measurements) {
  CHECK(last_vertex_.isValid());
  CHECK(target_vertex_id.isValid());
  CHECK_EQ(imu_timestamps.cols(), imu_measurements.cols());

  // Make sure the vertices are not yet connected.
  pose_graph::EdgeIdSet edges;
  constMap()->getVertex(last_vertex_).getOutgoingEdges(&edges);
  for (const pose_graph::EdgeId& outgoing_edge_id : edges) {
    CHECK(
        constMap()->getEdgeType(outgoing_edge_id) !=
        pose_graph::Edge::EdgeType::kViwls)
        << "Source vertex already has an outgoing ViwlsEdge.";
  }
  constMap()->getVertex(target_vertex_id).getIncomingEdges(&edges);
  for (const pose_graph::EdgeId& incoming_edge_id : edges) {
    CHECK(
        constMap()->getEdgeType(incoming_edge_id) !=
        pose_graph::Edge::EdgeType::kViwls)
        << "Source vertex already has an incoming ViwlsEdge.";
  }

  // The vi_map data structures requires that the last IMU measurement of
  // the previous edge to be be duplicated as the first measurement of the
  // following edge.
  pose_graph::EdgeIdList source_vertex_edge_incoming_ids;
  constMap()->getIncomingOfType(
      pose_graph::Edge::EdgeType::kViwls, last_vertex_,
      &source_vertex_edge_incoming_ids);
  CHECK_LE(source_vertex_edge_incoming_ids.size(), 1u)
      << "More than one ViwlsEdge found.";

  const bool first_edge = source_vertex_edge_incoming_ids.empty();
  if (!first_edge) {
    const vi_map::ViwlsEdge& prev_edge = map_->getEdgeAs<vi_map::ViwlsEdge>(
        source_vertex_edge_incoming_ids.front());
    const double kTolerance = 1.0e-12;
    const std::string kErrorMsg(
        "The vi_map data structures requires that the last IMU"
        "measurement of the previous edge to be be duplicated as the first "
        "measurement of the following edge.");
    CHECK_LT(
        (prev_edge.getImuTimestamps().tail<1>() - imu_timestamps.head(1))
                .cwiseAbs()
                .maxCoeff() +
            (prev_edge.getImuData().rightCols<1>() -
             imu_measurements.leftCols(1))
                .cwiseAbs()
                .maxCoeff(),
        kTolerance)
        << kErrorMsg;
  }
  // Add the edge.
  pose_graph::EdgeId edge_id = aslam::createRandomId<pose_graph::EdgeId>();
  map_->addEdge(aligned_unique<vi_map::ViwlsEdge>(
      edge_id, last_vertex_, target_vertex_id, imu_timestamps,
      imu_measurements));

  last_vertex_ = target_vertex_id;
}

void StreamMapBuilder::addWheelOdometryEdge(
    const pose_graph::VertexId& source_vertex_id,
    const pose_graph::VertexId& target_vertex_id,
    const aslam::Transformation& T_source_target) {
  VLOG(3) << "[StreamMapBuilder] Attaching wheel odometry edge from "
          << source_vertex_id << " to " << target_vertex_id
          << " with T_source_target:\n." << T_source_target;

  CHECK(last_vertex_.isValid());
  CHECK(target_vertex_id.isValid());

  // Make sure the vertices are not yet connected.
  pose_graph::EdgeIdSet edges;
  constMap()->getVertex(source_vertex_id).getOutgoingEdges(&edges);
  for (const pose_graph::EdgeId& outgoing_edge_id : edges) {
    CHECK(
        constMap()->getEdgeType(outgoing_edge_id) !=
        pose_graph::Edge::EdgeType::kWheelOdometry)
        << "Source vertex " << source_vertex_id
        << " already has an outgoing wheel odometry edge.";
  }
  constMap()->getVertex(target_vertex_id).getIncomingEdges(&edges);
  for (const pose_graph::EdgeId& incoming_edge_id : edges) {
    CHECK(
        constMap()->getEdgeType(incoming_edge_id) !=
        pose_graph::Edge::EdgeType::kWheelOdometry)
        << "Source vertex " << target_vertex_id
        << " already has an incoming wheel odometry edge.";
  }

  // Add the edge.
  // TODO(ben): use the correct covariance from config file
  const pose_graph::EdgeId edge_id =
      aslam::createRandomId<pose_graph::EdgeId>();
  const vi_map::WheelOdometry::Ptr odometry_sensor =
      getSelectedWheelOdometrySensor(constMap()->getSensorManager());
  if (!odometry_sensor) {
    LOG(ERROR) << "Cannot attach wheel odometry edges if there is no selected "
                  "wheel odometry sensor!";
    return;
  }

  const aslam::SensorId& sensor_id = odometry_sensor->getId();
  aslam::TransformationCovariance wheel_odometry_covariance;
  CHECK(odometry_sensor->get_T_St_Stp1_fixed_covariance(
      &wheel_odometry_covariance))
      << "No transformation covariance for wheel odometry sensor provided, "
      << "please set a fixed covariance in the wheel odometry sensor config "
         "file.";

  map_->addEdge(aligned_unique<vi_map::TransformationEdge>(
      pose_graph::Edge::EdgeType::kWheelOdometry, edge_id, source_vertex_id,
      target_vertex_id, T_source_target, wheel_odometry_covariance, sensor_id));
}

bool StreamMapBuilder::checkConsistency() const {
  bool is_consistent = false;
  if (!FLAGS_disable_consistency_check) {
    is_consistent = vi_map::checkMapConsistency(*CHECK_NOTNULL(constMap()));
  }

  return is_consistent;
}

void StreamMapBuilder::bufferAbsolute6DoFConstraint(
    const vi_map::Absolute6DoFMeasurement::Ptr& absolute_6dof_constraint) {
  CHECK_NOTNULL(map_);
  CHECK(absolute_6dof_constraint->isValid())
      << "[StreamMapBuilder] Absolute 6DoF constraint invalid!";

  absolute_6dof_measurment_buffer_.addValue(
      absolute_6dof_constraint->getTimestampNanoseconds(),
      absolute_6dof_constraint);
}

void StreamMapBuilder::bufferLoopClosureConstraint(
    const vi_map::LoopClosureMeasurement::ConstPtr& loop_closure_constraint) {
  CHECK_NOTNULL(map_);
  CHECK(loop_closure_constraint);
  CHECK(loop_closure_constraint->isValid())
      << "[StreamMapBuilder] Loop closure constraint invalid!";

  const int64_t newer_timestamp_ns = std::max(
      loop_closure_constraint->getTimestampNanosecondsA(),
      loop_closure_constraint->getTimestampNanosecondsB());
  const int64_t older_timestamp_ns = std::min(
      loop_closure_constraint->getTimestampNanosecondsA(),
      loop_closure_constraint->getTimestampNanosecondsB());
  CHECK_GT(newer_timestamp_ns, older_timestamp_ns);

  std::pair<int64_t, int64_t> timestamp_pair(
      older_timestamp_ns, newer_timestamp_ns);

  vi_map::LoopClosureTemporalMap::iterator it =
      loop_closure_measurement_buffer_.find(timestamp_pair);
  if (it != loop_closure_measurement_buffer_.end()) {
    CHECK(it->first.first == older_timestamp_ns);
    CHECK(it->first.second == newer_timestamp_ns);
    CHECK(it->second);
    CHECK(it->second->getSensorId() == loop_closure_constraint->getSensorId());
    LOG(INFO) << "[StreamMapBuilder] Updating loop closure constraint in "
              << "buffer.\nT_A_B_before: \n"
              << it->second->get_T_A_B() << "\nT_A_B_after: \n"
              << loop_closure_constraint->get_T_A_B();
  }

  loop_closure_measurement_buffer_[timestamp_pair] = loop_closure_constraint;
}

void StreamMapBuilder::bufferWheelOdometryConstraint(
    const vi_map::WheelOdometryMeasurement::ConstPtr&
        wheel_odometry_constraint) {
  CHECK_NOTNULL(map_);
  CHECK(wheel_odometry_constraint);
  CHECK(wheel_odometry_constraint->isValid())
      << "[StreamMapBuilder] Wheel odometry constraint invalid!";

  wheel_odometry_measurement_temporal_buffer_.addValue(
      wheel_odometry_constraint->getTimestampNanoseconds(),
      wheel_odometry_constraint);
}

void StreamMapBuilder::notifyBuffers() {
  notifyAbsolute6DoFConstraintBuffer();
  notifyLoopClosureConstraintBuffer();

  if (constMap()->getMission(mission_id_).hasWheelOdometrySensor()) {
    notifyWheelOdometryConstraintBuffer();
  }
}

void StreamMapBuilder::notifyAbsolute6DoFConstraintBuffer() {
  if (map_->numVertices() < 2u || absolute_6dof_measurment_buffer_.empty()) {
    return;
  }

  const int64_t newest_vertex_time_ns = newest_vertex_timestamp_ns_.load();
  const int64_t oldest_vertex_time_ns = oldest_vertex_timestamp_ns_.load();
  CHECK(aslam::time::isValidTime(newest_vertex_time_ns));
  CHECK(aslam::time::isValidTime(oldest_vertex_time_ns));

  const size_t dropped_constraints =
      absolute_6dof_measurment_buffer_.removeItemsBefore(oldest_vertex_time_ns);
  if (dropped_constraints > 0u) {
    LOG(WARNING) << "[StreamMapBuilder] Dropped " << dropped_constraints
                 << " absolute 6DoF constraints, because they are before the "
                 << "first vertex of the map.";
  }

  std::vector<vi_map::Absolute6DoFMeasurement::Ptr> constraints;
  absolute_6dof_measurment_buffer_.getValuesFromIncludingToIncluding(
      oldest_vertex_time_ns, newest_vertex_time_ns, &constraints);

  const size_t processed_constraints =
      absolute_6dof_measurment_buffer_.removeItemsBefore(newest_vertex_time_ns);

  VLOG(3) << "[StreamMapBuilder] Processing " << processed_constraints
          << " absolute 6DoF constraints.";

  for (const vi_map::Absolute6DoFMeasurement::Ptr constraint_ptr :
       constraints) {
    CHECK(constraint_ptr);
    CHECK_LE(constraint_ptr->getTimestampNanoseconds(), newest_vertex_time_ns);
    CHECK_GE(constraint_ptr->getTimestampNanoseconds(), oldest_vertex_time_ns);
    const vi_map::Absolute6DoFMeasurement& absolute_6dof_constraint =
        *constraint_ptr;

    const aslam::SensorId& absolute_6dof_sensor_id =
        absolute_6dof_constraint.getSensorId();

    CHECK(map_->getSensorManager().hasSensor(absolute_6dof_sensor_id))
        << "[StreamMapBuilder] The absolute 6DoF sensor of this constraint "
        << "is not yet present in the map's sensor manager!";

    const int64_t timestamp_ns_constraint =
        absolute_6dof_constraint.getTimestampNanoseconds();

    constexpr int64_t kMaxInterpolationTimeNs = aslam::time::milliseconds(500);
    pose_graph::VertexId closest_vertex_id;
    uint64_t delta_ns = 0u;
    if (!queries_.getClosestVertexIdByTimestamp(
            timestamp_ns_constraint, kMaxInterpolationTimeNs,
            &closest_vertex_id, &delta_ns)) {
      LOG(WARNING)
          << "[StreamMapBuilder] Could not attach absolute 6DoF "
          << "constraint, because the timestamp is not close enough to "
          << "a vertex in the pose graph (delta = " << delta_ns << "ns > "
          << kMaxInterpolationTimeNs
          << "ns)! timestamp_ns: " << timestamp_ns_constraint << ".";
      continue;
    }

    vi_map::Vertex& closest_vertex = map_->getVertex(closest_vertex_id);

    const aslam::Transformation& T_M_B_vertex = closest_vertex.get_T_M_I();
    const aslam::Transformation& T_B_S =
        map_->getSensorManager().getSensor_T_B_S(absolute_6dof_sensor_id);
    const aslam::Transformation& T_G_S_measurement =
        absolute_6dof_constraint.get_T_G_S();

    // Interpolate the offset of the robot base between the vertex and the time
    // the global constraint has been measured, but only if necessary.
    aslam::Transformation T_B_measurement_B_vertex;
    if (delta_ns == 0) {
      VLOG(3) << "[StreamMapBuilder] Absolute 6DoF constraint is synchronized "
              << "with vertex, no interpolation needed.";
      T_B_measurement_B_vertex.setIdentity();
    } else if (absolute_6dof_constraint.has_T_M_B_cached()) {
      VLOG(3) << "[StreamMapBuilder] Absolute 6DoF constraint was able to "
              << "reuse cached interpolated robot pose from buffer, no "
              << "interpolation needed.";
      aslam::Transformation T_M_B_measurement;
      absolute_6dof_constraint.get_T_M_B_cached(&T_M_B_measurement);
      T_B_measurement_B_vertex = T_M_B_measurement.inverse() * T_M_B_vertex;
    } else {
      VLOG(3) << "[StreamMapBuilder] Absolute 6DoF constraint interpolates "
              << "robot pose based on map vertices.";
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> constraint_timestamps_ns(1);
      constraint_timestamps_ns[0] = timestamp_ns_constraint;

      aslam::TransformationVector poses_M_B;
      pose_interpolator_.getPosesAtTime(
          *map_, mission_id_, constraint_timestamps_ns, &poses_M_B);
      CHECK_EQ(
          static_cast<int>(poses_M_B.size()), constraint_timestamps_ns.size());

      const aslam::Transformation& T_M_B_measurement = poses_M_B[0];
      T_B_measurement_B_vertex = T_M_B_measurement.inverse() * T_M_B_vertex;
    }

    const aslam::Transformation T_S_measurement_S_vertex =
        T_B_S.inverse() * T_B_measurement_B_vertex * T_B_S;

    const aslam::Transformation& T_G_S_vertex =
        T_G_S_measurement * T_S_measurement_S_vertex;

    const vi_map::Absolute6DoFMeasurement updated_6dof_measurement(
        absolute_6dof_sensor_id, closest_vertex.getMinTimestampNanoseconds(),
        T_G_S_vertex, absolute_6dof_constraint.get_T_G_S_covariance());

    closest_vertex.addAbsolute6DoFMeasurement(updated_6dof_measurement);

    VLOG(3) << "[StreamMapBuilder] Inserted new absolute 6DoF constraint for "
            << "vertex " << closest_vertex_id << ", which now has "
            << closest_vertex.getNumAbsolute6DoFMeasurements()
            << " such constraints.";
  }
}

void StreamMapBuilder::notifyLoopClosureConstraintBuffer() {
  if (map_->numVertices() < 1u || loop_closure_measurement_buffer_.empty()) {
    return;
  }

  const int64_t newest_vertex_time_ns = newest_vertex_timestamp_ns_.load();
  const int64_t oldest_vertex_time_ns = oldest_vertex_timestamp_ns_.load();
  CHECK(aslam::time::isValidTime(newest_vertex_time_ns));
  CHECK(aslam::time::isValidTime(oldest_vertex_time_ns));

  // Delete all constraints that can never be matched with the map because they
  // are relating to a time before the first vertex. At the same time extract
  // all elements that are within the range of the map.
  size_t dropped_constraints = 0u;
  std::vector<vi_map::LoopClosureMeasurement::ConstPtr> constraints_to_attach;
  vi_map::LoopClosureTemporalMap::iterator it =
      loop_closure_measurement_buffer_.begin();
  while (it != loop_closure_measurement_buffer_.end()) {
    const int64_t older_timestamp_ns = it->first.first;
    const int64_t newer_timestamp_ns = it->first.second;
    if (older_timestamp_ns < oldest_vertex_time_ns) {
      // This loop closure can never be attached to the pose graph.
      it = loop_closure_measurement_buffer_.erase(it);
      ++dropped_constraints;
    } else if (newer_timestamp_ns <= newest_vertex_time_ns) {
      // The lower timestamp is ok, since it didn't trigger the condition above,
      // hence both timestamps are withing the map boundaries and we can extract
      // the edges to be attached later.
      constraints_to_attach.push_back(it->second);
      it = loop_closure_measurement_buffer_.erase(it);
    } else {
      // THe newer timestamp is not yet in the map, hence we need to wait and
      // keep it in the buffer.
      ++it;
    }
  }
  if (dropped_constraints > 0u) {
    LOG(WARNING) << "[StreamMapBuilder] Dropped " << dropped_constraints
                 << " loop closure constraints, because they are before the "
                 << "first vertex of the map.";
  }
  if (loop_closure_measurement_buffer_.size() > 0u) {
    VLOG(3) << "[StreamMapBuilder] Left "
            << loop_closure_measurement_buffer_.size()
            << " loop closure constraints in the buffer, because they "
            << "newer than the latest vertex in the map.";
  }
  VLOG(3) << "[StreamMapBuilder] Attaching or updating "
          << constraints_to_attach.size() << " loop closure constraints.";

  for (const vi_map::LoopClosureMeasurement::ConstPtr constraint_ptr :
       constraints_to_attach) {
    CHECK(constraint_ptr);
    const vi_map::LoopClosureMeasurement& loop_closure_constraint =
        *constraint_ptr;

    const aslam::SensorId& loop_closure_sensor_id =
        loop_closure_constraint.getSensorId();

    CHECK(map_->getSensorManager().hasSensor(loop_closure_sensor_id))
        << "[StreamMapBuilder] The loop closure sensor of this constraint is "
           "not yet present in the map's sensor manager!";

    const aslam::Transformation& T_B_S =
        map_->getSensorManager().getSensor_T_B_S(loop_closure_sensor_id);

    const int64_t timestamp_A_ns =
        loop_closure_constraint.getTimestampNanosecondsA();
    const int64_t timestamp_B_ns =
        loop_closure_constraint.getTimestampNanosecondsB();
    CHECK(timestamp_A_ns != timestamp_B_ns)
        << "[StreamMapBuilder] A loop closure constraint should be between "
        << "two distinct points in time in the pose graph!";
    const aslam::Transformation& T_A_B = loop_closure_constraint.get_T_A_B();

    int64_t timestamp_from_ns = aslam::time::getInvalidTime();
    int64_t timestamp_to_ns = aslam::time::getInvalidTime();
    aslam::Transformation T_S_lc_from_S_lc_to;
    if (timestamp_A_ns > timestamp_B_ns) {
      timestamp_from_ns = timestamp_B_ns;
      timestamp_to_ns = timestamp_A_ns;
      T_S_lc_from_S_lc_to = T_A_B.inverse();
    } else if (timestamp_A_ns < timestamp_B_ns) {
      timestamp_from_ns = timestamp_A_ns;
      timestamp_to_ns = timestamp_B_ns;
      T_S_lc_from_S_lc_to = T_A_B;
    }
    CHECK(aslam::time::isValidTime(timestamp_from_ns));
    CHECK(aslam::time::isValidTime(timestamp_to_ns));
    CHECK_LT(timestamp_from_ns, timestamp_to_ns);
    CHECK(timestamp_from_ns >= oldest_vertex_time_ns)
        << "This should not happen, the temporal buffer should take care of "
        << "this! timestamp_from_ns: " << timestamp_from_ns
        << "ns vs oldest_vertex_time_ns: " << oldest_vertex_time_ns << "ns.";
    CHECK(timestamp_to_ns <= newest_vertex_time_ns)
        << "This should not happen, the temporal buffer should take care of "
        << "this! timestamp_to_ns: " << timestamp_to_ns
        << "ns vs newest_vertex_time_ns: " << newest_vertex_time_ns << "ns.";

    constexpr int64_t kMaxInterpolationTimeNs = aslam::time::milliseconds(500);

    pose_graph::VertexId vertex_id_from;
    if (!queries_.getClosestVertexIdByTimestamp(
            timestamp_from_ns, kMaxInterpolationTimeNs, &vertex_id_from,
            nullptr /*delta_ns*/)) {
      LOG(WARNING)
          << "[StreamMapBuilder] Could not attach loop closure "
          << "constraint, because the lower timestamp is not close enough to "
          << "a vertex in the pose graph (delta > " << kMaxInterpolationTimeNs
          << "ns)! timestamp_from: " << timestamp_from_ns
          << "ns timstamp_to: " << timestamp_to_ns;
      continue;
    }

    pose_graph::VertexId vertex_id_to;
    if (!queries_.getClosestVertexIdByTimestamp(
            timestamp_to_ns, kMaxInterpolationTimeNs, &vertex_id_to,
            nullptr /*delta_ns*/)) {
      LOG(WARNING)
          << "[StreamMapBuilder] Could not attach loop closure "
          << "constraint, because the upper timestamp is not close enough to "
          << "a vertex in the pose graph (delta > " << kMaxInterpolationTimeNs
          << "ns)! timestamp_from: " << timestamp_from_ns
          << "ns timstamp_to: " << timestamp_to_ns;
      continue;
    }

    CHECK(vertex_id_from.isValid());
    CHECK(vertex_id_to.isValid());

    const landmark_triangulation::PoseInterpolator pose_interpolator;
    Eigen::Matrix<int64_t, 1, Eigen::Dynamic> constraint_timestamps_ns(2);
    constraint_timestamps_ns[0] = timestamp_from_ns;
    constraint_timestamps_ns[1] = timestamp_to_ns;

    aslam::TransformationVector poses_M_B;
    pose_interpolator.getPosesAtTime(
        *map_, mission_id_, constraint_timestamps_ns, &poses_M_B);
    CHECK_EQ(
        static_cast<int>(poses_M_B.size()), constraint_timestamps_ns.size());

    const aslam::Transformation& T_M_B_lc_from = poses_M_B[0];
    const aslam::Transformation& T_M_B_lc_to = poses_M_B[1];

    const aslam::Transformation T_M_B_vertex_from =
        map_->getVertex(vertex_id_from).get_T_M_I();
    const aslam::Transformation T_M_B_vertex_to =
        map_->getVertex(vertex_id_to).get_T_M_I();

    const aslam::Transformation T_B_lc_from_B_vertex_from =
        T_M_B_lc_from.inverse() * T_M_B_vertex_from;
    const aslam::Transformation T_B_lc_to_B_vertex_to =
        T_M_B_lc_to.inverse() * T_M_B_vertex_to;

    const aslam::Transformation T_B_lc_from_B_lc_to =
        T_B_S * T_S_lc_from_S_lc_to * T_B_S.inverse();

    const aslam::Transformation T_B_vertex_to_B_vertex_from =
        T_B_lc_to_B_vertex_to.inverse() * T_B_lc_from_B_lc_to.inverse() *
        T_B_lc_from_B_vertex_from;

    // Replace the transformation above with this one for to test the lc logic,
    // this will enforce the current odometry estimate as lc edge.
    // const aslam::Transformation T_B_vertex_to_B_vertex_from =
    //     T_G_I_vertex_to.inverse() * T_G_I_vertex_from;

    constexpr double kSwitchVariable = 1.0;
    constexpr double kSwitchVariableVariance = 1e-3;

    // Either update existing loop closure edge or create a new one. We keep
    // track of all loop closures added this way for faster access to the edge.
    const std::pair<pose_graph::VertexId, pose_graph::VertexId> vertex_id_pair(
        vertex_id_from, vertex_id_to);
    LoopClosureEdgeLookup::const_iterator it =
        attached_loop_closures_lookup_.find(vertex_id_pair);
    if (it != attached_loop_closures_lookup_.end()) {
      CHECK(it->second.isValid());
      CHECK(it->first.first == vertex_id_from);
      CHECK(it->first.second == vertex_id_to);

      vi_map::LoopClosureEdge& lc_edge =
          map_->getEdgeAs<vi_map::LoopClosureEdge>(it->second);

      CHECK(lc_edge.from() == vertex_id_from);
      CHECK(lc_edge.to() == vertex_id_to);

      lc_edge.set_T_A_B(T_B_vertex_to_B_vertex_from.inverse());
      lc_edge.set_T_A_B_Covariance(
          loop_closure_constraint.get_T_A_B_covariance());

      VLOG(3) << "[StreamMapBuilder] Updated loop closure edge " << it->second
              << " between " << vertex_id_from << " and " << vertex_id_to
              << ". Number of loop closures: "
              << attached_loop_closures_lookup_.size();
    } else {
      pose_graph::EdgeId edge_id;
      aslam::generateId(&edge_id);
      attached_loop_closures_lookup_[vertex_id_pair] = edge_id;

      vi_map::LoopClosureEdge::UniquePtr lc_edge =
          aligned_unique<vi_map::LoopClosureEdge>(
              edge_id, vertex_id_from, vertex_id_to, kSwitchVariable,
              kSwitchVariableVariance, T_B_vertex_to_B_vertex_from.inverse(),
              loop_closure_constraint.get_T_A_B_covariance());
      map_->addEdge(std::move(lc_edge));

      VLOG(3) << "[StreamMapBuilder] Inserted new loop closure edge " << edge_id
              << " between " << vertex_id_from << " and " << vertex_id_to
              << ". Number of loop closures: "
              << attached_loop_closures_lookup_.size();
    }
  }
}

void StreamMapBuilder::notifyWheelOdometryConstraintBuffer() {
  if ((map_->numVertices() < 1u ||
       wheel_odometry_measurement_temporal_buffer_.empty())) {
    VLOG(3) << "[StreamMapBuilder] No wheel odometry measurements in buffer "
            << "yet; not adding wheel odom edge.";
    return;
  }

  // If current vertex should not be processed, look for a next one. If a next
  // vertex doesn't exist return and wait for next update
  if (done_current_vertex_wheel_odometry_) {
    if (map_->getNextVertex(
            vertex_processing_wheel_odometry_id_,
            map_->getGraphTraversalEdgeType(mission_id_),
            &vertex_processing_wheel_odometry_id_)) {
      VLOG(2) << "[StreamMapBuilder] Current vertex "
              << vertex_processing_wheel_odometry_id_ << " already "
              << "processed and will be skipped - moving to next vertex.";
      // We got a new vertex, try to process it
      done_current_vertex_wheel_odometry_ = false;
    } else {
      VLOG(2) << "[StreamMapBuilder] Current vertex "
              << vertex_processing_wheel_odometry_id_ << " already "
              << "processed but is also last vertex in buffer - returning.";
      return;
    }
  }

  int counter = 0;
  bool processing_measurements = true;
  while (processing_measurements) {
    const vi_map::Vertex& vertex =
        map_->getVertex(vertex_processing_wheel_odometry_id_);
    int64_t current_vertex_time_ns = vertex.getMinTimestampNanoseconds();
    CHECK(aslam::time::isValidTime(current_vertex_time_ns))
        << "Invalid vertex timestamp while adding wheel odometry edge. "
        << "This should not be possible!";

    int64_t t_before = 0;
    int64_t t_after = std::numeric_limits<int64_t>::max();
    vi_map::WheelOdometryMeasurement::ConstPtr measurement_before;
    vi_map::WheelOdometryMeasurement::ConstPtr measurement_after;
    bool value_before_exists =
        wheel_odometry_measurement_temporal_buffer_.getValueAtOrBeforeTime(
            current_vertex_time_ns, &t_before, &measurement_before);
    if (!value_before_exists) {
      LOG(WARNING)
          << "[StreamMapBuilder] No previous value found, thus not adding "
          << "wheel odometry edge. This can occur if root vertex is being "
          << "processed and no wheel odometry measurements arrived before. "
          << "This should not occur if any later frame is being processed.";
      done_current_vertex_wheel_odometry_ = true;
      return;
    }
    const bool value_after_exists =
        wheel_odometry_measurement_temporal_buffer_.getValueAtOrAfterTime(
            current_vertex_time_ns, &t_after, &measurement_after);

    if (value_after_exists) {
      CHECK(t_before < t_after);
    } else {
      VLOG(2)
          << "[StreamMapBuilder] Did not find value after current measurement "
          << "to interpolate. Stopping here, until next update.";
      return;
    }

    // TODO(ben): add logic for cases where time between measurements very large

    // Ow: wheel odometry origin frame
    // T_Ow_S_before: last available transformation before desired nframe
    //                timestamp. T_Ow_S_after analogous.
    // Bt: vertex at current time t in body frame
    // Btm1: previous vertex at time t minus 1 in body frame
    aslam::Transformation T_Ow_S_before = measurement_before->get_T_S0_St();
    aslam::Transformation T_Btm1_S_before =
        T_Ow_Btm1_.inverse() * T_Ow_S_before;

    aslam::Transformation T_M_Bt;
    // T_S_before_Bt is only set if we have a wheel odometry measurement from
    // after the most recent vertex.
    aslam::Transformation T_S_before_Bt;
    aslam::Transformation T_Ow_S_after;
    aslam::Transformation T_S_before_S_after;
    T_Ow_S_after = measurement_after->get_T_S0_St();
    T_S_before_S_after = T_Ow_S_before.inverse() * T_Ow_S_after;
    common::interpolateTransformation(
        t_before, T_Ow_S_before, t_after, T_Ow_S_after, current_vertex_time_ns,
        &T_M_Bt);
    T_S_before_Bt = T_Ow_S_before.inverse() * T_M_Bt;

    const aslam::Transformation T_Btm1_Bt = T_Btm1_S_before * T_S_before_Bt;
    // most recent Bt vertex will be next iteration's Btm1 vertex.
    T_Ow_Btm1_ = T_Ow_S_before * T_S_before_Bt;

    // We always have to skip the first time since with this we find the wheel
    // odometry origin and can't yet calculate a relative transform. When
    // submapping it can happen that we are initialized but processing the root
    // vertex in which case we also skip.
    if (found_wheel_odometry_origin_ &&
        vertex_processing_wheel_odometry_id_ != getRootVertexId()) {
      addWheelOdometryEdge(
          Btm1_vertex_id_, vertex_processing_wheel_odometry_id_, T_Btm1_Bt);
      Btm1_vertex_id_ = vertex_processing_wheel_odometry_id_;
      done_current_vertex_wheel_odometry_ = true;

      counter++;
    } else {
      VLOG(2)
          << "[StreamMapBuilder] Initialized wheel odometry origin or "
          << "currently processing root vertex. Skipping adding edge since no "
          << "relative transform can be calculated.";
      found_wheel_odometry_origin_ = true;
      Btm1_vertex_id_ = vertex_processing_wheel_odometry_id_;
      done_current_vertex_wheel_odometry_ = true;
    }

    std::unordered_set<pose_graph::EdgeId> outgoing_edges;
    vertex.getOutgoingEdges(&outgoing_edges);
    if (!map_->getNextVertex(
            vertex_processing_wheel_odometry_id_,
            map_->getGraphTraversalEdgeType(mission_id_),
            &vertex_processing_wheel_odometry_id_)) {
      VLOG(2) << "[StreamMapBuilder] Could not find a next vertex; terminating "
              << "adding of wheel odometry edges.";
      processing_measurements = false;
    } else {
      VLOG(3) << "[StreamMapBuilder] Found another vertex:"
              << vertex_processing_wheel_odometry_id_;
      // We got a new vertex, try to process it
      done_current_vertex_wheel_odometry_ = false;
    }
  }
  VLOG(2) << "[StreamMapBuilder] Added " << counter << " wheel odometry edges.";
}

}  // namespace online_map_builders
