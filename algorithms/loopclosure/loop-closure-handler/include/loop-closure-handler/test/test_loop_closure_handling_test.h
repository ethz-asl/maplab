#ifndef LOOP_CLOSURE_HANDLER_TEST_TEST_LOOP_CLOSURE_HANDLING_TEST_H_
#define LOOP_CLOSURE_HANDLER_TEST_TEST_LOOP_CLOSURE_HANDLING_TEST_H_
#include <memory>
#include <mutex>
#include <random>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/distortion-fisheye.h>
#include <aslam/frames/visual-frame.h>
#include <vi-map/landmark-index.h>
#include <vi-map/landmark.h>
#include <vi-map/mission-baseframe.h>
#include <vi-map/mission.h>
#include <vi-map/unique-id.h>
#include <vi-map/vertex.h>
#include <vi-map/viwls-edge.h>

#include <loop-closure-handler/loop-closure-constraint.h>
#include <loop-closure-handler/loop-closure-handler.h>
#include <maplab-common/pose_types.h>
#include <maplab-common/quaternion-math.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

DECLARE_double(lc_min_image_time_seconds);
DECLARE_double(vi_map_landmark_quality_max_distance_from_closest_observer);
DECLARE_bool(lc_filter_underconstrained_landmarks);

static constexpr unsigned int kNumOfLandmarks = 200;
static constexpr unsigned int kNumOfDuplicateLandmarks = 50;
static constexpr unsigned int kNumOfMapVertices = 5;
static constexpr unsigned int kNumOfQueryVertices = 1;

static constexpr int kVisualFrameIndex = 0;

class SubmapMergingTest;
struct ExpectedLandmarkMergeTriple {
  pose_graph::VertexId vertex_id;
  int idx;
  vi_map::LandmarkId new_landmark_id;
};

namespace vi_map {
void addObservedLandmarkId(
    const LandmarkId& landmark_id, vi_map::Vertex* vertex_ptr) {
  CHECK_NOTNULL(vertex_ptr);
  static constexpr unsigned int kVisualFrameIndex = 0;
  vertex_ptr->addObservedLandmarkId(kVisualFrameIndex, landmark_id);
}
}  // namespace vi_map
class LoopClosureHandlerTest : public ::testing::Test {
  friend ::SubmapMergingTest;  // Test.

 protected:
  typedef aslam::FisheyeDistortion DistortionType;
  typedef aslam::PinholeCamera CameraType;
  typedef std::unordered_map<vi_map::LandmarkId, vi_map::LandmarkId>
      LandmarkToLandmarkMap;

  LoopClosureHandlerTest();

  virtual void SetUp() {
    constructCamera();
    createMission();
    populatePosegraph();
    generateAndProjectLandmarksToMapKeyframes();
    addDuplicateLandmarksToQueryKeyframes();
    handler_ = std::shared_ptr<loop_closure_handler::LoopClosureHandler>(
        new loop_closure_handler::LoopClosureHandler(
            &map_, &landmark_id_old_to_new_));
    CHECK_LE(kNumOfDuplicateLandmarks, kNumOfLandmarks);
  }

  bool hasLandmark(const vi_map::LandmarkId& landmark_id) const;

  pose_graph::VertexId getVertexIdForLandmark(
      const vi_map::LandmarkId& landmark_id) const;

  void constructCamera();
  void createMission();
  void populatePosegraph();
  void generateAndProjectLandmarksToMapKeyframes();
  unsigned int addKeypointToVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addLandmarkToVertex(
      const Eigen::Vector3d& G_p_fi, const vi_map::LandmarkId& landmark_id,
      vi_map::Vertex* vertex_ptr);
  void addDuplicateLandmarksToQueryKeyframes();

  vi_map::VIMap map_;
  vi_map::PoseGraph& posegraph_;
  vi_map::LandmarkIndex& landmark_index_;
  vi_map::VIMissionMap& missions_;
  vi_map::MissionBaseFrameMap& mission_base_frames_;

  std::shared_ptr<loop_closure_handler::LoopClosureHandler> handler_;
  LandmarkToLandmarkMap landmark_id_old_to_new_;

  std::vector<pose_graph::VertexId> vertex_ids_;
  vi_map::MissionId mission_id_;
  aslam::NCamera::Ptr cameras_;
  aslam::Transformation T_C_B_;
  vi_map::LoopClosureConstraintVector constraints_;
  LandmarkToLandmarkMap duplicate_landmark_to_landmark_map_;
  std::vector<ExpectedLandmarkMergeTriple> expected_landmark_merges_;
  std::mt19937 gen_;
  std::uniform_real_distribution<> dis_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif  // LOOP_CLOSURE_HANDLER_TEST_TEST_LOOP_CLOSURE_HANDLING_TEST_H_
