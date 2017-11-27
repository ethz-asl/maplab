#ifndef LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_H_
#define LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_H_

#include <string>
#include <unordered_map>

#include <Eigen/Core>

#include <aslam/common/memory.h>
#include <maplab-common/macros.h>
#include <maplab-common/map-manager-config.h>
#include <vi-map/landmark.h>
#include <vi-map/unique-id.h>

#include "localization-summary-map/unique-id.h"

namespace summary_map {
class VIMap;
}  // namespace summary_map

namespace summary_map {

namespace proto {
class LocalizationSummaryMap;
}  // namespace proto

class LocalizationSummaryMap {
 public:
  MAPLAB_POINTER_TYPEDEFS(LocalizationSummaryMap);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline void setId(
      const LocalizationSummaryMapId& localization_summary_map_id) {
    id_ = localization_summary_map_id;
  }

  inline const LocalizationSummaryMapId& id() const {
    return id_;
  }

  void serialize(proto::LocalizationSummaryMap* proto) const;
  void deserialize(
      const LocalizationSummaryMapId& localization_summary_map_id,
      const proto::LocalizationSummaryMap& proto);

  bool loadFromFolder(const std::string& folder_path);
  bool loadFromFolder(
      const LocalizationSummaryMapId& summary_map_id,
      const std::string& folder_path);
  bool saveToFolder(
      const std::string& folder_path, const backend::SaveConfig& config);
  static bool hasMapOnFileSystem(const std::string& folder_path);

  void setGLandmarkPosition(const Eigen::Matrix3Xd& G_landmark_position);
  void initializeLandmarkIds(int num_landmarks);

  void setGObserverPosition(const Eigen::Matrix3Xd& G_observer_position);
  void setProjectedDescriptors(const Eigen::MatrixXf& descriptors);
  void setObserverIndices(
      const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>& observer_indices);
  void setObservationToLandmarkIndex(
      const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
          observation_to_landmark_index);

  const Eigen::Matrix3Xf& GLandmarkPosition() const;
  const Eigen::Matrix3Xf& GObserverPosition() const;
  const Eigen::MatrixXf& projectedDescriptors() const;
  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>& observerIndices() const;
  const Eigen::Matrix<unsigned int, Eigen::Dynamic, 1>&
  observationToLandmarkIndex() const;

  bool hasLandmark(const vi_map::LandmarkId& landmark_id) const;
  Eigen::Vector3d getGLandmarkPosition(
      const vi_map::LandmarkId& landmark_id) const;
  bool hasVertex(const pose_graph::VertexId& vertex_id) const;
  void getAllObserverIds(pose_graph::VertexIdList* observer_ids) const;
  void getAllLandmarkIds(vi_map::LandmarkIdList* landmark_ids) const;

  bool operator==(const LocalizationSummaryMap& other) const;
  bool operator!=(const LocalizationSummaryMap& other) const;

 private:
  static constexpr char kFileName[] = "localization_summary_map";

  // The goal here is to get the most compact representation (memory).
  // So instead of storing for every descriptor a vertex+frame id pair, we just
  // store an arbitrary integer id that tells us which landmarks are seen from
  // which frame. This is enough information to perform filtering in the
  // covisibility graph.

  LocalizationSummaryMapId id_;
  /// Mapping of landmark-ids to landmark indices.
  std::unordered_map<vi_map::LandmarkId, int> landmark_id_to_landmark_index_;
  /// The position of the landmarks in the global frame of reference.
  Eigen::Matrix3Xf G_landmark_position_;
  /// Mapping of vertex-ids to vertex indices.
  std::unordered_map<pose_graph::VertexId, int> vertex_id_to_index_;
  /// The position of the observers in the global frame of reference.
  Eigen::Matrix3Xf G_observer_position_;
  /// A set of projected_descriptors from observations of landmarks.
  Eigen::MatrixXf projected_descriptors_;
  /// An index of a key-frame for every observation (descriptor).
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> observer_indices_;
  /// A mapping from observation (descriptor) to index in G_landmark_position.
  Eigen::Matrix<unsigned int, Eigen::Dynamic, 1> observation_to_landmark_index_;
};

typedef std::unordered_map<LocalizationSummaryMapId,
                           LocalizationSummaryMap::Ptr>
    LocalizationSummaryMapMap;
typedef std::unordered_map<LocalizationSummaryMapId,
                           LocalizationSummaryMap::Ptr>
    LocalizationSummaryMapHashMap;

}  // namespace summary_map
#endif  // LOCALIZATION_SUMMARY_MAP_LOCALIZATION_SUMMARY_MAP_H_
