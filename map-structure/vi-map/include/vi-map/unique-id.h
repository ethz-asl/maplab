#ifndef VI_MAP_UNIQUE_ID_H_
#define VI_MAP_UNIQUE_ID_H_

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include <aslam/common/unique-id.h>
#include <maplab-common/file-system-tools.h>
#include <maplab-common/unique-id.h>
#include <posegraph/unique-id.h>

namespace vi_map {
UNIQUE_ID_DEFINE_ID(MissionId);
UNIQUE_ID_DEFINE_ID(MissionBaseFrameId);
UNIQUE_ID_DEFINE_ID(LandmarkId);
UNIQUE_ID_DEFINE_ID(ResourceId);

struct VisualFrameIdentifier {
  pose_graph::VertexId vertex_id;
  size_t frame_index;
  static constexpr size_t kInvalidFrameIndex =
      std::numeric_limits<size_t>::max();
  inline VisualFrameIdentifier()
      : vertex_id(), frame_index(kInvalidFrameIndex) {}
  inline VisualFrameIdentifier(
      const pose_graph::VertexId& _vertex_id, const size_t _frame_index)
      : vertex_id(_vertex_id), frame_index(_frame_index) {}
  inline bool operator==(const VisualFrameIdentifier& rhs) const {
    return vertex_id == rhs.vertex_id && frame_index == rhs.frame_index;
  }
  inline bool operator!=(const VisualFrameIdentifier& rhs) const {
    return vertex_id != rhs.vertex_id || frame_index != rhs.frame_index;
  }
  inline bool operator<(const VisualFrameIdentifier& rhs) const {
    if (vertex_id == rhs.vertex_id) {
      return frame_index < rhs.frame_index;
    } else {
      return vertex_id < rhs.vertex_id;
    }
  }
  inline bool isValid() const {
    return vertex_id.isValid() && frame_index != kInvalidFrameIndex;
  }
};
typedef std::vector<VisualFrameIdentifier> VisualFrameIdentifierList;
typedef std::unordered_map<aslam::FrameId, VisualFrameIdentifier>
    FrameIdToFrameIdentifierMap;

struct KeypointIdentifier {
  VisualFrameIdentifier frame_id;
  size_t keypoint_index;
  static constexpr size_t kInvalidKeypointIndex =
      std::numeric_limits<size_t>::max();

  inline KeypointIdentifier() : keypoint_index(kInvalidKeypointIndex) {}

  inline KeypointIdentifier(
      const VisualFrameIdentifier& _frame_id, const size_t _keypoint_index)
      : frame_id(_frame_id), keypoint_index(_keypoint_index) {}

  inline KeypointIdentifier(
      const pose_graph::VertexId& _vertex_id, const size_t _frame_index,
      const size_t _keypoint_index)
      : frame_id(_vertex_id, _frame_index), keypoint_index(_keypoint_index) {}

  inline bool operator==(const KeypointIdentifier& other) const {
    return frame_id == other.frame_id && keypoint_index == other.keypoint_index;
  }
  inline bool isValid() const {
    return frame_id.isValid() && keypoint_index != kInvalidKeypointIndex;
  }
};
typedef std::vector<KeypointIdentifier> KeypointIdentifierList;

// Converts a CSV string with ID hex strings into a vector of IDs. If the CSV
// string is empty it returns true and an empty vector. If one of the CSV
// entries is not a valid ID, it returns false and an empty vector.
template <typename Id>
bool csvIdStringToIdList(const std::string& csv_ids, std::vector<Id>* id_list) {
  CHECK_NOTNULL(id_list)->clear();
  if (csv_ids.empty()) {
    return true;
  }

  static const std::string kDelimiter = ",";

  std::vector<std::string> id_strings;
  common::tokenizeString(csv_ids, kDelimiter, &id_strings);

  if (id_strings.empty()) {
    // Nothing to do here.
    return true;
  }

  for (const std::string& id_string : id_strings) {
    Id id;
    id.fromHexString(id_string);
    if (!id.isValid()) {
      LOG(WARNING) << "\"" << id_string << "\" is not a valid ID string!";
      id_list->clear();
      return false;
    }
    id_list->push_back(id);
  }
  return true;
}

}  // namespace vi_map

UNIQUE_ID_DEFINE_ID_HASH(vi_map::LandmarkId);
UNIQUE_ID_DEFINE_ID_HASH(vi_map::MissionId);
UNIQUE_ID_DEFINE_ID_HASH(vi_map::MissionBaseFrameId);
UNIQUE_ID_DEFINE_ID_HASH(vi_map::ResourceId);
namespace std {

template <>
struct hash<vi_map::VisualFrameIdentifier> {
  std::size_t operator()(
      const vi_map::VisualFrameIdentifier& identifier) const {
    return identifier.vertex_id.hashToSizeT() ^ identifier.frame_index;
  }
};

template <>
struct hash<vi_map::KeypointIdentifier> {
  std::size_t operator()(const vi_map::KeypointIdentifier& identifier) const {
    return std::hash<vi_map::VisualFrameIdentifier>()(identifier.frame_id) ^
           std::hash<size_t>()(identifier.keypoint_index);
  }
};

inline ostream& operator<<(
    ostream& stream, const vi_map::VisualFrameIdentifier& identifier) {
  return stream << identifier.vertex_id << ":" << identifier.frame_index;
}

}  // namespace std

#endif  // VI_MAP_UNIQUE_ID_H_
