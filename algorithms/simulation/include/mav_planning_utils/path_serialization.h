#ifndef MAV_PLANNING_UTILS_PATH_SERIALIZATION_H_
#define MAV_PLANNING_UTILS_PATH_SERIALIZATION_H_

#include <string>

#include <simulation/visual-inertial-path-generator.h>

#include "mav_planning_utils/path_planning.h"

namespace mav_planning_utils {

namespace proto {
class SegmentVector;
class Vertex4DList;
class Path4D;
}  // namespace proto

namespace path_planning {

void serializeSegmentVector(
    const Path4D<kDefaultN>::SegmentVector& segment_vector,
    proto::SegmentVector* proto);

void deserializeSegmentVector(
    const proto::SegmentVector& proto,
    Path4D<kDefaultN>::SegmentVector* segment_vector);

void serializeVertices(
    const Path4D<kDefaultN>::Vertex4DList& vertices,
    proto::Vertex4DList* proto);

void deserializeVertices(
    const proto::Vertex4DList& proto,
    Path4D<kDefaultN>::Vertex4DList* vertices);

void serializePath4d(const Path4D<kDefaultN>& path_4d, proto::Path4D* proto);

void deserializePath4d(const proto::Path4D& proto, Path4D<kDefaultN>* path_4d);

// Returns true if there is a file name for the corresponding path type and
// false otherwise.
bool getFileNameForPathType(
    const test_trajectory_gen::Path path_type, std::string* file_name);

void writePath4dToFile(
    const test_trajectory_gen::Path path_type,
    const Path4D<kDefaultN>& path_4d);

void readPath4dFromFile(
    const test_trajectory_gen::Path path_type, Path4D<kDefaultN>* path_4d);

}  // namespace path_planning
}  // namespace mav_planning_utils

#endif  // MAV_PLANNING_UTILS_PATH_SERIALIZATION_H_
