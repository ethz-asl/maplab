#include "mav_planning_utils/path_serialization.h"

#include <fstream>  // NOLINT
#include <string>

#include <glog/logging.h>
#include <maplab-common/eigen-proto.h>
#include <maplab-common/file-system-tools.h>

#include "./path-4d.pb.h"

namespace mav_planning_utils {
namespace path_planning {

void serializeSegmentVector(
    const Path4D<kDefaultN>::SegmentVector& segment_vector,
    proto::SegmentVector* proto) {
  CHECK_NOTNULL(proto);
  for (const Path4D<kDefaultN>::SegmentType& segment : segment_vector) {
    proto::Segment* segment_proto = proto->add_segments();
    common::eigen_proto::serialize(
        segment.p.coefficients,
        segment_proto->mutable_polynomial_coefficients());
    segment_proto->set_time_seconds(segment.t);
  }
}

void deserializeSegmentVector(
    const proto::SegmentVector& proto,
    Path4D<kDefaultN>::SegmentVector* segment_vector) {
  CHECK_NOTNULL(segment_vector)->clear();
  for (const proto::Segment& segment_proto : proto.segments()) {
    Path4D<kDefaultN>::SegmentType segment;
    common::eigen_proto::deserialize(
        segment_proto.polynomial_coefficients(), &segment.p.coefficients);
    segment.t = segment_proto.time_seconds();
    segment_vector->emplace_back(segment);
  }
}

void serializeVertices(
    const Path4D<kDefaultN>::Vertex4DList& vertices,
    proto::Vertex4DList* proto) {
  CHECK_NOTNULL(proto);
  for (const Vertex4D& vertex : vertices) {
    proto::Vertex4D* vertex_proto = proto->add_vertices();
    for (const Vertex4D::Constraints::value_type& constraint :
         vertex.constraints) {
      vertex_proto->set_time_to_next(vertex.time_to_next);
      vertex_proto->set_derivative_to_optimize(vertex.derivative_to_optimize);
      proto::Constraint* constraint_proto = vertex_proto->add_constraints();
      constraint_proto->set_derivative(constraint.first);
      common::eigen_proto::serialize(
          constraint.second, constraint_proto->mutable_constraint_value());
    }
  }
}

void deserializeVertices(
    const proto::Vertex4DList& proto,
    Path4D<kDefaultN>::Vertex4DList* vertices) {
  CHECK_NOTNULL(vertices)->clear();
  for (const proto::Vertex4D& vertex_proto : proto.vertices()) {
    Vertex4D vertex;
    for (const proto::Constraint& constraint_proto :
         vertex_proto.constraints()) {
      Vertex4D::ConstraintValueT constraint_value;
      common::eigen_proto::deserialize(
          constraint_proto.constraint_value(), &constraint_value);
      vertex.constraints.emplace(
          constraint_proto.derivative(), constraint_value);
    }
    vertices->emplace_back(vertex);
  }
}

void serializePath4d(const Path4D<kDefaultN>& path_4d, proto::Path4D* proto) {
  CHECK_NOTNULL(proto);
  serializeSegmentVector(path_4d.getSegmentsX(), proto->mutable_segments_x());
  serializeSegmentVector(path_4d.getSegmentsY(), proto->mutable_segments_y());
  serializeSegmentVector(path_4d.getSegmentsZ(), proto->mutable_segments_z());
  serializeSegmentVector(
      path_4d.getSegmentsYaw(), proto->mutable_segments_yaw());
  serializeVertices(path_4d.getVertices(), proto->mutable_vertices());
}

void deserializePath4d(const proto::Path4D& proto, Path4D<kDefaultN>* path_4d) {
  CHECK_NOTNULL(path_4d);
  deserializeSegmentVector(proto.segments_x(), path_4d->getSegmentsXPtr());
  deserializeSegmentVector(proto.segments_y(), path_4d->getSegmentsYPtr());
  deserializeSegmentVector(proto.segments_z(), path_4d->getSegmentsZPtr());
  deserializeSegmentVector(proto.segments_yaw(), path_4d->getSegmentsYawPtr());
  deserializeVertices(proto.vertices(), path_4d->getVerticesPtr());
}

bool getFileNameForPathType(
    const test_trajectory_gen::Path path_type, std::string* file_path) {
  CHECK_NOTNULL(file_path)->clear();
  const char* simulation_share_dir = getenv("SIMULATION_SHARE_DIR");
  CHECK_NOTNULL(simulation_share_dir);
  std::string file_name;
  switch (path_type) {
    case test_trajectory_gen::Path::kCircular:
      file_name = "test_paths/circular.proto";
      break;
    case test_trajectory_gen::Path::kElliptical:
      file_name = "test_paths/elliptical.proto";
      break;
    case test_trajectory_gen::Path::kRotationOnly:
      file_name = "test_paths/rotation_only.proto";
      break;
    case test_trajectory_gen::Path::kTranslationOnly:
      file_name = "test_paths/translation_only.proto";
      break;
    case test_trajectory_gen::Path::kFromFile:
      file_name = "test_paths/file.proto";
      break;
    case test_trajectory_gen::Path::kFromCtor:
      file_name = "test_paths/ctor.proto";
      break;
    default:
      LOG(FATAL) << "Requested path type does not exist.";
      return false;
  }
  common::concatenateFolderAndFileName(
      std::string(simulation_share_dir), file_name, file_path);
  return true;
}

void writePath4dToFile(
    const test_trajectory_gen::Path path_type,
    const Path4D<kDefaultN>& path_4d) {
  // Verify that the version of the library that we linked against is compatible
  // with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  std::string file_path;
  CHECK(getFileNameForPathType(path_type, &file_path));
  CHECK(!file_path.empty());
  CHECK(common::fileExists(file_path));
  std::ofstream file(file_path);
  proto::Path4D proto;
  serializePath4d(path_4d, &proto);
  CHECK(proto.SerializeToOstream(&file));
}

void readPath4dFromFile(
    const test_trajectory_gen::Path path_type, Path4D<kDefaultN>* path_4d) {
  CHECK_NOTNULL(path_4d);
  // Verify that the version of the library that we linked against is compatible
  // with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  std::string file_path;
  CHECK(getFileNameForPathType(path_type, &file_path));
  CHECK(!file_path.empty());
  CHECK(common::fileExists(file_path)) << file_path;
  std::ifstream file(file_path);
  proto::Path4D proto;
  CHECK(proto.ParseFromIstream(&file));
  deserializePath4d(proto, path_4d);
}

}  // namespace path_planning
}  // namespace mav_planning_utils
