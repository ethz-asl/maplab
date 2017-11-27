#ifndef MAPLAB_COMMON_PROTO_SERIALIZATION_HELPER_H_
#define MAPLAB_COMMON_PROTO_SERIALIZATION_HELPER_H_

#include <string>

#include <google/protobuf/message.h>

namespace common {
namespace proto_serialization_helper {

/// Parses a proto file named \p file_name in the folder \p folder_path and
/// deserializes it into the given protobuf object \p proto.
bool parseProtoFromFile(
    const std::string& folder_path, const std::string& file_name,
    google::protobuf::Message* proto);
bool parseProtoFromFile(
    const std::string& folder_path, const std::string& file_name,
    google::protobuf::Message* proto, const bool is_text_format);

/// Serializes the protobuf object given by \p proto and saves it in a file
/// named \p file_name in \p folder_path.
bool serializeProtoToFile(
    const std::string& folder_path, const std::string& file_name,
    const google::protobuf::Message& proto);
bool serializeProtoToFile(
    const std::string& folder_path, const std::string& file_name,
    const google::protobuf::Message& proto, const bool use_text_format);

// Note: raw_data needs to be manually deleted afterwards, the caller takes
// ownership of the data!
void serializeToArray(
    const google::protobuf::Message& proto, void** raw_data,
    size_t* data_length_bytes);
void deserializeFromArray(
    const void* raw_data, const size_t data_length_bytes,
    google::protobuf::Message* proto);

}  // namespace proto_serialization_helper
}  // namespace common

#endif  // MAPLAB_COMMON_PROTO_SERIALIZATION_HELPER_H_
