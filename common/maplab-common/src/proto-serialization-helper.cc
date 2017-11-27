#include "maplab-common/proto-serialization-helper.h"

#include <fstream>  // NOLINT
#include <string>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>

#include "maplab-common/file-system-tools.h"

DEFINE_int32(
    proto_max_size_megabytes, 256,
    "The maximum allowed size for protobuf structures. The hard limit for this "
    "is 2 gigabytes.");

DEFINE_bool(
    proto_use_compression, true,
    "If enabled, the protobufs will be compressed when storing and "
    "decompressed when loading.");

namespace common {
namespace proto_serialization_helper {

bool parseProtoFromFile(
    const std::string& folder_path, const std::string& file_name,
    google::protobuf::Message* proto) {
  constexpr bool kIsTextFormat = false;
  return parseProtoFromFile(folder_path, file_name, proto, kIsTextFormat);
}

bool parseProtoFromFile(
    const std::string& folder_path, const std::string& file_name,
    google::protobuf::Message* proto, const bool is_text_format) {
  CHECK(!folder_path.empty());
  CHECK(!file_name.empty());
  CHECK_NOTNULL(proto);

  std::string complete_file_path;
  common::concatenateFolderAndFileName(
      folder_path, file_name, &complete_file_path);

  std::ifstream file_stream(complete_file_path);
  if (!file_stream.is_open()) {
    LOG(ERROR) << "Could not open file \"" << complete_file_path << "\"!";
    return false;
  }

  bool proto_parse_successful = false;
  if (is_text_format) {
    google::protobuf::io::IstreamInputStream proto_istream_input_stream(
        &file_stream);
    proto_parse_successful =
        google::protobuf::TextFormat::Parse(&proto_istream_input_stream, proto);
  } else {
    CHECK_GT(FLAGS_proto_max_size_megabytes, 0);
    constexpr int kProtobufHardLimitMegabytes = 2000;
    CHECK_LT(FLAGS_proto_max_size_megabytes, kProtobufHardLimitMegabytes);
    constexpr int kMegabytesToBytes = 1e6;
    const int max_proto_size_bytes =
        kMegabytesToBytes * FLAGS_proto_max_size_megabytes;
    if (FLAGS_proto_use_compression) {
      google::protobuf::io::IstreamInputStream proto_istream_input_stream(
          &file_stream);
      google::protobuf::io::GzipInputStream proto_gzip_input_stream(
          &proto_istream_input_stream);
      google::protobuf::io::CodedInputStream proto_coded_input_stream(
          &proto_gzip_input_stream);
      proto_coded_input_stream.SetTotalBytesLimit(
          max_proto_size_bytes, max_proto_size_bytes);
      proto_parse_successful =
          proto->ParseFromCodedStream(&proto_coded_input_stream);
    } else {
      proto_parse_successful = proto->ParseFromIstream(&file_stream);
    }
  }

  if (!proto_parse_successful) {
    LOG(ERROR) << "Error parsing file \"" << complete_file_path
               << "\" in protobuf. Either the format is invalid or the "
                  "protobuf structure is too large (try to increase the "
                  "max_proto_size_megabytes flag in this case).";
    return false;
  }
  return true;
}

bool serializeProtoToFile(
    const std::string& folder_path, const std::string& file_name,
    const google::protobuf::Message& proto) {
  constexpr bool kUseTextFormat = false;
  return serializeProtoToFile(folder_path, file_name, proto, kUseTextFormat);
}

bool serializeProtoToFile(
    const std::string& folder_path, const std::string& file_name,
    const google::protobuf::Message& proto, const bool use_text_format) {
  CHECK(!folder_path.empty());
  CHECK(!file_name.empty());
  std::string complete_file_path;
  common::concatenateFolderAndFileName(
      folder_path, file_name, &complete_file_path);

  std::ofstream file_stream(complete_file_path, std::ofstream::out);
  if (!file_stream.is_open()) {
    LOG(ERROR) << "Error writing to file\"" << complete_file_path << "\".";
    return false;
  }

  bool proto_serialization_successful = false;
  if (use_text_format) {
    google::protobuf::io::OstreamOutputStream proto_ostream_output_stream(
        &file_stream);
    proto_serialization_successful = google::protobuf::TextFormat::Print(
        proto, &proto_ostream_output_stream);
  } else {
    if (FLAGS_proto_use_compression) {
      google::protobuf::io::OstreamOutputStream proto_ostream_output_stream(
          &file_stream);
      google::protobuf::io::GzipOutputStream proto_gzip_output_stream(
          &proto_ostream_output_stream);
      google::protobuf::io::CodedOutputStream proto_coded_output_stream(
          &proto_gzip_output_stream);

      proto_serialization_successful =
          proto.SerializeToCodedStream(&proto_coded_output_stream);
    } else {
      proto_serialization_successful = proto.SerializeToOstream(&file_stream);
    }
  }
  if (!proto_serialization_successful) {
    LOG(ERROR) << "Error writing to file\"" << complete_file_path << "\".";
    return false;
  }
  return true;
}

void serializeToArray(
    const google::protobuf::Message& proto, void** raw_data,
    size_t* data_length_bytes) {
  CHECK_NOTNULL(raw_data);
  CHECK_NOTNULL(data_length_bytes);
  *data_length_bytes = proto.ByteSize();
  *raw_data = new uint8_t[*data_length_bytes];
  google::protobuf::io::ArrayOutputStream proto_array_output_stream(
      *raw_data, *data_length_bytes);
  google::protobuf::io::CodedOutputStream proto_coded_output_stream(
      &proto_array_output_stream);
  proto.SerializeToCodedStream(&proto_coded_output_stream);
}

void deserializeFromArray(
    const void* raw_data, const size_t data_length_bytes,
    google::protobuf::Message* proto) {
  CHECK_NOTNULL(raw_data);
  CHECK_NOTNULL(proto);
  constexpr int kMaxProtoSizeBytes = 128 * 1024 * 1024;
  google::protobuf::io::ArrayInputStream proto_array_input_stream(
      raw_data, data_length_bytes);
  google::protobuf::io::CodedInputStream proto_coded_input_stream(
      &proto_array_input_stream);
  proto_coded_input_stream.SetTotalBytesLimit(
      kMaxProtoSizeBytes, kMaxProtoSizeBytes);
  CHECK(proto->ParseFromCodedStream(&proto_coded_input_stream));
  CHECK(proto_coded_input_stream.ConsumedEntireMessage());
}

}  // namespace proto_serialization_helper
}  // namespace common
