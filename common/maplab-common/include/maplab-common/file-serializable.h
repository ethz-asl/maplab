#ifndef MAPLAB_COMMON_FILE_SERIALIZABLE_H_
#define MAPLAB_COMMON_FILE_SERIALIZABLE_H_

#include <fstream>  // NOLINT
#include <limits>
#include <string>

#include <glog/logging.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace common {

template <typename ProtoType>
class FileSerializable {
 public:
  virtual ~FileSerializable() {}

  virtual void serialize(ProtoType* proto) const = 0;
  virtual void deserialize(const ProtoType& proto) = 0;

  // Existing files get overwritten.
  bool serializeToFile(const std::string& file_name) const
      __attribute__((warn_unused_result)) {
    std::ofstream file(file_name);
    if (!file.is_open()) {
      LOG(ERROR) << "Couldn't open file " << file_name << ".";
      return false;
    }
    ProtoType proto;
    serialize(&proto);
    return proto.SerializeToOstream(&file);
  }

  bool deserializeFromFile(const std::string& file_name)
      __attribute__((warn_unused_result)) {
    std::ifstream file(file_name);
    if (!file.is_open()) {
      LOG(ERROR) << "Couldn't open file " << file_name << ".";
      return false;
    }
    google::protobuf::io::IstreamInputStream raw_in(&file);
    google::protobuf::io::CodedInputStream coded_in(&raw_in);
    coded_in.SetTotalBytesLimit(std::numeric_limits<int>::max(), -1);
    ProtoType proto;
    if (!proto.ParseFromCodedStream(&coded_in)) {
      LOG(ERROR) << "Failed to parse proto from stream " << file_name << ".";
      return false;
    }
    deserialize(proto);
    return true;
  }
};

}  // namespace common

#endif  // MAPLAB_COMMON_FILE_SERIALIZABLE_H_
