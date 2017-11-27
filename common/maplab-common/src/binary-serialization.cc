#include <maplab-common/binary-serialization.h>

#include <glog/logging.h>

namespace common {

void Serialize(const std::string& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  uint32_t length = value.size();
  Serialize(length, out);
  out->write(
      reinterpret_cast<const char*>(value.data()), length * sizeof(value[0]));
}

void Deserialize(std::string* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  uint32_t length;
  Deserialize(&length, in);
  value->resize(length);
  std::unique_ptr<char[]> mem(new char[length + 1]);
  in->read(mem.get(), length * sizeof(mem.get()[0]));
  CHECK_EQ(
      in->gcount(),
      static_cast<std::streamsize>(length * sizeof(mem.get()[0])));
  mem[length] = '\0';
  *value = std::string(mem.get());
}

void Serialize(const uint32_t& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  out->write(reinterpret_cast<const char*>(&value), sizeof(value));
}

void Deserialize(uint32_t* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(value), sizeof(*value));
  CHECK_EQ(in->gcount(), static_cast<std::streamsize>(sizeof(*value)));
}

void Serialize(
    const char* memory_start, unsigned int memory_block_length_bytes,
    std::ostream* out) {
  CHECK_NOTNULL(memory_start);
  CHECK_NOTNULL(out);
  out->write(memory_start, memory_block_length_bytes);
}

void Deserialize(
    char* memory_start, unsigned int memory_block_length_bytes,
    std::istream* in) {
  CHECK_NOTNULL(memory_start);
  CHECK_NOTNULL(in);
  in->read(memory_start, memory_block_length_bytes);
  CHECK_EQ(in->gcount(), memory_block_length_bytes);
}

void Serialize(
    const unsigned char* memory_start, unsigned int memory_block_length_bytes,
    std::ostream* out) {
  CHECK_NOTNULL(memory_start);
  CHECK_NOTNULL(out);
  out->write(
      reinterpret_cast<const char*>(memory_start), memory_block_length_bytes);
}

void Deserialize(
    unsigned char* memory_start, unsigned int memory_block_length_bytes,
    std::istream* in) {
  CHECK_NOTNULL(memory_start);
  CHECK_NOTNULL(in);
  in->read(reinterpret_cast<char*>(memory_start), memory_block_length_bytes);
  CHECK_EQ(in->gcount(), memory_block_length_bytes);
}

void Serialize(const aslam::HashId& value, std::ostream* out) {
  CHECK_NOTNULL(out);
  Serialize(value.hexString(), out);
}

void Deserialize(aslam::HashId* value, std::istream* in) {
  CHECK_NOTNULL(value);
  CHECK_NOTNULL(in);
  std::string string_value;
  Deserialize(&string_value, in);
  value->fromHexString(string_value);
}

}  // namespace common
