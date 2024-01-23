#include <glog/logging.h>
#include <aslam/common/channel-serialization.h>

namespace aslam {
namespace internal {
size_t HeaderInformation::size() const {
  return sizeof(rows) + sizeof(cols) + sizeof(depth) + sizeof(channels);
}

bool HeaderInformation::serializeToBuffer(char* buffer, size_t offset) const {
  CHECK_NOTNULL(buffer);
  buffer += offset;
  memcpy(buffer, &rows, sizeof(rows));
  buffer += sizeof(rows);
  memcpy(buffer, &cols, sizeof(cols));
  buffer += sizeof(cols);
  memcpy(buffer, &depth, sizeof(depth));
  buffer += sizeof(depth);
  memcpy(buffer, &channels, sizeof(channels));
  buffer += sizeof(channels);
  return true;
}

bool HeaderInformation::deSerializeFromBuffer(const char* const buffer_in, size_t offset) {
  CHECK_NOTNULL(buffer_in);
  const char* buffer = buffer_in;
  buffer += offset;
  memcpy(&rows, buffer, sizeof(rows));
  buffer += sizeof(rows);
  memcpy(&cols, buffer, sizeof(cols));
  buffer += sizeof(cols);
  memcpy(&depth, buffer, sizeof(depth));
  buffer += sizeof(depth);
  memcpy(&channels, buffer, sizeof(channels));
  buffer += sizeof(channels);
  return true;
}

bool serializeToString(const cv::Mat& image,
                       std::string* string) {
  CHECK(image.isContinuous()) << "This method only works if the image is stored "
      "in contiguous memory.";
  CHECK_EQ(image.total(), static_cast<size_t>(image.rows * image.cols)) <<
      "Unexpected number of pixels in the image.";
  CHECK_EQ(image.dims, 2) << "This method only works for 2D arrays";
  const char* const matrixData = reinterpret_cast<const char*>(image.data);
  bool success = false;
  // http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-depth
  switch(image.depth()) {
    case cv::DataType<uint8_t>::depth:
    success = serializeToString<uint8_t>(matrixData, image.rows,
                                         image.cols, image.channels(),
                                         string);
    break;
    case cv::DataType<int8_t>::depth:
    success = serializeToString<int8_t>(matrixData, image.rows,
                                        image.cols, image.channels(),
                                        string);
    break;
    case cv::DataType<uint16_t>::depth:
    success = serializeToString<uint16_t>(matrixData, image.rows,
                                          image.cols, image.channels(),
                                          string);
    break;
    case cv::DataType<int16_t>::depth:
    success = serializeToString<int16_t>(matrixData, image.rows,
                                         image.cols, image.channels(),
                                         string);
    break;
    case cv::DataType<int32_t>::depth:
    success = serializeToString<int32_t>(matrixData, image.rows,
                                     image.cols, image.channels(),
                                     string);
    break;
    case cv::DataType<double>::depth:
    success = serializeToString<double>(matrixData, image.rows,
                                        image.cols, image.channels(),
                                        string);
    break;
    case cv::DataType<float>::depth:
    success = serializeToString<float>(matrixData, image.rows,
                                       image.cols, image.channels(),
                                       string);
    break;
    default:
      LOG(FATAL) << "cv::Mat depth " << image.depth() << " is not supported for "
      << "serialization.";
      success = false;
      break;
  }
  return success;
}

bool deSerializeFromString(const std::string& string, cv::Mat* image) {
  CHECK_NOTNULL(image);
  return deSerializeFromBuffer(string.data(), string.size(), image);
}

template<typename SCALAR>
void deSerializeTypedFromBuffer(const char* const buffer, size_t size,
                                const HeaderInformation& header, cv::Mat* image) {
  CHECK_NOTNULL(buffer);
  CHECK_NOTNULL(image);
  size_t matrix_size = sizeof(SCALAR) * header.rows * header.cols * header.channels;
  size_t total_size = matrix_size + header.size();
  CHECK_EQ(size, total_size);
  int type = CV_MAKETYPE(cv::DataType<SCALAR>::depth, header.channels);
  // http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-create
  // Create should only allocate if necessary.
  image->create(header.rows, header.cols, type);
  memcpy(image->data, buffer + header.size(), matrix_size);
}

bool deSerializeFromBuffer(const char* const buffer, size_t size, cv::Mat* image) {
  CHECK_NOTNULL(image);
  HeaderInformation header;
  CHECK_GE(size, header.size());
  bool success = header.deSerializeFromBuffer(buffer, 0);
  if (!success) {
    LOG(FATAL) << "Failed to deserialize header from string: " <<
        std::string(buffer, size);
    return false;
  }

  // http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-depth
  switch(header.depth) {
    case cv::DataType<uint8_t>::depth:
    deSerializeTypedFromBuffer<uint8_t>(buffer, size, header, image);
    break;
    case cv::DataType<int8_t>::depth:
    deSerializeTypedFromBuffer<int8_t>(buffer, size, header, image);
    break;
    case cv::DataType<uint16_t>::depth:
    deSerializeTypedFromBuffer<uint16_t>(buffer, size, header, image);
    break;
    case cv::DataType<int16_t>::depth:
    deSerializeTypedFromBuffer<int16_t>(buffer, size, header, image);
    break;
    case cv::DataType<int32_t>::depth:
    deSerializeTypedFromBuffer<int32_t>(buffer, size, header, image);
    break;
    case cv::DataType<double>::depth:
    deSerializeTypedFromBuffer<double>(buffer, size, header, image);
    break;
    case cv::DataType<float>::depth:
    deSerializeTypedFromBuffer<float>(buffer, size, header, image);
    break;
    default:
      LOG(FATAL) << "cv::Mat depth " << header.depth << " is not supported for "
      << "serialization.";
      success = false;
      break;
  }

  return true;
}

bool serializeToBuffer(const cv::Mat& image, char** buffer, size_t* size) {
  CHECK(image.isContinuous()) << "This method only works if the image is stored "
      "in contiguous memory.";
  CHECK_EQ(image.total(), static_cast<size_t>(image.rows * image.cols)) <<
      "Unexpected number of pixels in the image.";
  CHECK_EQ(image.dims, 2) << "This method only works for 2D arrays";
  CHECK_NOTNULL(buffer);
  CHECK_NOTNULL(size);
  const char* const matrixData = reinterpret_cast<const char*>(image.data);
  bool success = false;
  // http://docs.opencv.org/modules/core/doc/basic_structures.html#mat-depth
  switch(image.depth()) {
    case cv::DataType<uint8_t>::depth:
    success = serializeToBuffer<uint8_t>(matrixData, image.rows,
                                         image.cols, image.channels(),
                                         buffer, size);
    break;
    case cv::DataType<int8_t>::depth:
    success = serializeToBuffer<int8_t>(matrixData, image.rows,
                                        image.cols, image.channels(),
                                        buffer, size);
    break;
    case cv::DataType<uint16_t>::depth:
    success = serializeToBuffer<uint16_t>(matrixData, image.rows,
                                          image.cols, image.channels(),
                                          buffer, size);
    break;
    case cv::DataType<int16_t>::depth:
    success = serializeToBuffer<int16_t>(matrixData, image.rows,
                                         image.cols, image.channels(),
                                         buffer, size);
    break;
    case cv::DataType<int32_t>::depth:
    success = serializeToBuffer<int32_t>(matrixData, image.rows,
                                     image.cols, image.channels(),
                                     buffer, size);
    break;
    case cv::DataType<double>::depth:
    success = serializeToBuffer<double>(matrixData, image.rows,
                                        image.cols, image.channels(),
                                        buffer, size);
    break;
    case cv::DataType<float>::depth:
    success = serializeToBuffer<float>(matrixData, image.rows,
                                       image.cols, image.channels(),
                                       buffer, size);
    break;
    default:
      LOG(FATAL) << "cv::Mat depth " << image.depth() << " is not supported for "
      << "serialization.";
      success = false;
      break;
  }
  return success;
}

}  // namespace internal
}  // namespace aslam
