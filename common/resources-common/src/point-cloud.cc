#include "resources-common/point-cloud.h"

#include <numeric>

#include <draco/compression/encode.h>
#include <draco/io/file_utils.h>
#include <draco/io/point_cloud_io.h>
#include <draco/point_cloud/point_cloud_builder.h>

namespace resources {

void PointCloud::writeToFileCompressed(const std::string& file_path) const {
  CHECK_GE(FLAGS_resources_pointcloud_compression_speed, 0);
  CHECK_GE(10, FLAGS_resources_pointcloud_compression_speed);
  CHECK_GE(FLAGS_resources_pointcloud_compression_quantization_bits, 1);
  CHECK_GE(31, FLAGS_resources_pointcloud_compression_quantization_bits);
  draco::PointCloudBuilder builder;
  builder.Start(size());

  const int position_att_id = builder.AddAttribute(
      draco::GeometryAttribute::POSITION, 3, draco::DT_FLOAT32);
  builder.SetAttributeValuesForAllPoints(position_att_id, xyz.data(), 0);

  if (!normals.empty()) {
    const int normals_att_id = builder.AddAttribute(
        draco::GeometryAttribute::NORMAL, 3, draco::DT_FLOAT32);
    builder.SetAttributeValuesForAllPoints(normals_att_id, normals.data(), 0);
  }
  if (!colors.empty()) {
    const int colors_att_id = builder.AddAttribute(
        draco::GeometryAttribute::COLOR, 3, draco::DT_UINT8);
    builder.SetAttributeValuesForAllPoints(colors_att_id, colors.data(), 0);
  }
  int scalars_att_id;
  if (!scalars.empty()) {
    scalars_att_id = builder.AddAttribute(
        draco::GeometryAttribute::GENERIC, 1, draco::DT_FLOAT32);
    builder.SetAttributeValuesForAllPoints(scalars_att_id, scalars.data(), 0);
  }
  int labels_att_id;
  if (!labels.empty()) {
    labels_att_id = builder.AddAttribute(
        draco::GeometryAttribute::GENERIC, 1, draco::DT_UINT32);
    builder.SetAttributeValuesForAllPoints(labels_att_id, labels.data(), 0);
  }

  // Draco encoding does not maintain point order.
  // Save point index in additional field entry.
  int index_att_id;
  if (FLAGS_resources_pointcloud_compression_add_indices) {
    std::vector<uint32_t> indices(size());
    std::iota(indices.begin(), indices.end(), 0);
    index_att_id = builder.AddAttribute(
        draco::GeometryAttribute::GENERIC, 1, draco::DT_UINT32);
    builder.SetAttributeValuesForAllPoints(index_att_id, indices.data(), 0);
  }

  std::unique_ptr<draco::PointCloud> draco_pointcloud = builder.Finalize(false);
  CHECK_NOTNULL(draco_pointcloud);

  if (FLAGS_resources_pointcloud_compression_add_indices) {
    std::unique_ptr<draco::AttributeMetadata> index_metadata =
        std::unique_ptr<draco::AttributeMetadata>(
            new draco::AttributeMetadata());
    index_metadata->AddEntryString("name", "index");
    draco_pointcloud->AddAttributeMetadata(
        index_att_id, std::move(index_metadata));
  }
  if (!scalars.empty()) {
    std::unique_ptr<draco::AttributeMetadata> scalars_metadata =
        std::unique_ptr<draco::AttributeMetadata>(
            new draco::AttributeMetadata());
    scalars_metadata->AddEntryString("name", "scalars");
    draco_pointcloud->AddAttributeMetadata(
        scalars_att_id, std::move(scalars_metadata));
  }
  if (!labels.empty()) {
    std::unique_ptr<draco::AttributeMetadata> labels_metadata =
        std::unique_ptr<draco::AttributeMetadata>(
            new draco::AttributeMetadata());
    labels_metadata->AddEntryString("name", "labels");
    draco_pointcloud->AddAttributeMetadata(
        labels_att_id, std::move(labels_metadata));
  }
  std::filebuf filebuf;
  filebuf.open(file_path, std::ios::out | std::ios::binary);
  CHECK(filebuf.is_open());
  std::ostream output_stream(&filebuf);
  draco::PointCloudEncodingMethod method = draco::POINT_CLOUD_KD_TREE_ENCODING;
  draco::EncoderOptions options = draco::EncoderOptions::CreateDefaultOptions();
  options.SetSpeed(
      FLAGS_resources_pointcloud_compression_speed,
      FLAGS_resources_pointcloud_compression_speed);
  options.SetGlobalInt(
      "quantization_bits",
      FLAGS_resources_pointcloud_compression_quantization_bits);
  draco::WritePointCloudIntoStream(
      draco_pointcloud.get(), output_stream, method, options);
}

bool PointCloud::loadFromCompressedFile(const std::string& file_path) {
  std::ifstream input_file(file_path, std::ios::binary);
  if (!input_file) {
    LOG(ERROR) << "Failed opening the input file.";
    return false;
  }

  // Read the file stream into a buffer.
  std::streampos file_size = 0;
  input_file.seekg(0, std::ios::end);
  file_size = input_file.tellg() - file_size;
  input_file.seekg(0, std::ios::beg);
  std::vector<char> data(file_size);
  input_file.read(data.data(), file_size);

  if (data.empty()) {
    LOG(ERROR) << "Empty input file.";
    return false;
  }

  draco::DecoderBuffer buffer;
  buffer.Init(data.data(), data.size());

  std::unique_ptr<draco::PointCloud> pc;

  auto type_statusor = draco::Decoder::GetEncodedGeometryType(&buffer);
  if (!type_statusor.ok()) {
    LOG(ERROR) << "Draco geometry type is unknown.";
    return false;
  }
  const draco::EncodedGeometryType geom_type = type_statusor.value();
  if (geom_type != draco::POINT_CLOUD) {
    LOG(ERROR) << "Draco Geometry Type " << geom_type << " is not supported";
    return false;
  }

  draco::Decoder decoder;
  auto statusor = decoder.DecodePointCloudFromBuffer(&buffer);
  if (!statusor.ok()) {
    LOG(ERROR) << "Draco decoding failed.";
    return false;
  }
  pc = std::move(statusor).value();
  CHECK_NOTNULL(pc);

  LOG(WARNING) << "lolo";
  draco::PointIndex::ValueType number_of_points = pc->num_points();
  std::vector<uint32_t> indices(number_of_points);
  int index_attribute_id = pc->GetAttributeIdByMetadataEntry("name", "index");
  LOG(WARNING) << "huhu";
  LOG(WARNING) << index_attribute_id;
  if (index_attribute_id >= 0) {
    const draco::PointAttribute* index_attribute =
        pc->attribute(index_attribute_id);
    CHECK_NOTNULL(index_attribute);
    for (draco::PointIndex::ValueType point_index = 0;
         point_index < number_of_points; point_index++) {
      uint32_t* out_data = &indices[point_index];
      index_attribute->GetValue(
          draco::AttributeValueIndex(point_index), out_data);
      CHECK_GE(number_of_points - 1u, *out_data);
    }
    LOG(WARNING) << "lasdf";
  } else {
    LOG(WARNING) << "hhugs";
    std::iota(indices.begin(), indices.end(), 0);
  }
  LOG(INFO) << "easy";
  const int32_t number_of_attributes = pc->num_attributes();
  for (uint32_t att_id = 0; att_id < number_of_attributes; att_id++) {
    const draco::PointAttribute* attribute = pc->attribute(att_id);

    if (!attribute->IsValid()) {
      LOG(ERROR) << "Attribute of Draco pointcloud is not valid!";
      continue;
    }

    if (attribute->attribute_type() == draco::GeometryAttribute::POSITION) {
      xyz.resize(3 * number_of_points);
      for (draco::PointIndex::ValueType point_index = 0;
           point_index < number_of_points; point_index++) {
        float* out_data = &xyz[3 * indices[point_index]];
        CHECK_NOTNULL(out_data);
        attribute->GetValue(draco::AttributeValueIndex(point_index), out_data);
      }
    } else if (
        attribute->attribute_type() == draco::GeometryAttribute::NORMAL) {
      normals.resize(3 * number_of_points);
      for (draco::PointIndex::ValueType point_index = 0;
           point_index < number_of_points; point_index++) {
        float* out_data = &normals[3 * indices[point_index]];
        CHECK_NOTNULL(out_data);
        attribute->GetValue(draco::AttributeValueIndex(point_index), out_data);
      }
    } else if (attribute->attribute_type() == draco::GeometryAttribute::COLOR) {
      colors.resize(3 * number_of_points);
      for (draco::PointIndex::ValueType point_index = 0;
           point_index < number_of_points; point_index++) {
        unsigned char* out_data = &colors[3 * indices[point_index]];
        CHECK_NOTNULL(out_data);
        attribute->GetValue(draco::AttributeValueIndex(point_index), out_data);
      }
    } else {
      auto metadata = pc->GetAttributeMetadataByAttributeId(att_id);
      CHECK_NOTNULL(metadata);
      std::string attribute_name;
      metadata->GetEntryString("name", &attribute_name);
      if (attribute_name == "scalars") {
        scalars.resize(number_of_points);
        for (draco::PointIndex::ValueType point_index = 0;
             point_index < number_of_points; point_index++) {
          float* out_data = &scalars[indices[point_index]];
          CHECK_NOTNULL(out_data);
          attribute->GetValue(
              draco::AttributeValueIndex(point_index), out_data);
        }
      } else if (attribute_name == "labels") {
        labels.resize(number_of_points);
        for (draco::PointIndex::ValueType point_index = 0;
             point_index < number_of_points; point_index++) {
          uint32_t* out_data = &labels[indices[point_index]];
          CHECK_NOTNULL(out_data);
          attribute->GetValue(
              draco::AttributeValueIndex(point_index), out_data);
        }
      }
    }
  }

  //  Check pointcloud consistency
  if (!colors.empty()) {
    CHECK(hasColor());
  }
  if (!normals.empty()) {
    CHECK(hasNormals());
  }
  if (!scalars.empty()) {
    CHECK(hasScalars());
  }
  if (!labels.empty()) {
    CHECK(hasLabels());
  }
  return true;
}
}  // namespace resources
