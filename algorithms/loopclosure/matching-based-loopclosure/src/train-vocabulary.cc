#include "matching-based-loopclosure/train-vocabulary.h"

#include <algorithm>
#include <cstdio>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/map-track-extractor.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <loopclosure-common/types.h>
#include <vi-map/vi-map.h>
#include <vocabulary-tree/tree-builder.h>

#include "matching-based-loopclosure/detector-settings.h"
#include "matching-based-loopclosure/inverted-index-interface.h"
#include "matching-based-loopclosure/inverted-multi-index-interface.h"

DECLARE_string(data_directory);
DEFINE_int32(
    lc_number_of_vocabulary_words, 1000, "Number of words in the vocabulary.");
DEFINE_int32(
    lc_num_descriptors_to_train, 100000,
    "Number of descriptors used for training.");
DEFINE_int32(
    lc_product_quantization_num_components, 1,
    "Number of components for product quantization.");
DEFINE_int32(
    lc_product_quantization_num_dim_per_component, 5,
    "Number of components for product quantization.");
DEFINE_int32(
    lc_product_quantization_num_words, 256,
    "Number of words in the product vocabulary.");

DECLARE_string(load_map);

using descriptor_projection::DescriptorVector;
using descriptor_projection::FeatureAllocator;
using descriptor_projection::ProjectedDescriptorType;

namespace loop_closure {
typedef loop_closure::TreeBuilder<
    ProjectedDescriptorType,
    loop_closure::distance::L2<ProjectedDescriptorType>, FeatureAllocator>
    ProjectedTreeBuilder;

void MakeVocabulary(
    int num_words, const DescriptorVector& descriptors,
    int descriptor_dimensionality, Eigen::MatrixXf* words) {
  CHECK_NOTNULL(words);
  CHECK(!descriptors.empty());

  ProjectedDescriptorType descriptor_zero;
  descriptor_zero.setConstant(descriptor_dimensionality, 1, 0);

  // Create tree.
  static constexpr int kLevels = 1;
  ProjectedTreeBuilder builder(descriptor_zero);
  builder.kmeans().SetRestarts(1);
  builder.Build(descriptors, num_words, kLevels);
  VLOG(3) << "Done. Got " << builder.tree().centers().size() << " centers";

  const descriptor_projection::DescriptorVector& centers =
      builder.tree().centers();
  words->resize(descriptor_dimensionality, centers.size());
  for (size_t i = 0; i < centers.size(); ++i) {
    CHECK_EQ(centers[i].rows(), descriptor_dimensionality);
    words->block(0, i, descriptor_dimensionality, 1) = centers[i];
  }
}

void LoadBinaryFeaturesFromDataset(
    const vi_map::VIMap& map, loop_closure::DescriptorContainer* descriptors) {
  CHECK_NOTNULL(descriptors);

  // Get the descriptor-length.
  unsigned int descriptor_size = -1;
  unsigned int raw_descriptor_matching_threshold = 70;
  if (FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorFREAK) {
    descriptor_size = loop_closure::kFreakDescriptorLengthBits;
  } else if (
      FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorBRISK) {
    descriptor_size = loop_closure::kBriskDescriptorLengthBits;
  } else {
    CHECK(false) << "Unknown feature descriptor "
                 << FLAGS_feature_descriptor_type;
  }

  CHECK_NOTNULL(descriptors);

  vi_map::MissionIdList all_mission_ids;
  map.getAllMissionIds(&all_mission_ids);

  loop_closure::DescriptorContainer all_descriptors;
  for (const vi_map::MissionId& mission_id : all_mission_ids) {
    std::vector<descriptor_projection::Track> tracks;

    loop_closure::DescriptorContainer mission_descriptors;
    using descriptor_projection::CollectAndConvertDescriptors;
    CollectAndConvertDescriptors(
        map, mission_id, descriptor_size, raw_descriptor_matching_threshold,
        &mission_descriptors, &tracks);
    int old_size = all_descriptors.cols();
    all_descriptors.conservativeResize(
        Eigen::NoChange, old_size + mission_descriptors.cols());
    all_descriptors.block(
        0, old_size, mission_descriptors.rows(), mission_descriptors.cols()) =
        mission_descriptors;
    if (all_descriptors.cols() >= FLAGS_lc_num_descriptors_to_train) {
      break;
    }
  }

  if (all_descriptors.cols() > FLAGS_lc_num_descriptors_to_train) {
    LOG(WARNING) << "Truncated number of descriptors "
                 << FLAGS_lc_num_descriptors_to_train;
    all_descriptors.conservativeResize(
        Eigen::NoChange, FLAGS_lc_num_descriptors_to_train);
  }
}

void ProjectDescriptors(
    const Eigen::MatrixXf& projection_matrix,
    const loop_closure::DescriptorContainer& descriptors,
    DescriptorVector* projected_descriptors) {
  CHECK_NOTNULL(projected_descriptors);
  VLOG(3) << "Got " << descriptors.cols()
          << " descriptors to train the vocabulary tree from.";

  CHECK_NE(descriptors.cols(), 0);

  ProjectedDescriptorType descriptor_zero;
  descriptor_zero.setConstant(FLAGS_lc_target_dimensionality, 1, 0);

  projected_descriptors->resize(descriptors.cols(), descriptor_zero);
  for (int i = 0; i < descriptors.cols(); ++i) {
    aslam::common::FeatureDescriptorConstRef raw_descriptor(
        &descriptors.coeffRef(0, i), descriptors.rows());
    descriptor_projection::ProjectDescriptor(
        raw_descriptor, projection_matrix, FLAGS_lc_target_dimensionality,
        (*projected_descriptors)[i]);
    if (i % 10000 == 0) {
      LOG(INFO) << "Projected " << i << "/" << descriptors.cols();
    }
  }
}

void MakeProductVocabularies(
    const DescriptorVector& input_descriptors,
    const Eigen::MatrixXf& base_vocabulary,
    Eigen::MatrixXf* product_vocabulary) {
  CHECK_NOTNULL(product_vocabulary);
  Aligned<std::vector, DescriptorVector> descriptors_v1;
  descriptors_v1.resize(base_vocabulary.cols());

  for (const ProjectedDescriptorType& projected_descriptor :
       input_descriptors) {
    int best_word = 0;
    float best_distance = std::numeric_limits<float>::max();
    for (int i = 0; i < base_vocabulary.cols(); ++i) {
      float distance =
          (base_vocabulary.col(i) - projected_descriptor).squaredNorm();
      if (distance < best_distance) {
        best_distance = distance;
        best_word = i;
      }
    }
    descriptors_v1[best_word].push_back(projected_descriptor);
  }

  const int kHalfDescriptorLength = FLAGS_lc_target_dimensionality / 2;
  CHECK_LE(
      FLAGS_lc_product_quantization_num_components *
          FLAGS_lc_product_quantization_num_dim_per_component,
      kHalfDescriptorLength);

  // Now train kNumComponents k-means on the descriptors of every word.
  // Split up dimensions here and then run k-means on them.
  const int num_imi_words = base_vocabulary.cols();
  const int num_pq_words = FLAGS_lc_product_quantization_num_words;
  const int num_components = FLAGS_lc_product_quantization_num_components;
  const int num_dim_per_component =
      FLAGS_lc_product_quantization_num_dim_per_component;
  product_vocabulary->resize(
      num_dim_per_component, num_imi_words * num_components * num_pq_words);

  int num_components_stored = 0;
  const unsigned int num_descriptors_per_training = num_pq_words * 200u;

  for (const DescriptorVector& word_descriptors : descriptors_v1) {
    for (int component = 0;
         component < FLAGS_lc_product_quantization_num_components;
         ++component) {
      LOG(INFO) << "Training component " << num_components_stored << "/"
                << num_imi_words * num_components;
      DescriptorVector dim_for_component;
      dim_for_component.reserve(word_descriptors.size());
      const int start_block =
          component * FLAGS_lc_product_quantization_num_dim_per_component;
      const int block_size =
          FLAGS_lc_product_quantization_num_dim_per_component;
      for (const ProjectedDescriptorType& descriptor : word_descriptors) {
        ProjectedDescriptorType sub_descriptor =
            descriptor.block(start_block, 0, block_size, 1);
        dim_for_component.push_back(sub_descriptor);
        if (dim_for_component.size() >= num_descriptors_per_training) {
          break;
        }
      }
      LOG(INFO) << "Using " << dim_for_component.size() << " descriptors.";
      Eigen::MatrixXf words_product_vocabulary;
      MakeVocabulary(
          FLAGS_lc_product_quantization_num_words, dim_for_component,
          block_size, &words_product_vocabulary);
      // Now store product vocabulary for component.
      CHECK_LE(
          num_components_stored * num_pq_words + num_pq_words,
          product_vocabulary->cols());
      product_vocabulary->block(
          0, num_components_stored * num_pq_words, num_dim_per_component,
          num_pq_words) = words_product_vocabulary;
      ++num_components_stored;
    }
  }
  LOG(INFO) << "Done with product vocabulary";
}

void MakeVocabularies(
    const Eigen::MatrixXf& projection_matrix,
    const DescriptorVector& projected_descriptors) {
  VLOG(3) << "Creating vocabulary with " << FLAGS_lc_number_of_vocabulary_words
          << " words.";
  VLOG(3) << "Got " << projected_descriptors.size() << " descriptors.";

  const matching_based_loopclosure::MatchingBasedEngineSettings
      matching_based_settings;

  typedef matching_based_loopclosure::MatchingBasedEngineSettings::
      DetectorEngineType DetectorEngineType;

  if (matching_based_settings.detector_engine_type ==
          DetectorEngineType::kMatchingLDInvertedMultiIndex ||
      matching_based_settings.detector_engine_type ==
          DetectorEngineType::
              kMatchingLDInvertedMultiIndexProductQuantization) {
    CHECK_EQ(FLAGS_lc_target_dimensionality % 2, 0)
        << "Number of dimensions must be divisible by two.";
    const int kHalfDescriptorLength = FLAGS_lc_target_dimensionality / 2;
    // Split the descriptors in half:
    DescriptorVector projected_descriptors_first_half;
    DescriptorVector projected_descriptors_second_half;
    projected_descriptors_first_half.reserve(projected_descriptors.size());
    projected_descriptors_second_half.reserve(projected_descriptors.size());
    for (size_t i = 0; i < projected_descriptors.size(); ++i) {
      const ProjectedDescriptorType& projected_descriptor =
          projected_descriptors[i];
      ProjectedDescriptorType first_half =
          projected_descriptor.block(0, 0, kHalfDescriptorLength, 1);
      projected_descriptors_first_half.push_back(first_half);

      ProjectedDescriptorType second_half = projected_descriptor.block(
          kHalfDescriptorLength, 0, kHalfDescriptorLength, 1);
      projected_descriptors_second_half.push_back(second_half);
    }

    if (matching_based_settings.detector_engine_type ==
        DetectorEngineType::kMatchingLDInvertedMultiIndex) {
      loop_closure::InvertedMultiIndexVocabulary vocabulary;
      vocabulary.projection_matrix_ = projection_matrix;
      vocabulary.target_dimensionality_ = FLAGS_lc_target_dimensionality;

      LOG(INFO) << "Creating first vocabulary.";
      MakeVocabulary(
          FLAGS_lc_number_of_vocabulary_words, projected_descriptors_first_half,
          kHalfDescriptorLength, &vocabulary.words_first_half_);

      LOG(INFO) << "Creating second vocabulary.";
      MakeVocabulary(
          FLAGS_lc_number_of_vocabulary_words,
          projected_descriptors_second_half, kHalfDescriptorLength,
          &vocabulary.words_second_half_);

      std::ofstream out(
          FLAGS_lc_projected_quantizer_filename.c_str(), std::ios_base::binary);
      CHECK(out.is_open()) << "Failed to write quantizer file to "
                           << FLAGS_lc_projected_quantizer_filename;

      vocabulary.Save(&out);
    } else if (
        matching_based_settings.detector_engine_type ==
        DetectorEngineType::kMatchingLDInvertedMultiIndexProductQuantization) {
      // Trains a product quantizer from a given set of data points, resulting
      // in a set of cluster centers that can be passed onto an instance of
      // ProductQuantization using the SetClusterCenters function.
      // The template parameters are the number of components to be used for
      // product
      // quantization, the number of dimensions for each component, and the
      // number of
      // cluster centers.

      loop_closure::InvertedMultiIndexProductVocabulary vocabulary;
      vocabulary.projection_matrix_ = projection_matrix;
      vocabulary.target_dimensionality_ = FLAGS_lc_target_dimensionality;

      LOG(INFO) << "Creating first vocabulary.";
      MakeVocabulary(
          FLAGS_lc_number_of_vocabulary_words, projected_descriptors_first_half,
          kHalfDescriptorLength, &vocabulary.words_first_half_);

      LOG(INFO) << "Creating second vocabulary.";
      MakeVocabulary(
          FLAGS_lc_number_of_vocabulary_words,
          projected_descriptors_second_half, kHalfDescriptorLength,
          &vocabulary.words_second_half_);

      vocabulary.number_of_components =
          FLAGS_lc_product_quantization_num_components;
      vocabulary.number_of_centers = FLAGS_lc_product_quantization_num_words;
      vocabulary.number_of_dimensions_per_component =
          FLAGS_lc_product_quantization_num_dim_per_component;

      MakeProductVocabularies(
          projected_descriptors_first_half, vocabulary.words_first_half_,
          &vocabulary.quantizer_centers_1);

      MakeProductVocabularies(
          projected_descriptors_second_half, vocabulary.words_second_half_,
          &vocabulary.quantizer_centers_2);

      std::ofstream out(
          FLAGS_lc_projected_quantizer_filename.c_str(), std::ios_base::binary);
      CHECK(out.is_open()) << "Failed to write quantizer file to "
                           << FLAGS_lc_projected_quantizer_filename;

      LOG(INFO) << "Storing " << FLAGS_lc_projected_quantizer_filename;
      vocabulary.Save(&out);
    } else {
      LOG(FATAL) << "Training for this type is not supported.";
    }
  } else if (
      matching_based_settings.detector_engine_type ==
      DetectorEngineType::kMatchingLDInvertedIndex) {
    loop_closure::InvertedIndexVocabulary vocabulary;
    vocabulary.projection_matrix_ = projection_matrix;
    vocabulary.target_dimensionality_ = FLAGS_lc_target_dimensionality;
    LOG(INFO) << "Creating vocabulary.";
    MakeVocabulary(
        FLAGS_lc_number_of_vocabulary_words, projected_descriptors,
        FLAGS_lc_target_dimensionality, &vocabulary.words_);
    std::ofstream out(
        FLAGS_lc_projected_quantizer_filename.c_str(), std::ios_base::binary);
    CHECK(out.is_open()) << "Failed to write quantizer file to "
                         << FLAGS_lc_projected_quantizer_filename;

    vocabulary.Save(&out);
  } else {
    LOG(FATAL) << "Training for type is not supported.";
  }
}

void TrainProjectedVocabulary(const vi_map::VIMap& map) {
  CHECK_NE(FLAGS_lc_projected_quantizer_filename, "")
      << "You have to provide a filename to write the quantizer to.";

  CHECK_NE(FLAGS_lc_projection_matrix_filename, "")
      << "You have to provide a filename for the projection matrix.";

  std::ifstream deserializer(FLAGS_lc_projection_matrix_filename);
  CHECK(deserializer.is_open()) << "Cannot load projection matrix from file: "
                                << FLAGS_lc_projection_matrix_filename;

  Eigen::MatrixXf projection_matrix;
  common::Deserialize(&projection_matrix, &deserializer);

  CHECK_NE(0, projection_matrix.rows());

  loop_closure::DescriptorContainer raw_descriptors;
  LoadBinaryFeaturesFromDataset(map, &raw_descriptors);

  DescriptorVector projected_descriptors;
  ProjectDescriptors(
      projection_matrix, raw_descriptors, &projected_descriptors);

  MakeVocabularies(projection_matrix, projected_descriptors);
  std::cout << "Done." << std::endl;
}
}  // namespace loop_closure
