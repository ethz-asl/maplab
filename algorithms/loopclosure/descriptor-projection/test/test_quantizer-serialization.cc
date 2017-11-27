#include <fstream>  // NOLINT
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <descriptor-projection/descriptor-projection.h>
#include <descriptor-projection/flags.h>
#include <descriptor-projection/projected-descriptor-quantizer.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <loopclosure-common/types.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>
#include <vocabulary-tree/helpers.h>
#include <vocabulary-tree/simple-kmeans.h>
#include <vocabulary-tree/tree-builder.h>

DECLARE_string(data_directory);

TEST(PlacelessLoopClosure, QuantizerSerialization) {
  std::mt19937 generator(42);
  static const uint32_t kNumWords = 100;
  static const int kDescriptorBytes = 64;

  static const int kTargetDimensionality = 10;

  std::string quantizer_filename = "./quantizer.dat";
  descriptor_projection::ProjectedDescriptorQuantizer quantizer(
      kTargetDimensionality);
  {
    quantizer.projection_matrix_.setRandom(
        loop_closure::kFreakDescriptorLengthBits,
        loop_closure::kFreakDescriptorLengthBits);

    descriptor_projection::ProjectedDescriptorType descriptor_zero;
    descriptor_zero.setConstant(kTargetDimensionality, 1, 0);

    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> descriptors;
    descriptors.setRandom(kDescriptorBytes, 40 * 100);

    descriptor_projection::DescriptorVector projected_descriptors;
    projected_descriptors.resize(descriptors.cols(), descriptor_zero);
    for (int i = 0; i < descriptors.cols(); ++i) {
      aslam::common::FeatureDescriptorConstRef raw_descriptor(
          &descriptors.coeffRef(0, i), kDescriptorBytes);

      descriptor_projection::ProjectDescriptor(
          raw_descriptor, quantizer.projection_matrix_,
          quantizer.target_dimensionality_, projected_descriptors[i]);
    }

    // Create tree.
    typedef loop_closure::TreeBuilder<
        descriptor_projection::ProjectedDescriptorType,
        loop_closure::distance::L2<
            descriptor_projection::ProjectedDescriptorType> >
        TreeBuilder;
    TreeBuilder builder(descriptor_zero);
    builder.kmeans().SetRestarts(5);
    builder.Build(projected_descriptors, kNumWords, 1);
    VLOG(3) << "Done. Got " << builder.tree().centers().size() << " centers";

    quantizer.vocabulary_ = builder.tree();

    std::ofstream ofs(quantizer_filename.c_str());
    ASSERT_TRUE(ofs.is_open());

    quantizer.Save(&ofs);
  }

  descriptor_projection::ProjectedDescriptorQuantizer quantizer_load(
      kTargetDimensionality);

  EXPECT_FALSE(quantizer_load.vocabulary_.IsBinaryEqual(quantizer.vocabulary_));

  std::ifstream ifs(quantizer_filename.c_str());
  ASSERT_TRUE(ifs.is_open());
  ASSERT_TRUE(ifs.good());
  quantizer_load.Load(&ifs);

  EXPECT_TRUE(
      quantizer_load.projection_matrix_ == quantizer.projection_matrix_);
  EXPECT_EQ(
      quantizer_load.target_dimensionality_, quantizer.target_dimensionality_);
  EXPECT_TRUE(quantizer_load.vocabulary_.IsBinaryEqual(quantizer.vocabulary_));
}

MAPLAB_UNITTEST_ENTRYPOINT
