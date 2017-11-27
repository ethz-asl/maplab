#include <cstdint>

#include <Eigen/Core>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

#include <product-quantization/product-quantization.h>

namespace product_quantization {

class ProductQuantizationTest : public ::testing::Test {
 public:
  void SetUp() {
    centers_ << 0.0, 1.0, 2.0, 3.0, 4.0, 0.0, 0.0, 0.0, 7.0, 1.0, 5.0, 1.0, 0.0,
        2.0, 1.0, 2.0, 3.0, 4.0, 1.0, 0.0;

    quantized_vectors_ << 1, 1, 3, 0, 2, 3, 4, 2;
  }

  Matrix<float, 2, 10> centers_;
  Matrix<uint8_t, 2, 4> quantized_vectors_;
};

TEST_F(ProductQuantizationTest, QuantizeWorks) {
  ProductQuantization<2, 2, 5, uint8_t> product_quantizer(centers_);

  Matrix<float, 4, Dynamic> vectors;
  vectors.resize(4, 4);
  vectors << 1.0, 0.0, 5.0, 1.0, 2.0, 1.0, 6.0, 4.0, 3.0, 4.0, 0.0, 1.0, 4.0,
      1.0, 0.0, 4.0;

  Matrix<uint8_t, 2, Dynamic> computed_quantized_vectors;
  computed_quantized_vectors.resize(2, 4);
  product_quantizer.Quantize(vectors, &computed_quantized_vectors);

  EXPECT_NEAR_EIGEN(quantized_vectors_, computed_quantized_vectors, 0.0);
}

TEST_F(ProductQuantizationTest, FillLUTWorks) {
  ProductQuantization<2, 2, 5, uint8_t> product_quantizer(centers_);

  Matrix<float, 4, 1> query_vector;
  query_vector << 0.0, 0.0, 2.0, 4.0;
  Matrix<float, 2, 5> lut;
  product_quantizer.FillLUT(query_vector, &lut);

  Matrix<float, 2, 5> expected_lut;
  expected_lut << 25.0, 2.0, 4.0, 13.0, 17.0, 8.0, 5.0, 4.0, 34.0, 17.0;

  EXPECT_NEAR_EIGEN(expected_lut, lut, 0.0);
}

TEST_F(ProductQuantizationTest, ComputeDistancesWorks) {
  ProductQuantization<2, 2, 5, uint8_t> product_quantizer(centers_);

  Matrix<float, 2, 5> lut;
  lut << 25.0, 2.0, 4.0, 13.0, 17.0, 8.0, 5.0, 4.0, 34.0, 17.0;

  Matrix<float, 1, Dynamic> distances;
  product_quantizer.ComputeDistances(lut, quantized_vectors_, &distances);

  ASSERT_EQ(distances.cols(), 4);
  Matrix<float, 1, Dynamic> expected_distances;
  expected_distances.resize(1, 4);
  expected_distances << 6.0, 36.0, 30.0, 29.0;

  EXPECT_NEAR_EIGEN(expected_distances, distances, 0.0);
}

TEST_F(ProductQuantizationTest, ComputeAndAddDistancesWorks) {
  ProductQuantization<2, 2, 5, uint8_t> product_quantizer(centers_);

  Matrix<float, 2, 5> lut;
  lut << 25.0, 2.0, 4.0, 13.0, 17.0, 8.0, 5.0, 4.0, 34.0, 17.0;

  Matrix<float, 1, Dynamic> distances;
  distances.resize(Eigen::NoChange, 4);
  distances << 1.0, 2.0, 3.0, 5.0;
  product_quantizer.ComputeAndAddDistances(lut, quantized_vectors_, &distances);

  ASSERT_EQ(distances.cols(), 4);
  Matrix<float, 1, Dynamic> expected_distances;
  expected_distances.resize(1, 4);
  expected_distances << 7.0, 38.0, 33.0, 34.0;

  EXPECT_NEAR_EIGEN(expected_distances, distances, 0.0);
}

}  // namespace product_quantization

MAPLAB_UNITTEST_ENTRYPOINT
