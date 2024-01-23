// TODO(tcies) reenable once sm timing is fixed in tango (linker errors).
//#include <aslam/common/entrypoint.h>
//#include <aslam/test/triangulation-fixture.h>
//#include <sm/timing/Timer.hpp>
//
//void sampleXYPlaneSine(const double x_min, const double x_max,
//                       const size_t num_samples, Eigen::Matrix3Xd* result) {
//  CHECK_NOTNULL(result)->resize(3, num_samples);
//  CHECK_GT(num_samples, 1);
//  for (size_t i = 0; i < num_samples; ++i) {
//    double x = x_min + i * (x_max - x_min) / (num_samples - 1);
//    result->block<3, 1>(0, i) << x, sin(x), 0;
//  }
//}
//
//template <typename MeasurementType>
//size_t handleOffset() {
//  CHECK(false);
//  return 0u;
//}
//
//template <>
//size_t handleOffset<Vector2dList>() { return 1000u; }
//
//template <>
//size_t handleOffset<Eigen::Matrix3Xd>() { return 2000u; }
//
//template <typename MeasurementType>
//void plotIfDone() {}
//
//constexpr size_t kMaxNumMeasurements = 50;
//
//template <>
//void plotIfDone<Eigen::Matrix3Xd>() {
//  FILE* plot = popen("gnuplot --persist", "w");
//  fprintf(plot, "set logscale y\n");
//  fprintf(plot, "plot '-' w l, '-' w l\n");
//  for (size_t i = 2; i <= kMaxNumMeasurements; ++i) {
//    fprintf(plot, "%lu %lf\n", i, sm::timing::Timing::getMeanSeconds(
//        std::to_string(handleOffset<Vector2dList>() + i)));
//  }
//  fprintf(plot, "e\n");
//  for (size_t i = 2; i <= kMaxNumMeasurements; ++i) {
//    fprintf(plot, "%lu %lf\n", i, sm::timing::Timing::getMeanSeconds(
//        std::to_string(handleOffset<Eigen::Matrix3Xd>() + i)));
//  }
//  fprintf(plot, "e\n");
//  fflush(plot);
//}
//
//TYPED_TEST(TriangulationFixture, Performance) {
//  constexpr double kDepth = 5;
//  constexpr size_t kNumSamples = 10;
//  this->p_W_L_ << 1, 1, kDepth;
//  CHECK_GT(kMaxNumMeasurements, 1);
//  for (size_t num_measurements = 2; num_measurements <= kMaxNumMeasurements;
//      ++num_measurements) {
//    this->setNMeasurements(num_measurements);
//    Eigen::Matrix3Xd p_W_B;
//    sampleXYPlaneSine(-100., 100., num_measurements, &p_W_B);
//    for (size_t i = 0; i < num_measurements; ++i) {
//      this->T_W_B_[i].getPosition() = p_W_B.block<3, 1>(0, i);
//    }
//    this->inferMeasurements();
//    for (size_t i = 0; i < kNumSamples; ++i) {
//      sm::timing::Timer timer(std::to_string(
//          handleOffset<TypeParam>() + num_measurements));
//      this->expectSuccess();
//      timer.stop();
//    }
//  }
//  plotIfDone<TypeParam>();
//}
//
//constexpr double kMaxAngleDisparity = 20. / 180. * M_PI;
//static_assert(kMaxAngleDisparity > 0. && kMaxAngleDisparity < M_PI / 2,
//              "Maximum angle disparity must be between 0 and Pi/2.");
//constexpr size_t kNumAngleDisparitySteps = 50u;
//static_assert(kNumAngleDisparitySteps > 0u, "Need at least one disparity step.");
//constexpr double kDAngleDisparity = kMaxAngleDisparity / kNumAngleDisparitySteps;
//constexpr size_t kMinNumNoisyMeasurements = 2u;
//constexpr size_t kMaxNumNoisyMeasurements = 18u;
//constexpr size_t kNumNoisyMeasurementsMultiplier = 3u;
//static_assert((kMaxNumNoisyMeasurements / kMinNumNoisyMeasurements) %
//              kNumNoisyMeasurementsMultiplier == 0u, "Inconsistent series.");
//constexpr size_t kNumNoisyMeasurementsSteps = kMaxNumNoisyMeasurements /
//    (kMinNumNoisyMeasurements * kNumNoisyMeasurementsMultiplier) + 1;
//
//double stats[2][kNumAngleDisparitySteps][kNumNoisyMeasurementsSteps];
//
//template <typename MeasurementType>
//void plotAngleNoiseIfDone() {}
//
//template <>
//void plotAngleNoiseIfDone<Eigen::Matrix3Xd>() {
//  FILE* plot = popen("gnuplot --persist", "w");
//
//  for (size_t measurement_i = 0u; measurement_i < 2 * kNumNoisyMeasurementsSteps;
//      ++measurement_i) {
//    if (measurement_i == 0) {
//      fprintf(plot, "plot ");
//    } else {
//      fprintf(plot, ", ");
//    }
//    fprintf(plot, "'-' w l");
//  }
//  fprintf(plot, "\n");
//  for (size_t measurement_i = 0u; measurement_i < kNumNoisyMeasurementsSteps;
//      ++measurement_i) {
//    for (size_t i = 0u; i < kNumAngleDisparitySteps; ++i) {
//      fprintf(plot, "%lf %lf\n", kDAngleDisparity * (1 + i) * 180 / M_PI,
//              stats[0][i][measurement_i]);
//    }
//    fprintf(plot, "e\n");
//  }
//  for (size_t measurement_i = 0u; measurement_i < kNumNoisyMeasurementsSteps;
//      ++measurement_i) {
//    for (size_t i = 0u; i < kNumAngleDisparitySteps; ++i) {
//      fprintf(plot, "%lf %lf\n", kDAngleDisparity * (1 + i) * 180 / M_PI,
//              stats[1][i][measurement_i]);
//    }
//    fprintf(plot, "e\n");
//  }
//
//  fflush(plot);
//}
//
//
//TYPED_TEST(TriangulationFixture, AngleNoise) {
//  constexpr double kDepth = 1;
//  constexpr size_t kNumSamples = 50;
//  this->p_W_L_ << 0, 0, kDepth;
//  CHECK_GT(kMinNumNoisyMeasurements, 1);
//  CHECK_GE(kMaxNumNoisyMeasurements, kMinNumNoisyMeasurements);
//  CHECK_GE(kNumNoisyMeasurementsMultiplier, 1);
//
//  for (size_t disparity_i = 0u; disparity_i < kNumAngleDisparitySteps; ++disparity_i) {
//    double disparity = kDAngleDisparity * (1 + disparity_i);
//    for (size_t measurement_i = 0; measurement_i < kNumNoisyMeasurementsSteps;
//        ++measurement_i) {
//      size_t num_measurements = kMinNumNoisyMeasurements *
//          pow(kNumNoisyMeasurementsMultiplier, measurement_i);
//      this->setNMeasurements(num_measurements);
//      Eigen::Matrix3Xd p_W_B;
//      sampleXYPlaneSine(-1., 1., num_measurements, &p_W_B);
//      for (size_t i = 0; i < num_measurements; ++i) {
//        this->T_W_B_[i].getPosition() = p_W_B.block<3, 1>(0, i);
//      }
//      Eigen::VectorXd samples(kNumSamples, 1);
//      for (size_t i = 0u; i < kNumSamples; ++i) {
//        this->inferMeasurements(disparity);
//        Eigen::Vector3d triangulation;
//        aslam::TriangulationResult result = this->triangulate(&triangulation);
//        EXPECT_TRUE(result.wasTriangulationSuccessful());
//        samples[i] = (this->p_W_L_ - triangulation).norm();
//      }
//      stats[handleOffset<TypeParam>() / 1000 - 1][disparity_i][measurement_i] =
//                  samples.mean();
//    }
//  }
//  plotAngleNoiseIfDone<TypeParam>();
//}
//
//ASLAM_UNITTEST_ENTRYPOINT
