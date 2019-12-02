#include <vector>

#include <maplab-common/signals.h>
#include <maplab-common/test/testing-entrypoint.h>
#include <maplab-common/test/testing-predicates.h>

TEST(TestSignalProcessing, FFT) {
  std::vector<double> impulse = {1, 0, 0, 0};
  std::vector<double> impulse_fft = {1, 1, 1, 1};

  std::vector<double> signal_A = {1, 0, 1, 0, 0, 0, 0, 0};
  std::vector<double> signal_B = {1, 1, 0, 1, 0, 1, 0, 0};
  CHECK_EQ(signal_A.size(), signal_B.size());

  std::vector<double> computed_fft, computed_signal;

  // FFT and IFFT of unit impulse
  common::signals::toComplexSignal(impulse, &impulse);
  common::signals::toComplexSignal(impulse_fft, &impulse_fft);
  common::signals::computeFFT(impulse, &computed_fft);
  EXPECT_TRUE(::common::VectorsEqual(impulse_fft, computed_fft, 1e-9));

  common::signals::computeIFFT(computed_fft, &computed_signal);
  common::signals::toRealSignal(computed_signal, &computed_signal);
  common::signals::toRealSignal(impulse, &impulse);
  EXPECT_TRUE(::common::VectorsEqual(impulse, computed_signal, 1e-9));

  // Linearity
  double a = 2.3;
  double b = -1.5;

  std::vector<double> signal_A_B, signal_A_B_fft1;

  signal_A_B.resize(signal_A.size());
  for (size_t i = 0; i < signal_A_B.size(); ++i) {
    signal_A_B[i] = a * signal_A[i] + b * signal_B[i];
  }

  common::signals::toComplexSignal(signal_A_B, &signal_A_B);
  common::signals::computeFFT(signal_A_B, &signal_A_B_fft1);

  std::vector<double> signal_A_fft, signal_B_fft, signal_A_B_fft2;
  common::signals::toComplexSignal(signal_A, &signal_A);
  common::signals::toComplexSignal(signal_B, &signal_B);
  common::signals::computeFFT(signal_A, &signal_A_fft);
  common::signals::computeFFT(signal_B, &signal_B_fft);

  signal_A_B_fft2.resize(signal_A_fft.size());
  for (size_t i = 0; i < signal_A_B_fft2.size(); i++) {
    signal_A_B_fft2[i] = a * signal_A_fft[i] + b * signal_B_fft[i];
  }

  EXPECT_TRUE(::common::VectorsEqual(signal_A_B_fft1, signal_A_B_fft2, 1e-9));
}

TEST(TestSignalProcessing, Convolution) {
  std::vector<double> A = {
      0.0784691584255, 1.39306642855, 1.0652345983200, 1.020180943590,
      1.8095595183100, 2.02758129934, 1.5953446641800, 0.518026840993,
      1.4201616681100, 1.59454143396, 1.2309986095000, 1.727122460820,
      1.9463538053500, 1.33889044368, 0.0397695217043, 0.387115325420,
      0.0561554893253};

  std::vector<double> B = {
      1.589497740750, 0.823594509183, 1.351135450360, 0.776620078155,
      0.840155743559, 0.723713719248, 1.516012663720, 1.486284703900,
      1.076355628190, 0.496874831752, 0.495853178658, 0.417350074242,
      1.197391464370};

  // Values taken from a trusted implementation
  std::vector<double> convolution_result_full = {
      0.1247265500360, 2.27890270892, 2.946532310580, 4.442058831420,
      6.3035920911100, 8.14604636106, 9.465059073410, 10.13876645490,
      12.442668856400, 13.3244189691, 14.53604684620, 16.72049536280,
      18.076144650700, 18.2446941899, 15.57132606810, 15.17115079330,
      14.212975417800, 13.3540410228, 11.84451598350, 9.794535292410,
      8.3071020876800, 6.37428437543, 4.528470768320, 4.064142958550,
      3.1618444421800, 1.83962841575, 0.237027173516, 0.486965084024,
      0.0672401035957};

  std::vector<double> convolution_result_same = {
      9.46505907341, 10.1387664549, 12.4426688564, 13.3244189691,
      14.5360468462, 16.7204953628, 18.0761446507, 18.2446941899,
      15.5713260681, 15.1711507933, 14.2129754178, 13.3540410228,
      11.8445159835, 9.79453529241, 8.30710208768, 6.37428437543,
      4.52847076832};

  std::vector<double> convolution_result_valid = {
      18.0761446507, 18.2446941899, 15.5713260681, 15.1711507933,
      14.2129754178};

  std::vector<double> correlation_result = {
      0.0939583005151, 1.70079495995, 1.895808374590, 2.395877459230,
      3.8973640432500, 5.83423896667, 7.496666180660, 8.040620288760,
      9.8682244846600, 11.9723315314, 13.39627954650, 15.02150119600,
      15.662459159100, 17.2293288404, 16.54585790040, 16.77051705960,
      16.893138432400, 15.7576042340, 13.60746553340, 10.49661405530,
      9.8754748473300, 9.21751055595, 7.447420607420, 6.554049370250,
      4.5979826959100, 2.72757401511, 0.457913293696, 0.661568287833,
      0.0892590234131};

  // Test the convolution operation
  std::vector<double> result;
  common::signals::convolve(A, B, &result);
  EXPECT_TRUE(::common::VectorsEqual(result, convolution_result_full, 1e-9));

  // Test the filter operation which just trims the result from the convolution
  common::signals::filter(A, B, &result, common::signals::FULL);
  EXPECT_TRUE(::common::VectorsEqual(result, convolution_result_full, 1e-9));
  common::signals::filter(A, B, &result, common::signals::SAME);
  EXPECT_TRUE(::common::VectorsEqual(result, convolution_result_same, 1e-9));
  common::signals::filter(A, B, &result, common::signals::VALID);
  EXPECT_TRUE(::common::VectorsEqual(result, convolution_result_valid, 1e-9));

  // Test correlation, which also is based on the convolution operation
  common::signals::correlate(A, B, &result);
  EXPECT_TRUE(::common::VectorsEqual(result, correlation_result, 1e-9));
}

MAPLAB_UNITTEST_ENTRYPOINT
