#ifndef MATCHING_BASED_LOOPCLOSURE_DETECTOR_SETTINGS_H_
#define MATCHING_BASED_LOOPCLOSURE_DETECTOR_SETTINGS_H_

#include <string>

#include <glog/logging.h>

namespace matching_based_loopclosure {

static const std::string kAccumulationString = "accumulation";
static const std::string kProbabilisticString = "probabilistic";

static const std::string kMatchingLDKdTreeString = "kd_tree";
static const std::string kMatchingLDInvertedIndexString = "inverted_index";
static const std::string kMatchingLDInvertedMultiIndexString =
    "inverted_multi_index";
static const std::string
    kMatchingLDInvertedMultiIndexProductQuantizationString =
        "inverted_multi_index_product_quantization";

struct MatchingBasedEngineSettings {
  MatchingBasedEngineSettings();

  enum class KeyframeScoringFunctionType {
    kAccumulation,
    kProbabilistic,
  };

  enum class DetectorEngineType {
    kMatchingLDKdTree,
    kMatchingLDInvertedIndex,
    kMatchingLDInvertedMultiIndex,
    kMatchingLDInvertedMultiIndexProductQuantization,
  };

  void setKeyframeScoringFunctionType(
      const std::string& scoring_function_string);
  void setDetectorEngineType(const std::string& detector_engine_string);
  static std::string getLoopClosureFilePath();

  KeyframeScoringFunctionType keyframe_scoring_function_type;
  std::string scoring_function_type_string;
  DetectorEngineType detector_engine_type;
  std::string detector_engine_type_string;

  std::string projection_matrix_filename;
  std::string projected_quantizer_filename;
  int num_closest_words_for_nn_search;
  double min_image_time_seconds;
  size_t min_verify_matches_num;
  float fraction_best_scores;
  int num_nearest_neighbors;
};

}  // namespace matching_based_loopclosure

#endif  // MATCHING_BASED_LOOPCLOSURE_DETECTOR_SETTINGS_H_
