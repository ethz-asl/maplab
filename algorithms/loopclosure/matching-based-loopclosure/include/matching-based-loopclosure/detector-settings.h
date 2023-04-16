#ifndef MATCHING_BASED_LOOPCLOSURE_DETECTOR_SETTINGS_H_
#define MATCHING_BASED_LOOPCLOSURE_DETECTOR_SETTINGS_H_

#include <glog/logging.h>
#include <string>

namespace matching_based_loopclosure {

static const std::string kAccumulationString = "accumulation";
static const std::string kProbabilisticString = "probabilistic";

static const std::string kMatchingInvertedMultiIndexString = "imi";
static const std::string kMatchingInvertedMultiIndexPQString = "imipq";
static const std::string kMatchingHNSWString = "hnsw";

struct MatchingBasedEngineSettings {
  MatchingBasedEngineSettings();

  enum class KeyframeScoringFunctionType {
    kAccumulation,
    kProbabilistic,
  };

  enum class DetectorEngineType {
    kMatchingInvertedMultiIndex,
    kMatchingInvertedMultiIndexPQ,
    kMatchingHNSW,
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

  size_t hnsw_m;
  size_t hnsw_ef_construction;
  size_t hnsw_ef_query;
};

}  // namespace matching_based_loopclosure

#endif  // MATCHING_BASED_LOOPCLOSURE_DETECTOR_SETTINGS_H_
