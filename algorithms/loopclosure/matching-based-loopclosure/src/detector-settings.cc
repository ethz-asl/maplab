#include <descriptor-projection/flags.h>
#include <glog/logging.h>
#include <loopclosure-common/flags.h>
#include <loopclosure-common/types.h>

#include "matching-based-loopclosure/detector-settings.h"

DEFINE_string(
    lc_detector_engine,
    matching_based_loopclosure::kMatchingInvertedMultiIndexString,
    "Which loop-closure engine to use. Options are:"
    " - imi:   inverted multi index (binary)"
    " - imipq: inverted multi index with product quantization (binary)"
    " - hsnw:  hierarchical navigable small world graphs (float)"
    "The default options for binary descriptors is \"imi\".");
DEFINE_string(
    lc_scoring_function, matching_based_loopclosure::kAccumulationString,
    "Type of scoring function to be used for scoring keyframes.");
DEFINE_double(
    lc_min_image_time_seconds, 10.0,
    "Minimum time between matching images to allow a loop closure.");
DEFINE_uint64(
    lc_min_verify_matches_num, 10u,
    "The minimum number of matches needed to verify geometry.");
DEFINE_double(
    lc_fraction_best_scores, 0.25,
    "Fraction of best scoring "
    "keyframes/vertices that are considered for covisibility filtering.");
DEFINE_int32(
    lc_num_neighbors, -1,
    "Number of neighbors to retrieve for loop-closure. -1 auto.");
DEFINE_int32(
    lc_num_words_for_nn_search, 10,
    "Number of nearest words to retrieve in the inverted index.");

DEFINE_int32(
    lc_hnsw_m, 12,
    "Number of bidirectional links in the HNSW search index. Ideal is "
    "somewhere between 8-64. See the readme for more details.");
DEFINE_int32(
    lc_hnsw_ef_construction, 50,
    "Parameter in HNSW that controls the time used during construction versus "
    "accuracy. After a point increasing this will have no effect.");
DEFINE_int32(
    lc_hnsw_ef_query, 50,
    "Parameter in HNSW that trades off between time and accuracy when "
    "querying. Must be bigger than the number of neighbours queried.");

namespace matching_based_loopclosure {
MatchingBasedEngineSettings::MatchingBasedEngineSettings()
    : projection_matrix_filename(FLAGS_lc_projection_matrix_filename),
      projected_quantizer_filename(FLAGS_lc_projected_quantizer_filename),
      num_closest_words_for_nn_search(FLAGS_lc_num_words_for_nn_search),
      min_image_time_seconds(FLAGS_lc_min_image_time_seconds),
      min_verify_matches_num(FLAGS_lc_min_verify_matches_num),
      fraction_best_scores(FLAGS_lc_fraction_best_scores),
      num_nearest_neighbors(FLAGS_lc_num_neighbors),
      hnsw_m(FLAGS_lc_hnsw_m),
      hnsw_ef_construction(FLAGS_lc_hnsw_ef_construction),
      hnsw_ef_query(FLAGS_lc_hnsw_ef_query) {
  CHECK_GT(num_closest_words_for_nn_search, 0);
  CHECK_GE(min_image_time_seconds, 0.0);
  CHECK_GE(min_verify_matches_num, 0u);
  CHECK_GT(fraction_best_scores, 0.f);
  CHECK_LT(fraction_best_scores, 1.f);
  CHECK_GE(num_nearest_neighbors, -1);

  // HNSW index specific parameters
  CHECK_GT(hnsw_m, 0u);
  CHECK_GT(hnsw_ef_construction, 0u);
  CHECK_GT(hnsw_ef_query, 0u);

  setKeyframeScoringFunctionType(FLAGS_lc_scoring_function);
  setDetectorEngineType(FLAGS_lc_detector_engine);

  const std::string loop_closure_files_path = getLoopClosureFilePath();
  const bool use_default_projection_matrix_filepath =
      projection_matrix_filename.empty();
  const bool use_default_projected_quantizer_filepath =
      projected_quantizer_filename.empty();
  if (FLAGS_feature_descriptor_type == loop_closure::kFeatureDescriptorFREAK) {
    if (use_default_projection_matrix_filepath) {
      projection_matrix_filename =
          std::string(loop_closure_files_path) + "/projection_matrix_freak.dat";
    }
    if (use_default_projected_quantizer_filepath) {
      projected_quantizer_filename =
          std::string(loop_closure_files_path) +
          "/inverted_multi_index_quantizer_freak.dat";
    }
  } else {
    CHECK_EQ(
        FLAGS_feature_descriptor_type, loop_closure::kFeatureDescriptorBRISK);
    if (use_default_projection_matrix_filepath) {
      projection_matrix_filename =
          std::string(loop_closure_files_path) + "/projection_matrix_brisk.dat";
    }
    if (use_default_projected_quantizer_filepath) {
      projected_quantizer_filename =
          std::string(loop_closure_files_path) +
          "/inverted_multi_index_quantizer_brisk.dat";
    }
  }
}

void MatchingBasedEngineSettings::setKeyframeScoringFunctionType(
    const std::string& scoring_function_string) {
  scoring_function_type_string = scoring_function_string;
  if (scoring_function_string == kAccumulationString) {
    keyframe_scoring_function_type = KeyframeScoringFunctionType::kAccumulation;
  } else if (scoring_function_string == kProbabilisticString) {
    keyframe_scoring_function_type =
        KeyframeScoringFunctionType::kProbabilistic;
  } else {
    LOG(FATAL) << "Unknown scoring function type: " << scoring_function_string;
  }
}

void MatchingBasedEngineSettings::setDetectorEngineType(
    const std::string& detector_engine_string) {
  detector_engine_type_string = detector_engine_string;
  if (detector_engine_string == kMatchingInvertedMultiIndexString) {
    detector_engine_type = DetectorEngineType::kMatchingInvertedMultiIndex;
  } else if (detector_engine_string == kMatchingInvertedMultiIndexPQString) {
    detector_engine_type = DetectorEngineType::kMatchingInvertedMultiIndexPQ;
  } else if (detector_engine_string == kMatchingHNSWString) {
    detector_engine_type = DetectorEngineType::kMatchingHNSW;
  } else {
    LOG(FATAL) << "Unknown loop detector engine type: "
               << detector_engine_string;
  }
}

std::string MatchingBasedEngineSettings::getLoopClosureFilePath() {
  const char* loop_closure_files_path = getenv("MAPLAB_LOOPCLOSURE_DIR");
  CHECK_NE(loop_closure_files_path, static_cast<char*>(NULL))
      << "MAPLAB_LOOPCLOSURE_DIR environment variable is not set.\n"
      << "Source the Maplab environment from your workspace:\n"
      << "source devel/setup.bash";
  return loop_closure_files_path;
}

}  // namespace matching_based_loopclosure
