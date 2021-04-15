#include "feature-tracking-pipelines/pnp-ransac.h"

int kPictureStretchingFactor = 4;
namespace feature_tracking_pipelines {
RansacSettings InitRansacSettingsFromGFlags() {
  RansacSettings settings;
  settings.ransac_threshold = FLAGS_NEW_feature_tracker_ransac_threshold;
  settings.ransac_max_iterations = FLAGS_NEW_feature_tracker_ransac_max_iters;
  settings.fix_random_seed = FLAGS_NEW_eature_tracker_ransac_fix_seed;
  return settings;
}

bool PerformTemporalFrameToFrameRansac(
    const aslam::Camera& camera, const KeyframeFeatures& keyframe_features_kp1,
    const KeyframeFeatures& keyframe_features_k, const RansacSettings& settings,
    const aslam::Quaternion& q_Ckp1_Ck,
    aslam::FrameToFrameMatches* inlier_matches_kp1_k,
    aslam::FrameToFrameMatches* outlier_matches_kp1_k) {
  CHECK_GT(settings.ransac_threshold, 0.0);
  CHECK_GT(settings.ransac_max_iterations, 0u);

  inlier_matches_kp1_k->clear();
  outlier_matches_kp1_k->clear();

  // Extract correspondences and convert to bearing vectors.
  aslam::FrameToFrameMatches matches_kp1_k;
  aslam::extractMatchesFromTrackIdChannel(
      keyframe_features_kp1.keypoint_track_ids.transpose(),
      keyframe_features_k.keypoint_track_ids.transpose(), &matches_kp1_k);

  aslam::geometric_vision::BearingVectors feature_vector_kp1;
  aslam::geometric_vision::BearingVectors feature_vector_k;
  feature_vector_kp1.reserve(matches_kp1_k.size());
  feature_vector_k.reserve(matches_kp1_k.size());

  for (const aslam::FrameToFrameMatch& match : matches_kp1_k) {
    const int keypoint_idx_kp1 = match.getKeypointIndexAppleFrame();
    const int keypoint_idx_k = match.getKeypointIndexBananaFrame();
    CHECK_LT(
        keypoint_idx_kp1, keyframe_features_kp1.keypoint_measurements.cols());
    CHECK_LT(keypoint_idx_k, keyframe_features_k.keypoint_measurements.cols());

    bool projection_successul = true;
    Eigen::Vector3d bearing_vector_kp1, bearing_vector_k;
    projection_successul &= camera.backProject3(
        keyframe_features_kp1.keypoint_measurements.col(keypoint_idx_kp1),
        &bearing_vector_kp1);
    projection_successul &= camera.backProject3(
        keyframe_features_k.keypoint_measurements.col(keypoint_idx_k),
        &bearing_vector_k);

    if (projection_successul) {
      feature_vector_kp1.emplace_back(bearing_vector_kp1.normalized());
      feature_vector_k.emplace_back(bearing_vector_k.normalized());
    }
  }

  // Run 2-pt RANSAC and prepare outputs.
  std::unordered_set<int> inlier_indices;
  bool ransac_success = aslam::geometric_vision::
      rejectOutlierFeatureMatchesTranslationRotationSAC(
          feature_vector_kp1, feature_vector_k, q_Ckp1_Ck,
          settings.fix_random_seed, settings.ransac_threshold,
          settings.ransac_max_iterations, &inlier_indices);

  int match_index = 0;
  for (const aslam::FrameToFrameMatch& match : matches_kp1_k) {
    if (inlier_indices.count(match_index)) {
      inlier_matches_kp1_k->emplace_back(match);
    } else {
      outlier_matches_kp1_k->emplace_back(match);
    }
    ++match_index;
  }
  CHECK_EQ(
      inlier_matches_kp1_k->size() + outlier_matches_kp1_k->size(),
      matches_kp1_k.size());
  return ransac_success;
}

static std::vector<int> getPxOffset(int lidar_mode) {
  auto repeat = [](int n, const std::vector<int>& v) {
    std::vector<int> res{};
    for (int i = 0; i < n; i++)
      res.insert(res.end(), v.begin(), v.end());
    return res;
  };

  switch (lidar_mode) {
    case 512:
      return repeat(16, {0, 3, 6, 9});
    case 1024:
      return repeat(16, {0, 6, 12, 18});
    case 2048:
      return repeat(16, {0, 12, 24, 36});
    default:
      return std::vector<int>{64, 0};
  }
}

bool backProject3d(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud, const cv::Mat* image,
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector3d* out_point_3d) {
  CHECK_NOTNULL(out_point_3d);
  const auto W = 1024;
  const auto H = 64;
  CHECK_EQ(kPictureStretchingFactor, image->cols / W);

  const int v = keypoint[0];
  const int u = keypoint[1];
  const auto px_offset = getPxOffset(W);
  const std::size_t offset = px_offset[u / kPictureStretchingFactor];
  const std::size_t index = ((v / kPictureStretchingFactor + offset) % W) * H +
                            u / kPictureStretchingFactor;
  pcl::PointXYZI projected = cloud->points[index];
  (*out_point_3d)[0] = projected.x;
  (*out_point_3d)[1] = projected.y;
  (*out_point_3d)[2] = projected.z;
}

bool backProject3d(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const Eigen::Ref<const Eigen::Vector2d>& keypoint,
    Eigen::Vector3d* out_point_3d) {
  CHECK_NOTNULL(out_point_3d);
  const auto W = 1024;
  const auto H = 64;
  const int v = keypoint[0];
  const int u = keypoint[1];
  const auto px_offset = getPxOffset(W);
  const std::size_t offset = px_offset[u / kPictureStretchingFactor];
  const std::size_t index = ((v / kPictureStretchingFactor + offset) % W) * H +
                            u / kPictureStretchingFactor;
  pcl::PointXYZI projected = cloud->points[index];
  (*out_point_3d)[0] = projected.x;
  (*out_point_3d)[1] = projected.y;
  (*out_point_3d)[2] = projected.z;
}

bool PerformTemporalFrameToFrameRansac(
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
    const KeyframeFeatures& keyframe_features_kp1,
    const KeyframeFeatures& keyframe_features_k,
    Eigen::Matrix3d* best_rotation_matrix, Eigen::Vector3d* best_translation,
    std::vector<size_t>* best_inliers, std::vector<size_t>* best_outliers) {
  LOG_IF(
      WARNING, keyframe_features_k.keypoint_measurements.cols() < 6 ||
                   keyframe_features_kp1.keypoint_measurements.cols() < 6)
      << "Ransac unsuccessful. Not enough "
         "features.";
  if (keyframe_features_k.keypoint_measurements.cols() < 6 ||
                   keyframe_features_kp1.keypoint_measurements.cols() < 6) {
    return false;
  }
  
  best_inliers->clear();
  best_outliers->clear();

  // Extract 3D positions from features
  std::vector<Eigen::Vector3d> feature_vector_kp1;
  std::vector<Eigen::Vector3d> feature_vector_k;

  for (int i = 0; i < keyframe_features_kp1.keypoint_measurements.cols(); i++) {
    Eigen::Vector3d feature_position_kp1, feature_position_k;
    backProject3d(
        cloud, keyframe_features_kp1.keypoint_measurements.col(i),
        &feature_position_kp1);
    backProject3d(
        cloud, keyframe_features_k.keypoint_measurements.col(i),
        &feature_position_k);
    feature_vector_k.push_back(feature_position_k);
    feature_vector_kp1.push_back(feature_position_kp1);
  }
  RansacSettings settings = InitRansacSettingsFromGFlags();

  RansacTransformationFor3DPoints(
      feature_vector_k, feature_vector_kp1, best_rotation_matrix,
      best_translation, best_inliers, best_outliers, settings.ransac_threshold,
      settings.ransac_max_iterations);
}
void RansacTransformationFor3DPoints(
    std::vector<Eigen::Vector3d> point_set_1,
    std::vector<Eigen::Vector3d> point_set_2,
    Eigen::Matrix3d* best_rotation_matrix, Eigen::Vector3d* best_translation,
    std::vector<size_t>* best_inliers, std::vector<size_t>* best_outliers,
    double ransac_threshold, int ransac_max_iterations) {
  CHECK_GT(ransac_threshold, 0.0);
  CHECK_GT(ransac_max_iterations, 0u);
  CHECK_EQ(point_set_1.size(), point_set_2.size());
  CHECK_GT(point_set_2.size(), 5u);
  for (int j = 0; j < ransac_max_iterations; ++j) {
    // Generate 3 random indeces for Ransac
    std::vector<int> ransac_indeces(6);

    ransac_indeces[0] = rand() % point_set_2.size();
    while (ransac_indeces[0] == ransac_indeces[1]) {
      ransac_indeces[1] = rand() % point_set_2.size();
    }
    while (ransac_indeces[0] == ransac_indeces[2] ||
           ransac_indeces[1] == ransac_indeces[2]) {
      ransac_indeces[2] = rand() % point_set_2.size();
    }
    while (ransac_indeces[0] == ransac_indeces[3] ||
           ransac_indeces[1] == ransac_indeces[3] ||
           ransac_indeces[2] == ransac_indeces[3]) {
      ransac_indeces[3] = rand() % point_set_2.size();
    }
    while (ransac_indeces[0] == ransac_indeces[4] ||
           ransac_indeces[1] == ransac_indeces[4] ||
           ransac_indeces[2] == ransac_indeces[4] ||
           ransac_indeces[3] == ransac_indeces[4]) {
      ransac_indeces[4] = rand() % point_set_2.size();
    }
    while (ransac_indeces[0] == ransac_indeces[5] ||
           ransac_indeces[1] == ransac_indeces[5] ||
           ransac_indeces[2] == ransac_indeces[5] ||
           ransac_indeces[3] == ransac_indeces[5] ||
           ransac_indeces[4] == ransac_indeces[5]) {
      ransac_indeces[5] = rand() % point_set_2.size();
    }

    // Generate transformation matrix
    // https://igl.ethz.ch/projects/ARAP/svd_rot.pdf
    Eigen::Matrix3d X, Y;
    X << point_set_1[ransac_indeces[0]], point_set_1[ransac_indeces[1]],
        point_set_1[ransac_indeces[2]], point_set_1[ransac_indeces[3]],
        point_set_1[ransac_indeces[4]], point_set_1[ransac_indeces[5]];
    Y << point_set_2[ransac_indeces[0]], point_set_2[ransac_indeces[1]],
        point_set_2[ransac_indeces[2]], point_set_2[ransac_indeces[3]],
        point_set_2[ransac_indeces[4]], point_set_2[ransac_indeces[5]];

    Eigen::VectorXd X_mean = X.rowwise().mean();
    Eigen::VectorXd Y_mean = Y.rowwise().mean();

    X.colwise() -= X_mean;
    Y.colwise() -= Y_mean;

    Eigen::Matrix3d S = X * Y.transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        S, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d reflection_handler = Eigen::Matrix3d::Identity();
    reflection_handler(2, 2) =
        (svd.matrixV() * svd.matrixU().transpose()).determinant();
    Eigen::Matrix3d R =
        svd.matrixV() * reflection_handler * svd.matrixU().transpose();
    Eigen::Vector3d t = Y_mean - R * X_mean;

    // Calculate Outliers for Transformation
    std::vector<size_t> outliers;
    std::vector<size_t> inliers;

    for (int i = 0; i < point_set_1.size(); ++i) {
      Eigen::Vector3d transformation_error =
          R * point_set_1[i] + t - point_set_2[i];
      if (transformation_error.norm() / point_set_2[i].norm() >
          ransac_threshold) {
        outliers.push_back(i);
      } else {
        inliers.push_back(i);
      }
    }
    // Compare results
    if (inliers.size() > best_inliers->size()) {
      *best_outliers = outliers;
      *best_rotation_matrix = R;
      *best_translation = t;
      *best_inliers = inliers;
    }
  }
}

}  // namespace feature_tracking_pipelines
