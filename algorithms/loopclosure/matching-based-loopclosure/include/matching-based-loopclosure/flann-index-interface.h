#ifndef MATCHING_BASED_LOOPCLOSURE_FLANN_INDEX_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_FLANN_INDEX_INTERFACE_H_
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <aslam/common/timer.h>
#include <maplab-common/binary-serialization.h>
#include <matching-based-loopclosure/helpers.h>
#include <matching-based-loopclosure/index-interface.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/features2d.hpp>

namespace loop_closure {
class FLANNIndexInterface : public IndexInterface {
 public:
  FLANNIndexInterface() {
    index_.reset(new cv::FlannBasedMatcher());
    cumulative_count.emplace_back(0);
  }

  virtual int GetNumDescriptorsInIndex() const {
    const size_t num_images = cumulative_count.size();
    return cumulative_count[num_images - 1];
  }

  virtual void Clear() {
    std::lock_guard<std::mutex> lock(index_mutex_);
    index_->clear();
    cumulative_count.clear();
    cumulative_count.emplace_back(0);
  }

  virtual void AddDescriptors(const Eigen::MatrixXf& descriptors) {
    // First flip since the ordering is reversed in opencv
    Eigen::MatrixXf transpose = descriptors.transpose();
    cv::Mat cv_descriptors;
    cv::eigen2cv(transpose, cv_descriptors);

    std::lock_guard<std::mutex> lock(index_mutex_);
    index_->add(cv_descriptors);
    cumulative_count.emplace_back(
        GetNumDescriptorsInIndex() + cv_descriptors.rows);
  }

  virtual void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixXf& query_features, int num_neighbors,
      Eigen::MatrixXi* indices, Eigen::MatrixXf* distances) const {
    CHECK_NOTNULL(indices);
    CHECK_NOTNULL(distances);

    Eigen::MatrixXf transpose = query_features.transpose();
    cv::Mat cv_query;
    cv::eigen2cv(transpose, cv_query);

    std::lock_guard<std::mutex> lock(index_mutex_);
    std::vector<std::vector<cv::DMatch>> knn_matches;
    index_->knnMatch(cv_query, knn_matches, num_neighbors);

    // TODO(smauq): Check that the eigen matrices are correctly preallocated

    for (size_t i = 0; i < knn_matches.size(); i++) {
      for (size_t j = 0; j < knn_matches[i].size(); j++) {
        const cv::DMatch& match = knn_matches[i][j];
        (*indices)(j, i) = cumulative_count[match.imgIdx] + match.trainIdx;
        (*distances)(j, i) = match.distance;
      }
    }
  }

  virtual void ProjectDescriptors(
      const DescriptorContainer& descriptors,
      Eigen::MatrixXf* projected_descriptors) const {
    CHECK_NOTNULL(projected_descriptors);

    *projected_descriptors = Eigen::Map<const Eigen::MatrixXf>(
        reinterpret_cast<const float*>(descriptors.data()),
        descriptors.rows() / 4, descriptors.cols());
  }

  virtual void ProjectDescriptors(
      const std::vector<aslam::common::FeatureDescriptorConstRef>& descriptors,
      Eigen::MatrixXf* projected_descriptors) const {
    LOG(FATAL) << "Not implemented";
  }

 private:
  std::shared_ptr<cv::FlannBasedMatcher> index_;
  std::vector<int> cumulative_count;
  mutable std::mutex index_mutex_;
};
}  // namespace loop_closure
#endif  // MATCHING_BASED_LOOPCLOSURE_FLANN_INDEX_INTERFACE_H_
