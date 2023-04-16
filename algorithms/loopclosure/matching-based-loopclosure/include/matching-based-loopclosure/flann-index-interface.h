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
    index_.reset(new cv::FlannBasedMatcher(
        cv::makePtr<cv::flann::KDTreeIndexParams>(5),
        cv::makePtr<cv::flann::SearchParams>(128)));
    initialized_ = false;
    num_descriptors = 0;
  }

  virtual void Initialize() {}

  virtual int GetNumDescriptorsInIndex() const {
    return num_descriptors;
  }

  virtual void Clear() {
    std::lock_guard<std::mutex> lock(index_mutex_);
    index_->clear();
    index_images_.clear();
    initialized_ = false;
    num_descriptors = 0;
  }

  virtual void AddDescriptors(const Eigen::MatrixXf& descriptors) {
    // Skip empty insertions, since it messes with the later concatenation and
    // doesn't affect the global indices of the descriptors
    if (descriptors.cols() == 0) {
      return;
    }

    // First flip since the ordering is reversed in opencv
    Eigen::MatrixXf transpose = descriptors.transpose();
    cv::Mat cv_descriptors;
    cv::eigen2cv(transpose, cv_descriptors);

    std::lock_guard<std::mutex> lock(index_mutex_);
    index_images_.emplace_back(cv_descriptors);
    initialized_ = false;
    num_descriptors += descriptors.cols();
  }

  virtual void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixXf& query_features, int num_neighbors,
      Eigen::MatrixXi* indices, Eigen::MatrixXf* distances) {
    CHECK_NOTNULL(indices);
    CHECK_NOTNULL(distances);

    Eigen::MatrixXf transpose = query_features.transpose();
    cv::Mat cv_query;
    cv::eigen2cv(transpose, cv_query);

    std::lock_guard<std::mutex> lock(index_mutex_);

    if (!initialized_) {
      const int kDescriptorBatchSize = 250000;
      std::vector<cv::Mat> batched_index_images;
      cumulative_count.clear();
      cumulative_count.emplace_back(0);

      int batch_start, batch_end, batch_size;
      batch_start = batch_size = 0;
      for (batch_end = 0; batch_end < index_images_.size(); batch_end++) {
        CHECK(index_images_[batch_end].rows <= kDescriptorBatchSize);
        if (batch_size + index_images_[batch_end].rows > kDescriptorBatchSize) {
          batched_index_images.emplace_back(cv::Mat());
          cv::vconcat(
              &index_images_[batch_start], batch_end - batch_start,
              batched_index_images.back());
          cumulative_count.emplace_back(cumulative_count.back() + batch_size);

          batch_start = batch_end;
          batch_size = index_images_[batch_end].rows;
        } else {
          batch_size += index_images_[batch_end].rows;
        }
      }

      // Insert final batch that never get done in the previous loop
      batched_index_images.emplace_back(cv::Mat());
      cv::vconcat(
          &index_images_[batch_start], batch_end - batch_start,
          batched_index_images.back());

      index_->add(batched_index_images);
      initialized_ = true;
    }

    std::vector<std::vector<cv::DMatch>> knn_matches;
    index_->knnMatch(cv_query, knn_matches, num_neighbors);

    // TODO(smauq): Check that the eigen matrices are correctly preallocated

    const std::vector<cv::Mat>& train = index_->getTrainDescriptors();
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
  bool initialized_;
  std::shared_ptr<cv::FlannBasedMatcher> index_;
  std::vector<cv::Mat> index_images_;
  std::vector<int> cumulative_count;
  size_t num_descriptors;
  mutable std::mutex index_mutex_;
};
}  // namespace loop_closure
#endif  // MATCHING_BASED_LOOPCLOSURE_FLANN_INDEX_INTERFACE_H_
