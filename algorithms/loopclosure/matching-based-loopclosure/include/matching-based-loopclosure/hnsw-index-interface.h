#ifndef MATCHING_BASED_LOOPCLOSURE_HSNW_INDEX_INTERFACE_H_
#define MATCHING_BASED_LOOPCLOSURE_HSNW_INDEX_INTERFACE_H_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <maplab-common/multi-threaded-progress-bar.h>
#include <maplab-common/parallel-process.h>
#include <memory>
#include <string>
#include <vector>

#include "matching-based-loopclosure/helpers.h"
#include "matching-based-loopclosure/hnswlib/hnswlib.h"
#include "matching-based-loopclosure/index-interface.h"

namespace loop_closure {
class HSNWIndexInterface : public IndexInterface {
 public:
  HSNWIndexInterface(size_t M, size_t ef_construction, size_t ef_query)
      : M_(M), ef_construction_(ef_construction), ef_query_(ef_query) {
    initialized_ = false;
    descriptor_size_ = 0;
    num_descriptors_ = 0;
  }

  virtual void Initialize() {
    if (!initialized_) {
      const size_t maxSize = new_descriptors_.size();
      CHECK_GT(maxSize, 0u) << "No descriptors added to database";

      space_.reset(new hnswlib::L2Space(descriptor_size_));
      index_.reset(new hnswlib::HierarchicalNSW<float>(
          space_.get(), maxSize, M_, ef_construction_));
      index_->setEf(ef_query_);

      common::MultiThreadedProgressBar progress_bar(1);
      std::function<void(const std::vector<size_t>&)> inserter =
          [&progress_bar, this](const std::vector<size_t>& batch) {
            progress_bar.setNumElements(batch.size());
            size_t num_processed = 0u;
            for (size_t i : batch) {
              index_->addPoint(new_descriptors_[i], i);
              delete new_descriptors_[i];
              progress_bar.update(++num_processed);
            }
          };

      VLOG(1) << "Building HNSW index for fast approximate nearest neighbour lookup.";
      static constexpr bool kAlwaysParallelize = true;
      const size_t num_threads = common::getNumHardwareThreads();
      common::ParallelProcess(
          maxSize, inserter, kAlwaysParallelize, num_threads);

      new_descriptors_.clear();
      initialized_ = true;
    }
  }

  virtual int GetNumDescriptorsInIndex() const {
    return num_descriptors_;
  }

  virtual void Clear() {
    space_.reset();
    index_.reset();
    initialized_ = false;
    descriptor_size_ = 0;
    num_descriptors_ = 0;

    for (float* descriptor : new_descriptors_) {
      delete descriptor;
    }
    new_descriptors_.clear();
  }

  virtual void AddDescriptors(const Eigen::MatrixXf& descriptors) {
    // Skip empty insertions, since it messes with the later concatenation and
    // doesn't affect the global indices of the descriptors
    if (descriptors.cols() == 0) {
      return;
    }

    // On the very first call determine the descriptor size
    if (!initialized_ && new_descriptors_.empty()) {
      descriptor_size_ = descriptors.rows();
      CHECK_GT(descriptor_size_, 0u);
    } else {
      CHECK_EQ(descriptor_size_, descriptors.rows());
    }

    // Brutally copy over the data from Eigen into temporary containers.
    // This lets us figure the max size before initializing the index.
    for (int i = 0; i < descriptors.cols(); i++) {
      float* descriptor = new float[descriptor_size_];
      for (size_t j = 0; j < descriptor_size_; j++) {
        descriptor[j] = descriptors.coeff(j, i);
      }
      new_descriptors_.emplace_back(descriptor);
    }

    num_descriptors_ += descriptors.cols();
  }

  virtual void GetNNearestNeighborsForFeatures(
      const Eigen::MatrixXf& query_features, int num_neighbors,
      Eigen::MatrixXi* indices, Eigen::MatrixXf* distances) {
    CHECK(initialized_);
    CHECK_NOTNULL(indices);
    CHECK_NOTNULL(distances);
    CHECK_EQ(descriptor_size_, query_features.rows());
    
    CHECK_EQ(indices->rows(), num_neighbors)
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(distances->rows(), num_neighbors)
        << "The distances parameter must be pre-allocated to hold all results.";
    CHECK_EQ(indices->cols(), query_features.cols())
        << "The indices parameter must be pre-allocated to hold all results.";
    CHECK_EQ(distances->cols(), query_features.cols())
        << "The distances parameter must be pre-allocated to hold all results.";

    float* query_descriptor = new float[descriptor_size_];
    for (int i = 0; i < query_features.cols(); i++) {
      for (int j = 0; j < query_features.rows(); j++) {
        query_descriptor[j] = query_features.coeff(j, i);
      }

      std::priority_queue<std::pair<float, size_t>> result =
          index_->searchKnn(query_descriptor, num_neighbors);
      CHECK_EQ(result.size(), num_neighbors);

      for (int j = 0; j < num_neighbors; j++) {
        indices->coeffRef(j, i) = result.top().second;
        distances->coeffRef(j, i) = result.top().first;
        result.pop();
      }
    }
    delete query_descriptor;
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
  int descriptor_size_;
  size_t num_descriptors_;
  std::vector<float*> new_descriptors_;

  size_t M_;
  size_t ef_construction_;
  size_t ef_query_;
  std::shared_ptr<hnswlib::L2Space> space_;
  std::shared_ptr<hnswlib::HierarchicalNSW<float>> index_;
};
}  // namespace loop_closure

#endif  // MATCHING_BASED_LOOPCLOSURE_HSNW_INDEX_INTERFACE_H_
