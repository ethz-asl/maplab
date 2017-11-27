// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_VOCABULARY_TREE_INL_H_
#define VOCABULARY_TREE_VOCABULARY_TREE_INL_H_
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include <aslam/common/memory.h>
#include <loopclosure-common/flags.h>
#include <maplab-common/binary-serialization.h>

namespace loop_closure {
template <class Feature, class Distance, class FeatureAllocator>
VocabularyTree<Feature, Distance, FeatureAllocator>::VocabularyTree(Distance d)
    : distance_(d), num_splits_(0), levels_(0), num_words_(0), word_start_(0) {}

template <class Feature, class Distance, class FeatureAllocator>
VocabularyTree<Feature, Distance, FeatureAllocator>::VocabularyTree(
    const std::string& file, Distance d)
    : distance_(d), num_splits_(0), levels_(0), num_words_(0), word_start_(0) {
  Load(file);
}

template <class Feature, class Distance, class FeatureAllocator>
Word VocabularyTree<Feature, Distance, FeatureAllocator>::Quantize(
    const Feature& f) const {
  CHECK(Initialized());
  CHECK(!search_accelerators_.empty());
  CHECK_EQ(f.size(), centers_[0].size());

  int32_t index = -1;  // Virtual "root" index, which has no associated center.
  for (unsigned level = 0; level < levels_; ++level) {
    // Calculate the offset to the first child of the current index.
    int32_t node = (index + 1) * splits();
    int32_t accelerator = node / splits();
    CHECK_LT(accelerator, static_cast<int32_t>(search_accelerators_.size()));
    const SearchAccelerator& search_accelerator =
        search_accelerators_[accelerator];
    int32_t closest_child = search_accelerator.FindIndexOfClosestFeature(f);
    index = node + closest_child;
  }

  return index - word_start_;
}

template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::
    GetNearestNeighborTopLevel(
        const Feature& feature, unsigned int num_nearest_neighbors,
        std::vector<Word>* nearest_neighbors,
        std::vector<DistanceType>* distances) const {
  CHECK(!search_accelerators_.empty());
  const SearchAccelerator& search_accelerator = search_accelerators_[0];
  search_accelerator.GetNNearestNeighbors(
      feature, num_nearest_neighbors, nearest_neighbors, distances);
}
template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::
    GetNearestNeighborTopLevel(
        const Eigen::MatrixXf& features, unsigned int num_nearest_neighbors,
        std::vector<Word>* nearest_neighbors) const {
  CHECK(!search_accelerators_.empty());
  const SearchAccelerator& search_accelerator = search_accelerators_[0];
  search_accelerator.GetNNearestNeighbors(
      features, num_nearest_neighbors, nearest_neighbors);
}

template <class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::CountLeaves()
    const {
  CHECK(Initialized());

  uint32_t count = 0;
  uint32_t index_lowest_level = (levels() - 1) * splits();
  for (uint32_t nodeidx = index_lowest_level; nodeidx < levels() * splits();
       ++nodeidx) {
    if (valid_centers_[nodeidx]) {
      ++count;
    } else {
      break;  // No more valid nodes at the lowest level.
    }
  }
  return count;
}

template <class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::levels() const {
  return levels_;
}

template <class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::splits() const {
  return num_splits_;
}

template <class Feature, class Distance, class FeatureAllocator>
uint32_t VocabularyTree<Feature, Distance, FeatureAllocator>::words() const {
  return num_words_;
}

template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::Clear() {
  centers_.clear();
  valid_centers_.clear();
  num_splits_ = levels_ = num_words_ = word_start_ = 0;
}

template <class Feature, class Distance, class FeatureAllocator>
bool VocabularyTree<Feature, Distance, FeatureAllocator>::IsBinaryEqual(
    const VocabularyTree<Feature, Distance, FeatureAllocator>& other) const {
  bool is_same = true;
  is_same = is_same && num_splits_ == other.num_splits_;
  is_same = is_same && levels_ == other.levels_;
  is_same = is_same && centers_.size() == other.centers_.size();
  is_same = is_same && valid_centers_ == other.valid_centers_;
  for (size_t i = 0; i < centers_.size() && is_same; ++i) {
    if (valid_centers_.at(i))
      is_same = is_same && centers_.at(i) == other.centers_.at(i);
  }
  return is_same;
}

template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::Save(
    const std::string& file) const {
  std::ofstream out(file.c_str(), std::ios_base::binary);
  Save(&out);
}

template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::Save(
    std::ofstream* out) const {
  CHECK_NOTNULL(out);
  CHECK(out->is_open());

  CHECK(Initialized());

  common::Serialize(kSerializationVersion, out);
  std::string descriptor_tree = FLAGS_feature_descriptor_type;

  common::Serialize(descriptor_tree, out);
  common::Serialize(num_splits_, out);
  common::Serialize(levels_, out);
  common::Serialize(centers_, out);
  common::Serialize(valid_centers_, out);
}

template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance,
                    FeatureAllocator>::InitializeSearchAccelerators() {
  if (centers_.empty()) {
    return;
  }
  search_accelerators_.clear();

  int32_t index = -1;
  for (unsigned level = 0; level < levels_; ++level) {
    int32_t accelerators_on_this_level = pow(splits(), level);
    for (int32_t accelerator = 0; accelerator < accelerators_on_this_level;
         ++accelerator) {
      typedef std::vector<Feature, FeatureAllocator> FeatureVector;
      std::shared_ptr<FeatureVector> centers = aligned_shared<FeatureVector>();
      int32_t first_child = (index + 1 + accelerator) * splits();
      for (int32_t child = first_child;
           child < first_child + static_cast<int32_t>(splits()); ++child) {
        if (!valid_centers_[child])
          break;  // Fewer than splits() children.
        centers->push_back(centers_[child]);
      }
      search_accelerators_.push_back(SearchAccelerator(centers, distance_));
    }
    index = (index + 1) * splits();
  }
}

template <class Feature, class Distance, class FeatureAllocator>
bool VocabularyTree<Feature, Distance, FeatureAllocator>::Load(
    const std::string& file) {
  std::ifstream in;
  in.open(file.c_str(), std::ios_base::binary);
  if (!in.is_open()) {
    VLOG(3) << "Failed to load vocabulary file: " + file;
    return false;
  }
  return Load(&in);
}

template <class Feature, class Distance, class FeatureAllocator>
bool VocabularyTree<Feature, Distance, FeatureAllocator>::Load(
    std::ifstream* in) {
  CHECK_NOTNULL(in);
  if (!in->is_open()) {
    VLOG(3) << "Failed to load vocabulary file";
    return false;
  }

  Clear();

  int deserialized_version;
  std::string deserialized_descriptor;

  common::Deserialize(&deserialized_version, in);
  CHECK_EQ(deserialized_version, kSerializationVersion)
      << "This vocabulary file was saved with a different version.";
  common::Deserialize(&deserialized_descriptor, in);
  std::string descriptor_tree = FLAGS_feature_descriptor_type;
  CHECK_EQ(deserialized_descriptor, descriptor_tree)
      << "This vocabulary tree file was trained for a different descriptor "
      << "type.";
  common::Deserialize(&num_splits_, in);
  common::Deserialize(&levels_, in);
  common::Deserialize(&centers_, in);
  common::Deserialize(&valid_centers_, in);

  SetNodeCounts();
  CHECK_EQ(centers_.size(), num_words_ + word_start_);
  return true;
}

template <class Feature, class Distance, class FeatureAllocator>
void VocabularyTree<Feature, Distance, FeatureAllocator>::SetNodeCounts() {
  num_words_ = num_splits_;
  word_start_ = 0;
  for (uint32_t i = 0; i < levels_ - 1; ++i) {
    word_start_ += num_words_;
    num_words_ *= num_splits_;
  }
  InitializeSearchAccelerators();
}
}  // namespace loop_closure
#endif  // VOCABULARY_TREE_VOCABULARY_TREE_INL_H_
