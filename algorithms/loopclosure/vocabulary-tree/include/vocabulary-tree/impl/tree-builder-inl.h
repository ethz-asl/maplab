// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_TREE_BUILDER_INL_H_
#define VOCABULARY_TREE_TREE_BUILDER_INL_H_

#include <deque>
#include <vector>

#include <vocabulary-tree/mutable-tree.h>
#include <vocabulary-tree/simple-kmeans.h>

namespace loop_closure {

template <class Feature, class Distance, class FeatureAllocator>
TreeBuilder<Feature, Distance, FeatureAllocator>::TreeBuilder(
    const Feature& zero, Distance d)
    : tree_(d), kmeans_(zero, d), zero_(zero), distance_(d) {}

template <class Feature, class Distance, class FeatureAllocator>
void TreeBuilder<Feature, Distance, FeatureAllocator>::Build(
    const FeatureVector& training_features, uint32_t k, uint32_t levels) {
  // Initial setup and memory allocation for the tree
  CHECK_LT(
      pow(static_cast<double>(k), static_cast<double>(levels)),
      5 * pow(static_cast<double>(10), static_cast<double>(6)))
      << "Double check your settings for levels and splits. The tree you "
         "wanted to build is likely too large to fit in memory.";

  tree_.Clear();
  tree_.SetSize(levels, k);
  tree_.centers().reserve(tree_.nodes());
  tree_.validCenters().reserve(tree_.nodes());

  std::mt19937 generator(10);

  // We keep a queue of disjoint feature subsets to cluster.
  // Feature* is used to avoid copying features.
  std::deque<std::vector<Feature*> > subset_queue(1);

  // At first the queue contains one "subset" containing all the features.
  std::vector<Feature*>& feature_ptrs = subset_queue.front();
  feature_ptrs.reserve(training_features.size());
  for (const Feature& f : training_features)
    feature_ptrs.push_back(const_cast<Feature*>(&f));

  std::shared_ptr<FeatureVector> centers(new FeatureVector);
  for (uint32_t level = 0u; level < levels; ++level) {
    std::vector<unsigned int> membership;

    for (size_t i = 0u, ie = subset_queue.size(); i < ie; ++i) {
      std::vector<Feature*>& subset = subset_queue.front();

      // Check that we have k different points:
      std::vector<Feature*> unique_clusters;
      for (size_t subset_idx = 0; subset_idx < subset.size(); ++subset_idx) {
        bool found_different_point = true;
        for (size_t l = 0; l < unique_clusters.size(); ++l) {
          float dist = distance_(*subset[subset_idx], *unique_clusters[l]);
          if (dist == 0) {
            found_different_point = false;
          }
        }
        if (found_different_point) {
          unique_clusters.push_back(subset[subset_idx]);
        }
        if (unique_clusters.size() > k) {
          break;
        }
      }
      // If the subset already has k or fewer elements, just use those as the
      // centers.
      if (unique_clusters.size() <= k) {
        for (size_t j = 0; j < unique_clusters.size(); ++j) {
          tree_.centers().push_back(*unique_clusters[j]);
          tree_.validCenters().push_back(1);
        }
        // Mark non-existent centers as invalid.
        tree_.centers().insert(
            tree_.centers().end(), k - unique_clusters.size(), zero_);
        tree_.validCenters().insert(
            tree_.validCenters().end(), k - unique_clusters.size(), 0);

        // Push k empty subsets into the queue so all children get marked
        // invalid.
        subset_queue.pop_front();
        subset_queue.insert(subset_queue.end(), k, std::vector<Feature*>());
      } else {
        int random_seed = generator();

        // Cluster the current subset into k centers.
        kmeans_.ClusterPointers(subset, k, random_seed, &centers, &membership);

        // Add the centers and mark them as valid.
        tree_.centers().insert(
            tree_.centers().end(), centers->begin(), centers->end());
        tree_.validCenters().insert(tree_.validCenters().end(), k, 1);

        // Partition the current subset into k new subsets based on the cluster
        // assignments.
        std::vector<std::vector<Feature*> > new_subsets(k);
        for (size_t j = 0; j < subset.size(); ++j) {
          new_subsets[membership[j]].push_back(subset[j]);
        }

        // Update the queue
        subset_queue.pop_front();
        subset_queue.insert(
            subset_queue.end(), new_subsets.begin(), new_subsets.end());
      }
    }
  }
  tree_.SetNodeCounts();
}

}  // namespace loop_closure
#endif  // VOCABULARY_TREE_TREE_BUILDER_INL_H_
