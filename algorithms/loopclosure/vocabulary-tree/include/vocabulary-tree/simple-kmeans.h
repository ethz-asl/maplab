// Original code Copyright Willowgarage as part of ROS, adapted here (BSD).
// http://ros.org/wiki/vocabulary_tree
#ifndef VOCABULARY_TREE_SIMPLE_KMEANS_H_
#define VOCABULARY_TREE_SIMPLE_KMEANS_H_

#include <algorithm>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <vector>

#include <maplab-common/parallel-process.h>

#include "vocabulary-tree/distance.h"
#include "vocabulary-tree/feature-allocator.h"

namespace loop_closure {

// Forward declare function objects for choosing the initial centers
template <typename Feature>
struct InitKMeansPlusPlus;
template <typename Feature>
struct InitRandom;
template <typename Feature>
struct InitGiven;

// Class for performing K-means clustering, optimized for a particular feature
// type and metric. The standard Lloyd's algorithm is used. By default, cluster
// centers are initialized randomly.
template <class Feature, class Distance = distance::L2<Feature>,
          class FeatureAllocator = typename DefaultAllocator<Feature>::type>
class SimpleKmeans {
 public:
  typedef typename Distance::result_type SquaredDistanceType;
  typedef std::shared_ptr<std::vector<Feature, FeatureAllocator> > Centers;
  typedef std::function<void(
      const std::vector<Feature*>&, size_t, Distance, int random_seed,
      std::vector<Feature, FeatureAllocator>*)>
      Initializer;

  // - zero Object representing zero in the feature space
  // - d    Functor for calculating squared distance
  SimpleKmeans(const Feature& zero = Feature(), const Distance& d = Distance());

  // Set function object used to choose initial cluster centers.
  void SetInitMethod(const Initializer& init) {
    choose_centers_ = init;
  }

  size_t GetMaxIterations() const {
    return max_iterations_;
  }
  void SetMaxIterations(size_t iters) {
    max_iterations_ = iters;
  }

  size_t GetRestarts() const {
    return restarts_;
  }
  void SetRestarts(size_t restarts) {
    restarts_ = restarts;
  }

  // Partition a set of features into k clusters.
  // - features   The features to be clustered.
  // - k          The number of clusters.
  // - centers    A set of k cluster centers.
  // - membership Cluster assignment for each feature
  SquaredDistanceType Cluster(
      const std::vector<Feature, FeatureAllocator>& features, size_t k,
      int random_seed, std::vector<unsigned int>* const membership,
      Centers* const centers) const;

  // Partition a set of features into k clusters.
  // This version is more convenient for hierarchical clustering, as you do not
  // have to copy feature objects.

  // - features   The features to be clustered.
  // - k          The number of clusters.
  // - centers    A set of k cluster centers.
  // -  membership Cluster assignment for each feature
  SquaredDistanceType ClusterPointers(
      const std::vector<Feature*>& features, size_t k, int random_seed,
      Centers* const centers,
      std::vector<unsigned int>* const membership) const;

 private:
  SquaredDistanceType ClusterOnce(
      const std::vector<Feature*>& features, size_t k, int random_seed,
      Centers* const centers,
      std::vector<unsigned int>* const membership) const;

  Feature zero_;
  Distance distance_;
  Initializer choose_centers_;
  size_t max_iterations_;
  size_t restarts_;
};

// Initializer for K-means that randomly selects k features as the cluster
// centers.
template <class Feature>
struct InitRandom {
  explicit InitRandom(const Feature& zero) : zero_(zero) {}

  template <class Distance, class FeatureAllocator>
  void operator()(
      const std::vector<Feature*>& features, size_t k, Distance /*distance*/,
      int random_seed, std::vector<Feature, FeatureAllocator>* centers) {
    CHECK_NOTNULL(centers);
    centers->clear();
    centers->resize(k, zero_);
    std::mt19937 generator(random_seed);
    // Construct a random permutation of the features using a Fisher-Yates
    // shuffle.
    std::vector<Feature*> features_perm = features;
    for (size_t i = features.size(); i > 1; --i) {
      size_t k = generator() % i;
      std::swap(features_perm[i - 1], features_perm[k]);
    }
    // Take the first k permuted features as the initial centers
    for (size_t i = 0; i < centers->size(); ++i) {
      centers[i] = *features_perm[i];
    }
  }

 private:
  Feature zero_;
};

// Initializer for K-means that selects the cluster centers folowing the
// kmeans++ algorithm.
template <class Feature>
struct InitKMeansPlusPlus {
  explicit InitKMeansPlusPlus(const Feature& zero) : zero_(zero) {}

  template <class Distance, class FeatureAllocator>
  void operator()(
      const std::vector<Feature*>& descriptors, size_t k, Distance distance,
      int random_seed, std::vector<Feature, FeatureAllocator>* centers) {
    CHECK_NOTNULL(centers);
    std::mt19937 generator(random_seed);
    centers->resize(k, zero_);
    std::vector<double> minimum_distances(
        descriptors.size(), std::numeric_limits<double>::max());
    int descriptor_idx = generator() % descriptors.size();

    // Helper struct and method for parallel distance computation of
    // existing center to descriptors.
    struct ParallelDistance {
      ParallelDistance(
          Distance _distance, const Feature& _feature,
          const std::vector<Feature*>& _features,
          std::vector<double>* _distances)
          : distance(_distance),
            feature(_feature),
            features(_features),
            distances(_distances) {}
      Distance distance;
      const Feature& feature;
      const std::vector<Feature*>& features;
      std::vector<double>* distances;
      void operator()(const std::vector<size_t>& range) const {
        for (size_t i : range) {
          (*distances)[i] = distance(feature, *features[i]);
        }
      }
    };

    std::function<void(
        const Distance& distance, const Feature&, const std::vector<Feature*>&,
        std::vector<double>* distances)>
        parallel_distance =
            [](const Distance& distance, const Feature& feature,
               const std::vector<Feature*>& features,
               std::vector<double>* distances) {
              CHECK_NOTNULL(distances);
              distances->resize(features.size());

              ParallelDistance distance_functor(
                  distance, feature, features, distances);

              const bool kAlwaysParallelize = false;
              const size_t num_threads = common::getNumHardwareThreads();
              common::ParallelProcess(
                  features.size(), distance_functor, kAlwaysParallelize,
                  num_threads);
            };

    size_t num_centers = 0;
    // Create first cluster.
    (*centers)[0] = (*descriptors[descriptor_idx]);
    ++num_centers;

    // Compute the initial distances.
    std::vector<double>::iterator minimum_distance_iterator;
    minimum_distance_iterator = minimum_distances.begin();

    std::vector<double> distances;
    parallel_distance(
        distance, (*centers)[num_centers - 1], descriptors, &distances);

    for (size_t i = 0; i < descriptors.size();
         ++i, ++minimum_distance_iterator) {
      *minimum_distance_iterator = distances[i];
    }

    while (num_centers < k) {
      minimum_distance_iterator = minimum_distances.begin();
      std::vector<double> distances;
      parallel_distance(
          distance, (*centers)[num_centers - 1], descriptors, &distances);
      for (size_t i = 0; i < descriptors.size();
           ++i, ++minimum_distance_iterator) {
        if (*minimum_distance_iterator > 0) {
          double dist = distances[i];
          if (dist < *minimum_distance_iterator)
            *minimum_distance_iterator = dist;
        }
      }
      double distance_sum = std::accumulate(
          minimum_distances.begin(), minimum_distances.end(), 0.0);

      if (distance_sum > 0) {
        double random_cutoff;
        do {
          random_cutoff =
              (static_cast<double>(generator()) / RAND_MAX) * distance_sum;
        } while (random_cutoff == 0.0);

        double partial_sum = 0;
        for (minimum_distance_iterator = minimum_distances.begin();
             minimum_distance_iterator != minimum_distances.end();
             ++minimum_distance_iterator) {
          partial_sum += *minimum_distance_iterator;
          if (partial_sum >= random_cutoff)
            break;
        }

        if (minimum_distance_iterator == minimum_distances.end()) {
          descriptor_idx = descriptors.size() - 1;
        } else {
          descriptor_idx =
              minimum_distance_iterator - minimum_distances.begin();
        }
        (*centers)[num_centers] = (*descriptors[descriptor_idx]);
        ++num_centers;
      } else {
        break;
      }
    }
  }

 private:
  Feature zero_;
};

//  Dummy initializer for K-means that leaves the centers as-is.
template <class Feature>
struct InitGiven {
  explicit InitGiven(const Feature& zero) : zero_(zero) {}
  template <class Distance, class FeatureAllocator>
  void operator()(
      const std::vector<Feature*>& /*features*/, size_t /*k*/,
      Distance /*distance*/, int /*random_seed*/,
      std::vector<Feature, FeatureAllocator>* /*centers*/) {}

 private:
  Feature zero_;
};

}  // namespace loop_closure

#include "impl/simple-kmeans-inl.h"
#endif  // VOCABULARY_TREE_SIMPLE_KMEANS_H_
