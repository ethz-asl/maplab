#ifndef SEMANTIFY_PLUGIN_K_MEDOIDS_CLUSTERING_MANAGER_H_
#define SEMANTIFY_PLUGIN_K_MEDOIDS_CLUSTERING_MANAGER_H_

#include <unordered_map>
#include <memory>

#include <Eigen/Dense>

#include <aslam/frames/visual-frame.h>
// #include <K-Medoids-Clustering/kMedoids.h>

#include "semantify-plugin/k-medoids.hpp"

namespace semantify {

class KMedoidsClusteringManager {
public:
  typedef std::unordered_map<int, std::vector<clustering::object_id>> KMedoidsIndexMap;

  KMedoidsClusteringManager();

  void createKMedoids(
      int track_id, 
      const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors, int num_cluster);
  void displayKMedoidsScores();
  KMedoidsClusteringManager::KMedoidsIndexMap getKMedoidsIndiciesForTrackId(int track_id);
  const aslam::VisualFrame::SemanticObjectDescriptorsT& getKMedoidsDescriptors(int track_id);
private:
  void build_dissimilarity_matrix(
    const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors,
    clustering::dissimilarity_matrix* distance);

  void compute_silhouette_score(const clustering::dissimilarity_matrix& distance);

// if we dont store this, then need to rebuild it every time
std::unordered_map <int,aslam::VisualFrame::SemanticObjectDescriptorsT> track_id_to_descriptors_;
std::unordered_map <int,clustering::dissimilarity_matrix> track_id_to_dissimilarity_matrix_;

std::unordered_map <int, std::unique_ptr<clustering::kmedoids>> track_id_to_kMedoids_1_;
std::unordered_map <int, std::unique_ptr<clustering::kmedoids>> track_id_to_kMedoids_2_;
std::unordered_map <int, std::unique_ptr<clustering::kmedoids>> track_id_to_kMedoids_3_;
std::unordered_map <int, std::unique_ptr<clustering::kmedoids>> track_id_to_kMedoids_4_;

};
}  // namespace semantify
#endif  // SEMANTIFY_PLUGIN_K_MEDOIDS_CLUSTERING_MANAGER_H_