#include "semantify-plugin/k-medoids-clustering-manager.h"

#include <iostream>
#include <string>

namespace semantify {
KMedoidsClusteringManager::KMedoidsClusteringManager() {

}

void KMedoidsClusteringManager::createKMedoids( int track_id,
  const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors, int num_cluster) {
  CHECK_GT(track_id, 0u);
  CHECK_GT(num_cluster, 0u);
  CHECK_GT(descriptors.cols(), 0u);
  size_t num_objects = descriptors.cols();
  clustering::dissimilarity_matrix distance;
  distance.resize(num_objects,num_objects);
  build_dissimilarity_matrix(descriptors, &distance);
  track_id_to_dissimilarity_matrix_[track_id] = distance;
  track_id_to_descriptors_[track_id] = descriptors;
  std::unique_ptr<clustering::kmedoids> medoids_1(new clustering::kmedoids(num_objects));
  track_id_to_kMedoids_1_[track_id] = std::move(medoids_1);
  track_id_to_kMedoids_1_[track_id]->pam(distance,1,NULL);

  std::unique_ptr<clustering::kmedoids> medoids_2(new clustering::kmedoids(num_objects));
  track_id_to_kMedoids_2_[track_id] = std::move(medoids_2);
  track_id_to_kMedoids_2_[track_id]->pam(distance,2,NULL);

  std::unique_ptr<clustering::kmedoids> medoids_3(new clustering::kmedoids(num_objects));
  track_id_to_kMedoids_3_[track_id] = std::move(medoids_3);
  track_id_to_kMedoids_3_[track_id]->pam(distance,3,NULL);

  std::unique_ptr<clustering::kmedoids> medoids_4(new clustering::kmedoids(num_objects));
  track_id_to_kMedoids_4_[track_id] = std::move(medoids_4);
  track_id_to_kMedoids_4_[track_id]->pam(distance,4,NULL);
}

void KMedoidsClusteringManager::displayKMedoidsScores() {
  for(const auto& pair: track_id_to_kMedoids_1_) {
    int track_id = pair.first;
    const clustering::dissimilarity_matrix& distance = track_id_to_dissimilarity_matrix_[track_id];
    
    LOG(INFO)<<"track_id: "<< track_id; 
    LOG(INFO)<<"for k = 1:";
    for(size_t i  = 0; i<track_id_to_kMedoids_1_[track_id]->cluster_list.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" size: "<< track_id_to_kMedoids_1_[track_id]->cluster_list[i].size();
    }
    LOG(INFO)<<"Mean dist_from_medoid: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_1_[track_id]->mean_dist_from_medoid.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean_dist_from_medoid: "<<track_id_to_kMedoids_1_[track_id]->mean_dist_from_medoid[i];
    }
    LOG(INFO)<<"Mean a_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_1_[track_id]->mean_a_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean a_score: "<<track_id_to_kMedoids_1_[track_id]->mean_a_scores[i];
    }
    // LOG(INFO)<<"a_scores:";
    // for(size_t i  = 0; i<track_id_to_kMedoids_1_[track_id]->a_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" a_score: "<<track_id_to_kMedoids_1_[track_id]->a_scores[i];
    // }
    
    LOG(INFO)<<"for k = 2:";
    for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->cluster_list.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" size: "<< track_id_to_kMedoids_2_[track_id]->cluster_list[i].size();
    }
    LOG(INFO)<<"Mean dist_from_medoid: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->mean_dist_from_medoid.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean_dist_from_medoid: "<<track_id_to_kMedoids_2_[track_id]->mean_dist_from_medoid[i];
    }
    LOG(INFO)<<"dist_between_medoids: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->medoid_ids.size(); i++) {
      for(size_t j  = i+1; j<track_id_to_kMedoids_2_[track_id]->medoid_ids.size(); j++) {
        size_t index_i = track_id_to_kMedoids_2_[track_id]->medoid_ids[i];
        size_t index_j = track_id_to_kMedoids_2_[track_id]->medoid_ids[j];
        LOG(INFO)<<"cluster index: "<<i <<" vs "<<j<<" : "<<distance(index_i,index_j);
      }
    }
    LOG(INFO)<<"Mean s_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->mean_silhouette_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean s_score: "<<track_id_to_kMedoids_2_[track_id]->mean_silhouette_scores[i];
    }
    LOG(INFO)<<"Mean a_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->mean_a_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean a_score: "<<track_id_to_kMedoids_2_[track_id]->mean_a_scores[i];
    }
    LOG(INFO)<<"Mean b_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->mean_b_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean b_score: "<<track_id_to_kMedoids_2_[track_id]->mean_b_scores[i];
    }
    // for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->silhouette_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" s_score: "<<track_id_to_kMedoids_2_[track_id]->silhouette_scores[i];
    // }
    // LOG(INFO)<<"a_scores:";
    // for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->a_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" a_score: "<<track_id_to_kMedoids_2_[track_id]->a_scores[i];
    // }
    // LOG(INFO)<<"b_scores:";
    // for(size_t i  = 0; i<track_id_to_kMedoids_2_[track_id]->b_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" b_score: "<<track_id_to_kMedoids_2_[track_id]->b_scores[i];
    // }
    LOG(INFO)<<"for k = 3:";
    for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->cluster_list.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" size: "<< track_id_to_kMedoids_3_[track_id]->cluster_list[i].size();
    }
    LOG(INFO)<<"Mean dist_from_medoid: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->mean_dist_from_medoid.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean_dist_from_medoid: "<<track_id_to_kMedoids_3_[track_id]->mean_dist_from_medoid[i];
    }
    LOG(INFO)<<"dist_between_medoids: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->medoid_ids.size(); i++) {
      for(size_t j  = i+1; j<track_id_to_kMedoids_3_[track_id]->medoid_ids.size(); j++) {
        size_t index_i = track_id_to_kMedoids_3_[track_id]->medoid_ids[i];
        size_t index_j = track_id_to_kMedoids_3_[track_id]->medoid_ids[j];
        LOG(INFO)<<"cluster index: "<<i <<" vs "<<j<<" : "<<distance(index_i,index_j);
      }
    }
    LOG(INFO)<<"Mean s_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->mean_silhouette_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean s_score: "<<track_id_to_kMedoids_3_[track_id]->mean_silhouette_scores[i];
    }
    LOG(INFO)<<"Mean a_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->mean_a_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean a_score: "<<track_id_to_kMedoids_3_[track_id]->mean_a_scores[i];
    }
    LOG(INFO)<<"Mean b_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->mean_b_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean b_score: "<<track_id_to_kMedoids_3_[track_id]->mean_b_scores[i];
    }
    // for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->silhouette_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" s_score: "<<track_id_to_kMedoids_3_[track_id]->silhouette_scores[i];
    // }
    // LOG(INFO)<<"a_scores:";
    // for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->a_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" a_score: "<<track_id_to_kMedoids_3_[track_id]->a_scores[i];
    // }
    // LOG(INFO)<<"b_scores:";
    // for(size_t i  = 0; i<track_id_to_kMedoids_3_[track_id]->b_scores.size(); i++) {
    //   LOG(INFO)<<"index: "<<i <<" b_score: "<<track_id_to_kMedoids_3_[track_id]->b_scores[i];
    // }

    LOG(INFO)<<"for k = 4:";
    for(size_t i  = 0; i<track_id_to_kMedoids_4_[track_id]->cluster_list.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" size: "<< track_id_to_kMedoids_4_[track_id]->cluster_list[i].size();
    }
    LOG(INFO)<<"Mean dist_from_medoid: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_4_[track_id]->mean_dist_from_medoid.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean_dist_from_medoid: "<<track_id_to_kMedoids_4_[track_id]->mean_dist_from_medoid[i];
    }
    LOG(INFO)<<"dist_between_medoids: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_4_[track_id]->medoid_ids.size(); i++) {
      for(size_t j  = i+1; j<track_id_to_kMedoids_4_[track_id]->medoid_ids.size(); j++) {
        size_t index_i = track_id_to_kMedoids_4_[track_id]->medoid_ids[i];
        size_t index_j = track_id_to_kMedoids_4_[track_id]->medoid_ids[j];
        LOG(INFO)<<"cluster index: "<<i <<" vs "<<j<<" : "<<distance(index_i,index_j);
      }
    }
    LOG(INFO)<<"Mean s_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_4_[track_id]->mean_silhouette_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean s_score: "<<track_id_to_kMedoids_4_[track_id]->mean_silhouette_scores[i];
    }
    LOG(INFO)<<"Mean a_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_4_[track_id]->mean_a_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean a_score: "<<track_id_to_kMedoids_4_[track_id]->mean_a_scores[i];
    }
    LOG(INFO)<<"Mean b_scores: ";
    for(size_t i  = 0; i<track_id_to_kMedoids_4_[track_id]->mean_b_scores.size(); i++) {
      LOG(INFO)<<"cluster index: "<<i <<" mean b_score: "<<track_id_to_kMedoids_4_[track_id]->mean_b_scores[i];
    }
  }
}

KMedoidsClusteringManager::KMedoidsIndexMap KMedoidsClusteringManager::getKMedoidsIndiciesForTrackId(int track_id) {
  KMedoidsClusteringManager::KMedoidsIndexMap map;
  map[1] = track_id_to_kMedoids_1_[track_id]->medoid_ids;
  map[2] = track_id_to_kMedoids_2_[track_id]->medoid_ids;
  map[3] = track_id_to_kMedoids_3_[track_id]->medoid_ids;
  map[4] = track_id_to_kMedoids_4_[track_id]->medoid_ids;
  return map;
}

const aslam::VisualFrame::SemanticObjectDescriptorsT& KMedoidsClusteringManager::getKMedoidsDescriptors(int track_id) {
  return track_id_to_descriptors_[track_id];
}

// private
void KMedoidsClusteringManager::build_dissimilarity_matrix(
  const aslam::VisualFrame::SemanticObjectDescriptorsT& descriptors,
  clustering::dissimilarity_matrix* distance) {
  for(int i = 0; i<distance->cols(); i++){
    for(int j = 0; j<distance->rows(); j++){
      (*distance)(i,j) = (descriptors.col(i) - descriptors.col(j)).norm();
      if (std::isnan((*distance)(i,j))) {
        LOG(ERROR)<<"The distance matrix contains NAN value";
      }
    }
  }
}

} // namespace semantify