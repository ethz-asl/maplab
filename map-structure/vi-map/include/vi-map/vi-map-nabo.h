#ifndef VI_MAP_VI_MAP_NABO_H_
#define VI_MAP_VI_MAP_NABO_H_

#include <vector>

#include <Eigen/Dense>
#include <glog/logging.h>
#include <nabo/nabo.h>

#include "vi-map/vi-map.h"

namespace vi_map {

// "libnabo" wrapper for VIMap. Works for vertices or landmarks.
// See SpatialDatabase for "all within radius" queries.
// TODO(tcies) should this be consolidated with VectorizedMission?
template <typename ObjectIdType>
class VIMapNabo {
 public:
  explicit VIMapNabo(const VIMap& map) {
    map.template getAllIds<ObjectIdType>(&id_for_index_);
    initializeDatabaseFromIds(map);
  }

  VIMapNabo(const VIMap& map, const MissionId& mission_to_ignore) {
    map.forEachMission([&](const MissionId& mission_id) {
      if (mission_id != mission_to_ignore) {
        std::vector<ObjectIdType> objects_in_mission;
        map.template getAllIdsInMission(mission_id, &objects_in_mission);
        id_for_index_.insert(
            id_for_index_.end(), objects_in_mission.begin(),
            objects_in_mission.end());
      }
    });
    initializeDatabaseFromIds(map);
  }

  void knn(
      const Eigen::Vector3d& p_G_query, const size_t k,
      std::vector<ObjectIdType>* result, Eigen::VectorXd* dists2) const {
    CHECK_NOTNULL(result)->resize(k);
    CHECK_NOTNULL(dists2)->resize(k, 1);

    CHECK(database_);
    CHECK_GE(database_->cloud.cols(), k) << "Query must not exceed cloud size!";
    // Nabo needs dynamic vectors.
    Eigen::VectorXd query = p_G_query;
    Eigen::VectorXi indices(k);
    database_->knn(query, indices, *dists2, k);

    for (int i = 0; i < k; ++i) {
      CHECK_LT(indices(i), id_for_index_.size());
      (*result)[i] = id_for_index_[indices(i)];
    }
  }

  ObjectIdType closest(
      const Eigen::Vector3d& p_G_query, double* const dist2) const {
    CHECK_NOTNULL(dist2);
    std::vector<ObjectIdType> result;
    Eigen::VectorXd dists2;
    knn(p_G_query, 1u, &result, &dists2);
    *dist2 = dists2(0);
    CHECK(!result.empty());
    return result.front();
  }

 private:
  void initializeDatabaseFromIds(const VIMap& map) {
    CHECK(!id_for_index_.empty());

    Eigen::MatrixXd database(3, id_for_index_.size());

    for (int i = 0; i < id_for_index_.size(); ++i) {
      database.col(i) = map.get_p_G(id_for_index_[i]);
    }

    database_.reset(Nabo::NNSearchD::createKDTreeLinearHeap(database));
    CHECK(database_);
  }

  std::unique_ptr<Nabo::NNSearchD> database_;
  std::vector<ObjectIdType> id_for_index_;
};

}  // namespace vi_map

#endif  // VI_MAP_VI_MAP_NABO_H_
