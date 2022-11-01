#ifndef VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
#define VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_

#include <functional>
#include <limits>
#include <unordered_set>

#include <glog/logging.h>

namespace vi_map_helpers {

template <typename QueryType, typename DataType>
bool VIMapNearestNeighborLookupBase<QueryType, DataType>::empty() const {
  return size() == 0u;
}

template <typename QueryType, typename DataType>
size_t VIMapNearestNeighborLookupBase<QueryType, DataType>::size() const {
  CHECK_GE(nn_index_data_.cols(), 0);
  return static_cast<size_t>(nn_index_data_.cols());
}

template <typename QueryType, typename DataType>
bool VIMapNearestNeighborLookupBase<QueryType, DataType>::getClosestDataItem(
    const QueryType& query, DataType* closest_data_item) const {
  CHECK_NOTNULL(closest_data_item);
  if (empty()) {
    LOG(WARNING) << "The nearest-neighbor index is empty. "
                 << "Can't look for closest data item.";
    return false;
  }
  CHECK(nn_index_);
  CHECK_EQ(size(), data_items_.size());

  constexpr int kNumNeighbors = 1;

  Eigen::VectorXi index = Eigen::VectorXi::Constant(kNumNeighbors, -1);
  Eigen::VectorXd distance_squared = Eigen::VectorXd::Constant(
      kNumNeighbors, std::numeric_limits<double>::infinity());
  constexpr double kSearchNNEpsilon = 0.0;
  const int kOptionFlags = Nabo::NNSearchD::ALLOW_SELF_MATCH;
  const Eigen::VectorXd query_vector = queryTypeToVector(query);
  CHECK_EQ(query_vector.rows(), nn_index_data_.rows());
  nn_index_->knn(
      query_vector, index, distance_squared, kNumNeighbors, kSearchNNEpsilon,
      kOptionFlags);
  const double nn_distance_squared = distance_squared(0);
  const int nn_index = index(0);

  if (nn_index >= 0 &&
      nn_distance_squared != std::numeric_limits<double>::infinity()) {
    CHECK_LT(static_cast<size_t>(nn_index), data_items_.size());
    *closest_data_item = data_items_[nn_index];
    return true;
  }
  return false;
}

template <typename QueryType, typename DataType>
template <typename Allocator>
void VIMapNearestNeighborLookupBase<QueryType, DataType>::
    getAllDataItemsWithinRadius(
        const QueryType& query, const double search_radius,
        std::unordered_set<DataType, std::hash<DataType>,
                           std::equal_to<DataType>, Allocator>*
            data_items_within_search_radius) const {
  CHECK_NOTNULL(data_items_within_search_radius)->clear();
  if (empty()) {
    LOG(WARNING) << "The nearest-neighbor index is empty. "
                 << "Can't look for data items within a radius.";
    return;
  }
  CHECK(nn_index_);
  CHECK_EQ(size(), data_items_.size());

  const int num_neighbors = size();
  CHECK_GT(num_neighbors, 0);

  Eigen::VectorXi index = Eigen::VectorXi::Constant(num_neighbors, -1);

  Eigen::VectorXd distance_squared = Eigen::VectorXd::Constant(
      num_neighbors, std::numeric_limits<double>::infinity());
  constexpr double kSearchNNEpsilon = 0.0;
  const int kOptionFlags = Nabo::NNSearchD::ALLOW_SELF_MATCH;
  const Eigen::VectorXd query_vector = queryTypeToVector(query);
  CHECK_EQ(query_vector.rows(), nn_index_data_.rows());
  nn_index_->knn(
      query_vector, index, distance_squared, num_neighbors, kSearchNNEpsilon,
      kOptionFlags, search_radius);

  size_t result_idx = 0u;
  while (result_idx < static_cast<size_t>(num_neighbors) &&
         distance_squared[result_idx] <
             std::numeric_limits<double>::infinity()) {
    const int nn_index = index(result_idx);
    CHECK_LT(nn_index, static_cast<int>(data_items_.size()));
    data_items_within_search_radius->emplace(data_items_[nn_index]);
    ++result_idx;
  }
}

template <>
inline Eigen::VectorXd queryTypeToVector(const aslam::Position3D& p_G_I) {
  return p_G_I;
}

template <typename QueryType, typename DataType>
void BasicMapElementsVIMapNearestNeighborLookup<
    QueryType, DataType>::buildIndexImpl(const vi_map::VIMap& map) {
  map.getAllIds(BaseType::getDataItemsMutable());
  const size_t num_items_in_map = BaseType::getDataItems().size();
  if (num_items_in_map == 0u) {
    LOG(WARNING) << "Aborting building nearest-neighbor index because the "
                 << "given map contains zero items.";
    return;
  }
  BaseType::getDataItemsMutable()->reserve(num_items_in_map);
  constexpr size_t kPositionDimensions = 3u;
  BaseType::resizeIndexData(kPositionDimensions, num_items_in_map);

  int col_idx = 0;
  for (const DataType& item_id : BaseType::getDataItems()) {
    CHECK(item_id.isValid());
    const QueryType data_item = map.get_p_G(item_id);
    BaseType::setNearestNeighborIndexDataColumn(
        queryTypeToVector(data_item), col_idx);
    ++col_idx;
  }
  BaseType::initializeIndex();
}

}  // namespace vi_map_helpers

#endif  // VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_INL_H_
