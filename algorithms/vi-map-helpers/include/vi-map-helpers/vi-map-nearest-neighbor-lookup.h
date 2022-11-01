#ifndef VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_
#define VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_

#include <functional>
#include <memory>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <aslam/common/memory.h>
#include <gtest/gtest.h>
#include <maplab-common/macros.h>
#include <nabo/nabo.h>
#include <posegraph/unique-id.h>
#include <vi-map/unique-id.h>
#include <vi-map/vi-map.h>

namespace vi_map_helpers {

template <typename QueryType, typename DataType>
class VIMapNearestNeighborLookupBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FRIEND_TEST(VIMapNearestNeighborLookupTest, MultipleVertexMap);
  FRIEND_TEST(VIMapNearestNeighborLookupTest, MultipleVertexRadiusLookup);
  VIMapNearestNeighborLookupBase() = default;
  virtual ~VIMapNearestNeighborLookupBase() = default;

  // Returns the data item in the map that is closest (L2 norm) to the given
  // query object.
  bool getClosestDataItem(
      const QueryType& query, DataType* closest_data_item) const;

  // Returns all items found to be within the given search radius (L2 norm) of
  // the given query object.
  template <typename Allocator>
  void getAllDataItemsWithinRadius(
      const QueryType& query, const double search_radius,
      std::unordered_set<DataType, std::hash<DataType>, std::equal_to<DataType>,
                         Allocator>* data_items_within_search_radius) const;

  inline size_t size() const;
  inline bool empty() const;

 protected:
  void buildIndex(const vi_map::VIMap& map) {
    LOG_IF(WARNING, nn_index_)
        << "The already existing k-d tree index will be overwritten.";
    data_items_.clear();
    buildIndexImpl(map);
    CHECK(empty() || nn_index_);
    CHECK_EQ(nn_index_data_.cols(), static_cast<int>(data_items_.size()));
    CHECK_EQ(size(), data_items_.size());
  }

  std::vector<DataType>* getDataItemsMutable() {
    return &data_items_;
  }

  const std::vector<DataType>& getDataItems() const {
    return data_items_;
  }

  void resizeIndexData(const size_t num_rows, const size_t num_cols) {
    nn_index_data_.resize(num_rows, num_cols);
  }

  template <typename... _Args>
  void emplaceBackDataItem(_Args&&... __args) {
    data_items_.emplace_back(std::forward<_Args>(__args)...);
  }

  void setNearestNeighborIndexDataColumn(
      const Eigen::VectorXd& data, const size_t col_idx) {
    CHECK_EQ(data.rows(), nn_index_data_.rows());
    CHECK_LT(col_idx, static_cast<size_t>(nn_index_data_.cols()));
    nn_index_data_.col(col_idx) = data;
  }

  void initializeIndex() {
    CHECK_GT(nn_index_data_.rows(), 0);
    CHECK_GT(nn_index_data_.cols(), 0);
    CHECK(!nn_index_);
    nn_index_.reset(
        Nabo::NNSearchD::createKDTreeLinearHeap(
            nn_index_data_, nn_index_data_.rows()));
  }

 private:
  std::unique_ptr<Nabo::NNSearchD> nn_index_;
  Eigen::MatrixXd nn_index_data_;
  std::vector<DataType> data_items_;

  virtual void buildIndexImpl(const vi_map::VIMap& map) = 0;
};

template <typename QueryType, typename DataType>
class BasicMapElementsVIMapNearestNeighborLookup final
    : public VIMapNearestNeighborLookupBase<QueryType, DataType> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  MAPLAB_POINTER_TYPEDEFS(BasicMapElementsVIMapNearestNeighborLookup);
  explicit BasicMapElementsVIMapNearestNeighborLookup(
      const vi_map::VIMap& map) {
    static_assert(
        std::is_same<DataType, vi_map::LandmarkId>::value ||
            std::is_same<DataType, pose_graph::VertexId>::value,
        "Invalid data type. The data type must be either LandmarkId or "
        "VertexId");
    BasicMapElementsVIMapNearestNeighborLookup<QueryType, DataType>::buildIndex(
        map);
  }
  ~BasicMapElementsVIMapNearestNeighborLookup() = default;

 private:
  typedef VIMapNearestNeighborLookupBase<QueryType, DataType> BaseType;
  void buildIndexImpl(const vi_map::VIMap& map) override;
};

typedef BasicMapElementsVIMapNearestNeighborLookup<aslam::Position3D,
                                                   pose_graph::VertexId>
    VIMapNearestNeighborLookupVertexId;
typedef BasicMapElementsVIMapNearestNeighborLookup<aslam::Position3D,
                                                   vi_map::LandmarkId>
    VIMapNearestNeighborLookupLandmarkId;

template <class QueryType>
inline Eigen::VectorXd queryTypeToVector(const QueryType& query);

template <class QueryType>
inline size_t getQueryTypeDimension();

}  // namespace vi_map_helpers

#include "./vi-map-nearest-neighbor-lookup-inl.h"

#endif  // VI_MAP_HELPERS_VI_MAP_NEAREST_NEIGHBOR_LOOKUP_H_
