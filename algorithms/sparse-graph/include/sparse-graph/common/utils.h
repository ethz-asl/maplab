#ifndef SPARSE_GRAPH_DENSE_UTILS_H_
#define SPARSE_GRAPH_DENSE_UTILS_H_

#include <ros/ros.h>

namespace spg {

class Utils {
public:
  static ros::Time CreateRosTimestamp(const int64_t ts_ns);

};

} // namespace spg

#endif  // SPARSE_GRAPH_DENSE_UTILS_H_
