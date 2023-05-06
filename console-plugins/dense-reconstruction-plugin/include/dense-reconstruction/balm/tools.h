#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <unordered_map>

#define HASH_P 116101
#define MAX_N 10000000000
#define PLM(a)                     \
  vector<                          \
      Eigen::Matrix<double, a, a>, \
      Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a)                     \
  vector<                          \
      Eigen::Matrix<double, a, 1>, \
      Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

typedef pcl::PointXYZINormal PointType;
using namespace std;

class VOXEL_LOC {
 public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

namespace std {
template <>
struct hash<VOXEL_LOC> {
  size_t operator()(const VOXEL_LOC& s) const {
    return (((hash<int64_t>()(s.z) * HASH_P) % MAX_N + hash<int64_t>()(s.y)) *
            HASH_P) %
               MAX_N +
           hash<int64_t>()(s.x);
  }
};
}  // namespace std

Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d Omega;
  Omega << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return Omega;
}

void down_sampling_voxel(
    pcl::PointCloud<PointType>& pl_feat, double voxel_size) {
  if (voxel_size < 0.001)
    return;

  unordered_map<VOXEL_LOC, PointType> feat_map;
  float loc_xyz[3];
  for (PointType& p_c : pl_feat.points) {
    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position(
        (int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter == feat_map.end()) {
      PointType pp = p_c;
      pp.curvature = 1;
      feat_map[position] = pp;
    } else {
      PointType& pp = iter->second;
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1);
      pp.curvature += 1;
    }
  }

  // pl_feat.clear();
  pcl::PointCloud<PointType> pl_feat2;
  pl_feat.swap(pl_feat2);
  pl_feat.reserve(feat_map.size());
  for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter)
    pl_feat.push_back(iter->second);
}

void pl_transform(
    pcl::PointCloud<PointType>& pl1, const aslam::Transformation& T) {
  for (PointType& ap : pl1.points) {
    Eigen::Vector3d point(ap.x, ap.y, ap.z);
    point = T * point;
    ap.x = point[0];
    ap.y = point[1];
    ap.z = point[2];
  }
}

class PointCluster {
 public:
  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  PointCluster() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear() {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void push(const Eigen::Vector3d& vec) {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  Eigen::Matrix3d cov() {
    Eigen::Vector3d center = v / N;
    return P / N - center * center.transpose();
  }

  PointCluster& operator+=(const PointCluster& sigv) {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;

    return *this;
  }

  void transform(const PointCluster& sigv, const aslam::Transformation& T) {
    const Eigen::Matrix3d R = T.getRotationMatrix();
    const Eigen::Vector3d p = T.getPosition();
    const Eigen::Matrix3d rp = R * sigv.v * p.transpose();

    N = sigv.N;
    v = R * sigv.v + N * p;
    P = R * sigv.P * R.transpose() + rp + rp.transpose() +
        N * p * p.transpose();
  }
};

#endif
