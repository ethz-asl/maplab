#ifndef BALM_H_
#define BALM_H_

#include <Eigen/Core>
#include <aslam/common/pose-types.h>
#include <kindr/minimal/common.h>
#include <resources-common/point-cloud.h>

#include <thread>
#include <vector>

#define PLM(a)                     \
  std::vector<                     \
      Eigen::Matrix<double, a, a>, \
      Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a)                     \
  std::vector<                     \
      Eigen::Matrix<double, a, 1>, \
      Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

size_t layer_limit = 2;
float eigen_value_array[4] = {1.0 / 16, 1.0 / 16, 1.0 / 16, 1.0 / 16};
size_t min_ps = 15;
double one_three = (1.0 / 3.0);

double voxel_size = 1;
int win_size = 20;

class OctoTreeNode;
typedef std::unordered_map<resources::VoxelPosition, OctoTreeNode*> SurfaceMap;

class PointCluster {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  PointCluster() {
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

Eigen::Matrix3d hat(const Eigen::Vector3d& v) {
  Eigen::Matrix3d Omega;
  Omega << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return Omega;
}

class VoxHess {
 public:
  std::vector<const PointCluster*> sig_vecs;
  std::vector<const std::vector<PointCluster>*> plvec_voxels;
  std::vector<double> coeffs;

  void push_voxel(
      const std::vector<PointCluster>* vec_orig, const PointCluster* fix) {
    int process_size = 0;
    for (int i = 0; i < win_size; i++) {
      if ((*vec_orig)[i].N != 0) {
        process_size++;
      }
    }

    if (process_size < 2) {
      return;
    }

    double coe = 0;
    for (int i = 0; i < win_size; i++) {
      coe += (*vec_orig)[i].N;
    }

    plvec_voxels.push_back(vec_orig);
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
  }

  void acc_evaluate2(
      const aslam::TransformationVector& xs, int head, int end,
      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT, double& residual) {
    Hess.setZero();
    JacT.setZero();
    residual = 0;
    std::vector<PointCluster> sig_tran(win_size);
    const int kk = 0;

    PLV(3) viRiTuk(win_size);
    PLM(3) viRiTukukT(win_size);

    std::vector<
        Eigen::Matrix<double, 3, 6>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>>
        Auk(win_size);
    Eigen::Matrix3d umumT;

    for (int a = head; a < end; a++) {
      const std::vector<PointCluster>& sig_orig = *plvec_voxels[a];
      double coe = coeffs[a];

      PointCluster sig = *sig_vecs[a];
      for (int i = 0; i < win_size; i++)
        if (sig_orig[i].N != 0) {
          sig_tran[i].transform(sig_orig[i], xs[i]);
          sig += sig_tran[i];
        }

      const Eigen::Vector3d& vBar = sig.v / sig.N;
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(
          sig.P / sig.N - vBar * vBar.transpose());
      const Eigen::Vector3d& lmbd = saes.eigenvalues();
      const Eigen::Matrix3d& U = saes.eigenvectors();
      int NN = sig.N;

      Eigen::Vector3d u[3] = {U.col(0), U.col(1), U.col(2)};

      const Eigen::Vector3d& uk = u[kk];
      Eigen::Matrix3d ukukT = uk * uk.transpose();
      umumT.setZero();
      for (int i = 0; i < 3; i++)
        if (i != kk)
          umumT += 2.0 / (lmbd[kk] - lmbd[i]) * u[i] * u[i].transpose();

      for (int i = 0; i < win_size; i++)
        if (sig_orig[i].N != 0) {
          Eigen::Matrix3d Pi = sig_orig[i].P;
          Eigen::Vector3d vi = sig_orig[i].v;
          Eigen::Matrix3d Ri = xs[i].getRotationMatrix();
          double ni = sig_orig[i].N;

          Eigen::Matrix3d vihat = kindr::minimal::skewMatrix(vi);
          Eigen::Vector3d RiTuk = Ri.transpose() * uk;
          Eigen::Matrix3d RiTukhat = kindr::minimal::skewMatrix(RiTuk);

          Eigen::Vector3d PiRiTuk = Pi * RiTuk;
          viRiTuk[i] = vihat * RiTuk;
          viRiTukukT[i] = viRiTuk[i] * uk.transpose();

          Eigen::Vector3d ti_v = xs[i].getPosition() - vBar;
          double ukTti_v = uk.dot(ti_v);

          Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
          Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
          Auk[i].block<3, 3>(0, 0) =
              (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
          Auk[i].block<3, 3>(0, 3) =
              combo2 * uk.transpose() +
              combo2.dot(uk) * Eigen::Matrix3d::Identity();
          Auk[i] /= NN;

          const Eigen::Matrix<double, 6, 1>& jjt = Auk[i].transpose() * uk;
          JacT.block<6, 1>(6 * i, 0) += coe * jjt;

          const Eigen::Matrix3d& HRt =
              2.0 / NN * (1.0 - ni / NN) * viRiTukukT[i];
          Eigen::Matrix<double, 6, 6> Hb = Auk[i].transpose() * umumT * Auk[i];
          Hb.block<3, 3>(0, 0) +=
              2.0 / NN * (combo1 - RiTukhat * Pi) * RiTukhat -
              2.0 / NN / NN * viRiTuk[i] * viRiTuk[i].transpose() -
              0.5 * hat(jjt.block<3, 1>(0, 0));
          Hb.block<3, 3>(0, 3) += HRt;
          Hb.block<3, 3>(3, 0) += HRt.transpose();
          Hb.block<3, 3>(3, 3) += 2.0 / NN * (ni - ni * ni / NN) * ukukT;

          Hess.block<6, 6>(6 * i, 6 * i) += coe * Hb;
        }

      for (int i = 0; i < win_size - 1; i++)
        if (sig_orig[i].N != 0) {
          double ni = sig_orig[i].N;
          for (int j = i + 1; j < win_size; j++)
            if (sig_orig[j].N != 0) {
              double nj = sig_orig[j].N;
              Eigen::Matrix<double, 6, 6> Hb =
                  Auk[i].transpose() * umumT * Auk[j];
              Hb.block<3, 3>(0, 0) +=
                  -2.0 / NN / NN * viRiTuk[i] * viRiTuk[j].transpose();
              Hb.block<3, 3>(0, 3) += -2.0 * nj / NN / NN * viRiTukukT[i];
              Hb.block<3, 3>(3, 0) +=
                  -2.0 * ni / NN / NN * viRiTukukT[j].transpose();
              Hb.block<3, 3>(3, 3) += -2.0 * ni * nj / NN / NN * ukukT;

              Hess.block<6, 6>(6 * i, 6 * j) += coe * Hb;
            }
        }

      residual += coe * lmbd[kk];
    }

    for (int i = 1; i < win_size; i++)
      for (int j = 0; j < i; j++)
        Hess.block<6, 6>(6 * i, 6 * j) =
            Hess.block<6, 6>(6 * j, 6 * i).transpose();
  }

  void evaluate_only_residual(
      const aslam::TransformationVector& xs, double& residual) {
    residual = 0;
    std::vector<PointCluster> sig_tran(win_size);
    int kk = 0;  // The kk-th lambda value

    int gps_size = plvec_voxels.size();

    std::vector<double> ress(gps_size);

    for (int a = 0; a < gps_size; a++) {
      const std::vector<PointCluster>& sig_orig = *plvec_voxels[a];
      PointCluster sig = *sig_vecs[a];

      for (int i = 0; i < win_size; i++) {
        sig_tran[i].transform(sig_orig[i], xs[i]);
        sig += sig_tran[i];
      }

      Eigen::Vector3d vBar = sig.v / sig.N;
      Eigen::Matrix3d cmt = sig.P / sig.N - vBar * vBar.transpose();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(cmt);
      Eigen::Vector3d lmbd = saes.eigenvalues();

      residual += coeffs[a] * lmbd[kk];

      ress[a] = lmbd[kk];
    }
  }
};

class OctoTreeNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  int octo_state;  // 0(unknown), 1(node), 2(leaf)
  size_t layer;
  std::vector<PLV(3)> vec_orig, vec_tran;
  std::vector<PointCluster> sig_orig, sig_tran;
  PointCluster fix_point;

  OctoTreeNode* leaves[8];
  float voxel_center[3];
  float quater_length;

  OctoTreeNode(size_t num_scans) {
    octo_state = 0;
    layer = 0;

    vec_orig.resize(num_scans);
    vec_tran.resize(num_scans);

    sig_orig.resize(num_scans);
    sig_tran.resize(num_scans);

    for (size_t i = 0; i < 8; ++i) {
      leaves[i] = nullptr;
    }
  }

  bool judge_eigen() {
    PointCluster covMat = fix_point;
    for (const PointCluster& p : sig_tran) {
      covMat += p;
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat.cov());
    double decision = saes.eigenvalues()[0] / saes.eigenvalues()[1];

    return (decision < eigen_value_array[layer]);
  }

  void cut_func(size_t ci) {
    PLV(3)& pvec_orig = vec_orig[ci];
    PLV(3)& pvec_tran = vec_tran[ci];

    const size_t num_scans = vec_orig.size();
    for (size_t j = 0; j < pvec_tran.size(); ++j) {
      const int x = pvec_tran[j][0] > voxel_center[0];
      const int y = pvec_tran[j][1] > voxel_center[1];
      const int z = pvec_tran[j][2] > voxel_center[2];
      const int leafnum = (x << 2) + (y << 1) + z;

      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] = new OctoTreeNode(num_scans);
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * x - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * y - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * z - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
        leaves[leafnum]->layer = layer + 1;
      }

      leaves[leafnum]->vec_orig[ci].push_back(pvec_orig[j]);
      leaves[leafnum]->vec_tran[ci].push_back(pvec_tran[j]);
      leaves[leafnum]->sig_orig[ci].push(pvec_orig[j]);
      leaves[leafnum]->sig_tran[ci].push(pvec_tran[j]);
    }

    PLV(3)().swap(pvec_orig);
    PLV(3)().swap(pvec_tran);
  }

  bool recut() {
    size_t point_size = 0;
    for (const PointCluster& p : sig_orig) {
      point_size += p.N;
    }

    if (point_size <= min_ps) {
      return false;
    }

    if (judge_eigen()) {
      octo_state = 2;
      return true;
    } else if (layer == layer_limit) {
      return false;
    }

    octo_state = 1;
    std::vector<PointCluster>().swap(sig_orig);
    std::vector<PointCluster>().swap(sig_tran);
    for (size_t i = 0; i < vec_orig.size(); ++i) {
      cut_func(i);
    }

    bool keep = false;
    for (size_t i = 0; i < 8; ++i) {
      if (leaves[i] != nullptr) {
        if (leaves[i]->recut()) {
          keep = true;
        } else {
          delete leaves[i];
          leaves[i] = nullptr;
        }
      }
    }

    return keep;
  }

  ~OctoTreeNode() {
    for (size_t i = 0; i < 8; ++i) {
      if (leaves[i] != nullptr) {
        delete leaves[i];
      }
    }
  }

  void tras_display(resources::PointCloud* points_G) {
    if (octo_state != 1) {
      const float intensity = 255.0 * rand() / (RAND_MAX + 1.0f);

      for (size_t i = 0; i < vec_tran.size(); ++i) {
        for (Eigen::Vector3d point : vec_tran[i]) {
          points_G->xyz.emplace_back(point.x());
          points_G->xyz.emplace_back(point.y());
          points_G->xyz.emplace_back(point.z());
          points_G->scalars.emplace_back(intensity);
        }
      }
    } else {
      for (size_t i = 0; i < 8; ++i) {
        if (leaves[i] != nullptr) {
          leaves[i]->tras_display(points_G);
        }
      }
    }
  }

  void tras_opt(VoxHess& vox_opt) {
    if (octo_state != 1) {
      vox_opt.push_voxel(&sig_orig, &fix_point);
    } else {
      for (size_t i = 0; i < 8; ++i) {
        if (leaves[i] != nullptr) {
          leaves[i]->tras_opt(vox_opt);
        }
      }
    }
  }
};

class BALM2 {
 public:
  BALM2() {}

  double divide_thread(
      const aslam::TransformationVector& x_stats, VoxHess& voxhess,
      Eigen::MatrixXd& Hess, Eigen::VectorXd& JacT) {
    int thd_num = 4;
    double residual = 0;
    Hess.setZero();
    JacT.setZero();
    PLM(-1) hessians(thd_num);
    PLV(-1) jacobins(thd_num);

    for (int i = 0; i < thd_num; i++) {
      hessians[i].resize(6 * win_size, 6 * win_size);
      jacobins[i].resize(6 * win_size);
    }

    int tthd_num = thd_num;
    std::vector<double> resis(tthd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < tthd_num)
      tthd_num = 1;

    std::vector<std::thread*> mthreads(tthd_num);
    double part = 1.0 * g_size / tthd_num;
    for (int i = 0; i < tthd_num; i++)
      mthreads[i] = new std::thread(
          &VoxHess::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1),
          std::ref(hessians[i]), std::ref(jacobins[i]), std::ref(resis[i]));

    for (int i = 0; i < tthd_num; i++) {
      mthreads[i]->join();
      Hess += hessians[i];
      JacT += jacobins[i];
      residual += resis[i];
      delete mthreads[i];
    }

    return residual;
  }

  double only_residual(
      const aslam::TransformationVector& x_stats, VoxHess& voxhess) {
    double residual = 0;

    voxhess.evaluate_only_residual(x_stats, residual);
    return residual;
  }

  void damping_iter(aslam::TransformationVector& x_stats, VoxHess& voxhess) {
    std::vector<int> planes(x_stats.size(), 0);
    for (size_t i = 0; i < voxhess.plvec_voxels.size(); i++) {
      for (size_t j = 0; j < voxhess.plvec_voxels[i]->size(); j++)
        if (voxhess.plvec_voxels[i]->at(j).N != 0)
          planes[j]++;
    }
    sort(planes.begin(), planes.end());
    if (planes[0] < 20) {
      printf("Initial error too large.\n");
      printf("Please loose plane determination criteria for more planes.\n");
      printf("The optimization is terminated.\n");
      exit(0);
    }

    double u = 0.01, v = 2;
    Eigen::MatrixXd D(6 * win_size, 6 * win_size),
        Hess(6 * win_size, 6 * win_size);
    Eigen::VectorXd JacT(6 * win_size), dxi(6 * win_size);

    D.setIdentity();
    double residual1, residual2, q;
    bool is_calc_hess = true;
    aslam::TransformationVector x_stats_temp = x_stats;

    for (int i = 0; i < 10; i++) {
      if (is_calc_hess) {
        residual1 = divide_thread(x_stats, voxhess, Hess, JacT);
      }

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u * D).ldlt().solve(-JacT);

      for (int j = 0; j < win_size; j++) {
        x_stats_temp[j].getRotation() =
            x_stats[j].getRotation() *
            aslam::Quaternion::exp(dxi.block<3, 1>(6 * j, 0));
        x_stats_temp[j].getPosition() =
            x_stats[j].getPosition() + dxi.block<3, 1>(6 * j + 3, 0);
      }
      double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

      residual2 = only_residual(x_stats_temp, voxhess);

      q = (residual1 - residual2);
      printf(
          "iter%d: (%lf %lf) u: %lf v: %.1lf q: %.3lf %lf %lf\n", i, residual1,
          residual2, u, v, q / q1, q1, q);

      if (q > 0) {
        x_stats = x_stats_temp;

        q = q / q1;
        v = 2;
        q = 1 - pow(2 * q - 1, 3);
        u *= (q < one_three ? one_three : q);
        is_calc_hess = true;
      } else {
        u = u * v;
        v = 2 * v;
        is_calc_hess = false;
      }

      if (fabs(residual1 - residual2) / residual1 < 1e-6)
        break;
    }

    aslam::Transformation es0 = x_stats[0].inverse();
    for (uint i = 0; i < x_stats.size(); i++) {
      x_stats[i] = es0 * x_stats[i];
    }
  }
};

void cut_voxel(
    SurfaceMap& surface_map, const resources::PointCloud& points_S,
    const aslam::Transformation& T_G_S, size_t index, size_t num_scans) {
  resources::PointCloud points_G;
  points_G.appendTransformed(points_S, T_G_S);

  Eigen::Map<const Eigen::Matrix3Xf> xyz_S(
      points_S.xyz.data(), 3, points_S.size());
  Eigen::Map<const Eigen::Matrix3Xf> xyz_G(
      points_G.xyz.data(), 3, points_G.size());

  for (size_t i = 0; i < points_S.size(); ++i) {
    const Eigen::Vector3d pvec_orig = xyz_S.col(i).cast<double>();
    const Eigen::Vector3d pvec_tran = xyz_G.col(i).cast<double>();

    resources::VoxelPosition position(
        pvec_tran[0], pvec_tran[1], pvec_tran[2], voxel_size);
    auto iter = surface_map.find(position);
    if (iter != surface_map.end()) {
      if (iter->second->octo_state != 2) {
        iter->second->vec_orig[index].push_back(pvec_orig);
        iter->second->vec_tran[index].push_back(pvec_tran);
      }

      if (iter->second->octo_state != 1) {
        iter->second->sig_orig[index].push(pvec_orig);
        iter->second->sig_tran[index].push(pvec_tran);
      }
    } else {
      OctoTreeNode* node = new OctoTreeNode(num_scans);
      node->vec_orig[index].push_back(pvec_orig);
      node->vec_tran[index].push_back(pvec_tran);
      node->sig_orig[index].push(pvec_orig);
      node->sig_tran[index].push(pvec_tran);

      node->voxel_center[0] = (0.5 + position.x) * voxel_size;
      node->voxel_center[1] = (0.5 + position.y) * voxel_size;
      node->voxel_center[2] = (0.5 + position.z) * voxel_size;
      node->quater_length = voxel_size / 4.0;
      node->layer = 0;

      surface_map[position] = node;
    }
  }
}

#endif  // BALM_H_
