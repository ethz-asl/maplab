#ifndef BAVOXEL_HPP
#define BAVOXEL_HPP

#include "dense-reconstruction/balm/tools.h"

#include <Eigen/Eigenvalues>
#include <thread>

int layer_limit = 2;
int layer_size[] = {30, 30, 30, 30};
// float eigen_value_array[] = {1.0/4.0, 1.0/4.0, 1.0/4.0};
float eigen_value_array[4] = {1.0 / 16, 1.0 / 16, 1.0 / 16, 1.0 / 16};
int min_ps = 15;
double one_three = (1.0 / 3.0);

double voxel_size = 1;
int win_size = 20;

class VOX_HESS {
 public:
  vector<const PointCluster*> sig_vecs;
  vector<const vector<PointCluster>*> plvec_voxels;
  vector<double> coeffs, coeffs_back;

  vector<pcl::PointCloud<PointType>::Ptr> plptrs;

  void push_voxel(
      const vector<PointCluster>* vec_orig, const PointCluster* fix,
      double feat_eigen, int layer) {
    int process_size = 0;
    for (int i = 0; i < win_size; i++)
      if ((*vec_orig)[i].N != 0)
        process_size++;

    if (process_size < 2)
      return;  // 改

    double coe = 1 - feat_eigen / eigen_value_array[layer];
    coe = coe * coe;
    coe = 1;
    coe = 0;
    for (int j = 0; j < win_size; j++)
      coe += (*vec_orig)[j].N;

    plvec_voxels.push_back(vec_orig);
    sig_vecs.push_back(fix);
    coeffs.push_back(coe);
    pcl::PointCloud<PointType>::Ptr plptr(new pcl::PointCloud<PointType>());
    plptrs.push_back(plptr);
  }

  void acc_evaluate2(
      const vector<IMUST>& xs, int head, int end, Eigen::MatrixXd& Hess,
      Eigen::VectorXd& JacT, double& residual) {
    Hess.setZero();
    JacT.setZero();
    residual = 0;
    vector<PointCluster> sig_tran(win_size);
    const int kk = 0;

    PLV(3) viRiTuk(win_size);
    PLM(3) viRiTukukT(win_size);

    vector<
        Eigen::Matrix<double, 3, 6>,
        Eigen::aligned_allocator<Eigen::Matrix<double, 3, 6>>>
        Auk(win_size);
    Eigen::Matrix3d umumT;

    for (int a = head; a < end; a++) {
      const vector<PointCluster>& sig_orig = *plvec_voxels[a];
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
        // for(int i=1; i<win_size; i++)
        if (sig_orig[i].N != 0) {
          Eigen::Matrix3d Pi = sig_orig[i].P;
          Eigen::Vector3d vi = sig_orig[i].v;
          Eigen::Matrix3d Ri = xs[i].R;
          double ni = sig_orig[i].N;

          Eigen::Matrix3d vihat;
          vihat << SKEW_SYM_MATRX(vi);
          Eigen::Vector3d RiTuk = Ri.transpose() * uk;
          Eigen::Matrix3d RiTukhat;
          RiTukhat << SKEW_SYM_MATRX(RiTuk);

          Eigen::Vector3d PiRiTuk = Pi * RiTuk;
          viRiTuk[i] = vihat * RiTuk;
          viRiTukukT[i] = viRiTuk[i] * uk.transpose();

          Eigen::Vector3d ti_v = xs[i].p - vBar;
          double ukTti_v = uk.dot(ti_v);

          Eigen::Matrix3d combo1 = hat(PiRiTuk) + vihat * ukTti_v;
          Eigen::Vector3d combo2 = Ri * vi + ni * ti_v;
          Auk[i].block<3, 3>(0, 0) =
              (Ri * Pi + ti_v * vi.transpose()) * RiTukhat - Ri * combo1;
          Auk[i].block<3, 3>(0, 3) =
              combo2 * uk.transpose() + combo2.dot(uk) * I33;
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
        // for(int i=1; i<win_size-1; i++)
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

  void evaluate_only_residual(const vector<IMUST>& xs, double& residual) {
    residual = 0;
    vector<PointCluster> sig_tran(win_size);
    int kk = 0;  // The kk-th lambda value

    int gps_size = plvec_voxels.size();

    vector<double> ress(gps_size);

    for (int a = 0; a < gps_size; a++) {
      const vector<PointCluster>& sig_orig = *plvec_voxels[a];
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

  ~VOX_HESS() {
    int vsize = sig_vecs.size();
  }
};

class OCTO_TREE_NODE {
 public:
  int octo_state;  // 0(unknown), 1(mid node), 2(plane)
  int push_state;
  int layer;
  vector<PLV(3)> vec_orig, vec_tran;
  vector<PointCluster> sig_orig, sig_tran;
  PointCluster fix_point;

  OCTO_TREE_NODE* leaves[8];
  float voxel_center[3];
  float quater_length;

  double decision, ref;

  OCTO_TREE_NODE() {
    octo_state = 0;
    push_state = 0;
    layer = 0;

    vec_orig.resize(win_size);
    vec_tran.resize(win_size);

    sig_orig.resize(win_size);
    sig_tran.resize(win_size);

    for (int i = 0; i < 8; i++) {
      leaves[i] = nullptr;
    }

    ref = 255.0 * rand() / (RAND_MAX + 1.0f);
  }

  bool judge_eigen(int win_count) {
    PointCluster covMat = fix_point;
    for (int i = 0; i < win_count; i++) {
      covMat += sig_tran[i];
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat.cov());
    decision = saes.eigenvalues()[0] / saes.eigenvalues()[1];

    return (decision < eigen_value_array[layer]);
  }

  void cut_func(int ci) {
    PLV(3)& pvec_orig = vec_orig[ci];
    PLV(3)& pvec_tran = vec_tran[ci];

    uint a_size = pvec_tran.size();
    for (uint j = 0; j < a_size; j++) {
      int xyz[3] = {0, 0, 0};
      for (uint k = 0; k < 3; k++)
        if (pvec_tran[j][k] > voxel_center[k])
          xyz[k] = 1;
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves[leafnum] == nullptr) {
        leaves[leafnum] = new OCTO_TREE_NODE();
        leaves[leafnum]->voxel_center[0] =
            voxel_center[0] + (2 * xyz[0] - 1) * quater_length;
        leaves[leafnum]->voxel_center[1] =
            voxel_center[1] + (2 * xyz[1] - 1) * quater_length;
        leaves[leafnum]->voxel_center[2] =
            voxel_center[2] + (2 * xyz[2] - 1) * quater_length;
        leaves[leafnum]->quater_length = quater_length / 2;
        leaves[leafnum]->layer = layer + 1;
      }

      leaves[leafnum]->vec_orig[ci].push_back(pvec_orig[j]);
      leaves[leafnum]->vec_tran[ci].push_back(pvec_tran[j]);

      if (leaves[leafnum]->octo_state != 1) {
        leaves[leafnum]->sig_orig[ci].push(pvec_orig[j]);
        leaves[leafnum]->sig_tran[ci].push(pvec_tran[j]);
      }
    }

    PLV(3)().swap(pvec_orig);
    PLV(3)().swap(pvec_tran);
  }

  void recut(int win_count) {
    if (octo_state != 1) {
      int point_size = fix_point.N;
      for (int i = 0; i < win_count; i++)
        point_size += sig_orig[i].N;

      push_state = 0;
      if (point_size <= min_ps)
        return;

      if (judge_eigen(win_count)) {
        if (octo_state == 0 && point_size > layer_size[layer])
          octo_state = 2;

        point_size -= fix_point.N;
        if (point_size > min_ps)
          push_state = 1;
        return;
      } else if (layer == layer_limit) {
        octo_state = 2;
        return;
      }

      octo_state = 1;
      vector<PointCluster>().swap(sig_orig);
      vector<PointCluster>().swap(sig_tran);
      for (int i = 0; i < win_count; i++)
        cut_func(i);
    } else
      cut_func(win_count - 1);

    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        leaves[i]->recut(win_count);
  }

  ~OCTO_TREE_NODE() {
    for (int i = 0; i < 8; i++)
      if (leaves[i] != nullptr)
        delete leaves[i];
  }

  void tras_display(pcl::PointCloud<PointType>& pl_feat, int win_count) {
    if (octo_state != 1) {
      if (push_state != 1)
        return;

      PointType ap;
      ap.intensity = ref;

      int tsize = 0;
      for (int i = 0; i < win_count; i++)
        tsize += vec_tran[i].size();
      if (tsize < 100)
        return;

      for (int i = 0; i < win_count; i++)
        for (Eigen::Vector3d pvec : vec_tran[i]) {
          ap.x = pvec.x();
          ap.y = pvec.y();
          ap.z = pvec.z();
          pl_feat.push_back(ap);
        }

    } else {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr)
          leaves[i]->tras_display(pl_feat, win_count);
    }
  }

  void tras_opt(VOX_HESS& vox_opt, int win_count) {
    if (octo_state != 1) {
      int points_size = 0;
      for (int i = 0; i < win_count; i++)
        points_size += sig_orig[i].N;

      if (points_size < min_ps)
        return;

      if (push_state == 1)
        vox_opt.push_voxel(&sig_orig, &fix_point, decision, layer);
    } else {
      for (int i = 0; i < 8; i++)
        if (leaves[i] != nullptr)
          leaves[i]->tras_opt(vox_opt, win_count);
    }
  }
};

class BALM2 {
 public:
  BALM2() {}

  double divide_thread(
      vector<IMUST>& x_stats, VOX_HESS& voxhess, vector<IMUST>& x_ab,
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
    vector<double> resis(tthd_num, 0);
    int g_size = voxhess.plvec_voxels.size();
    if (g_size < tthd_num)
      tthd_num = 1;

    vector<thread*> mthreads(tthd_num);
    double part = 1.0 * g_size / tthd_num;
    for (int i = 0; i < tthd_num; i++)
      mthreads[i] = new thread(
          &VOX_HESS::acc_evaluate2, &voxhess, x_stats, part * i, part * (i + 1),
          ref(hessians[i]), ref(jacobins[i]), ref(resis[i]));

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
      vector<IMUST>& x_stats, VOX_HESS& voxhess, vector<IMUST>& x_ab) {
    double residual1 = 0, residual2 = 0;

    voxhess.evaluate_only_residual(x_stats, residual2);
    return (residual1 + residual2);
  }

  void damping_iter(vector<IMUST>& x_stats, VOX_HESS& voxhess) {
    vector<int> planes(x_stats.size(), 0);
    for (int i = 0; i < voxhess.plvec_voxels.size(); i++) {
      for (int j = 0; j < voxhess.plvec_voxels[i]->size(); j++)
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
    vector<IMUST> x_stats_temp = x_stats;

    vector<IMUST> x_ab(win_size);
    x_ab[0] = x_stats[0];
    for (int i = 1; i < win_size; i++) {
      x_ab[i].p =
          x_stats[i - 1].R.transpose() * (x_stats[i].p - x_stats[i - 1].p);
      x_ab[i].R = x_stats[i - 1].R.transpose() * x_stats[i].R;
    }

    for (int i = 0; i < 10; i++) {
      if (is_calc_hess)
        residual1 = divide_thread(x_stats, voxhess, x_ab, Hess, JacT);

      D.diagonal() = Hess.diagonal();
      dxi = (Hess + u * D).ldlt().solve(-JacT);

      for (int j = 0; j < win_size; j++) {
        x_stats_temp[j].R = x_stats[j].R * Exp(dxi.block<3, 1>(DVEL * j, 0));
        x_stats_temp[j].p = x_stats[j].p + dxi.block<3, 1>(DVEL * j + 3, 0);
        // x_stats_temp[j].p = x_stats[j].p + x_stats[j].R * dxi.block<3,
        // 1>(DVEL*j+3, 0);

        // Eigen::Matrix3d dR = Exp(dxi.block<3, 1>(DVEL*j, 0));
        // x_stats_temp[j].R = dR * x_stats[j].R;
        // x_stats_temp[j].p = dR * x_stats[j].p + dxi.block<3, 1>(DVEL*j+3, 0);
      }
      double q1 = 0.5 * dxi.dot(u * D * dxi - JacT);

      residual2 = only_residual(x_stats_temp, voxhess, x_ab);

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

    IMUST es0 = x_stats[0];
    for (uint i = 0; i < x_stats.size(); i++) {
      x_stats[i].p = es0.R.transpose() * (x_stats[i].p - es0.p);
      x_stats[i].R = es0.R.transpose() * x_stats[i].R;
    }
  }
};

void cut_voxel(
    unordered_map<VOXEL_LOC, OCTO_TREE_NODE*>& feat_map,
    pcl::PointCloud<PointType>& pl_feat, const IMUST& x_key, int fnum) {
  float loc_xyz[3];
  for (PointType& p_c : pl_feat.points) {
    Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
    Eigen::Vector3d pvec_tran = x_key.R * pvec_orig + x_key.p;

    for (int j = 0; j < 3; j++) {
      loc_xyz[j] = pvec_tran[j] / voxel_size;
      if (loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position(
        (int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if (iter != feat_map.end()) {
      if (iter->second->octo_state != 2) {
        iter->second->vec_orig[fnum].push_back(pvec_orig);
        iter->second->vec_tran[fnum].push_back(pvec_tran);
      }

      if (iter->second->octo_state != 1) {
        iter->second->sig_orig[fnum].push(pvec_orig);
        iter->second->sig_tran[fnum].push(pvec_tran);
      }
    } else {
      OCTO_TREE_NODE* ot = new OCTO_TREE_NODE();
      ot->vec_orig[fnum].push_back(pvec_orig);
      ot->vec_tran[fnum].push_back(pvec_tran);
      ot->sig_orig[fnum].push(pvec_orig);
      ot->sig_tran[fnum].push(pvec_tran);

      ot->voxel_center[0] = (0.5 + position.x) * voxel_size;
      ot->voxel_center[1] = (0.5 + position.y) * voxel_size;
      ot->voxel_center[2] = (0.5 + position.z) * voxel_size;
      ot->quater_length = voxel_size / 4.0;
      ot->layer = 0;
      
      feat_map[position] = ot;
    }
  }
}

#endif
