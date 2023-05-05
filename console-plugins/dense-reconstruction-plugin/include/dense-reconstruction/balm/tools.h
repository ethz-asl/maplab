#ifndef TOOLS_HPP
#define TOOLS_HPP

#include <Eigen/Core>
#include <unordered_map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HASH_P 116101
#define MAX_N 10000000000
#define SKEW_SYM_MATRX(v) 0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0
#define PLM(a) vector<Eigen::Matrix<double, a, a>, Eigen::aligned_allocator<Eigen::Matrix<double, a, a>>>
#define PLV(a) vector<Eigen::Matrix<double, a, 1>, Eigen::aligned_allocator<Eigen::Matrix<double, a, 1>>>

#define G_m_s2 9.81
#define DIMU 18
#define DIM 15
#define DNOI 12
#define NMATCH 5
#define DVEL 6

typedef pcl::PointXYZINormal PointType;
using namespace std;

Eigen::Matrix3d I33(Eigen::Matrix3d::Identity());
Eigen::Matrix<double, DIMU, DIMU> I_imu(Eigen::Matrix<double, DIMU, DIMU>::Identity());
Eigen::Matrix<double, 12, 12> I12(Eigen::Matrix<double, 12, 12>::Identity());

class VOXEL_LOC
{
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx=0, int64_t vy=0, int64_t vz=0): x(vx), y(vy), z(vz){}

  bool operator == (const VOXEL_LOC &other) const
  {
    return (x==other.x && y==other.y && z==other.z);
  }
};

namespace std
{
  template<>
  struct hash<VOXEL_LOC>
  {
    size_t operator() (const VOXEL_LOC &s) const
    {
      using std::size_t; using std::hash;
      // return ((hash<int64_t>()(s.x) ^ (hash<int64_t>()(s.y) << 1)) >> 1) ^ (hash<int64_t>()(s.z) << 1);
      return (((hash<int64_t>()(s.z)*HASH_P)%MAX_N + hash<int64_t>()(s.y))*HASH_P)%MAX_N + hash<int64_t>()(s.x);
    }
  };
}

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang)
{
  double ang_norm = ang.norm();
  // if (ang_norm >= 0.0000001)
  if (ang_norm >= 1e-11)
  {
    Eigen::Vector3d r_axis = ang / ang_norm;
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  
  return I33;
  
}

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt)
{
  double ang_vel_norm = ang_vel.norm();
  if (ang_vel_norm > 0.0000001)
  {
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << SKEW_SYM_MATRX(r_axis);
    double r_ang = ang_vel_norm * dt;

    /// Roderigous Tranformation
    return I33 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }
  
  return I33;
  
}

Eigen::Vector3d Log(const Eigen::Matrix3d &R)
{
  double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Vector3d K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

Eigen::Matrix3d jr(Eigen::Vector3d vec)
{
  double ang = vec.norm();

  if(ang < 1e-9)
  {
    return I33;
  }
  else
  {
    vec /= ang;
    double ra = sin(ang)/ang;
    return ra*I33 + (1-ra)*vec*vec.transpose() - (1-cos(ang))/ang * hat(vec);
  }
}

Eigen::Matrix3d jr_inv(const Eigen::Matrix3d &rotR)
{
  Eigen::AngleAxisd rot_vec(rotR);
  Eigen::Vector3d axi = rot_vec.axis();
  double ang = rot_vec.angle();

  if(ang < 1e-9)
  {
    return I33;
  }
  else
  {
    double ctt = ang / 2 / tan(ang/2);
    return ctt*I33 + (1-ctt)*axi*axi.transpose() + ang/2 * hat(axi);
  }
}

struct IMUST
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  double t;
  Eigen::Matrix3d R;
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d bg;
  Eigen::Vector3d ba;
  Eigen::Vector3d g;
  
  IMUST()
  {
    setZero();
  }

  IMUST(double _t, const Eigen::Matrix3d &_R, const Eigen::Vector3d &_p, const Eigen::Vector3d &_v, const Eigen::Vector3d &_bg, const Eigen::Vector3d &_ba, const Eigen::Vector3d &_g = Eigen::Vector3d(0, 0, -G_m_s2)) : t(_t), R(_R), p(_p), v(_v), bg(_bg), ba(_ba), g(_g) {}

  IMUST &operator+=(const Eigen::Matrix<double, DIMU, 1> &ist)
  {
    this->R = this->R * Exp(ist.block<3, 1>(0, 0));
    this->p += ist.block<3, 1>(3, 0);
    this->v += ist.block<3, 1>(6, 0);
    this->bg += ist.block<3, 1>(9, 0);
    this->ba += ist.block<3, 1>(12, 0);
    this->g += ist.block<3, 1>(15, 0);
    return *this;
  }

  Eigen::Matrix<double, DIMU, 1> operator-(const IMUST &b) 
  {
    Eigen::Matrix<double, DIMU, 1> a;
    a.block<3, 1>(0, 0) = Log(b.R.transpose() * this->R);
    a.block<3, 1>(3, 0) = this->p - b.p;
    a.block<3, 1>(6, 0) = this->v - b.v;
    a.block<3, 1>(9, 0) = this->bg - b.bg;
    a.block<3, 1>(12, 0) = this->ba - b.ba;
    a.block<3, 1>(15, 0) = this->g - b.g;
    return a;
  }

  IMUST &operator=(const IMUST &b)
  {
    this->R = b.R;
    this->p = b.p;
    this->v = b.v;
    this->bg = b.bg;
    this->ba = b.ba;
    this->g = b.g;
    this->t = b.t;
    return *this;
  }

  void setZero()
  {
    t = 0; R.setIdentity();
    p.setZero(); v.setZero();
    bg.setZero(); ba.setZero();
    g << 0, 0, -G_m_s2;
  }

};

void down_sampling_voxel(pcl::PointCloud<PointType> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.001) return;

  unordered_map<VOXEL_LOC, PointType> feat_map;
  float loc_xyz[3];
  for(PointType &p_c : pl_feat.points)
  {
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())
    {
      PointType pp = p_c;
      pp.curvature = 1;
      feat_map[position] = pp;
    }
    else
    {
      PointType &pp = iter->second;
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1);
      pp.curvature += 1;
    }
  }

  // pl_feat.clear();
  pcl::PointCloud<PointType> pl_feat2;
  pl_feat.swap(pl_feat2); pl_feat.reserve(feat_map.size());
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
    pl_feat.push_back(iter->second);
  
}

void down_sampling_serie(pcl::PointCloud<PointType> &pl_feat, int num)
{
  if(num < 1) num = 1;

  pcl::PointCloud<PointType> pl_down;
  int psize = pl_feat.size();
  pl_down.reserve(psize);
  for(int i=0; i<psize; i+=num)
    pl_down.push_back(pl_feat[i]);
  pl_feat.swap(pl_down);
}

void pl_transform(pcl::PointCloud<PointType> &pl1, const Eigen::Matrix3d &rr, const Eigen::Vector3d &tt)
{
  for(PointType &ap : pl1.points)
  {
    Eigen::Vector3d pvec(ap.x, ap.y, ap.z);
    pvec = rr * pvec + tt;
    ap.x = pvec[0];
    ap.y = pvec[1];
    ap.z = pvec[2];
  }
}

void pl_transform(pcl::PointCloud<PointType> &pl1, const IMUST &xx)
{
  for(PointType &ap : pl1.points)
  {
    Eigen::Vector3d pvec(ap.x, ap.y, ap.z);
    pvec = xx.R * pvec + xx.p;
    ap.x = pvec[0];
    ap.y = pvec[1];
    ap.z = pvec[2];
  }
}

void plvec_trans(PLV(3) &porig, PLV(3) &ptran, IMUST &stat)
{
  uint asize = porig.size();
  ptran.resize(asize);
  for(uint i=0; i<asize; i++)
    ptran[i] = stat.R * porig[i] + stat.p;
}

bool time_compare(PointType &x, PointType &y) {return (x.curvature < y.curvature);}

class PointCluster
{
public:
  Eigen::Matrix3d P;
  Eigen::Vector3d v;
  int N;

  PointCluster()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void clear()
  {
    P.setZero();
    v.setZero();
    N = 0;
  }

  void push(const Eigen::Vector3d &vec)
  {
    N++;
    P += vec * vec.transpose();
    v += vec;
  }

  Eigen::Matrix3d cov()
  {
    Eigen::Vector3d center = v / N;
    return P/N - center*center.transpose();
  }

  PointCluster & operator+=(const PointCluster &sigv)
  {
    this->P += sigv.P;
    this->v += sigv.v;
    this->N += sigv.N;

    return *this;
  }

  void transform(const PointCluster &sigv, const IMUST &stat)
  {
    N = sigv.N;
    v = stat.R*sigv.v + N*stat.p;
    Eigen::Matrix3d rp = stat.R * sigv.v * stat.p.transpose();
    P = stat.R*sigv.P*stat.R.transpose() + rp + rp.transpose() + N*stat.p*stat.p.transpose();
  }

};


/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/

const double threshold = 0.1;
bool esti_plane(Eigen::Vector4d &pca_result, const pcl::PointCloud<PointType> &point)
{
  Eigen::Matrix<double, NMATCH, 3> A;
  Eigen::Matrix<double, NMATCH, 1> b;
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NMATCH; j++) 
  {
    A(j, 0) = point[j].x;
    A(j, 1) = point[j].y;
    A(j, 2) = point[j].z;
  }

  Eigen::Vector3d normvec = A.colPivHouseholderQr().solve(b);

  for (int j = 0; j < NMATCH; j++) 
  {
    if (fabs(normvec.dot(A.row(j)) + 1.0) > threshold) 
      return false;
  }

  double n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;
  return true;
}

#endif
