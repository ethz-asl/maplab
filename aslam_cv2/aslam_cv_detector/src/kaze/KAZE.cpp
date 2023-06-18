//=============================================================================
//
// KAZE.cpp
// Authors: Pablo F. Alcantarilla
// Date: 11/12/2014
// Email: pablofdezalc@gmail.com
//
// KAZE Features Copyright 2014, Pablo F. Alcantarilla
// All Rights Reserved
// See LICENSE for the license information
//=============================================================================

/**
 * @file KAZE.cpp
 * @brief Main class for detecting and describing features in a nonlinear
 * scale space
 * @date Dec 11, 2014
 * @author Pablo F. Alcantarilla
 */

#include "kaze/KAZE.h"

using namespace std;
using namespace libKAZE;

/* ************************************************************************* */
KAZE::KAZE(KAZEOptions& options) : options_(options) {

  ncycles_ = 0;
  reordering_ = true;
  Allocate_Memory_Evolution();
}

/* ************************************************************************* */
KAZE::~KAZE(void) {

  evolution_.clear();
}

/* ************************************************************************* */
/**
 * @brief This method allocates the memory for the nonlinear diffusion evolution
*/
void KAZE::Allocate_Memory_Evolution() {

  cv::Size size(options_.img_width, options_.img_height);

  // Allocate the dimension of the matrices for the evolution
  for (int i = 0; i <= options_.omax-1; i++) {
    for (int j = 0; j <= options_.nsublevels-1; j++) {

      TEvolution step;
      step.Lx.create(size, CV_32F);
      step.Ly.create(size, CV_32F);
      step.Lxx.create(size, CV_32F);
      step.Lxy.create(size, CV_32F);
      step.Lyy.create(size, CV_32F);
      step.Lt.create(size, CV_32F);
      step.Lsmooth.create(size, CV_32F);
      step.Ldet.create(size, CV_32F);
      step.esigma = options_.soffset*pow((float)2.0,(float)(j)/(float)(options_.nsublevels) + i);
      step.etime = 0.5*(step.esigma*step.esigma);
      step.sigma_size = fRound(step.esigma);
      step.octave = i;
      step.sublevel = j;
      evolution_.push_back(step);
    }
  }

  // Allocate memory for the FED number of cycles and time steps
  if (options_.use_fed) {
    for (size_t i = 1; i < evolution_.size(); i++) {
      int naux = 0;
      vector<float> tau;
      float ttime = 0.0;
      ttime = evolution_[i].etime-evolution_[i-1].etime;
      naux = fed_tau_by_process_time(ttime, 1, 0.25, reordering_, tau);
      nsteps_.push_back(naux);
      tsteps_.push_back(tau);
      ncycles_++;
    }
  }
  // Allocate memory for the auxiliary variables that are used in the AOS scheme
  else {
    Ltx_.create(size, CV_32F);
    Lty_.create(size, CV_32F);
    px_.create(size, CV_32F);
    py_.create(size, CV_32F);
    ax_.create(size, CV_32F);
    ay_.create(size, CV_32F);
    bx_ = cv::Mat::zeros(options_.img_height-1, options_.img_width, CV_32F);
    by_ = cv::Mat::zeros(options_.img_height-1, options_.img_width, CV_32F);
    qr_ = cv::Mat::zeros(options_.img_height-1, options_.img_width, CV_32F);
    qc_ = cv::Mat::zeros(options_.img_height, options_.img_width-1, CV_32F);
  }
}

/* ************************************************************************* */
int KAZE::Create_Nonlinear_Scale_Space(const cv::Mat& img) {

  double t2 = 0.0, t1 = 0.0;

  if (evolution_.size() == 0) {
    cout << "Error generating the nonlinear scale space!!" << endl;
    cout << "Firstly you need to call KAZE::Allocate_Memory_Evolution()" << endl;
    return -1;
  }

  t1 = cv::getTickCount();

  // Copy the original image to the first level of the evolution
  img.copyTo(evolution_[0].Lt);
  gaussian_2D_convolution(evolution_[0].Lt, evolution_[0].Lt, 0, 0, options_.soffset);
  gaussian_2D_convolution(evolution_[0].Lt, evolution_[0].Lsmooth, 0, 0, options_.sderivatives);

  // Allocate memory for the flow and step images
  cv::Mat Lflow = cv::Mat::zeros(evolution_[0].Lt.rows, evolution_[0].Lt.cols, CV_32F);
  cv::Mat Lstep = cv::Mat::zeros(evolution_[0].Lt.rows, evolution_[0].Lt.cols, CV_32F);

  // Firstly compute the kcontrast factor
  Compute_KContrast(img);

  t2 = cv::getTickCount();
  timing_.kcontrast = 1000.0*(t2-t1) / cv::getTickFrequency();

  if (options_.verbosity == true) {
    cout << "Computed image evolution step. Evolution time: " << evolution_[0].etime <<
            " Sigma: " << evolution_[0].esigma << endl;
  }

  // Now generate the rest of evolution levels
  for (size_t i = 1; i < evolution_.size(); i++) {

    evolution_[i-1].Lt.copyTo(evolution_[i].Lt);
    gaussian_2D_convolution(evolution_[i-1].Lt, evolution_[i].Lsmooth, 0, 0, options_.sderivatives);

    // Compute the Gaussian derivatives Lx and Ly
    cv::Scharr(evolution_[i].Lsmooth, evolution_[i].Lx, CV_32F, 1, 0, 1, 0, cv::BORDER_DEFAULT);
    cv::Scharr(evolution_[i].Lsmooth, evolution_[i].Ly, CV_32F, 0, 1, 1, 0, cv::BORDER_DEFAULT);

    // Compute the conductivity equation
    switch (options_.diffusivity) {
      case PM_G1:
        pm_g1(evolution_[i].Lx, evolution_[i].Ly, Lflow, options_.kcontrast);
      break;
      case PM_G2:
        pm_g2(evolution_[i].Lx, evolution_[i].Ly, Lflow, options_.kcontrast);
      break;
      case WEICKERT:
        weickert_diffusivity(evolution_[i].Lx, evolution_[i].Ly, Lflow, options_.kcontrast);
      break;
      case CHARBONNIER:
        charbonnier_diffusivity(evolution_[i].Lx, evolution_[i].Ly, Lflow, options_.kcontrast);
      break;
      default:
        cerr << "Diffusivity: " << options_.diffusivity << " is not supported" << endl;
    }

    // Perform FED n inner steps
    if (options_.use_fed) {
      for (int j = 0; j < nsteps_[i-1]; j++)
        nld_step_scalar(evolution_[i].Lt, Lflow, Lstep, tsteps_[i-1][j]);
    }
    // Perform the evolution step with AOS
    else
      AOS_Step_Scalar(evolution_[i].Lt, evolution_[i-1].Lt, Lflow, evolution_[i].etime-evolution_[i-1].etime);

    if (options_.verbosity == true) {
      cout << "Computed image evolution step " << i << " Evolution time: " << evolution_[i].etime <<
              " Sigma: " << evolution_[i].esigma << endl;
    }
  }

  t2 = cv::getTickCount();
  timing_.scale = 1000.0*(t2-t1) / cv::getTickFrequency();

  return 0;
}

/* ************************************************************************* */
void KAZE::Compute_KContrast(const cv::Mat& img) {

  if (options_.verbosity == true) {
    cout << "Computing Kcontrast factor." << endl;
  }

  options_.kcontrast = compute_k_percentile(img,options_.kcontrast_percentile,
                                            options_.sderivatives,options_.kcontrast_nbins,0,0);

  if (options_.verbosity == true) {
    cout << "kcontrast = " << options_.kcontrast << endl;
    cout << endl << "Now computing the nonlinear scale space!!" << endl;
  }
}

/* ************************************************************************* */
void KAZE::Compute_Multiscale_Derivatives() {

  double t2 = 0.0, t1 = 0.0;
  t1 = cv::getTickCount();

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < (int)(evolution_.size()); i++) {

    if (options_.verbosity == true) {
      cout << "Computing multiscale derivatives. Evolution time: " << evolution_[i].etime
           << " Step (pixels): " << evolution_[i].sigma_size << endl;
    }

    // Compute multiscale derivatives for the detector
    compute_scharr_derivatives(evolution_[i].Lsmooth, evolution_[i].Lx, 1,0, evolution_[i].sigma_size);
    compute_scharr_derivatives(evolution_[i].Lsmooth, evolution_[i].Ly, 0, 1, evolution_[i].sigma_size);
    compute_scharr_derivatives(evolution_[i].Lx, evolution_[i].Lxx, 1, 0, evolution_[i].sigma_size);
    compute_scharr_derivatives(evolution_[i].Ly, evolution_[i].Lyy, 0, 1, evolution_[i].sigma_size);
    compute_scharr_derivatives(evolution_[i].Lx, evolution_[i].Lxy, 0, 1, evolution_[i].sigma_size);

    evolution_[i].Lx = evolution_[i].Lx*((evolution_[i].sigma_size));
    evolution_[i].Ly = evolution_[i].Ly*((evolution_[i].sigma_size));
    evolution_[i].Lxx = evolution_[i].Lxx*((evolution_[i].sigma_size)*(evolution_[i].sigma_size));
    evolution_[i].Lxy = evolution_[i].Lxy*((evolution_[i].sigma_size)*(evolution_[i].sigma_size));
    evolution_[i].Lyy = evolution_[i].Lyy*((evolution_[i].sigma_size)*(evolution_[i].sigma_size));
  }

  t2 = cv::getTickCount();
  timing_.derivatives = 1000.0*(t2-t1) / cv::getTickFrequency();
}

/* ************************************************************************* */
void KAZE::Compute_Detector_Response() {

  // Firstly compute the multiscale derivatives
  Compute_Multiscale_Derivatives();

  for (size_t i = 0; i < evolution_.size(); i++) {

    // Determinant of the Hessian
    if (options_.verbosity == true)
      cout << "Computing detector response. Determinant of Hessian. Evolution time: " << evolution_[i].etime << endl;

    for (int ix = 0; ix < options_.img_height; ix++) {

      const float* lxx = evolution_[i].Lxx.ptr<float>(ix);
      const float* lxy = evolution_[i].Lxy.ptr<float>(ix);
      const float* lyy = evolution_[i].Lyy.ptr<float>(ix);
      float* ldet = evolution_[i].Ldet.ptr<float>(ix);

      for (int jx = 0; jx < options_.img_width; jx++)
       ldet[jx] = (lxx[jx]*lyy[jx]-lxy[jx]*lxy[jx]);
    }
  }
}

/* ************************************************************************* */
void KAZE::Feature_Detection(std::vector<cv::KeyPoint>& kpts) {

  double t2 = 0.0, t1 = 0.0;
  t1 = cv::getTickCount();

  Compute_Detector_Response();
  Determinant_Hessian_Parallel(kpts);
  Do_Subpixel_Refinement(kpts);

  t2 = cv::getTickCount();
  timing_.detector = 1000.0*(t2-t1) / cv::getTickFrequency();
}

/* ************************************************************************* */
void KAZE::Determinant_Hessian_Parallel(std::vector<cv::KeyPoint>& kpts) {

  int level = 0;
  float dist = 0.0, smax = 3.0;
  int npoints = 0, id_repeated = 0;
  int left_x = 0, right_x = 0, up_y = 0, down_y = 0;
  bool is_extremum = false, is_repeated = false, is_out = false;

  // Delete the memory of the vector of keypoints vectors
  // In case we use the same kaze object for multiple images
  for (size_t i = 0; i < kpts_par_.size(); i++)
    vector<cv::KeyPoint>().swap(kpts_par_[i]);

  kpts_par_.clear();
  vector<cv::KeyPoint> aux;

  // Allocate memory for the vector of vectors
  for (size_t i = 1; i < evolution_.size()-1; i++)
    kpts_par_.push_back(aux);

#ifdef _OPENMP
#pragma omp parallel for
#endif
  for (size_t i = 1; i < evolution_.size()-1; i++)
    Find_Extremum_Threading(i);

  // Now fill the vector of keypoints!!!
  for (size_t i = 0; i < kpts_par_.size(); i++) {
    for (size_t j = 0; j < kpts_par_[i].size(); j++) {
      level = i+1;
      is_extremum = true;
      is_repeated = false;
      is_out = false;

      // Check in case we have the same point as maxima in previous evolution levels
      for (size_t ik = 0; ik < kpts.size(); ik++) {
        if (kpts[ik].class_id == level || kpts[ik].class_id == level+1 || kpts[ik].class_id == level-1) {

          dist = (kpts_par_[i][j].pt.x-kpts[ik].pt.x)*(kpts_par_[i][j].pt.x-kpts[ik].pt.x) +
              (kpts_par_[i][j].pt.y-kpts[ik].pt.y)*(kpts_par_[i][j].pt.y-kpts[ik].pt.y);

          if (dist < evolution_[level].sigma_size*evolution_[level].sigma_size) {
            if (kpts_par_[i][j].response > kpts[ik].response) {
              id_repeated = ik;
              is_repeated = true;
            }
            else {
              is_extremum = false;
            }

            break;
          }
        }
      }

      if (is_extremum == true) {
        // Check that the point is under the image limits for the descriptor computation
        left_x = fRound(kpts_par_[i][j].pt.x-smax*kpts_par_[i][j].size);
        right_x = fRound(kpts_par_[i][j].pt.x+smax*kpts_par_[i][j].size);
        up_y = fRound(kpts_par_[i][j].pt.y-smax*kpts_par_[i][j].size);
        down_y = fRound(kpts_par_[i][j].pt.y+smax*kpts_par_[i][j].size);

        if (left_x < 0 || right_x >= evolution_[level].Ldet.cols ||
            up_y < 0 || down_y >= evolution_[level].Ldet.rows) {
          is_out = true;
        }

        is_out = false;

        if (is_out == false) {
          if (is_repeated == false) {
            kpts.push_back(kpts_par_[i][j]);
            npoints++;
          }
          else {
            kpts[id_repeated] = kpts_par_[i][j];
          }
        }
      }
    }
  }
}

/* ************************************************************************* */
void KAZE::Find_Extremum_Threading(const int level) {

  float value = 0.0;
  bool is_extremum = false;

  for (int ix = 1; ix < options_.img_height-1; ix++) {
    for (int jx = 1; jx < options_.img_width-1; jx++) {

      is_extremum = false;
      value = *(evolution_[level].Ldet.ptr<float>(ix)+jx);

      // Filter the points with the detector threshold
      if (value > options_.dthreshold && value >= 0.00001) {
        if (value >= *(evolution_[level].Ldet.ptr<float>(ix)+jx-1)) {
          // First check on the same scale
          if (check_maximum_neighbourhood(evolution_[level].Ldet,1,value,ix,jx,1)) {
            // Now check on the lower scale
            if (check_maximum_neighbourhood(evolution_[level-1].Ldet,1,value,ix,jx,0)) {
              // Now check on the upper scale
              if (check_maximum_neighbourhood(evolution_[level+1].Ldet,1,value,ix,jx,0)) {
                is_extremum = true;
              }
            }
          }
        }
      }

      // Add the point of interest!!
      if (is_extremum == true) {
        cv::KeyPoint point;
        point.pt.x = jx;
        point.pt.y = ix;
        point.response = fabs(value);
        point.size = evolution_[level].esigma;
        point.octave = evolution_[level].octave;
        point.class_id = level;

        // We use the angle field for the sublevel value
        // Then, we will replace this angle field with the main orientation
        point.angle = evolution_[level].sublevel;
        kpts_par_[level-1].push_back(point);
      }
    }
  }
}

/* ************************************************************************* */
void KAZE::Do_Subpixel_Refinement(std::vector<cv::KeyPoint>& kpts) {

  int step = 1;
  int x = 0, y = 0;
  float Dx = 0.0, Dy = 0.0, Ds = 0.0, dsc = 0.0;
  float Dxx = 0.0, Dyy = 0.0, Dss = 0.0, Dxy = 0.0, Dxs = 0.0, Dys = 0.0;
  cv::Matx33f A(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  cv::Vec3f b(0.0, 0.0, 0.0);
  cv::Vec3f dst(0.0, 0.0, 0.0);
  double t2 = 0.0, t1 = 0.0;

  t1 = cv::getTickCount();
  vector<cv::KeyPoint> kpts_(kpts);

  for (size_t i = 0; i < kpts_.size(); i++) {

    x = kpts_[i].pt.x;
    y = kpts_[i].pt.y;

    // Compute the gradient
    Dx = (1.0/(2.0*step))*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x+step)
        -*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x-step));
    Dy = (1.0/(2.0*step))*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y+step)+x)
        -*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y-step)+x));
    Ds = 0.5*(*(evolution_[kpts_[i].class_id+1].Ldet.ptr<float>(y)+x)
        -*(evolution_[kpts_[i].class_id-1].Ldet.ptr<float>(y)+x));

    // Compute the Hessian
    Dxx = (1.0/(step*step))*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x+step)
        + *(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x-step)
        -2.0*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x)));

    Dyy = (1.0/(step*step))*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y+step)+x)
        + *(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y-step)+x)
        -2.0*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x)));

    Dss = *(evolution_[kpts_[i].class_id+1].Ldet.ptr<float>(y)+x)
        + *(evolution_[kpts_[i].class_id-1].Ldet.ptr<float>(y)+x)
        -2.0*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y)+x));

    Dxy = (1.0/(4.0*step))*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y+step)+x+step)
        +(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y-step)+x-step)))
        -(1.0/(4.0*step))*(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y-step)+x+step)
        +(*(evolution_[kpts_[i].class_id].Ldet.ptr<float>(y+step)+x-step)));

    Dxs = (1.0/(4.0*step))*(*(evolution_[kpts_[i].class_id+1].Ldet.ptr<float>(y)+x+step)
        +(*(evolution_[kpts_[i].class_id-1].Ldet.ptr<float>(y)+x-step)))
        -(1.0/(4.0*step))*(*(evolution_[kpts_[i].class_id+1].Ldet.ptr<float>(y)+x-step)
        +(*(evolution_[kpts_[i].class_id-1].Ldet.ptr<float>(y)+x+step)));

    Dys = (1.0/(4.0*step))*(*(evolution_[kpts_[i].class_id+1].Ldet.ptr<float>(y+step)+x)
        +(*(evolution_[kpts_[i].class_id-1].Ldet.ptr<float>(y-step)+x)))
        -(1.0/(4.0*step))*(*(evolution_[kpts_[i].class_id+1].Ldet.ptr<float>(y-step)+x)
        +(*(evolution_[kpts_[i].class_id-1].Ldet.ptr<float>(y+step)+x)));

    // Solve the linear system
    A(0,0) = Dxx;
    A(1,1) = Dyy;
    A(2,2) = Dss;
    A(0,1) = A(1,0) = Dxy;
    A(0,2) = A(2,0) = Dxs;
    A(1,2) = A(2,1) = Dys;
    b(0) = -Dx;
    b(1) = -Dy;
    b(2) = -Ds;

    cv::solve(A, b, dst, cv::DECOMP_LU);

    if (fabs(dst(0)) <= 1.0 && fabs(dst(1)) <= 1.0 && fabs(dst(2)) <= 1.0) {
      kpts_[i].pt.x += dst(0);
      kpts_[i].pt.y += dst(1);
      dsc = kpts_[i].octave + (kpts_[i].angle+dst(2))/((float)(options_.nsublevels));

      // In OpenCV the size of a keypoint is the diameter!!
      kpts_[i].size = 2.0*options_.soffset*pow(2.0f, dsc);
      kpts_[i].angle = 0.0;
    }
    // Set the points to be deleted after the for loop
    else {
      kpts_[i].response = -1;
    }
  }

  // Clear the vector of keypoints
  kpts.clear();

  for (size_t i = 0; i < kpts_.size(); i++) {
    if (kpts_[i].response != -1) {
      kpts.push_back(kpts_[i]);
    }
  }

  t2 = cv::getTickCount();
  timing_.subpixel = 1000.0*(t2-t1) / cv::getTickFrequency();
}

/* ************************************************************************* */
void KAZE::Compute_Descriptors(std::vector<cv::KeyPoint> &kpts, cv::Mat &desc) {

  double t2 = 0.0, t1 = 0.0;
  t1 = cv::getTickCount();

  // Allocate memory for the matrix of descriptors
  if (options_.descriptor == SURF_EXTENDED ||
      options_.descriptor == SURF_EXTENDED_UPRIGHT ||
      options_.descriptor == MSURF_EXTENDED ||
      options_.descriptor == MSURF_EXTENDED_UPRIGHT ||
      options_.descriptor == GSURF_EXTENDED ||
      options_.descriptor == GSURF_EXTENDED_UPRIGHT) {
    desc = cv::Mat::zeros(kpts.size(), 128, CV_32FC1);
  }
  else {
    desc = cv::Mat::zeros(kpts.size(), 64, CV_32FC1);
  }

  switch (options_.descriptor) {

    case SURF_UPRIGHT :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++)
        Get_SURF_Upright_Descriptor_64(kpts[i],desc.ptr<float>(i));
    }
    break;
    case SURF :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++) {
        Compute_Main_Orientation(kpts[i]);
        Get_SURF_Descriptor_64(kpts[i],desc.ptr<float>(i));
      }
    }
    break;
    case SURF_EXTENDED :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++) {
        Compute_Main_Orientation(kpts[i]);
        Get_SURF_Descriptor_128(kpts[i],desc.ptr<float>(i));
      }
    }
    break;
    case SURF_EXTENDED_UPRIGHT :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++)
        Get_SURF_Upright_Descriptor_128(kpts[i],desc.ptr<float>(i));
    }
    break;

    case MSURF_UPRIGHT :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++)
        Get_MSURF_Upright_Descriptor_64(kpts[i],desc.ptr<float>(i));
    }
    break;
    case MSURF :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++) {
        Compute_Main_Orientation(kpts[i]);
        Get_MSURF_Descriptor_64(kpts[i],desc.ptr<float>(i));
      }
    }
    break;
    case MSURF_EXTENDED :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++) {
        Compute_Main_Orientation(kpts[i]);
        Get_MSURF_Descriptor_128(kpts[i],desc.ptr<float>(i));
      }
    }
    break;
    case MSURF_EXTENDED_UPRIGHT :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++)
        Get_MSURF_Upright_Descriptor_128(kpts[i],desc.ptr<float>(i));
    }
    break;

    case GSURF_UPRIGHT :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++)
        Get_GSURF_Upright_Descriptor_64(kpts[i],desc.ptr<float>(i));
    }
    break;
    case GSURF :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++) {
        Compute_Main_Orientation(kpts[i]);
        Get_GSURF_Descriptor_64(kpts[i],desc.ptr<float>(i));
      }
    }
    break;
    case GSURF_EXTENDED :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++) {
        Compute_Main_Orientation(kpts[i]);
        Get_GSURF_Descriptor_128(kpts[i],desc.ptr<float>(i));
      }
    }
    break;
    case GSURF_EXTENDED_UPRIGHT :
    {
#ifdef _OPENMP
#pragma omp parallel for
#endif
      for (int i = 0; i < (int)(kpts.size()); i++)
        Get_GSURF_Upright_Descriptor_128(kpts[i],desc.ptr<float>(i));
    }
    break;
  }

  t2 = cv::getTickCount();
  timing_.descriptor = 1000.0*(t2-t1) / cv::getTickFrequency();
}

/* ************************************************************************* */
void KAZE::Compute_Main_Orientation(cv::KeyPoint& kpt) {

  int ix = 0, iy = 0, idx = 0, s = 0, level = 0;
  float xf = 0.0, yf = 0.0, gweight = 0.0;
  vector<float> resX(109), resY(109), Ang(109);

  // Variables for computing the dominant direction
  float sumX = 0.0, sumY = 0.0, max = 0.0, ang1 = 0.0, ang2 = 0.0;

  // Get the information from the keypoint
  xf = kpt.pt.x;
  yf = kpt.pt.y;
  level = kpt.class_id;
  s = fRound(kpt.size/2.0f);

  // Calculate derivatives responses for points within radius of 6*scale
  for (int i = -6; i <= 6; ++i) {
    for (int j = -6; j <= 6; ++j) {
      if (i*i + j*j < 36) {
        iy = fRound(yf + j*s);
        ix = fRound(xf + i*s);

        if (iy >= 0 && iy < options_.img_height && ix >= 0 && ix < options_.img_width) {
          gweight = gaussian(iy-yf, ix-xf, 2.5*s);
          resX[idx] = gweight*(*(evolution_[level].Lx.ptr<float>(iy)+ix));
          resY[idx] = gweight*(*(evolution_[level].Ly.ptr<float>(iy)+ix));
        }
        else {
          resX[idx] = 0.0;
          resY[idx] = 0.0;
        }

        Ang[idx] = cv::fastAtan2(resY[idx], resX[idx])*(CV_PI/180.0);
        ++idx;
      }
    }
  }

  // Loop slides pi/3 window around feature point
  for (ang1 = 0; ang1 < 2.0*CV_PI;  ang1+=0.15f) {
    ang2 =(ang1+CV_PI/3.0f > 2.0*CV_PI ? ang1-5.0f*CV_PI/3.0f : ang1+CV_PI/3.0f);
    sumX = sumY = 0.f;

    for (size_t k = 0; k < Ang.size(); ++k) {
      // Get angle from the x-axis of the sample point
      const float& ang = Ang[k];

      // Determine whether the point is within the window
      if (ang1 < ang2 && ang1 < ang && ang < ang2) {
        sumX += resX[k];
        sumY += resY[k];
      }
      else if (ang2 < ang1 && ((ang > 0 && ang < ang2) || (ang > ang1 && ang < 2.0*CV_PI))) {
        sumX += resX[k];
        sumY += resY[k];
      }
    }

    // if the vector produced from this window is longer than all
    // previous vectors then this forms the new dominant direction
    if (sumX*sumX + sumY*sumY > max) {
      // store largest orientation
      max = sumX*sumX + sumY*sumY;
      kpt.angle =  cv::fastAtan2(sumY, sumX)*(CV_PI/180.0);
    }
  }
}

/* ************************************************************************* */
void KAZE::Get_SURF_Upright_Descriptor_64(const cv::KeyPoint& kpt, float* desc) {

  float dx = 0.0, dy = 0.0, mdx = 0.0, mdy = 0.0;
  float rx = 0.0, ry = 0.0, len = 0.0, xf = 0.0, yf = 0.0, sample_x = 0.0, sample_y = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 64;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  level = kpt.class_id;
  scale = fRound(kpt.size/2.0f);

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dx=dy=mdx=mdy=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          sample_y = k*scale + yf;
          sample_x = l*scale + xf;

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          // Sum the derivatives to the cumulative descriptor
          dx += rx;
          dy += ry;
          mdx += fabs(rx);
          mdy += fabs(ry);
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dx;
      desc[dcount++] = dy;
      desc[dcount++] = mdx;
      desc[dcount++] = mdy;

      // Store the current length^2 of the vector
      len += dx*dx + dy*dy + mdx*mdx + mdy*mdy;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_SURF_Descriptor_64(const cv::KeyPoint &kpt, float *desc) {

  float dx = 0.0, dy = 0.0, mdx = 0.0, mdy = 0.0;
  float rx = 0.0, ry = 0.0, rrx = 0.0, rry = 0.0, len = 0.0, xf = 0.0, yf = 0.0;
  float sample_x = 0.0, sample_y = 0.0, co = 0.0, si = 0.0, angle = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 64;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  angle = kpt.angle;
  level = kpt.class_id;
  co = cos(angle);
  si = sin(angle);

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {
      dx=dy=mdx=mdy=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          // Get the coordinates of the sample point on the rotated axis
          sample_y = yf + (l*scale*co + k*scale*si);
          sample_x = xf + (-l*scale*si + k*scale*co);

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          // Get the x and y derivatives on the rotated axis
          rry = rx*co + ry*si;
          rrx = -rx*si + ry*co;

          // Sum the derivatives to the cumulative descriptor
          dx += rrx;
          dy += rry;
          mdx += fabs(rrx);
          mdy += fabs(rry);
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dx;
      desc[dcount++] = dy;
      desc[dcount++] = mdx;
      desc[dcount++] = mdy;

      // Store the current length^2 of the vector
      len += dx*dx + dy*dy + mdx*mdx + mdy*mdy;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_MSURF_Upright_Descriptor_64(const cv::KeyPoint& kpt, float* desc) {

  float dx = 0.0, dy = 0.0, mdx = 0.0, mdy = 0.0, gauss_s1 = 0.0, gauss_s2 = 0.0;
  float rx = 0.0, ry = 0.0, len = 0.0, xf = 0.0, yf = 0.0, ys = 0.0, xs = 0.0;
  float sample_x = 0.0, sample_y = 0.0;
  int x1 = 0, y1 = 0, sample_step = 0, pattern_size = 0;
  int x2 = 0, y2 = 0, kx = 0, ky = 0, i = 0, j = 0, dcount = 0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  int dsize = 0, scale = 0, level = 0;

  // Subregion centers for the 4x4 gaussian weighting
  float cx = -0.5, cy = 0.5;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 64;
  sample_step = 5;
  pattern_size = 12;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  level = kpt.class_id;

  i = -8;

  // Calculate descriptor for this interest point
  // Area of size 24 s x 24 s
  while (i < pattern_size) {
    j = -8;
    i = i-4;

    cx += 1.0;
    cy = -0.5;

    while (j < pattern_size) {

      dx=dy=mdx=mdy=0.0;
      cy += 1.0;
      j = j-4;

      ky = i + sample_step;
      kx = j + sample_step;

      ys = yf + (ky*scale);
      xs = xf + (kx*scale);

      for (int k = i; k < i+9; k++) {
        for (int l = j; l < j+9; l++) {

          sample_y = k*scale + yf;
          sample_x = l*scale + xf;

          //Get the gaussian weighted x and y responses
          gauss_s1 = gaussian(xs-sample_x,ys-sample_y,2.5*scale);

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          rx = gauss_s1*rx;
          ry = gauss_s1*ry;

          // Sum the derivatives to the cumulative descriptor
          dx += rx;
          dy += ry;
          mdx += fabs(rx);
          mdy += fabs(ry);
        }
      }

      // Add the values to the descriptor vector
      gauss_s2 = gaussian(cx-2.0f,cy-2.0f,1.5f);

      desc[dcount++] = dx*gauss_s2;
      desc[dcount++] = dy*gauss_s2;
      desc[dcount++] = mdx*gauss_s2;
      desc[dcount++] = mdy*gauss_s2;

      len += (dx*dx + dy*dy + mdx*mdx + mdy*mdy)*gauss_s2*gauss_s2;

      j += 9;
    }

    i += 9;
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_MSURF_Descriptor_64(const cv::KeyPoint& kpt, float* desc) {

  float dx = 0.0, dy = 0.0, mdx = 0.0, mdy = 0.0, gauss_s1 = 0.0, gauss_s2 = 0.0;
  float rx = 0.0, ry = 0.0, rrx = 0.0, rry = 0.0, len = 0.0, xf = 0.0, yf = 0.0, ys = 0.0, xs = 0.0;
  float sample_x = 0.0, sample_y = 0.0, co = 0.0, si = 0.0, angle = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0;
  int kx = 0, ky = 0, i = 0, j = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Subregion centers for the 4x4 gaussian weighting
  float cx = -0.5, cy = 0.5;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 64;
  sample_step = 5;
  pattern_size = 12;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  angle = kpt.angle;
  level = kpt.class_id;
  co = cos(angle);
  si = sin(angle);

  i = -8;

  // Calculate descriptor for this interest point
  // Area of size 24 s x 24 s
  while (i < pattern_size) {

    j = -8;
    i = i-4;

    cx += 1.0;
    cy = -0.5;

    while (j < pattern_size) {

      dx=dy=mdx=mdy=0.0;
      cy += 1.0;
      j = j - 4;

      ky = i + sample_step;
      kx = j + sample_step;

      xs = xf + (-kx*scale*si + ky*scale*co);
      ys = yf + (kx*scale*co + ky*scale*si);

      for (int k = i; k < i + 9; ++k) {
        for (int l = j; l < j + 9; ++l) {

          // Get coords of sample point on the rotated axis
          sample_y = yf + (l*scale*co + k*scale*si);
          sample_x = xf + (-l*scale*si + k*scale*co);

          // Get the gaussian weighted x and y responses
          gauss_s1 = gaussian(xs-sample_x,ys-sample_y,2.5*scale);
          y1 = fRound(sample_y-.5);
          x1 = fRound(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          // Get the x and y derivatives on the rotated axis
          rry = gauss_s1*(rx*co + ry*si);
          rrx = gauss_s1*(-rx*si + ry*co);

          // Sum the derivatives to the cumulative descriptor
          dx += rrx;
          dy += rry;
          mdx += fabs(rrx);
          mdy += fabs(rry);
        }
      }

      // Add the values to the descriptor vector
      gauss_s2 = gaussian(cx-2.0f,cy-2.0f,1.5f);
      desc[dcount++] = dx*gauss_s2;
      desc[dcount++] = dy*gauss_s2;
      desc[dcount++] = mdx*gauss_s2;
      desc[dcount++] = mdy*gauss_s2;
      len += (dx*dx + dy*dy + mdx*mdx + mdy*mdy)*gauss_s2*gauss_s2;
      j += 9;
    }
    i += 9;
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_GSURF_Upright_Descriptor_64(const cv::KeyPoint& kpt, float* desc) {

  float dx = 0.0, dy = 0.0, mdx = 0.0, mdy = 0.0;
  float rx = 0.0, ry = 0.0, rxx = 0.0, rxy = 0.0, ryy = 0.0, len = 0.0, xf = 0.0, yf = 0.0;
  float sample_x = 0.0, sample_y = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float lvv = 0.0, lww = 0.0, modg = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 64;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  level = kpt.class_id;

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dx=dy=mdx=mdy=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          // Get the coordinates of the sample point on the rotated axis
          sample_y = yf + l*scale;
          sample_x = xf + k*scale;

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          modg = pow(rx,2) + pow(ry,2);

          if (modg != 0.0) {

            res1 = *(evolution_[level].Lxx.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxx.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxx.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxx.ptr<float>(y2)+x2);
            rxx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lxy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxy.ptr<float>(y2)+x2);
            rxy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lyy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lyy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lyy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lyy.ptr<float>(y2)+x2);
            ryy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            // Lww = (Lx^2 * Lxx + 2*Lx*Lxy*Ly + Ly^2*Lyy) / (Lx^2 + Ly^2)
            lww = (pow(rx,2)*rxx + 2.0*rx*rxy*ry + pow(ry,2)*ryy) / (modg);

            // Lvv = (-2*Lx*Lxy*Ly + Lxx*Ly^2 + Lx^2*Lyy) / (Lx^2 + Ly^2)
            lvv = (-2.0*rx*rxy*ry + rxx*pow(ry,2) + pow(rx,2)*ryy) /(modg);
          }
          else {
            lww = 0.0;
            lvv = 0.0;
          }

          // Sum the derivatives to the cumulative descriptor
          dx += lww;
          dy += lvv;
          mdx += fabs(lww);
          mdy += fabs(lvv);
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dx;
      desc[dcount++] = dy;
      desc[dcount++] = mdx;
      desc[dcount++] = mdy;

      // Store the current length^2 of the vector
      len += dx*dx + dy*dy + mdx*mdx + mdy*mdy;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_GSURF_Descriptor_64(const cv::KeyPoint &kpt, float *desc) {

  float dx = 0.0, dy = 0.0, mdx = 0.0, mdy = 0.0;
  float rx = 0.0, ry = 0.0, rxx = 0.0, rxy = 0.0, ryy = 0.0, len = 0.0, xf = 0.0, yf = 0.0;
  float sample_x = 0.0, sample_y = 0.0, co = 0.0, si = 0.0, angle = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float lvv = 0.0, lww = 0.0, modg = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 64;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  angle = kpt.angle;
  level = kpt.class_id;
  co = cos(angle);
  si = sin(angle);

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dx=dy=mdx=mdy=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          // Get the coordinates of the sample point on the rotated axis
          sample_y = yf + (l*scale*co + k*scale*si);
          sample_x = xf + (-l*scale*si + k*scale*co);

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          modg = pow(rx,2) + pow(ry,2);

          if (modg != 0.0) {

            res1 = *(evolution_[level].Lxx.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxx.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxx.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxx.ptr<float>(y2)+x2);
            rxx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lxy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxy.ptr<float>(y2)+x2);
            rxy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lyy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lyy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lyy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lyy.ptr<float>(y2)+x2);
            ryy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            // Lww = (Lx^2 * Lxx + 2*Lx*Lxy*Ly + Ly^2*Lyy) / (Lx^2 + Ly^2)
            lww = (pow(rx,2)*rxx + 2.0*rx*rxy*ry + pow(ry,2)*ryy) / (modg);

            // Lvv = (-2*Lx*Lxy*Ly + Lxx*Ly^2 + Lx^2*Lyy) / (Lx^2 + Ly^2)
            lvv = (-2.0*rx*rxy*ry + rxx*pow(ry,2) + pow(rx,2)*ryy) /(modg);
          }
          else {
            lww = 0.0;
            lvv = 0.0;
          }

          // Sum the derivatives to the cumulative descriptor
          dx += lww;
          dy += lvv;
          mdx += fabs(lww);
          mdy += fabs(lvv);
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dx;
      desc[dcount++] = dy;
      desc[dcount++] = mdx;
      desc[dcount++] = mdy;

      // Store the current length^2 of the vector
      len += dx*dx + dy*dy + mdx*mdx + mdy*mdy;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_SURF_Upright_Descriptor_128(const cv::KeyPoint& kpt, float* desc) {

  float rx = 0.0, ry = 0.0, len = 0.0, xf = 0.0, yf = 0.0, sample_x = 0.0, sample_y = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float dxp = 0.0, dyp = 0.0, mdxp = 0.0, mdyp = 0.0;
  float dxn = 0.0, dyn = 0.0, mdxn = 0.0, mdyn = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 128;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  level = kpt.class_id;

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dxp=dxn=mdxp=mdxn=0.0;
      dyp=dyn=mdyp=mdyn=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          sample_y = k*scale + yf;
          sample_x = l*scale + xf;

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          // Sum the derivatives to the cumulative descriptor
          if (ry >= 0.0) {
            dxp += rx;
            mdxp += fabs(rx);
          }
          else {
            dxn += rx;
            mdxn += fabs(rx);
          }

          if (rx >= 0.0) {
            dyp += ry;
            mdyp += fabs(ry);
          }
          else {
            dyn += ry;
            mdyn += fabs(ry);
          }
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dxp;
      desc[dcount++] = dxn;
      desc[dcount++] = mdxp;
      desc[dcount++] = mdxn;
      desc[dcount++] = dyp;
      desc[dcount++] = dyn;
      desc[dcount++] = mdyp;
      desc[dcount++] = mdyn;

      // Store the current length^2 of the vector
      len += dxp*dxp + dxn*dxn + mdxp*mdxp + mdxn*mdxn +
          dyp*dyp + dyn*dyn + mdyp*mdyp + mdyn*mdyn;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_SURF_Descriptor_128(const cv::KeyPoint& kpt, float* desc) {

  float rx = 0.0, ry = 0.0, rrx = 0.0, rry = 0.0, len = 0.0, xf = 0.0, yf = 0.0;
  float sample_x = 0.0, sample_y = 0.0, co = 0.0, si = 0.0, angle = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float dxp = 0.0, dyp = 0.0, mdxp = 0.0, mdyp = 0.0;
  float dxn = 0.0, dyn = 0.0, mdxn = 0.0, mdyn = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 128;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  angle = kpt.angle;
  level = kpt.class_id;
  co = cos(angle);
  si = sin(angle);

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dxp=dxn=mdxp=mdxn=0.0;
      dyp=dyn=mdyp=mdyn=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          // Get the coordinates of the sample point on the rotated axis
          sample_y = yf + (l*scale*co + k*scale*si);
          sample_x = xf + (-l*scale*si + k*scale*co);

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          // Get the x and y derivatives on the rotated axis
          rry = rx*co + ry*si;
          rrx = -rx*si + ry*co;

          // Sum the derivatives to the cumulative descriptor
          if (rry >= 0.0) {
            dxp += rrx;
            mdxp += fabs(rrx);
          }
          else {
            dxn += rrx;
            mdxn += fabs(rrx);
          }

          if (rrx >= 0.0) {
            dyp += rry;
            mdyp += fabs(rry);
          }
          else {
            dyn += rry;
            mdyn += fabs(rry);
          }
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dxp;
      desc[dcount++] = dxn;
      desc[dcount++] = mdxp;
      desc[dcount++] = mdxn;
      desc[dcount++] = dyp;
      desc[dcount++] = dyn;
      desc[dcount++] = mdyp;
      desc[dcount++] = mdyn;

      // Store the current length^2 of the vector
      len += dxp*dxp + dxn*dxn + mdxp*mdxp + mdxn*mdxn +
          dyp*dyp + dyn*dyn + mdyp*mdyp + mdyn*mdyn;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_MSURF_Upright_Descriptor_128(const cv::KeyPoint& kpt, float* desc) {

  float gauss_s1 = 0.0, gauss_s2 = 0.0;
  float rx = 0.0, ry = 0.0, len = 0.0, xf = 0.0, yf = 0.0, ys = 0.0, xs = 0.0;
  float sample_x = 0.0, sample_y = 0.0;
  int x1 = 0, y1 = 0, sample_step = 0, pattern_size = 0;
  int x2 = 0, y2 = 0, kx = 0, ky = 0, i = 0, j = 0, dcount = 0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float dxp = 0.0, dyp = 0.0, mdxp = 0.0, mdyp = 0.0;
  float dxn = 0.0, dyn = 0.0, mdxn = 0.0, mdyn = 0.0;
  int dsize = 0, scale = 0, level = 0;

  // Subregion centers for the 4x4 gaussian weighting
  float cx = -0.5, cy = 0.5;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 128;
  sample_step = 5;
  pattern_size = 12;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  level = kpt.class_id;

  i = -8;

  // Calculate descriptor for this interest point
  // Area of size 24 s x 24 s
  while (i < pattern_size) {

    j = -8;
    i = i-4;

    cx += 1.0;
    cy = -0.5;

    while (j < pattern_size) {

      dxp=dxn=mdxp=mdxn=0.0;
      dyp=dyn=mdyp=mdyn=0.0;

      cy += 1.0;
      j = j-4;

      ky = i + sample_step;
      kx = j + sample_step;

      ys = yf + (ky*scale);
      xs = xf + (kx*scale);

      for (int k = i; k < i+9; k++) {
        for (int l = j; l < j+9; l++) {

          sample_y = k*scale + yf;
          sample_x = l*scale + xf;

          //Get the gaussian weighted x and y responses
          gauss_s1 = gaussian(xs-sample_x,ys-sample_y,2.50*scale);

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          rx = gauss_s1*rx;
          ry = gauss_s1*ry;

          // Sum the derivatives to the cumulative descriptor
          if (ry >= 0.0) {
            dxp += rx;
            mdxp += fabs(rx);
          }
          else {
            dxn += rx;
            mdxn += fabs(rx);
          }

          if (rx >= 0.0) {
            dyp += ry;
            mdyp += fabs(ry);
          }
          else {
            dyn += ry;
            mdyn += fabs(ry);
          }
        }
      }

      // Add the values to the descriptor vector
      gauss_s2 = gaussian(cx-2.0f,cy-2.0f,1.5f);

      desc[dcount++] = dxp*gauss_s2;
      desc[dcount++] = dxn*gauss_s2;
      desc[dcount++] = mdxp*gauss_s2;
      desc[dcount++] = mdxn*gauss_s2;
      desc[dcount++] = dyp*gauss_s2;
      desc[dcount++] = dyn*gauss_s2;
      desc[dcount++] = mdyp*gauss_s2;
      desc[dcount++] = mdyn*gauss_s2;

      // Store the current length^2 of the vector
      len += (dxp*dxp + dxn*dxn + mdxp*mdxp + mdxn*mdxn +
              dyp*dyp + dyn*dyn + mdyp*mdyp + mdyn*mdyn)*gauss_s2*gauss_s2;

      j += 9;
    }

    i += 9;
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_MSURF_Descriptor_128(const cv::KeyPoint& kpt, float* desc) {

  float gauss_s1 = 0.0, gauss_s2 = 0.0;
  float rx = 0.0, ry = 0.0, rrx = 0.0, rry = 0.0, len = 0.0, xf = 0.0, yf = 0.0, ys = 0.0, xs = 0.0;
  float sample_x = 0.0, sample_y = 0.0, co = 0.0, si = 0.0, angle = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float dxp = 0.0, dyp = 0.0, mdxp = 0.0, mdyp = 0.0;
  float dxn = 0.0, dyn = 0.0, mdxn = 0.0, mdyn = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0;
  int kx = 0, ky = 0, i = 0, j = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Subregion centers for the 4x4 gaussian weighting
  float cx = -0.5, cy = 0.5;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 128;
  sample_step = 5;
  pattern_size = 12;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  angle = kpt.angle;
  level = kpt.class_id;
  co = cos(angle);
  si = sin(angle);

  i = -8;

  // Calculate descriptor for this interest point
  // Area of size 24 s x 24 s
  while (i < pattern_size) {

    j = -8;
    i = i-4;

    cx += 1.0;
    cy = -0.5;

    while (j < pattern_size) {

      dxp=dxn=mdxp=mdxn=0.0;
      dyp=dyn=mdyp=mdyn=0.0;

      cy += 1.0f;
      j = j - 4;

      ky = i + sample_step;
      kx = j + sample_step;

      xs = xf + (-kx*scale*si + ky*scale*co);
      ys = yf + (kx*scale*co + ky*scale*si);

      for (int k = i; k < i + 9; ++k) {
        for (int l = j; l < j + 9; ++l) {

          // Get coords of sample point on the rotated axis
          sample_y = yf + (l*scale*co + k*scale*si);
          sample_x = xf + (-l*scale*si + k*scale*co);

          // Get the gaussian weighted x and y responses
          gauss_s1 = gaussian(xs-sample_x,ys-sample_y,2.5*scale);

          y1 = fRound(sample_y-.5);
          x1 = fRound(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          // Get the x and y derivatives on the rotated axis
          rry = gauss_s1*(rx*co + ry*si);
          rrx = gauss_s1*(-rx*si + ry*co);

          // Sum the derivatives to the cumulative descriptor
          if (rry >= 0.0) {
            dxp += rrx;
            mdxp += fabs(rrx);
          }
          else {
            dxn += rrx;
            mdxn += fabs(rrx);
          }

          if (rrx >= 0.0) {
            dyp += rry;
            mdyp += fabs(rry);
          }
          else {
            dyn += rry;
            mdyn += fabs(rry);
          }
        }
      }

      // Add the values to the descriptor vector
      gauss_s2 = gaussian(cx-2.0f,cy-2.0f,1.5f);

      desc[dcount++] = dxp*gauss_s2;
      desc[dcount++] = dxn*gauss_s2;
      desc[dcount++] = mdxp*gauss_s2;
      desc[dcount++] = mdxn*gauss_s2;
      desc[dcount++] = dyp*gauss_s2;
      desc[dcount++] = dyn*gauss_s2;
      desc[dcount++] = mdyp*gauss_s2;
      desc[dcount++] = mdyn*gauss_s2;

      // Store the current length^2 of the vector
      len += (dxp*dxp + dxn*dxn + mdxp*mdxp + mdxn*mdxn +
              dyp*dyp + dyn*dyn + mdyp*mdyp + mdyn*mdyn)*gauss_s2*gauss_s2;

      j += 9;
    }

    i += 9;
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_GSURF_Upright_Descriptor_128(const cv::KeyPoint& kpt, float* desc) {

  float len = 0.0, xf = 0.0, yf = 0.0, sample_x = 0.0, sample_y = 0.0;
  float rx = 0.0, ry = 0.0, rxx = 0.0, rxy = 0.0, ryy = 0.0, modg = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float dxp = 0.0, dyp = 0.0, mdxp = 0.0, mdyp = 0.0;
  float dxn = 0.0, dyn = 0.0, mdxn = 0.0, mdyn = 0.0, lvv = 0.0, lww = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 128;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  level = kpt.class_id;

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for(int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dxp=dxn=mdxp=mdxn=0.0;
      dyp=dyn=mdyp=mdyn=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          sample_y = k*scale + yf;
          sample_x = l*scale + xf;

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          modg = pow(rx,2) + pow(ry,2);

          if (modg != 0.0) {

            res1 = *(evolution_[level].Lxx.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxx.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxx.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxx.ptr<float>(y2)+x2);
            rxx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lxy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxy.ptr<float>(y2)+x2);
            rxy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lyy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lyy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lyy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lyy.ptr<float>(y2)+x2);
            ryy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            // Lww = (Lx^2 * Lxx + 2*Lx*Lxy*Ly + Ly^2*Lyy) / (Lx^2 + Ly^2)
            lww = (pow(rx,2)*rxx + 2.0*rx*rxy*ry + pow(ry,2)*ryy) / (modg);

            // Lvv = (-2*Lx*Lxy*Ly + Lxx*Ly^2 + Lx^2*Lyy) / (Lx^2 + Ly^2)
            lvv = (-2.0*rx*rxy*ry + rxx*pow(ry,2) + pow(rx,2)*ryy) /(modg);
          }
          else {
            lww = 0.0;
            lvv = 0.0;
          }

          // Sum the derivatives to the cumulative descriptor
          if (lww >= 0.0) {
            dxp += lvv;
            mdxp += fabs(lvv);
          }
          else {
            dxn += lvv;
            mdxn += fabs(lvv);
          }

          if (lvv >= 0.0) {
            dyp += lww;
            mdyp += fabs(lww);
          }
          else {
            dyn += lww;
            mdyn += fabs(lww);
          }
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dxp;
      desc[dcount++] = dxn;
      desc[dcount++] = mdxp;
      desc[dcount++] = mdxn;
      desc[dcount++] = dyp;
      desc[dcount++] = dyn;
      desc[dcount++] = mdyp;
      desc[dcount++] = mdyn;

      // Store the current length^2 of the vector
      len += dxp*dxp + dxn*dxn + mdxp*mdxp + mdxn*mdxn +
          dyp*dyp + dyn*dyn + mdyp*mdyp + mdyn*mdyn;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::Get_GSURF_Descriptor_128(const cv::KeyPoint& kpt, float* desc) {

  float len = 0.0, xf = 0.0, yf = 0.0;
  float rx = 0.0, ry = 0.0, rxx = 0.0, rxy = 0.0, ryy = 0.0;
  float sample_x = 0.0, sample_y = 0.0, co = 0.0, si = 0.0, angle = 0.0;
  float fx = 0.0, fy = 0.0, res1 = 0.0, res2 = 0.0, res3 = 0.0, res4 = 0.0;
  float dxp = 0.0, dyp = 0.0, mdxp = 0.0, mdyp = 0.0;
  float dxn = 0.0, dyn = 0.0, mdxn = 0.0, mdyn = 0.0;
  float lvv = 0.0, lww = 0.0, modg = 0.0;
  int x1 = 0, y1 = 0, x2 = 0, y2 = 0, sample_step = 0, pattern_size = 0, dcount = 0;
  int dsize = 0, scale = 0, level = 0;

  // Set the descriptor size and the sample and pattern sizes
  dsize = 128;
  sample_step = 5;
  pattern_size = 10;

  // Get the information from the keypoint
  yf = kpt.pt.y;
  xf = kpt.pt.x;
  scale = fRound(kpt.size/2.0f);
  angle = kpt.angle;
  level = kpt.class_id;
  co = cos(angle);
  si = sin(angle);

  // Calculate descriptor for this interest point
  for (int i = -pattern_size; i < pattern_size; i+=sample_step) {
    for (int j = -pattern_size; j < pattern_size; j+=sample_step) {

      dxp=dxn=mdxp=mdxn=0.0;
      dyp=dyn=mdyp=mdyn=0.0;

      for (int k = i; k < i + sample_step; k++) {
        for (int l = j; l < j + sample_step; l++) {

          // Get the coordinates of the sample point on the rotated axis
          sample_y = yf + (l*scale*co + k*scale*si);
          sample_x = xf + (-l*scale*si + k*scale*co);

          y1 = (int)(sample_y-.5);
          x1 = (int)(sample_x-.5);

          checkDescriptorLimits(x1,y1,options_.img_width,options_.img_height);

          y2 = (int)(sample_y+.5);
          x2 = (int)(sample_x+.5);

          checkDescriptorLimits(x2,y2,options_.img_width,options_.img_height);

          fx = sample_x-x1;
          fy = sample_y-y1;

          res1 = *(evolution_[level].Lx.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Lx.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Lx.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Lx.ptr<float>(y2)+x2);
          rx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          res1 = *(evolution_[level].Ly.ptr<float>(y1)+x1);
          res2 = *(evolution_[level].Ly.ptr<float>(y1)+x2);
          res3 = *(evolution_[level].Ly.ptr<float>(y2)+x1);
          res4 = *(evolution_[level].Ly.ptr<float>(y2)+x2);
          ry = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

          modg = pow(rx,2) + pow(ry,2);

          if (modg != 0.0) {
            res1 = *(evolution_[level].Lxx.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxx.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxx.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxx.ptr<float>(y2)+x2);
            rxx = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lxy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lxy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lxy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lxy.ptr<float>(y2)+x2);
            rxy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            res1 = *(evolution_[level].Lyy.ptr<float>(y1)+x1);
            res2 = *(evolution_[level].Lyy.ptr<float>(y1)+x2);
            res3 = *(evolution_[level].Lyy.ptr<float>(y2)+x1);
            res4 = *(evolution_[level].Lyy.ptr<float>(y2)+x2);
            ryy = (1.0-fx)*(1.0-fy)*res1 + fx*(1.0-fy)*res2 + (1.0-fx)*fy*res3 + fx*fy*res4;

            // Lww = (Lx^2 * Lxx + 2*Lx*Lxy*Ly + Ly^2*Lyy) / (Lx^2 + Ly^2)
            lww = (pow(rx,2)*rxx + 2.0*rx*rxy*ry + pow(ry,2)*ryy) / (modg);

            // Lvv = (-2*Lx*Lxy*Ly + Lxx*Ly^2 + Lx^2*Lyy) / (Lx^2 + Ly^2)
            lvv = (-2.0*rx*rxy*ry + rxx*pow(ry,2) + pow(rx,2)*ryy) /(modg);
          }
          else {
            lww = 0.0;
            lvv = 0.0;
          }

          // Sum the derivatives to the cumulative descriptor
          if (lww >= 0.0) {
            dxp += lvv;
            mdxp += fabs(lvv);
          }
          else {
            dxn += lvv;
            mdxn += fabs(lvv);
          }

          if (lvv >= 0.0) {
            dyp += lww;
            mdyp += fabs(lww);
          }
          else {
            dyn += lww;
            mdyn += fabs(lww);
          }
        }
      }

      // Add the values to the descriptor vector
      desc[dcount++] = dxp;
      desc[dcount++] = dxn;
      desc[dcount++] = mdxp;
      desc[dcount++] = mdxn;
      desc[dcount++] = dyp;
      desc[dcount++] = dyn;
      desc[dcount++] = mdyp;
      desc[dcount++] = mdyn;

      // Store the current length^2 of the vector
      len += dxp*dxp + dxn*dxn + mdxp*mdxp + mdxn*mdxn +
          dyp*dyp + dyn*dyn + mdyp*mdyp + mdyn*mdyn;
    }
  }

  // convert to unit vector
  len = sqrt(len);

  for (int i = 0; i < dsize; i++)
    desc[i] /= len;
}

/* ************************************************************************* */
void KAZE::AOS_Step_Scalar(cv::Mat& Ld, const cv::Mat& Ldprev, const cv::Mat& c, const float stepsize) {

#ifdef _OPENMP
#pragma omp sections
  {
#pragma omp section
    {
      AOS_Rows(Ldprev,c,stepsize);
    }
#pragma omp section
    {
      AOS_Columns(Ldprev,c,stepsize);
    }
  }
#else
  AOS_Rows(Ldprev, c, stepsize);
  AOS_Columns(Ldprev, c, stepsize);
#endif

  Ld = 0.5*(Lty_+Ltx_.t());
}

/* ************************************************************************* */
void KAZE::AOS_Rows(const cv::Mat& Ldprev, const cv::Mat& c, const float stepsize) {

  // Operate on rows
  for (int i = 0; i < qr_.rows; i++) {
    for (int j = 0; j < qr_.cols; j++) {
      *(qr_.ptr<float>(i)+j) = *(c.ptr<float>(i)+j) + *(c.ptr<float>(i+1)+j);
    }
  }

  for (int j = 0; j < py_.cols; j++) {
    *(py_.ptr<float>(0)+j) = *(qr_.ptr<float>(0)+j);
  }

  for (int j = 0; j < py_.cols; j++) {
    *(py_.ptr<float>(py_.rows-1)+j) = *(qr_.ptr<float>(qr_.rows-1)+j);
  }

  for (int i = 1; i < py_.rows-1; i++) {
    for (int j = 0; j < py_.cols; j++) {
      *(py_.ptr<float>(i)+j) = *(qr_.ptr<float>(i-1)+j) + *(qr_.ptr<float>(i)+j);
    }
  }

  // a = 1 + t.*p; (p is -1*p)
  // b = -t.*q;
  ay_ = 1.0 + stepsize*py_; // p is -1*p
  by_ = -stepsize*qr_;

  // Do Thomas algorithm to solve the linear system of equations
  Thomas(ay_,by_,Ldprev,Lty_);
}

/* ************************************************************************* */
void KAZE::AOS_Columns(const cv::Mat& Ldprev, const cv::Mat& c, const float stepsize) {

  // Operate on columns
  for (int j = 0; j < qc_.cols; j++) {
    for (int i = 0; i < qc_.rows; i++) {
      *(qc_.ptr<float>(i)+j) = *(c.ptr<float>(i)+j) + *(c.ptr<float>(i)+j+1);
    }
  }

  for (int i = 0; i < px_.rows; i++) {
    *(px_.ptr<float>(i)) = *(qc_.ptr<float>(i));
  }

  for (int i = 0; i < px_.rows; i++) {
    *(px_.ptr<float>(i)+px_.cols-1) = *(qc_.ptr<float>(i)+qc_.cols-1);
  }

  for (int j = 1; j < px_.cols-1; j++) {
    for (int i = 0; i < px_.rows; i++) {
      *(px_.ptr<float>(i)+j) = *(qc_.ptr<float>(i)+j-1) + *(qc_.ptr<float>(i)+j);
    }
  }

  // a = 1 + t.*p';
  ax_ = 1.0 + stepsize*px_.t();

  // b = -t.*q';
  bx_ = -stepsize*qc_.t();

  // But take care since we need to transpose the solution!!
  cv::Mat Ldprevt = Ldprev.t();

  // Do Thomas algorithm to solve the linear system of equations
  Thomas(ax_,bx_,Ldprevt,Ltx_);
}

/* ************************************************************************* */
void KAZE::Thomas(const cv::Mat& a, const cv::Mat &b, const cv::Mat &Ld, cv::Mat &x) {

  // Auxiliary variables
  int n = a.rows;
  cv::Mat m = cv::Mat::zeros(a.rows, a.cols,CV_32F);
  cv::Mat l = cv::Mat::zeros(b.rows, b.cols,CV_32F);
  cv::Mat y = cv::Mat::zeros(Ld.rows, Ld.cols,CV_32F);

  /** A*x = d;																		   	   */
  /**	/ a1 b1  0  0 0  ...    0 \  / x1 \ = / d1 \										   */
  /**	| c1 a2 b2  0 0  ...    0 |  | x2 | = | d2 |										   */
  /**	|  0 c2 a3 b3 0  ...    0 |  | x3 | = | d3 |										   */
  /**	|  :  :  :  : 0  ...    0 |  |  : | = |  : |										   */
  /**	|  :  :  :  : 0  cn-1  an |  | xn | = | dn |										   */

  /** 1. LU decomposition
   / L = / 1				 \		U = / m1 r1			   \
   /     | l1 1 			 |	        |    m2 r2		   |
   /     |    l2 1          |			|		m3 r3	   |
   /	  |     : : :        |			|       :  :  :	   |
   /	  \           ln-1 1 /			\				mn /	*/
  for (int j = 0; j < m.cols; j++) {
    *(m.ptr<float>(0)+j) = *(a.ptr<float>(0)+j);
  }

  for (int j = 0; j < y.cols; j++) {
    *(y.ptr<float>(0)+j) = *(Ld.ptr<float>(0)+j);
  }

  // 1. Forward substitution L*y = d for y
  for (int k = 1; k < n; k++) {
    for (int j=0; j < l.cols; j++) {
      *(l.ptr<float>(k-1)+j) = *(b.ptr<float>(k-1)+j) / *(m.ptr<float>(k-1)+j);
    }

    for (int j=0; j < m.cols; j++) {
      *(m.ptr<float>(k)+j) = *(a.ptr<float>(k)+j) - *(l.ptr<float>(k-1)+j)*(*(b.ptr<float>(k-1)+j));
    }

    for (int j=0; j < y.cols; j++) {
      *(y.ptr<float>(k)+j) = *(Ld.ptr<float>(k)+j) - *(l.ptr<float>(k-1)+j)*(*(y.ptr<float>(k-1)+j));
    }
  }

  // 2. Backward substitution U*x = y
  for (int j=0; j < y.cols; j++) {
    *(x.ptr<float>(n-1)+j) = (*(y.ptr<float>(n-1)+j))/(*(m.ptr<float>(n-1)+j));
  }

  for (int i = n-2; i >= 0; i--) {
    for(int j = 0; j < x.cols; j++) {
      *(x.ptr<float>(i)+j) = (*(y.ptr<float>(i)+j) - (*(b.ptr<float>(i)+j))*(*(x.ptr<float>(i+1)+j)))/(*(m.ptr<float>(i)+j));
    }
  }
}

/* ************************************************************************* */
void KAZE::Save_Scale_Space() {

  cv::Mat img_aux;
  char outputFile[500];

  for (size_t i = 0; i < evolution_.size(); i++) {
    convert_scale(evolution_[i].Lt);
    evolution_[i].Lt.convertTo(img_aux, CV_8U, 255.0, 0);
    sprintf(outputFile,"../output/images/nl_evolution_%02ld.jpg",i);
    cv::imwrite(outputFile, img_aux);
  }
}

/* ************************************************************************* */
void KAZE::Save_Detector_Responses() {

  cv::Mat img_aux;
  char outputFile[500];

  for (size_t i = 0; i < evolution_.size(); i++) {
    convert_scale(evolution_[i].Ldet);
    evolution_[i].Ldet.convertTo(img_aux, CV_8U, 255.0, 0);
    sprintf(outputFile,"../output/images/nl_detector_%02ld.jpg",i);
    cv::imwrite(outputFile, img_aux);
  }
}

/* ************************************************************************* */
inline float libKAZE::gaussian(float x, float y, float sig) {
  return exp(-(x*x+y*y)/(2.0f*sig*sig));
}

/* ************************************************************************* */
inline void libKAZE::checkDescriptorLimits(int& x, int& y, const int width, const int height) {

  if (x < 0)
    x = 0;

  if (y < 0)
    y = 0;

  if (x > width-1)
    x = width-1;

  if (y > height-1)
    y = height-1;
}

/* ************************************************************************* */
inline int libKAZE::fRound(const float flt) {
  return (int)(flt+0.5f);
}
