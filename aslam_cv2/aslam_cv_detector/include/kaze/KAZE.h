/**
 * @file KAZE.h
 * @brief Main program for detecting and computing descriptors in a nonlinear
 * scale space
 * @date Dec 11, 2014
 * @author Pablo F. Alcantarilla
 */

#pragma once

/* ************************************************************************* */
#include "KAZEConfig.h"
#include "nldiffusion_functions.h"
#include "fed.h"
#include "utils.h"

/* ************************************************************************* */
namespace libKAZE {

  /// KAZE Timing structure
  struct KAZETiming {

    KAZETiming() {
      kcontrast = 0.0;
      scale = 0.0;
      derivatives = 0.0;
      detector = 0.0;
      extrema = 0.0;
      subpixel = 0.0;
      descriptor = 0.0;
    }

    double kcontrast;       ///< Contrast factor computation time in ms
    double scale;           ///< Nonlinear scale space computation time in ms
    double derivatives;     ///< Multiscale derivatives computation time in ms
    double detector;        ///< Feature detector computation time in ms
    double extrema;         ///< Scale space extrema computation time in ms
    double subpixel;        ///< Subpixel refinement computation time in ms
    double descriptor;      ///< Descriptors computation time in ms
  };

  /// KAZE Class Declaration
  class KAZE {

  private:

    KAZEOptions options_;                ///< Configuration options for AKAZE
    std::vector<TEvolution> evolution_;	/// Vector for nonlinear diffusion evolution

    /// Vector of keypoint vectors for finding extrema in multiple threads
    std::vector<std::vector<cv::KeyPoint> > kpts_par_;

    /// FED parameters
    int ncycles_;                  ///< Number of cycles
    bool reordering_;              ///< Flag for reordering time steps
    std::vector<std::vector<float > > tsteps_;  ///< Vector of FED dynamic time steps
    std::vector<int> nsteps_;      ///< Vector of number of steps per cycle

    /// Some auxiliary variables used in the AOS step
    cv::Mat Ltx_, Lty_, px_, py_, ax_, ay_, bx_, by_, qr_, qc_;

    /// Computation times variables in ms
    KAZETiming timing_;

  public:

    /// KAZE constructor with input options
    /// @param options KAZE configuration options
    /// @note The constructor allocates memory for the nonlinear scale space
    KAZE(KAZEOptions& options);

    /// Destructor
    ~KAZE();

    /// Allocates the memory for the nonlinear scale space
    void Allocate_Memory_Evolution();

    /// This method creates the nonlinear scale space for a given image
    /// @param img Input image for which the nonlinear scale space needs to be created
    /// @return 0 if the nonlinear scale space was created successfully. -1 otherwise
    int Create_Nonlinear_Scale_Space(const cv::Mat& img);

    /// This method selects interesting keypoints with local-maximum response in the nonlinear scale space
    /// @param kpts Vector of keypoints
    void Feature_Detection(std::vector<cv::KeyPoint>& kpts);

    /// This method computes the descriptors in the nonlinear scale space
    /// @param kpts Vector of keypoints
    /// @param desc Matrix with the feature descriptors
    void Compute_Descriptors(std::vector<cv::KeyPoint>& kpts, cv::Mat& desc);

    /// This method saves the nonlinear scale space into jpg images for visualization or debugging purposes
    void Save_Scale_Space();

    /// This method saves the feature detector responses of the nonlinear scale space for visualization or debugging purposes
    void Save_Detector_Responses();

  private:

    /// This method computes the k contrast factor
    /// @param img Input image
    /// @param kpercentile Percentile of the gradient histogram
    void Compute_KContrast(const cv::Mat& img);

    /// This method computes the multiscale derivatives for the nonlinear scale space
    void Compute_Multiscale_Derivatives();

    /// This method computes the feature detector response for the nonlinear scale space
    /// @note We use the Hessian determinant as feature detector
    void Compute_Detector_Response();

    /// This method performs the detection of keypoints by using the normalized score of the Hessian determinant
    /// @param kpts Vector of keypoints
    /// @note We compute features for each of the nonlinear scale space level in a different processing thread
    void Determinant_Hessian_Parallel(std::vector<cv::KeyPoint>& kpts);

    /// This method is called by the thread which is responsible of finding extrema at a given nonlinear scale level
    /// @param level Index in the nonlinear scale space evolution
    void Find_Extremum_Threading(const int level);

    /// This method performs subpixel refinement of the detected keypoints
    /// @param kpts Vector of detected keypoints
    void Do_Subpixel_Refinement(std::vector<cv::KeyPoint>& kpts);

    /// This method performs a scalar non-linear diffusion step using AOS schemes
    /// @param Ld Image at a given evolution step
    /// @param Ldprev Image at a previous evolution step
    /// @param c Conductivity image
    /// @param stepsize Stepsize for the nonlinear diffusion evolution
    /// @note If c is constant, the diffusion will be linear
    /// If c is a matrix of the same size as Ld, the diffusion will be nonlinear
    /// The stepsize can be arbitrarilly large
    void AOS_Step_Scalar(cv::Mat& Ld, const cv::Mat& Ldprev, const cv::Mat& c, const float stepsize);

    /// This method performs performs 1D-AOS for the image rows
    /// @param Ldprev Image at a previous evolution step
    /// @param c Conductivity image
    /// @param stepsize Stepsize for the nonlinear diffusion evolution
    void AOS_Rows(const cv::Mat& Ldprev, const cv::Mat& c, const float stepsize);

    /// This method performs performs 1D-AOS for the image columns
    /// @param Ldprev Image at a previous evolution step
    /// @param c Conductivity image
    /// @param stepsize Stepsize for the nonlinear diffusion evolution
    void AOS_Columns(const cv::Mat& Ldprev, const cv::Mat& c, const float stepsize);

    /// This method does the Thomas algorithm for solving a tridiagonal linear system
    /// @note The matrix A must be strictly diagonally dominant for a stable solution
    void Thomas(const cv::Mat& a, const cv::Mat& b, const cv::Mat& Ld, cv::Mat& x);

    /// Compute the main orientation for a given keypoint
    /// @param kpt Input keypoint
    /// @note The orientation is computed using a similar approach as described in the
    /// original SURF method. See Bay et al., Speeded Up Robust Features, ECCV 2006
    void Compute_Main_Orientation(cv::KeyPoint& kpt);

    /// These methods compute the different descriptors (upright, rotation invariant, extended) of the provided keypoint using first
    /// order derivatives from the nonlinear scale space using an inspired-SURF descriptor
    ///  @param kpt Input keypoint
    ///  @param desc Descriptor vector
    ///  @note Rectangular grid of 20 s x 20 s. Descriptor Length 64. No additional
    ///  Gaussian weighting is performed. The descriptor is inspired from Bay et al.,
    ///  Speeded Up Robust Features, ECCV, 2006
    void Get_SURF_Upright_Descriptor_64(const cv::KeyPoint& kpt, float* desc);
    void Get_SURF_Descriptor_64(const cv::KeyPoint& kpt, float* desc);
    void Get_SURF_Upright_Descriptor_128(const cv::KeyPoint& kpt, float* desc);
    void Get_SURF_Descriptor_128(const cv::KeyPoint& kpt, float* desc);

    /// These methods compute the different descriptors (upright, rotation invariant, extended) of the provided keypoint using first
    /// order derivatives from the nonlinear scale space using an inspired-MSURF descriptor
    /// @param kpt Input keypoint
    /// @param desc Descriptor vector
    /// @note Rectangular grid of 24 s x 24 s. Descriptor Length 64. The descriptor is inspired
    /// from Agrawal et al., CenSurE: Center Surround Extremas for Realtime Feature Detection and Matching, ECCV 2008
    void Get_MSURF_Upright_Descriptor_64(const cv::KeyPoint& kpt, float* desc);
    void Get_MSURF_Descriptor_64(const cv::KeyPoint& kpt, float* desc);
    void Get_MSURF_Upright_Descriptor_128(const cv::KeyPoint& kpt, float* desc);
    void Get_MSURF_Descriptor_128(const cv::KeyPoint& kpt, float* desc);

    /// These methods compute the different descriptors (upright, rotation invariant, extended) of the provided keypoint using first
    /// order derivatives from the nonlinear scale space using an inspired-GSURF descriptor
    /// @param kpt Input keypoint
    /// @param desc Descriptor vector
    /// @note Rectangular grid of 20 s x 20 s. Descriptor Length 64. No additional
    /// G-SURF descriptor as described in Pablo F. Alcantarilla, Luis M. Bergasa and
    /// Andrew J. Davison, Gauge-SURF Descriptors, Image and Vision Computing 31(1), 2013
    void Get_GSURF_Upright_Descriptor_64(const cv::KeyPoint& kpt, float* desc);
    void Get_GSURF_Descriptor_64(const cv::KeyPoint& kpt, float* desc);
    void Get_GSURF_Upright_Descriptor_128(const cv::KeyPoint& kpt, float* desc);
    void Get_GSURF_Descriptor_128(const cv::KeyPoint& kpt, float* desc);

  public:

    /// Return the computation times
    KAZETiming Get_Computation_Times() const {
      return timing_;
    }
  };

  /* ************************************************************************* */
  /// This function computes the value of a 2D Gaussian function
  float gaussian(float x, float y, float sig);

  /// This function checks descriptor limits
  void checkDescriptorLimits(int& x, int& y, const int width, const int height);

  /// This funtion rounds float to nearest integer
  int fRound(const float flt);
}
