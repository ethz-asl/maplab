/**
 * @file utils.h
 * @brief Some useful functions
 * @date Dec 29, 2011
 * @author Pablo F. Alcantarilla
 */

#pragma once

/* ************************************************************************* */
// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// System
#include <string>
#include <vector>

/* ************************************************************************* */
/// Computes the minimum value of a float image
void compute_min_32F(const cv::Mat& src, float& value);

/// Compute the maximum value of a float image
void compute_max_32F(const cv::Mat& src, float& value);

/// This function converts the scale of the input image prior to visualization
/// @param src Input/Output image
/// @param value Maximum value
void convert_scale(cv::Mat& src);

/// This function copies the input image and converts the scale of the copied image prior visualization
void copy_and_convert_scale(const cv::Mat& src, cv::Mat& dst);

/// This function draws the list of detected keypoints
void draw_keypoints(cv::Mat& img, const std::vector<cv::KeyPoint>& kpts);

///  This function saves the interest points to a regular ASCII file
/// @note The format is compatible with Mikolajczy and Schmid evaluation
/// @param sFileName Name of the output file where the points will be stored
/// @param kpts Vector of points of interest
/// @param desc Descriptors
/// @param bLaplacian Set to 1 if we want to write the sign of the Laplacian
/// into the descriptor information
/// @param bVerbose Set to 1 for some verbosity information
int save_keypoints(std::string& keypointsFile, const std::vector<cv::KeyPoint>& kpts, const cv::Mat& desc, bool bVerbose);

/// This function converts matches to points using nearest neighbor distance
/// ratio matching strategy
/// @param train Vector of keypoints from the first image
/// @param query Vector of keypoints from the second image
/// @param matches Vector of nearest neighbors for each keypoint
/// @param pmatches Vector of putative matches
/// @param nndr Nearest neighbor distance ratio value
void matches2points_nndr(const std::vector<cv::KeyPoint>& train, const std::vector<cv::KeyPoint>& query,
                         const std::vector<std::vector<cv::DMatch> >& matches,
                         std::vector<cv::Point2f>& pmatches, const float nndr);

/// This function computes the set of inliers estimating the fundamental matrix
/// or a planar homography in a RANSAC procedure
/// @param matches Vector of putative matches
/// @param inliers Vector of inliers
/// @param error The minimum pixelic error to accept an inlier
/// @param use_fund Set to true if you want to compute a fundamental matrix
void compute_inliers_ransac(const std::vector<cv::Point2f>& matches, std::vector<cv::Point2f>& inliers,
                            const float error, const bool use_fund);

/// This function computes the set of inliers given a ground truth homography
///  @param matches Vector of putative matches
///  @param inliers Vector of inliers
///  @param H Ground truth homography matrix 3x3
///  @param min_error The minimum pixelic error to accept an inlier
void compute_inliers_homography(const std::vector<cv::Point2f>&matches, std::vector<cv::Point2f>& inliers,
                                const cv::Mat& H, const float min_error);


/// This function draws the set of the inliers between the two images
/// @param img1 First image
/// @param img2 Second image
/// @param img_com Image with the inliers
/// @param ptpairs Vector of point pairs with the set of inliers
void draw_inliers(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& img_com,
                  const std::vector<cv::Point2f>& ptpairs);

/// This function draws the set of the inliers between the two images
/// @param img1 First image
/// @param img2 Second image
/// @param img_com Image with the inliers
/// @param ptpairs Vector of point pairs with the set of inliers
/// @param color The color for each method
void draw_inliers(const cv::Mat& img1, const cv::Mat& img2, cv::Mat& img_com,
                  const std::vector<cv::Point2f>& ptpairs, int color);

/// Function for reading the ground truth homography from a txt file
/// @param homography_file Path for the file that contains the ground truth homography
/// @param HG Matrix to store the ground truth homography
bool read_homography(const std::string& homography_path, cv::Mat& H1toN);

/// This function shows the possible command line configuration options
void show_input_options_help(int example);

/// This function displays text in the image with the matching statistics
void display_text(cv::Mat& img_rgb, int npoints1, int npoints2, int nmatches,
                  int ninliers, float ratio, float dratio, int index);
