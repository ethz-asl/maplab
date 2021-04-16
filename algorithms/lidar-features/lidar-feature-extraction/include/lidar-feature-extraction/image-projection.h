#ifndef LIDAR_FEATURE_EXTRACTION_IMAGE_PROJECTION_H_
#define LIDAR_FEATURE_EXTRACTION_IMAGE_PROJECTION_H_

#include <sensors/lidar.h>
#include <vio-common/pose-lookup-buffer.h>

#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/photo/photo.hpp>

#include <utility>
#include <vector>

namespace LidarFeatureExtraction {

class ImageProjection {
 public:
  explicit ImageProjection(const vio_common::PoseLookupBuffer& T_M_B_buffer);

  bool projectToImage(const vi_map::RosLidarMeasurement::ConstPtr& cloud);

  const cv::Mat& getRangeImage() const;
  const cv::Mat& getIntensityImage() const;
  const cv::Mat& getFeatureImage() const;
  pcl::PointCloud<pcl::PointXYZI>::ConstPtr getPointcloud();

 private:
  void convertPointCloudMsgToPcl(
      const vi_map::RosLidarMeasurement::ConstPtr& cloud);
  void projectPointCloud();
  std::vector<std::size_t> getPxOffset(const std::size_t lidar_mode);
  void createFeatureImage();

  const vio_common::PoseLookupBuffer& T_M_B_buffer_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_pointer_;
  pcl::PointCloud<pcl::PointXYZI> old_cloud_;
  cv::Ptr<cv::CLAHE> clahe_range_;
  cv::Ptr<cv::CLAHE> clahe_intensity_;
  cv::Ptr<cv::CLAHE> clahe_hdr_;

  cv::Ptr<cv::MergeMertens> merge_mertens_;
  cv::Mat range_image_;
  cv::Mat intensity_image_;
  cv::Mat feature_image_;
  cv::Mat inpaint_mask_;

  // Parameters for range and signal scaling.
  const float flatness_range_;
  const float close_point_;
  const float far_point_;
  const float flatness_intensity_;
  const float max_int_;
  const float min_int_;
  float a_;
  float b_;
  float c_;
  float d_;

  // Filter for horizontal lines.
  cv::Mat horizontal_lines_filter_kernel_;

  std::vector<std::size_t> px_offset_;
};

}  // namespace LidarFeatureExtraction

#endif
