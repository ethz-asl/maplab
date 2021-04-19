#include <lidar-feature-extraction/image-projection.h>
#include <lidar-feature-extraction/ouster-configuration.h>

#include <cmath>
#include <omp.h>

#include <glog/logging.h>
#include <pcl_conversions/pcl_conversions.h>

#define USE_SSE2
#include <lidar-feature-extraction/sse-mathfun-extension.h>
#include <lidar-feature-extraction/vec-helper.h>

DEFINE_double(
    lidar_image_projection_flatness_range, 1,
    "Flatness of the range projection function.");
DEFINE_double(
    lidar_image_projection_close_point, 1.0,
    "Minimal distance that can be mapped onto the image [m].");
DEFINE_double(
    lidar_image_projection_far_point, 50.0,
    "Maximal distance that can be mapped onto the image [m].");
DEFINE_double(
    lidar_image_projection_flatness_intensity, 5,
    "Flatness of the intensity projection function.");
DEFINE_double(
    lidar_image_projection_min_intensity, 2,
    "Minimal intensity that can be mapped onto the image.");
DEFINE_double(
    lidar_image_projection_max_intensity, 3000,
    "Maximal intenisty that can be mapped onto the image [m].");
DEFINE_bool(
    lidar_image_projection_draw, false,
    "Draw the intemediate projection and filtering results.");

namespace LidarFeatureExtraction {
ImageProjection::ImageProjection(
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : T_M_B_buffer_(T_M_B_buffer),
      input_cloud_pointer_(new pcl::PointCloud<pcl::PointXYZI>),
      close_point_(FLAGS_lidar_image_projection_close_point),
      far_point_(FLAGS_lidar_image_projection_far_point),
      flatness_range_(FLAGS_lidar_image_projection_flatness_range),
      min_int_(FLAGS_lidar_image_projection_min_intensity),
      max_int_(FLAGS_lidar_image_projection_max_intensity),
      flatness_intensity_(FLAGS_lidar_image_projection_flatness_intensity) {
  range_image_ = cv::Mat(beam_size_, ring_size_, CV_8U, cv::Scalar::all(0u));
  intensity_image_ =
      cv::Mat(beam_size_, ring_size_, CV_8U, cv::Scalar::all(0u));
  feature_image_ = cv::Mat(beam_size_, ring_size_, CV_8U, cv::Scalar::all(0u));
  inpaint_mask_ = cv::Mat(beam_size_, ring_size_, CV_8U, cv::Scalar::all(0u));

  px_offset_ = getPxOffset(ring_size_);

  // Configure clahe.
  clahe_intensity_ = cv::createCLAHE(6, cv::Size(4, 12));

  // Setup hdr.
  merge_mertens_ = cv::createMergeMertens();

  // Setup horizontal lines filter.
  horizontal_lines_filter_kernel_ = cv::Mat::zeros(5, 3, CV_32F);
  horizontal_lines_filter_kernel_.at<float>(0, 2) = 0.125;
  horizontal_lines_filter_kernel_.at<float>(1, 2) = 0.25;
  horizontal_lines_filter_kernel_.at<float>(2, 2) = 0.25;
  horizontal_lines_filter_kernel_.at<float>(3, 2) = 0.25;
  horizontal_lines_filter_kernel_.at<float>(4, 2) = 0.125;

  // Calculate parameters for range scaling.
  CHECK_NE(close_point_, far_point_);
  CHECK_NE(min_int_, max_int_);
  CHECK_GE(flatness_range_, 0);
  CHECK_GE(flatness_intensity_, 0);
  a_ = 255 /
       log(1 - close_point_ * flatness_range_ + far_point_ * flatness_range_);
  b_ = close_point_ - 1 / flatness_range_;
  c_ = 255 /
       log(1 - min_int_ * flatness_intensity_ + max_int_ * flatness_intensity_);
  d_ = min_int_ - 1 / flatness_intensity_;
}  // namespace LidarFeatureExtraction

bool ImageProjection::projectToImage(
    const vi_map::RosLidarMeasurement::ConstPtr& cloud) {
  CHECK_NOTNULL(cloud);
  convertPointCloudMsgToPcl(cloud);
  projectPointCloud();
  createFeatureImage();
  return true;
}

void ImageProjection::convertPointCloudMsgToPcl(
    const vi_map::RosLidarMeasurement::ConstPtr& cloud) {
  const sensor_msgs::PointCloud2& msg = cloud->getPointCloud();
  pcl::fromROSMsg(msg, *input_cloud_pointer_);
}

std::vector<std::size_t> ImageProjection::getPxOffset(
    const std::size_t lidar_mode) {
  // TODO(lbern): there is probably a STL function that does this much better.
  auto repeat = [](const std::size_t n, std::vector<std::size_t>&& v) {
    std::vector<std::size_t> res{};
    for (std::size_t i = 0u; i < n; ++i)
      res.insert(res.end(), v.begin(), v.end());
    return res;
  };

  switch (lidar_mode) {
    case 512u:
      return repeat(16u, {0u, 3u, 6u, 9u});
    case 1024u:
      return repeat(16u, {0u, 6u, 12u, 18u});
    case 2048u:
      return repeat(16u, {0u, 12u, 24u, 36u});
    default:
      return std::vector<std::size_t>{64u, 0u};
  }
}

void ImageProjection::projectPointCloud() {
  // Example from :
  // https://github.com/ouster-lidar/ouster_example/blob/f3dc5ec292e4bbd260e8bac8d686069b9a3d2ec6/ouster_ros/src/img_node.cpp#L90
  const std::size_t& W = ring_size_;
  const std::size_t& H = beam_size_;

  inpaint_mask_ = 0u;
  for (std::size_t u = 0u; u < H; ++u) {
    const std::size_t offset = px_offset_[u];
    for (std::size_t v = 0u; v < W; ++v) {
      const std::size_t img_index = u * W + v;
      const std::size_t pcl_index = ((v + offset) % W) * H + u;
      const pcl::PointXYZI& curPoint = input_cloud_pointer_->points[pcl_index];
      const float squaredXY = curPoint.x * curPoint.x + curPoint.y * curPoint.y;
      const float range =
          sqrt(squaredXY + curPoint.z * curPoint.z) <= close_point_
              ? 0.0f
              : a_ * log((range - b_) * flatness_range_);

      // Scale range to fit close_point_ [m] to far_point_ [m] (logarithmically)
      // into 8 bit image.
      if (range <= 0) {
        range_image_.data[img_index] = 0;
        inpaint_mask_.data[img_index] = 255u;
      } else {
        range_image_.data[img_index] =
            255u - static_cast<uint8_t>(std::min(std::round(range), 255.0f));
      }

      // Scale intenisty to fit min_int_ to max_int_ (logarithmicly)
      // into 8 bit image.
      const float intensity =
          curPoint.intensity <= min_int_
              ? 0.0f
              : c_ * log((curPoint.intensity - d_) * flatness_intensity_);
      intensity_image_.data[img_index] =
          static_cast<uint8_t>(std::min(std::round(intensity), 255.f));
    }
  }
}

const cv::Mat& ImageProjection::getRangeImage() const {
  return range_image_;
}

const cv::Mat& ImageProjection::getIntensityImage() const {
  return intensity_image_;
}

const cv::Mat& ImageProjection::getFeatureImage() const {
  return feature_image_;
}

void ImageProjection::createFeatureImage() {
  CHECK_NOTNULL(clahe_intensity_);
  CHECK_NOTNULL(merge_mertens_);

  // Perform histogram equalization on the intensity image.
  cv::Mat intensity_image_eq;
  clahe_intensity_->apply(intensity_image_, intensity_image_eq);

  // Remove horizontal lines from intensity image.
  cv::filter2D(
      intensity_image_eq, intensity_image_eq, CV_8U,
      horizontal_lines_filter_kernel_);

  cv::Mat inpainted_range_image;
  constexpr double inpaint_radius = 5.0;
  cv::inpaint(
      range_image_, inpaint_mask_, inpainted_range_image, inpaint_radius,
      cv::INPAINT_TELEA);

  cv::GaussianBlur(
      inpainted_range_image, inpainted_range_image, cv::Size(5, 5), 1, 1,
      cv::BORDER_DEFAULT);

  cv::Mat range_grad_x, range_grad_y;
  cv::Mat range_abs_grad_x, range_abs_grad_y;
  const int scale = 1;
  const int delta = 0;

  /// Gradient X
  cv::Sobel(
      inpainted_range_image, range_grad_x, CV_8U, 1, 0, 3, scale, delta,
      cv::BORDER_DEFAULT);
  /// Gradient Y
  cv::Sobel(
      inpainted_range_image, range_grad_y, CV_8U, 0, 1, 3, scale, delta,
      cv::BORDER_DEFAULT);

  cv::convertScaleAbs(range_grad_x, range_abs_grad_x);
  cv::convertScaleAbs(range_grad_y, range_abs_grad_y);

  cv::Mat range_gradient;
  addWeighted(range_abs_grad_x, 0.5, range_abs_grad_y, 0.5, 0, range_gradient);

  // Merge the two images into one HDR image.
  std::vector<cv::Mat> images;
  images.emplace_back(std::move(range_gradient));
  images.emplace_back(std::move(intensity_image_eq));

  cv::Mat hdr_image;
  merge_mertens_->process(images, hdr_image);
  hdr_image = hdr_image * 255u;
  hdr_image.convertTo(feature_image_, CV_8UC1);
}

pcl::PointCloud<pcl::PointXYZI>::ConstPtr ImageProjection::getPointcloud() {
  if (!old_cloud_.empty()) {
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr old_cloud_pointer_ =
        old_cloud_.makeShared();
    old_cloud_ = *input_cloud_pointer_;
    return old_cloud_pointer_;
  } else {
    old_cloud_ = *input_cloud_pointer_;
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr old_cloud_pointer_ =
        old_cloud_.makeShared();
    return input_cloud_pointer_;
  }
}

}  // namespace LidarFeatureExtraction
