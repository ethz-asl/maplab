#include <lidar-feature-extraction/loam-features.h>
#include <lidar-feature-extraction/ouster-configuration.h>

#define USE_SSE2
#include <lidar-feature-extraction/sse-mathfun-extension.h>
#include <lidar-feature-extraction/vec-helper.h>

#include <pcl_conversions/pcl_conversions.h>

namespace LidarFeatureExtraction {

LidarFeatureExtraction::LidarFeatureExtraction(
    const vio_common::PoseLookupBuffer& T_M_B_buffer)
    : T_M_B_buffer_(T_M_B_buffer) {
  corner_features_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  surf_features_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
}

bool LidarFeatureExtraction::extractLidarFeatures(
    const vi_map::RosLidarMeasurement::ConstPtr& cloud) {
  convertPointCloudMsg(cloud);
  projectPointCloud();
  cloudSegmentation();
  calculateSmoothness();
  extractFeatures();

  return true;
}

void LidarFeatureExtraction::convertPointCloudMsg(
    const vi_map::RosLidarMeasurement::ConstPtr& cloud) {
  const sensor_msgs::PointCloud2& msg = cloud->getPointCloud();
  pcl::fromROSMsg(msg, *input_cloud_);
}


void LidarFeatureExtraction::projectPointCloud() {
  std::size_t cloudSize = input_cloud_->points.size();
  __m128 lowerVecXY = _mm_setzero_ps();
  __m128 upperVecXY = _mm_setzero_ps();
  __m128 vecX = _mm_setzero_ps();
  __m128 vecY = _mm_setzero_ps();
  __m128 vecZ = _mm_setzero_ps();
  __m128 verticalAngle = _mm_setzero_ps();
  __m128 horizonAngle = _mm_setzero_ps();
  __m128 sumSquaredXY = _mm_setzero_ps();
  __m128 rowIdn = _mm_setzero_ps();
  __m128 columnIdn = _mm_setzero_ps();
  __m128 range = _mm_setzero_ps();
  __m128 largerThanHorizon = _mm_setzero_ps();
  __m128 validRow = _mm_setzero_ps();
  __m128 intensity = _mm_setzero_ps();
  __m128 index = _mm_setzero_ps();
  uint32_t cond = 0;
  std::size_t i = 0;
  for (; i < cloudSize; i += 4) {
    const auto& point1 = input_cloud_->points[i];
    const auto& point2 = input_cloud_->points[i + 1];
    const auto& point3 = input_cloud_->points[i + 2];
    const auto& point4 = input_cloud_->points[i + 3];
    lowerVecXY[0] = point1.x;
    lowerVecXY[1] = point1.y;
    lowerVecXY[2] = point2.x;
    lowerVecXY[3] = point2.y;
    upperVecXY[0] = point3.x;
    upperVecXY[1] = point3.y;
    upperVecXY[2] = point4.x;
    upperVecXY[3] = point4.y;
    vecX[0] = point1.x;
    vecX[1] = point2.x;
    vecX[2] = point3.x;
    vecX[3] = point4.x;
    vecY[0] = point1.y;
    vecY[1] = point2.y;
    vecY[2] = point3.y;
    vecY[3] = point4.y;
    vecZ[0] = point1.z;
    vecZ[1] = point2.z;
    vecZ[2] = point3.z;
    vecZ[3] = point4.z;

    // Compute the vertical angle alpha, i.e.
    // alpha = atan2(z, sqrt(x^2 + y^2)).
    sumSquaredXY = _mm_hadd_ps(
        _mm_mul_ps(lowerVecXY, lowerVecXY), _mm_mul_ps(upperVecXY, upperVecXY));
    verticalAngle = rad2deg_ps(atan2_ps(vecZ, _mm_sqrt_ps(sumSquaredXY)));

    rowIdn = _mm_round_ps(
        _mm_div_ps(_mm_add_ps(verticalAngle, ang_bottom_ps), ang_res_y_ps),
        _MM_FROUND_TO_ZERO);

    horizonAngle = rad2deg_ps(atan2_ps(vecX, vecY));
    columnIdn = _mm_sub_ps(
        horizon_scan_half_ps,
        _mm_round_ps(
            _mm_div_ps(_mm_sub_ps(horizonAngle, degree90_ps), ang_res_x_ps),
            _MM_FROUND_TO_ZERO));

    // columnIdn = columnIdn >= Horizon_SCAN ?
    //   columnIdn - Horizon_SCAN : columnIdn
    largerThanHorizon = _mm_cmpge_ps(columnIdn, horizon_scan_ps);
    columnIdn = _mm_blendv_ps(
        columnIdn, _mm_sub_ps(columnIdn, horizon_scan_ps), largerThanHorizon);

    range = _mm_sqrt_ps(_mm_add_ps(sumSquaredXY, _mm_mul_ps(vecZ, vecZ)));
    intensity = _mm_add_ps(rowIdn, _mm_div_ps(columnIdn, tenThousand_ps));

    validRow = _mm_and_ps(
        _mm_cmpge_ps(rowIdn, zero_ps), _mm_cmplt_ps(rowIdn, n_scan_ps));
    cond = _mm_movemask_epi8(_mm_castps_si128(validRow));
    index = _mm_add_ps(columnIdn, _mm_mul_ps(rowIdn, horizon_scan_ps));

    if (cond & 0x000F) {
      rangeMat_.at<float>(rowIdn[0], columnIdn[0]) = range[0];
    }
    if (cond & 0x00F0) {
      rangeMat_.at<float>(rowIdn[1], columnIdn[1]) = range[1];
    }
    if (cond & 0x0F00) {
      rangeMat_.at<float>(rowIdn[2], columnIdn[2]) = range[2];
    }
    if (cond & 0xF000) {
      rangeMat_.at<float>(rowIdn[3], columnIdn[3]) = range[3];
    }
  }

  // Process the remaining points in the cloud sequentially.
  float verticalAngleSeq, horizonAngleSeq, rangeSeq;
  std::size_t rowIdnSeq, columnIdnSeq, indexSeq;
  for (; i < cloudSize; ++i) {
    auto& curPoint = input_cloud_->points[i];
    const auto squaredXY = curPoint.x * curPoint.x + curPoint.y * curPoint.y;

    verticalAngleSeq = atan2(curPoint.z, sqrt(squaredXY)) * 180 / M_PI;
    rowIdnSeq = (verticalAngleSeq + ang_bottom_) / ang_res_y_;
    if (rowIdnSeq < 0 || rowIdnSeq >= beam_size_)
      continue;

    horizonAngleSeq = atan2(curPoint.x, curPoint.y) * 180 / M_PI;

    columnIdnSeq =
        ring_size_ / 2 - round((horizonAngleSeq - 90.0) / ang_res_x_);
    if (columnIdnSeq >= ring_size_)
      columnIdnSeq -= ring_size_;

    rangeSeq = sqrt(squaredXY + curPoint.z * curPoint.z);
    rangeMat_.at<float>(rowIdnSeq, columnIdnSeq) = rangeSeq;
  }
}

void LidarFeatureExtraction::calculateSmoothness() {
  cloudSmoothness_.resize(beam_size_ * ring_size_);
  const std::size_t cloud_size = input_cloud_->points.size() - 5u;
  for (std::size_t i = 5u; i < cloud_size; ++i) {
    const float diff_range = cloud_info_[i - 5u].range
      + cloud_info_[i - 4u].range + cloud_info_[i - 3u].range
      + cloud_info_[i - 2u].range + cloud_info_[i - 1u].range
      - cloud_info_[i].range * 10 + cloud_info_[i + 1u].range
      + cloud_info_[i + 2u].range + cloud_info_[i + 3u].range
      + cloud_info_[i + 4u].range + cloud_info_[i + 5u].range;
    cloudSmoothness_[i].first = diff_range * diff_range;
    cloudSmoothness_[i].second = i;
  }
}

void LidarFeatureExtraction::cloudSegmentation() {
  int sizeOfSegCloud = 0;
  cloud_info_.resize(beam_size_ * ring_size_);
  for (size_t i = 0; i < beam_size_; ++i) {
    cloud_info_[i].start_ring_index = sizeOfSegCloud - 1 + 5;
    for (size_t j = 0; j < ring_size_; ++j) {

      cloud_info_[sizeOfSegCloud].range = rangeMat_.at<float>(i, j);
      cloud_info_[sizeOfSegCloud].col_index = j;
      ++sizeOfSegCloud;
    }

    cloud_info_[i].end_ring_index = sizeOfSegCloud - 1 - 5;
  }
}

void LidarFeatureExtraction::markOccludedPoints() {
  neighbor_picked_.resize(beam_size_ * ring_size_);
  const std::size_t cloud_size = input_cloud_->points.size() - 6u;
  for (std::size_t i = 5u; i < cloud_size; ++i) {
    const float depth1 = cloud_info_[i].range;
    const float depth2 = cloud_info_[i+1].range;
    const std::size_t column_diff = std::abs(
        int(cloud_info_[i+1].col_index - cloud_info_[i].col_index));

    if (column_diff < 10) {
      if (depth1 - depth2 > 0.3) {
        neighbor_picked_[i - 5] = true;
        neighbor_picked_[i - 4] = true;
        neighbor_picked_[i - 3] = true;
        neighbor_picked_[i - 2] = true;
        neighbor_picked_[i - 1] = true;
        neighbor_picked_[i] = 1;
      } else if (depth2 - depth1 > 0.3) {
        neighbor_picked_[i + 1] = true;
        neighbor_picked_[i + 2] = true;
        neighbor_picked_[i + 3] = true;
        neighbor_picked_[i + 4] = true;
        neighbor_picked_[i + 5] = true;
        neighbor_picked_[i + 6] = true;
      }
    }
    const float current_range = cloud_info_[i].range;
    const float diff1 = std::abs(cloud_info_[i-1].range - current_range);
    const float diff2 = std::abs(cloud_info_[i+1].range - current_range);

    if (diff1 > 0.02 * current_range && diff2 > 0.02 * current_range) {
      neighbor_picked_[i] = true;
    }
  }
}

void LidarFeatureExtraction::extractFeatures() {
  corner_features_->clear();
  surf_features_->clear();

  for (unsigned int i = 0u; i < beam_size_; ++i) {
    const std::size_t start_ring_index = cloud_info_[i].start_ring_index;
    const std::size_t end_ring_index = cloud_info_[i].end_ring_index;
    for (std::size_t j = 0; j < grid_; ++j) {
      int sp = (start_ring_index * (grid_ - j) + end_ring_index * j) / 6;
      int ep = (start_ring_index * (grid_ - 1 - j) 
          + end_ring_index * (j + 1)) / 6 - 1;

      if (sp >= ep)
        continue;
      std::sort(cloudSmoothness_.begin() + sp, cloudSmoothness_.begin() + ep, 
          [] (const smoothness_t &lhs, const smoothness_t &rhs) {
            return lhs.first < rhs.first;
          });

      int largestPickedNum = 0;
      for (int k = ep; k >= sp; --k) {
        const std::size_t ind = cloudSmoothness_[k].second;
      }

    }
  }
}

}  // namespace LidarFeatureExtraction
