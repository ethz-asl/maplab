#ifndef LIDAR_FEATURE_EXTRACTION_LOAM_FEATURES_H_
#define LIDAR_FEATURE_EXTRACTION_LOAM_FEATURES_H_

#include <vio-common/pose-lookup-buffer.h>
#include <sensors/lidar.h>

#include <opencv/cv.h>

#include <vector>
#include <utility>

namespace LidarFeatureExtraction {

struct CloudInformation {
  int start_ring_index;
  int end_ring_index;
  float range;
  std::size_t col_index;
};

class LidarFeatureExtraction {
  public:
    explicit LidarFeatureExtraction
      (const vio_common::PoseLookupBuffer& T_M_B_buffer);

    bool extractLidarFeatures(
        const vi_map::RosLidarMeasurement::ConstPtr& cloud);

  private:
    void convertPointCloudMsg(
        const vi_map::RosLidarMeasurement::ConstPtr& cloud);
    void projectPointCloud(); 
    void cloudSegmentation();
    void calculateSmoothness();
    void markOccludedPoints();
    void extractFeatures();

    const vio_common::PoseLookupBuffer& T_M_B_buffer_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr corner_features_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr surf_features_;

    using smoothness_t = std::pair<float, std::size_t>;
    std::vector<smoothness_t> cloudSmoothness_;
    std::vector<CloudInformation> cloud_info_;
    std::vector<bool> neighbor_picked_;

    cv::Mat rangeMat_;
};

}

#endif
