#include <gflags/gflags.h>
#include <glog/logging.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include "registration-toolbox/common/base-controller.h"
#include "registration-toolbox/common/supported.h"
#include "registration-toolbox/pcl-icp-controller.h"

DEFINE_string(target_cloud, "", "Defines the path to the target cloud.");
DEFINE_string(source_cloud, "", "Defines the path to the source cloud.");
DEFINE_string(reg_cloud, "", "Defines the path to the registered cloud.");

namespace regbox {

static pcl::PointCloud<pcl::PointXYZI>::Ptr readPointCloud(
    const std::string& cloud) {
  CHECK(!cloud.empty());
  LOG(INFO) << "Reading point cloud from " << cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  if (pcl::io::loadPCDFile<pcl::PointXYZI>(cloud, *input_cloud) == -1) {
    LOG(FATAL) << "Unable to load point cloud.";
    return nullptr;
  }
  LOG(INFO) << "Point cloud loaded with size " << input_cloud->size();
  return input_cloud;
}

static void writePointCloud(
    const std::string& reg_file, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) {
  CHECK(!reg_file.empty());
  CHECK_NOTNULL(cloud);
  pcl::io::savePCDFileASCII(reg_file, *cloud);
  LOG(INFO) << "Wrote registered cloud to " << reg_file;
}

static void registerCloud(
    const std::string& target, const std::string& source,
    const std::string& reg_cloud) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud = readPointCloud(target);
  pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = readPointCloud(source);
  CHECK_NOTNULL(target_cloud);
  CHECK_NOTNULL(source_cloud);

  aslam::Transformation prior;
  prior.setIdentity();

  auto aligner =
      regbox::BaseController::make("regbox::PclIcpController", "PCL ICP");
  CHECK_NOTNULL(aligner);
  const regbox::RegistrationResult result =
      aligner->align(target_cloud, source_cloud, prior);

  LOG(INFO) << "Registration result: \n" << result.get_T_target_source();
  writePointCloud(reg_cloud, result.getRegisteredCloud());
}

}  // namespace regbox

int main(int argc, char** argv) {
  ros::init(argc, argv, "registration_toolbox_driver");
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  regbox::registerCloud(
      FLAGS_target_cloud, FLAGS_source_cloud, FLAGS_reg_cloud);

  return 0;
}
