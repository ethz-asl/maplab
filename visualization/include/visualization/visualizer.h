#ifndef VISUALIZATION_VISUALIZER_H_
#define VISUALIZATION_VISUALIZER_H_

#include <string>
#include <vector>

#include <Eigen/Dense>
#include <aslam/cameras/ncamera.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <vi-map/vi-map.h>

namespace visualization {

class Visualizer {
 public:
  explicit Visualizer(vi_map::VIMap* map);
  bool visualizeCvMatResources(backend::ResourceType type);
  typedef std::unordered_map<aslam::CameraId, aslam::Camera::Ptr> CameraCache;

 private:
  void getOpenCvWindowsForNCamera(
      const aslam::NCamera& n_camera, std::vector<std::string>* named_windows);

  void destroyAllWindows(const std::vector<std::string>& windows_names);

 private:
  vi_map::VIMap* map_;
};

}  // namespace visualization

#endif  // VISUALIZATION_VISUALIZER_H_
