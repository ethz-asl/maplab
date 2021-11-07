#include "semantify-plugin/semantify-common.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace semantify {

DEFINE_double(
    semantify_tracking_confidence_threshold, 0.8,
    "The minimum number of confidence required for the bounding box to be "
    " eligible for tracking by the tracker. Default: 0.8");

DEFINE_double(
    semantify_visualization_frequency, 1,
    "Frequency of the visualization in semantify plugin in Hz.");

std::vector<std::string> mask_rcnn_class_names = {"BG",
                                                  "person",
                                                  "bicycle",
                                                  "car",
                                                  "motorcycle",
                                                  "airplane",
                                                  "bus",
                                                  "train",
                                                  "truck",
                                                  "boat",
                                                  "traffic light",
                                                  "fire hydrant",
                                                  "stop sign",
                                                  "parking meter",
                                                  "bench",
                                                  "bird",
                                                  "cat",
                                                  "dog",
                                                  "horse",
                                                  "sheep",
                                                  "cow",
                                                  "elephant",
                                                  "bear",
                                                  "zebra",
                                                  "giraffe",
                                                  "backpack",
                                                  "umbrella",
                                                  "handbag",
                                                  "tie",
                                                  "suitcase",
                                                  "frisbee",
                                                  "skis",
                                                  "snowboard",
                                                  "sports ball",
                                                  "kite",
                                                  "baseball bat",
                                                  "baseball glove",
                                                  "skateboard",
                                                  "surfboard",
                                                  "tennis racket",
                                                  "bottle",
                                                  "wine glass",
                                                  "cup",
                                                  "fork",
                                                  "knife",
                                                  "spoon",
                                                  "bowl",
                                                  "banana",
                                                  "apple",
                                                  "sandwich",
                                                  "orange",
                                                  "broccoli",
                                                  "carrot",
                                                  "hot dog",
                                                  "pizza",
                                                  "donut",
                                                  "cake",
                                                  "chair",
                                                  "couch",
                                                  "potted plant",
                                                  "bed",
                                                  "dining table",
                                                  "toilet",
                                                  "tv",
                                                  "laptop",
                                                  "mouse",
                                                  "remote",
                                                  "keyboard",
                                                  "cell phone",
                                                  "microwave",
                                                  "oven",
                                                  "toaster",
                                                  "sink",
                                                  "refrigerator",
                                                  "book",
                                                  "clock",
                                                  "vase",
                                                  "scissors",
                                                  "teddy bear",
                                                  "hair drier",
                                                  "toothbrush"};

std::string input_vis_window_name = "input_bb";

bool getSemanticResources(
    const vi_map::VIMap& map, const vi_map::Vertex& vertex, const size_t& idx,
    cv::Mat* image, cv::Mat* image_masks,
    resources::ObjectInstanceBoundingBoxes* boxes) {
  if (!map.getFrameResource(
          vertex, idx, backend::ResourceType::kRawColorImage, image)) {
    VLOG(1) << "Cannot get color image from vertex " << vertex.id()
            << " and frame " << idx;
    return false;
  }
  if (!map.getFrameResource(
          vertex, idx, backend::ResourceType::kObjectInstanceMasks,
          image_masks)) {
    VLOG(1) << "Cannot get the mask image from vertex " << vertex.id()
            << " and frame " << idx;
    return false;
  }
  if (!map.getFrameResource(
          vertex, idx, backend::ResourceType::kObjectInstanceBoundingBoxes,
          boxes)) {
    VLOG(1) << "Cannot get bounding boxes from vertex " << vertex.id()
            << " and frame " << idx;
    return false;
  }
  return true;
}

void visualizeInput(
    const cv::Mat& input_image,
    const resources::ObjectInstanceBoundingBoxes& input_boxes,
    const Eigen::Matrix2Xd& input_keypoints,
    const Eigen::VectorXi& input_track_ids, const cv::Mat& input_mask,
    bool look_up_class_name, bool visualize_mask) {
  cv::Mat image = input_image.clone();
  if (visualize_mask) {
    CHECK(&input_mask);
  }

  std::vector<cv::Vec3b> colors;
  colors.resize(input_boxes.size());
  generateRandomColor(colors);

  for (size_t i = 0; i < input_boxes.size(); i++) {
    auto const& box = input_boxes[i];
    // skips the box if confidence is below threshold
    if (box.confidence < FLAGS_semantify_tracking_confidence_threshold) {
      continue;
    }
    int track_id = input_track_ids(i);
    // cv::Scalar color(255, 255, 255);
    // cv::Scalar color(0, 255, 0);
    cv::Vec3b& color = colors[i];
    cv::rectangle(
        image, box.bounding_box.tl(), box.bounding_box.br(), color, 2);
    // adds text
    std::string class_name;
    if (look_up_class_name) {
      class_name = mask_rcnn_class_names[box.class_number];
    } else {
      class_name = box.class_name;
    }
    cv::putText(
        image, class_name + " class_id: " + std::to_string(box.class_number),
        cv::Point(box.bounding_box.x, box.bounding_box.y - 50),  // Coordinates
        cv::FONT_HERSHEY_COMPLEX_SMALL,                          // Font
        0.5,     // Scale. 2.0 = 2x bigger
        color,   // BGR Color
        0.6,     // Line Thickness (Optional)
        CV_AA);  // Anti-alias (Optional)
    cv::putText(
        image, "score: " + std::to_string(box.confidence),
        cv::Point(box.bounding_box.x, box.bounding_box.y - 30),  // Coordinates
        cv::FONT_HERSHEY_COMPLEX_SMALL,                          // Font
        0.5,    // Scale. 2.0 = 2x bigger
        color,  // BGR Color
        0.6,    // Line Thickness (Optional)
        CV_AA);
    cv::putText(
        image, "track_id: " + std::to_string(track_id),
        cv::Point(box.bounding_box.x, box.bounding_box.y - 10),  // Coordinates
        cv::FONT_HERSHEY_COMPLEX_SMALL,                          // Font
        0.5,    // Scale. 2.0 = 2x bigger
        color,  // BGR Color
        0.6,    // Line Thickness (Optional)
        CV_AA);
    if (visualize_mask) {
      cv::Mat results[3];
      cv::split(input_mask, results);
      cv::Mat class_number_mask = (results[0] == box.class_number);
      cv::Mat instance_number_mask = (results[1] == box.instance_number);
      cv::Mat object_mask(results[0].rows, results[0].cols, CV_8UC1);
      cv::bitwise_and(class_number_mask, instance_number_mask, object_mask);
      applyMask(image, object_mask, color);
    }
  }
  // keypoints visualization
  for (int i = 0; i < input_keypoints.cols(); ++i) {
    double x = input_keypoints(0, i);
    double y = input_keypoints(1, i);
    cv::circle(image, cv::Point(x, y), 1, cv::Scalar(255, 255, 0), 2, CV_AA);
  }
  cv::imshow(input_vis_window_name, image);
}

void destroyVisualizationWindow() {
  cv::destroyWindow(input_vis_window_name);
}

void generateRandomColor(std::vector<cv::Vec3b>& colors) {
  CHECK_GT(colors.size(), 0u);
  float N = static_cast<float>(colors.size());
  for (size_t i = 0; i < colors.size(); i++) {
    cv::Vec3f hsv(i / N * 360, 1.0, 1.0);
    // cvtColor only takes cv::Mat
    cv::Mat_<cv::Vec3f> hsvMat(hsv);
    cv::Mat_<cv::Vec3f> bgrMat;
    cv::cvtColor(hsvMat, bgrMat, CV_HSV2BGR);
    bgrMat *= 255;
    colors[i] = bgrMat(0);
  }
}

void applyMask(cv::Mat& image, const cv::Mat& mask, const cv::Vec3b& color) {
  cv::Mat image_masked;
  image.copyTo(image_masked);
  cv::Mat image_masked_split[3];
  cv::split(image_masked, image_masked_split);
  std::vector<cv::Mat> image_split(3);
  cv::split(image, image_split);
  for (int i = 0; i < 3; i++) {
    image_masked_split[i] = image_masked_split[i] * 0.5 + 0.5 * color[i];
    image_masked_split[i].copyTo(image_split[i], mask);
  }
  cv::merge(image_split, image);
}

}  // namespace semantify
