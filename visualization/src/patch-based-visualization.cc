#include "visualization/patch-based-visualization.h"

#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "visualization/color.h"

namespace visualization {

static const int kBoundingBoxWidthPixels = 2;
static const int kKeypointCircleRadiusPixels = 3;

void drawCenterCircleOntoPatch(cv::Mat* patch) {
  CHECK_NOTNULL(patch);
  const int kPatchMinSideLengthPixels = (2 * kKeypointCircleRadiusPixels);
  int patch_width = patch->cols;
  int patch_height = patch->rows;
  CHECK_GT(patch_width, kPatchMinSideLengthPixels);
  CHECK_GT(patch_height, kPatchMinSideLengthPixels);
  double center_x =
      static_cast<double>(std::floor(static_cast<double>(patch_width) / 2.0));
  double center_y =
      static_cast<double>(std::floor(static_cast<double>(patch_height) / 2.0));

  cv::circle(
      *patch, cv::Point2d(center_x, center_y), kKeypointCircleRadiusPixels,
      kCvGreen);
}

void drawBoundingBoxOntoPatch(cv::Mat* patch) {
  CHECK_NOTNULL(patch);
  cv::rectangle(
      *patch, cv::Point2d(0.0, 0.0),
      cv::Point2d(
          static_cast<double>(patch->cols), static_cast<double>(patch->rows)),
      kCvRed, kBoundingBoxWidthPixels);
}

bool getImagePatch(
    const cv::Mat& image, int patch_center_x, int patch_center_y,
    int half_patch_size_pixels, cv::Mat* image_patch) {
  CHECK_NOTNULL(image_patch);
  CHECK_NOTNULL(image.data);
  CHECK_GT(half_patch_size_pixels, 0)
      << "The half patch size needs ot be positive. You provided "
      << half_patch_size_pixels;

  const int image_height = image.rows;
  const int image_width = image.cols;

  // The patch center must lie within the inner range of the image such that
  // the whole patch can be extracted.
  if ((patch_center_x >= half_patch_size_pixels) &&
      (patch_center_y >= half_patch_size_pixels) &&
      (patch_center_x < image_width - half_patch_size_pixels) &&
      (patch_center_y < image_height - half_patch_size_pixels)) {
    const int patch_upper_left_x = patch_center_x - half_patch_size_pixels;
    const int patch_upper_left_y = patch_center_y - half_patch_size_pixels;

    const size_t patch_size_pixels = (2 * half_patch_size_pixels) + 1;
    image_patch->create(patch_size_pixels, patch_size_pixels, CV_8UC3);

    image(
        cv::Rect(
            patch_upper_left_x, patch_upper_left_y, patch_size_pixels,
            patch_size_pixels))
        .copyTo(*image_patch);
    return true;
  } else {
    // Patch outside valid image zone.
    return false;
  }
}

void createColorImageFromImage(const cv::Mat& image, cv::Mat* color_image) {
  CHECK_NOTNULL(color_image);

  const int image_height = image.rows;
  const int image_width = image.cols;
  color_image->create(image_height, image_width, CV_8UC3);

  switch (image.type()) {
    case CV_8UC1:
      cv::cvtColor(image, *color_image, CV_GRAY2BGR);
      break;
    case CV_8UC3:
      image.copyTo(*color_image);
      break;
    default:
      LOG(FATAL) << "Unhandled CV::Mat image type: " << image.type();
  }
}

void createBloatedColorImageFromColorImage(
    const cv::Mat& color_image, int border_size_pixels,
    cv::Mat* bloated_color_image) {
  CHECK_NOTNULL(bloated_color_image);
  CHECK_EQ(color_image.type(), CV_8UC3)
      << "The given image is not a color image.";

  const int image_height = color_image.rows;
  const int image_width = color_image.cols;
  const int bloated_image_height = image_height + (2 * border_size_pixels);
  const int bloated_image_width = image_width + (2 * border_size_pixels);
  bloated_color_image->create(
      bloated_image_height, bloated_image_width, CV_8UC3);
  bloated_color_image->setTo(0);
  color_image.copyTo(
      (*bloated_color_image)(
          cv::Rect(
              border_size_pixels, border_size_pixels, image_width,
              image_height)));

  CHECK_NOTNULL(bloated_color_image->data);
}

bool getCopyOfImageAsColorImage(
    const vi_map::Vertex& vertex, size_t camera_index, const vi_map::VIMap& map,
    cv::Mat* image) {
  CHECK_NOTNULL(image);

  vi_map::VIMap& nonconst_map = const_cast<vi_map::VIMap&>(map);
  cv::Mat image_from_vertex;
  if (nonconst_map.getRawImage(
          vertex, static_cast<int>(camera_index), &image_from_vertex)) {
  } else if (
      nonconst_map.getRawColorImage(
          vertex, static_cast<int>(camera_index), &image_from_vertex)) {
  } else {
    LOG(WARNING) << "No raw image available for vertex "
                 << vertex.id().hexString() << " and camera index "
                 << camera_index;
    return false;
  }
  CHECK_NOTNULL(image_from_vertex.data);
  createColorImageFromImage(image_from_vertex, image);
  CHECK_NOTNULL(image->data);
  return true;
}

bool getCopyOfImageAsColorImage(
    const aslam::VisualFrame& frame, cv::Mat* image) {
  CHECK_NOTNULL(image);

  if (frame.hasRawImage()) {
    createColorImageFromImage(frame.getRawImage(), image);
    CHECK_NOTNULL(image->data);
    return true;
  } else {
    LOG(WARNING) << "Unable to extract the image. "
                 << "The given frame " << frame.getId().hexString()
                 << " does not have a raw image.";
    return false;
  }
}

bool getCopyOfImageAsBloatedColorImage(
    const aslam::VisualFrame& frame, int border_size_pixels, cv::Mat* image) {
  CHECK_NOTNULL(image);
  CHECK_GT(border_size_pixels, 0);

  cv::Mat color_image_from_visual_frame;
  if (getCopyOfImageAsColorImage(frame, &color_image_from_visual_frame)) {
    CHECK_NOTNULL(color_image_from_visual_frame.data);
    createBloatedColorImageFromColorImage(
        color_image_from_visual_frame, border_size_pixels, image);
    CHECK_NOTNULL(image->data);
    return true;
  } else {
    LOG(WARNING) << "Unable to extract the image. "
                 << "The given frame " << frame.getId().hexString()
                 << " does not have a raw image.";
    return false;
  }
}

bool getCopyOfImageAsBloatedColorImage(
    const vi_map::Vertex& vertex, size_t camera_index, int border_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image) {
  CHECK_NOTNULL(image);
  CHECK_GT(border_size_pixels, 0);

  cv::Mat color_image_from_vertex;
  if (getCopyOfImageAsColorImage(
          vertex, camera_index, map, &color_image_from_vertex)) {
    CHECK_NOTNULL(color_image_from_vertex.data);
    createBloatedColorImageFromColorImage(
        color_image_from_vertex, border_size_pixels, image);
    CHECK_NOTNULL(image->data);
    return true;
  } else {
    return false;
  }
}

void getImagePatchesForLandmark(
    const vi_map::LandmarkId& landmark_id,
    int half_patch_size_pixels, bool draw_bounding_boxes,
    bool draw_center_circle, const vi_map::VIMap& map,
    std::vector<cv::Mat>* patches) {
  CHECK_NOTNULL(patches);
  CHECK(map.hasLandmark(landmark_id));
  CHECK_GT(half_patch_size_pixels, 0);

  const vi_map::Landmark& landmark = map.getLandmark(landmark_id);
  const vi_map::KeypointIdentifierList& keypoint_identifiers =
      landmark.getObservations();
  const size_t num_observations = keypoint_identifiers.size();

  if (num_observations == 0u) {
    LOG(WARNING) << "Zero observations for landmark with id "
                 << landmark_id.hexString();
    return;
  }
  patches->resize(num_observations);

  for (size_t observation_idx = 0u; observation_idx < num_observations;
       ++observation_idx) {
    const vi_map::KeypointIdentifier& keypoint_identifier =
        keypoint_identifiers[observation_idx];
    const size_t frame_index = keypoint_identifier.frame_id.frame_index;

    const vi_map::Vertex& observer_vertex =
        map.getVertex(keypoint_identifier.frame_id.vertex_id);

    CHECK_LT(frame_index, observer_vertex.numFrames());
    const aslam::VisualFrame& visual_frame =
        observer_vertex.getVisualFrame(frame_index);
    CHECK(visual_frame.hasKeypointMeasurements());
    const size_t keypoint_index = keypoint_identifier.keypoint_index;
    CHECK_LT(keypoint_index, visual_frame.getNumKeypointMeasurements());
    const Eigen::Vector2d& keypoint =
        visual_frame.getKeypointMeasurement(keypoint_index);
    const int keypoint_x = static_cast<int>(std::round(keypoint(0)));
    const int keypoint_y = static_cast<int>(std::round(keypoint(1)));
    const int keypoint_bloated_x =
        getBloatedCoordinate<int>(keypoint_x, half_patch_size_pixels);
    const int keypoint_bloated_y =
        getBloatedCoordinate<int>(keypoint_y, half_patch_size_pixels);

    cv::Mat bloated_map_image;
    // Bloating the image by half the path size on each side.
    CHECK(
        getCopyOfImageAsBloatedColorImage(
            observer_vertex, frame_index, half_patch_size_pixels, map,
            &bloated_map_image));

    CHECK(
        getImagePatch(
            bloated_map_image, keypoint_bloated_x, keypoint_bloated_y,
            half_patch_size_pixels, &(patches->at(observation_idx))));

    if (draw_bounding_boxes) {
      drawBoundingBoxOntoPatch(&(patches->at(observation_idx)));
    }
    if (draw_center_circle) {
      drawCenterCircleOntoPatch(&(patches->at(observation_idx)));
    }
  }
}

void getImagePatchesForLandmark(
    const vi_map::LandmarkId& landmark_id,
    int half_patch_size_pixels, bool draw_bounding_boxes,
    bool draw_center_circle, const vi_map::VIMap& map, cv::Mat* patch_row) {
  CHECK_NOTNULL(patch_row);

  std::vector<cv::Mat> patches;
  getImagePatchesForLandmark(
      landmark_id, half_patch_size_pixels, draw_bounding_boxes,
      draw_center_circle, map, &patches);

  const int num_patches = static_cast<int>(patches.size());

  const int patch_size_pixels = ((2 * half_patch_size_pixels) + 1);
  size_t patch_row_width = num_patches * patch_size_pixels;

  patch_row->create(patch_size_pixels, patch_row_width, CV_8UC3);

  // Copy all patches onto a single row.
  for (int patch_idx = 0; patch_idx < num_patches; ++patch_idx) {
    patches[patch_idx].copyTo(
        (*patch_row)(
            cv::Rect(
                patch_idx * patch_size_pixels, 0, patch_size_pixels,
                patch_size_pixels)));
  }
}

void getBloatedImageCoordinatesOfPatchUpperLeftCorner(
    const Eigen::Vector2d& keypoint, int half_patch_size_pixels,
    int* x_bloated_upper_left, int* y_bloated_upper_left) {
  CHECK_NOTNULL(x_bloated_upper_left);
  CHECK_NOTNULL(y_bloated_upper_left);

  const int x_bloated = getBloatedCoordinate<int>(
      static_cast<int>(std::round(keypoint(0))), half_patch_size_pixels);
  const int y_bloated = getBloatedCoordinate<int>(
      static_cast<int>(std::round(keypoint(1))), half_patch_size_pixels);

  (*x_bloated_upper_left) = x_bloated - half_patch_size_pixels;
  (*y_bloated_upper_left) = y_bloated - half_patch_size_pixels;
}

void copyPatchOntoBloatedImage(
    const cv::Mat& patch, const Eigen::Vector2d& keypoint,
    int half_patch_size_pixels, cv::Mat* image) {
  CHECK_NOTNULL(image);
  CHECK_NOTNULL(image->data);
  CHECK_GT(half_patch_size_pixels, 0);

  int x_bloated_upper_left, y_bloated_upper_left;
  getBloatedImageCoordinatesOfPatchUpperLeftCorner(
      keypoint, half_patch_size_pixels, &x_bloated_upper_left,
      &y_bloated_upper_left);

  const int patch_size_pixels = ((2 * half_patch_size_pixels) + 1);
  CHECK_EQ(patch.rows, patch_size_pixels);
  CHECK_EQ(patch.cols, patch_size_pixels);
  patch.copyTo(
      (*image)(
          cv::Rect(
              x_bloated_upper_left, y_bloated_upper_left, patch_size_pixels,
              patch_size_pixels)));
}

void visualizeImage(const cv::Mat& image, const std::string& label) {
  cv::namedWindow(label);
  cv::imshow(label, image);
  cv::waitKey(0);
}

void visualizePatchesOfLandmarkKeypointMatches(
    const aslam::VisualFrame& frame,
    const std::vector<size_t>& keypoint_indices,
    const vi_map::LandmarkIdList& landmark_ids,
    const std::vector<double>& scores, int half_patch_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image) {
  CHECK_NOTNULL(image);
  const size_t num_matches = keypoint_indices.size();
  CHECK_EQ(num_matches, landmark_ids.size());
  CHECK_EQ(num_matches, scores.size());

  CHECK(frame.hasRawImage());
  const Eigen::Matrix2Xd& keypoints = frame.getKeypointMeasurements();

  // Figure out the descirptor size used.
  CHECK(frame.hasDescriptors());
  const size_t kNumBitsPerByte = 8u;
  const size_t descriptor_size_bits =
      frame.getDescriptorSizeBytes() * kNumBitsPerByte;
  CHECK_GT(descriptor_size_bits, 0u);

  cv::Mat bloated_query_image;
  // Bloating the image by half the path size on each side.
  CHECK(
      getCopyOfImageAsBloatedColorImage(
          frame, half_patch_size_pixels, &bloated_query_image));
  CHECK_NOTNULL(bloated_query_image.data);

  const int patch_size_pixels = ((2 * half_patch_size_pixels) + 1);
  const int kVerticalBarWidth = 5;
  const int kTextWidthPixels = 30;
  const int kRowHeightPixels = (patch_size_pixels + kTextWidthPixels);

  std::vector<cv::Mat> image_rows(num_matches);
  size_t max_num_observations = 0;

  // Iterating over all matches.
  for (size_t match_idx = 0u; match_idx < num_matches; ++match_idx) {
    const size_t keypoint_index = keypoint_indices[match_idx];
    CHECK_LT(static_cast<int>(keypoint_index), keypoints.cols());

    vi_map::LandmarkId store_landmark_id = landmark_ids[match_idx];
    CHECK(map.hasLandmark(store_landmark_id));

    const double score = scores[match_idx];

    const vi_map::Landmark& landmark = map.getLandmark(store_landmark_id);

    const size_t num_observations = landmark.getObservations().size();
    CHECK_GT(num_observations, 0u);

    max_num_observations = std::max(num_observations, max_num_observations);

    cv::Mat& image_row = image_rows[match_idx];
    const int row_width_pixels =
        (((static_cast<int>(num_observations) + 1) * patch_size_pixels) +
         kVerticalBarWidth);

    image_row.create(kRowHeightPixels, row_width_pixels, CV_8UC3);
    image_row.setTo(0);

    // Copy the query patch to the beginning of the row.
    cv::Mat query_patch;
    const Eigen::Vector2d& query_keypoint = keypoints.col(keypoint_index);

    // Get the image patch of the query keypoint.
    const int query_keypoint_x =
        static_cast<int>(std::round(query_keypoint(0)));
    const int query_keypoint_y =
        static_cast<int>(std::round(query_keypoint(1)));
    const int query_keypoint_bloated_x =
        getBloatedCoordinate<int>(query_keypoint_x, half_patch_size_pixels);
    const int query_keypoint_bloated_y =
        getBloatedCoordinate<int>(query_keypoint_y, half_patch_size_pixels);
    CHECK(
        getImagePatch(
            bloated_query_image, query_keypoint_bloated_x,
            query_keypoint_bloated_y, half_patch_size_pixels, &query_patch));
    drawCenterCircleOntoPatch(&query_patch);
    // Copy the query patch onto the beginning of the image row.
    query_patch.copyTo(
        image_row(cv::Rect(0, 0, patch_size_pixels, patch_size_pixels)));

    const bool kDrawBoundingBoxForLandmarkPatches = false;
    const bool kDrawCenterCircle = true;
    cv::Mat landmark_patches_row;
    getImagePatchesForLandmark(
        store_landmark_id, half_patch_size_pixels,
        kDrawBoundingBoxForLandmarkPatches, kDrawCenterCircle, map,
        &landmark_patches_row);

    int landmark_patches_width = landmark_patches_row.cols;
    CHECK_GT(landmark_patches_width, 0);
    CHECK_EQ(landmark_patches_row.rows, patch_size_pixels);

    // Copy the landmark patches patch onto the image row,
    // right of the query patch.
    landmark_patches_row.copyTo(
        image_row(
            cv::Rect(
                (patch_size_pixels + kVerticalBarWidth), 0,
                landmark_patches_width, patch_size_pixels)));

    // Draw a blue bar between the query patch and the landmark patches to
    // visually separate the two.
    cv::rectangle(
        image_row, cv::Point2f(patch_size_pixels, 0),
        cv::Point2f(
            patch_size_pixels + kVerticalBarWidth, patch_size_pixels - 1),
        kCvBlue, -1);

    // Add some text indicating the matching score.
    const int hamming_distance = static_cast<int>(
        std::round((1.0 - score) * static_cast<double>(descriptor_size_bits)));
    const std::string score_text = "Match Score: " + std::to_string(score) +
                                   ", Hamming Distance: " +
                                   std::to_string(hamming_distance);
    const cv::Scalar kWhite(255.0, 255.0, 255.0);
    const int kFontFace = CV_FONT_NORMAL;
    const double kFontScaling = 0.5;
    const int y_bottom_left =
        kRowHeightPixels -
        static_cast<int>(
            std::floor(static_cast<double>(kTextWidthPixels) / 2.0));
    cv::putText(
        image_row, score_text, cv::Point2d(0, y_bottom_left), kFontFace,
        kFontScaling, kWhite);
  }
  // Assemble the image rows into one image.
  // Calculate the dimensions.
  const int image_with_rows_for_each_match_height =
      (static_cast<int>(num_matches) * kRowHeightPixels);
  const int image_with_rows_for_each_match_width =
      kVerticalBarWidth +
      ((static_cast<int>(max_num_observations) + 1) * patch_size_pixels);

  image->create(
      image_with_rows_for_each_match_height,
      image_with_rows_for_each_match_width, CV_8UC3);

  for (size_t match_idx = 0u; match_idx < num_matches; ++match_idx) {
    const int width = image_rows[match_idx].cols;
    CHECK_GT(width, 0);
    CHECK_LE(width, image_with_rows_for_each_match_width);
    const int row_index_px = static_cast<int>(match_idx) * (kRowHeightPixels);
    VLOG(30) << "Copying image patch " << match_idx << " with dimensions "
             << image_rows[match_idx].rows << "/" << image_rows[match_idx].cols
             << " to 0/" << row_index_px << " patch size: " << kRowHeightPixels
             << "/" << width;
    image_rows[match_idx].copyTo(
        (*image)(cv::Rect(0, row_index_px, width, kRowHeightPixels)));
  }
}

void visualizePatchesOfProjectedMatchedLandmarksInVisualFrameImage(
    const aslam::VisualFrame& frame,
    const std::vector<size_t>& keypoint_indices,
    const vi_map::LandmarkIdList& landmark_ids,
    const Eigen::Matrix3Xd& p_C_landmarks, int half_patch_size_pixels,
    const vi_map::VIMap& map, cv::Mat* bloated_image) {
  CHECK_NOTNULL(bloated_image);
  const size_t num_matches = keypoint_indices.size();
  CHECK_EQ(num_matches, landmark_ids.size())
      << "The number of keypoint indices and store landmark ids must be equal, "
      << "because they are expected to denote matches.";
  CHECK_EQ(static_cast<int>(num_matches), p_C_landmarks.cols())
      << "The number of keypoint indices and landmarks in p_C_landmarks must "
         "be "
      << "equal, because they are expected to denote matches.";

  // Bloating the image by half the path size on each side.
  CHECK(
      getCopyOfImageAsBloatedColorImage(
          frame, half_patch_size_pixels, bloated_image));
  CHECK_NOTNULL(bloated_image->data);

  const int bloated_image_height = bloated_image->rows;
  const int bloated_image_width = bloated_image->cols;
  CHECK_GT(bloated_image_height, 0);
  CHECK_GT(bloated_image_width, 0);

  const aslam::Camera::ConstPtr& camera = frame.getCameraGeometry();
  const int image_height = static_cast<int>(camera->imageHeight());
  const int image_width = static_cast<int>(camera->imageWidth());

  const Eigen::Matrix2Xd& keypoints = frame.getKeypointMeasurements();

  for (size_t match_idx = 0u; match_idx < num_matches; ++match_idx) {
    const size_t keypoint_index = keypoint_indices[match_idx];
    CHECK_LT(static_cast<int>(keypoint_index), keypoints.cols());

    const vi_map::LandmarkId& landmark_id = landmark_ids[match_idx];
    CHECK(map.hasLandmark(landmark_id));

    const Eigen::Vector2d& keypoint = keypoints.col(keypoint_index);
    const int keypoint_x = static_cast<int>(std::round(keypoint(0)));
    const int keypoint_y = static_cast<int>(std::round(keypoint(1)));
    checkIfCoordinateValid(
        keypoint_x, keypoint_y, 0, 0, image_width, image_height);

    const double keypoint_bloated_x_fp =
        getBloatedCoordinate<double>(keypoint(0), half_patch_size_pixels);
    const double keypoint_bloated_y_fp =
        getBloatedCoordinate<double>(keypoint(1), half_patch_size_pixels);

    Eigen::Vector2d landmark_projection;
    const Eigen::Vector3d& p_C_landmark = p_C_landmarks.col(match_idx);
    CHECK(
        (camera->project3(p_C_landmark, &landmark_projection))
            .isKeypointVisible());

    int landmark_projection_x_upper_left, landmark_projection_y_upper_left;
    getBloatedImageCoordinatesOfPatchUpperLeftCorner(
        landmark_projection, half_patch_size_pixels,
        &landmark_projection_x_upper_left, &landmark_projection_y_upper_left);
    checkIfCoordinateValid(
        landmark_projection_x_upper_left, landmark_projection_y_upper_left, 0,
        0, image_width, image_height);

    std::vector<cv::Mat> landmark_patches;
    const bool kDrawBoundingBoxes = true;
    const bool kDrawCenterCircle = true;
    getImagePatchesForLandmark(
        landmark_id, half_patch_size_pixels, kDrawBoundingBoxes,
        kDrawCenterCircle, map, &landmark_patches);
    CHECK_GT(landmark_patches.size(), 0u);

    // TODO(mbuerki): Don't just copy the first patch, but copy the one
    // that actually got matched.
    const int patch_size_pixels = ((2 * half_patch_size_pixels) + 1);
    landmark_patches[0].copyTo(
        (*bloated_image)(
            cv::Rect(
                landmark_projection_x_upper_left,
                landmark_projection_y_upper_left, patch_size_pixels,
                patch_size_pixels)));

    const int kCircleRadiusPixels = 5;
    cv::circle(
        (*bloated_image),
        cv::Point2d(keypoint_bloated_x_fp, keypoint_bloated_y_fp),
        kCircleRadiusPixels, kCvRed);

    const double landmark_bloated_x = getBloatedCoordinate<double>(
        landmark_projection(0), half_patch_size_pixels);
    const double landmark_bloated_y = getBloatedCoordinate<double>(
        landmark_projection(1), half_patch_size_pixels);

    cv::line(
        (*bloated_image),
        cv::Point2d(keypoint_bloated_x_fp, keypoint_bloated_y_fp),
        cv::Point(landmark_bloated_x, landmark_bloated_y), kCvBlue, 2);
  }
}

void visualizePatchesOfLandmarksOfVertexAsProjectionsOntoTheVisualFrame(
    const pose_graph::VertexId& vertex_id, size_t frame_index,
    int half_patch_size_pixels, const vi_map::VIMap& map, cv::Mat* image) {
  CHECK_NOTNULL(image);
  CHECK(vertex_id.isValid());
  VLOG(1) << "Visualizing frame and landmarks of vertex "
          << vertex_id.hexString();

  const vi_map::Vertex& vertex = map.getVertex(vertex_id);

  const size_t num_frames = vertex.numFrames();
  CHECK_LT(frame_index, num_frames);
  const aslam::VisualFrame& frame = vertex.getVisualFrame(frame_index);

  // Bloating the image by half the path size on each side.
  CHECK(
      getCopyOfImageAsBloatedColorImage(
          vertex, frame_index, half_patch_size_pixels, map, image));

  // Get all global landmark ids of this frame.
  vi_map::LandmarkIdList landmark_ids;
  vertex.getFrameObservedLandmarkIds(frame_index, &landmark_ids);

  const Eigen::Matrix2Xd& keypoints = frame.getKeypointMeasurements();

  const aslam::Camera::ConstPtr& camera = frame.getCameraGeometry();
  CHECK(camera);
  const int image_height = static_cast<int>(camera->imageHeight());
  CHECK_GT(image_height, 0);
  const int image_width = static_cast<int>(camera->imageWidth());
  CHECK_GT(image_width, 0);

  aslam::Transformation T_I_G = map.getVertex_T_G_I(vertex_id).inverse();

  cv::Mat vertex_image_standard_size;
  CHECK(
      getCopyOfImageAsColorImage(
          vertex, frame_index, map, &vertex_image_standard_size));

  size_t num_valid = 0u;
  // Iterate over all valid global landmark ids and get the image patches.
  const size_t num_landmark_ids = landmark_ids.size();
  CHECK_EQ(static_cast<int>(num_landmark_ids), keypoints.cols());

  const aslam::Transformation& T_C_B =
      vertex.getVisualNFrame().get_T_C_B(frame_index);

  for (size_t landmark_id_idx = 0u; landmark_id_idx < num_landmark_ids;
      ++landmark_id_idx) {
    const vi_map::LandmarkId& landmark_id = landmark_ids[landmark_id_idx];
    if (landmark_id.isValid()) {
      bool kDrawBoundingBoxes = true;
      bool kDrawCenterCircle = true;
      std::vector<cv::Mat> patches;
      getImagePatchesForLandmark(
          landmark_id, half_patch_size_pixels, kDrawBoundingBoxes,
          kDrawCenterCircle, map, &patches);

      Eigen::Vector2d real_keypoint = keypoints.col(landmark_id_idx);
      const double real_keypoint_bloated_x_fp = getBloatedCoordinate<double>(
          real_keypoint(0), half_patch_size_pixels);
      const double real_keypoint_bloated_y_fp = getBloatedCoordinate<double>(
          real_keypoint(1), half_patch_size_pixels);

      Eigen::Vector3d p_G_landmark = map.getLandmark_G_p_fi(landmark_id);
      Eigen::Vector3d p_C_landmark = (T_C_B * T_I_G).transform(p_G_landmark);

      Eigen::Vector2d projected_keypoint;
      aslam::ProjectionResult projection_result =
          camera->project3(p_C_landmark, &projected_keypoint);
      CHECK(projection_result.isKeypointVisible());

      const double projected_keypoint_bloated_x_fp =
          getBloatedCoordinate<double>(
              projected_keypoint(0), half_patch_size_pixels);
      const double projected_keypoint_bloated_y_fp =
          getBloatedCoordinate<double>(
              projected_keypoint(1), half_patch_size_pixels);

      int bloated_x_upper_left, bloated_y_upper_left;
      getBloatedImageCoordinatesOfPatchUpperLeftCorner(
          projected_keypoint, half_patch_size_pixels, &bloated_x_upper_left,
          &bloated_y_upper_left);

      CHECK_GT(patches.size(), 0u);
      const int patch_size_pixels = ((2 * half_patch_size_pixels) + 1);
      CHECK_EQ(patches[0].rows, patch_size_pixels);
      CHECK_EQ(patches[0].cols, patch_size_pixels);
      patches[0].copyTo(
          (*image)(
              cv::Rect(
                  bloated_x_upper_left, bloated_y_upper_left, patch_size_pixels,
                  patch_size_pixels)));

      const int kKeypointCircleRadiusPixels = 5;
      cv::circle(
          (*image),
          cv::Point2d(real_keypoint_bloated_x_fp, real_keypoint_bloated_y_fp),
          kKeypointCircleRadiusPixels, kCvRed);

      cv::line(
          (*image),
          cv::Point2d(real_keypoint_bloated_x_fp, real_keypoint_bloated_y_fp),
          cv::Point(
              projected_keypoint_bloated_x_fp, projected_keypoint_bloated_y_fp),
          kCvBlue, 2);

      ++num_valid;
    }
    break;
  }
}

void visualizePatchesOfLandmarksInVisualFrameImage(
    const aslam::VisualFrame& frame,
    const vi_map::LandmarkIdList& landmark_ids,
    const Eigen::Matrix3Xd& p_C_landmarks, int half_patch_size_pixels,
    const vi_map::VIMap& map, cv::Mat* image) {
  CHECK_NOTNULL(image);
  const size_t num_landmarks = landmark_ids.size();
  CHECK_EQ(p_C_landmarks.cols(), static_cast<int>(num_landmarks));

  // Bloating the image by half the path size on each side.
  CHECK(
      getCopyOfImageAsBloatedColorImage(frame, half_patch_size_pixels, image));
  CHECK_NOTNULL(image->data);

  const int bloated_image_height = image->rows;
  const int bloated_image_width = image->cols;
  CHECK_GT(bloated_image_height, 0);
  CHECK_GT(bloated_image_width, 0);

  const aslam::Camera::ConstPtr camera = frame.getCameraGeometry();
  CHECK(camera);

  size_t num_valid = 0u;
  size_t num_invalid = 0u;

  // Iterate over all landmarks, find their global landmark id, get the patch
  // and put it in the image.
  for (size_t landmark_idx = 0u; landmark_idx < num_landmarks; ++landmark_idx) {
    vi_map::LandmarkId landmark_id = landmark_ids[landmark_idx];
    CHECK(landmark_id.isValid());

    const bool kDrawBoundingBoxes = true;
    const bool kDrawCenterCircle = true;
    std::vector<cv::Mat> landmark_patches;
    getImagePatchesForLandmark(
        landmark_id, half_patch_size_pixels, kDrawBoundingBoxes,
        kDrawCenterCircle, map, &landmark_patches);
    CHECK_GT(landmark_patches.size(), 0u);

    const Eigen::Vector3d& p_C_landmark = p_C_landmarks.col(landmark_idx);

    Eigen::Vector2d projected_keypoint;
    aslam::ProjectionResult projection_result =
        camera->project3(p_C_landmark, &projected_keypoint);

    if (projection_result.isKeypointVisible()) {
      copyPatchOntoBloatedImage(
          landmark_patches.front(), projected_keypoint, half_patch_size_pixels,
          image);
      ++num_valid;
    } else {
      ++num_invalid;
    }
  }
}
}  // namespace visualization
