#ifndef VISUAL_NPIPELINE_H_
#define VISUAL_NPIPELINE_H_

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <vector>

#include <opencv2/core/core.hpp>

#include <aslam/cameras/ncamera.h>
#include <aslam/common/macros.h>
#include <aslam/common/thread-pool.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <aslam/pipeline/visual-pipeline.h>

namespace aslam {

/// \class VisualNPipeline
/// \brief An interface for pipelines that turn images into VisualNFrames
///
/// This is the abstract interface for visual pipelines that turn raw images
/// into VisualNFrame data. The underlying pipeline may include undistortion
/// or rectification, image contrast enhancement, feature detection and
/// descriptor computation, or other operations.
///
/// The class has two NCameras calibration structs that represent the
/// intrinsic and extrinsic calibration of the camera system.
/// The "input" calibration (getInputNCameras()) represents the calibration of
/// raw camera system, before any image processing, resizing, or undistortion
/// has taken place. The "output" calibration (getOutputNCameras())
/// represents the calibration parameters of the images and keypoints that get
/// set in the VisualNFrames struct. These are the camera parameters after
/// image processing, resizing, undistortion, etc.
///
/// The class should synchronize images with nearby timestamps and handle
/// out-of-order images. When all frames of a VisualNFrame are complete,
/// they are added to a list of output frames in the order that they are
/// completed. This list should be sorted by time (oldest first) and the number
/// of elements can be queried by numVisualNFramesComplete(). The getNext()
/// function retrieves the oldest complete VisualNFrames and leaves the remaining.
/// The getLatestAndClear() function gets the newest VisualNFrames and discards
/// anything older.
class VisualNPipeline final {
 public:
  ASLAM_POINTER_TYPEDEFS(VisualNPipeline);
  ASLAM_DISALLOW_EVIL_CONSTRUCTORS(VisualNPipeline);

  /// \brief Initialize a working pipeline.
  ///
  /// \param[in] num_threads            The number of processing threads.
  /// \param[in] pipelines              The ordered image pipelines, one pipeline
  ///                                   per camera in the same order as they are
  ///                                   indexed in the camera system.
  /// \param[in] input_camera_system    The camera system of the raw images.
  /// \param[in] output_camera_system   The camera system of the processed images.
  /// \param[in] timestamp_tolerance_ns How close should two image timestamps be
  ///                                   for us to consider them part of the same
  ///                                   synchronized frame?
  VisualNPipeline(size_t num_threads,
                  const std::vector<VisualPipeline::Ptr>& pipelines,
                  const NCamera::Ptr& input_camera_system,
                  const NCamera::Ptr& output_camera_system,
                  int64_t timestamp_tolerance_ns);

  ~VisualNPipeline();

  /// Shutdown the thread pool and release blocking waiters.
  void shutdown();

  /// \brief Add an image to the visual pipeline
  ///
  /// This function is called by a user when an image is received.
  /// The pipeline then processes the images and constructs VisualNFrames
  ///
  /// \param[in] camera_index The index of the camera that this image corresponds to
  /// \param[in] image the image data
  /// \param[in] timestamp the time in integer nanoseconds.
  void processImage(size_t camera_index, const cv::Mat& image, int64_t timestamp);

  /// \brief Same as \ref processImage with the difference that the function call blocks if the
  ///        output queue exceeds the specified limit.
  ///
  /// \param[in] camera_index The index of the camera that this image corresponds to.
  /// \param[in] image the image data.
  /// \param[in] timestamp the time in integer nanoseconds.
  /// \param[in] max_output_queue_size the max. size of the output queue. The function call will
  ///            block once this limit has been reached. As the frames are processed in a thread
  ///            pool it is possible that the real queue size will exceed the defined size by the
  ///            number of currently processed nframes.
  /// @return    Returns false if the queue is shut down.
  bool processImageBlockingIfFull(size_t camera_index, const cv::Mat& image, int64_t timestamp,
                                  size_t max_output_queue_size);

  /// \brief Same as \ref processImage with the difference that the function
  ///        call erases the oldest element of the queue if it exceeds the maximum size.
  ///
  /// \param[in] camera_index The index of the camera that this image corresponds to.
  /// \param[in] image the image data.
  /// \param[in] timestamp the time in integer nanoseconds.
  /// \param[in] max_output_queue_size the max. size of the output queue. The
  ///            function call will erase the oldest element of the output queue
  ///            once this limit has been reached. As the frames are processed
  ///            in a thread pool it is possible that the real queue size will
  ///            exceed the defined size by the number of currently processed
  ///            nframes.
  /// @return    Returns true if oldest nframe has been dropped.
  bool processImageNonBlockingDroppingOldestNFrameIfFull(
      size_t camera_index, const cv::Mat &image, int64_t timestamp,
      size_t max_output_queue_size);

  /// How many completed VisualNFrames are waiting to be retrieved?
  size_t getNumFramesComplete() const;

  /// Get the number of frames being processed.
  size_t getNumFramesProcessing() const;

  /// Get the next available set of processed frames.
  /// This may not be the latest data, it is simply the next in a FIFO queue.
  /// If there are no VisualNFrames waiting, this returns a NULL pointer.
  std::shared_ptr<VisualNFrame> getNext();

  /// Get the next available set of processed frames. Blocks if the queue is
  /// empty.
  /// @return Returns false if the queue is shut down.
  bool getNextBlocking(std::shared_ptr<VisualNFrame>* nframe);

  /// Get the latest available data and clear anything older.
  /// @return If there are no VisualNFrames waiting, this returns a NULL
  ///         pointer.
  std::shared_ptr<VisualNFrame> getLatestAndClear();

  /// Get the latest available data and clear anything older. Block if the
  /// queue is empty.
  /// @return Returns false if the queue is shut down.
  bool getLatestAndClearBlocking(std::shared_ptr<VisualNFrame>* nframe);

  /// Get the input camera system that corresponds to the images
  /// passed in to processImage().
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera systems may not be the same.
  std::shared_ptr<const NCamera> getInputNCameras() const;

  /// Get the output camera system that corresponds to the VisualNFrame
  /// data that comes out.
  /// Because this pipeline may do things like image undistortion or
  /// rectification, the input and output camera systems may not be the same.
  std::shared_ptr<const NCamera> getOutputNCameras() const;

  /// Blocks until all waiting frames are processed.
  void waitForAllWorkToComplete() const;

  /// \brief  Create a test visual npipeline.
  ///
  /// @param[in]  num_cameras   The number of cameras in the pipeline (determines the number of
  ///                           frames).
  /// @param[in]  num_threads   The number of threads used in the pipeline for processing the data.
  /// @param[in]  timestamp_tolerance_ns  Timestamp tolerance for frames to be considered
  ///                                     belonging together. [ns]
  /// @return  Pointer to the visual npipeline.
  static VisualNPipeline::Ptr createTestVisualNPipeline(size_t num_cameras,
      size_t num_threads, int64_t timestamp_tolerance_ns);

 private:
  /// \brief A local function to be passed to the thread pool.
  ///
  /// \param[in] camera_index The index of the camera that this image corresponds to.
  /// \param[in] image The image data.
  /// \param[in] timestamp_nanoseconds The time in integer nanoseconds.
  void work(size_t camera_index, const cv::Mat& image, int64_t timestamp_nanoseconds);

  std::shared_ptr<VisualNFrame> getNextImpl();

  void processImageImpl(size_t camera_index, const cv::Mat& image,
                        int64_t timestamp);

  /// One visual pipeline for each camera.
  std::vector<std::shared_ptr<VisualPipeline>> pipelines_;

  /// A mutex to protect the processing and completed queues.
  mutable std::mutex mutex_;
  /// Condition variable signaling that the output queue is not full.
  std::condition_variable condition_not_full_;
  /// Condition variable signaling that the output queue is not empty.
  std::condition_variable condition_not_empty_;
  /// A flag indicating a system shutdown.
  std::atomic<bool> shutdown_;

  typedef std::map<int64_t, std::shared_ptr<VisualNFrame>> TimestampVisualNFrameMap;
  /// The frames that are in progress.
  TimestampVisualNFrameMap processing_;
  /// The output queue of completed frames.
  TimestampVisualNFrameMap completed_;

  /// A thread pool for processing.
  std::shared_ptr<aslam::ThreadPool> thread_pool_;

  /// The camera system of the raw images.
  std::shared_ptr<NCamera> input_camera_system_;
  /// The camera system of the processed images.
  std::shared_ptr<NCamera> output_camera_system_;

  /// The tolerance for associating host timestamps as being captured at the same time
  int64_t timestamp_tolerance_ns_;
};
}  // namespace aslam
#endif // VISUAL_NPIPELINE_H_
