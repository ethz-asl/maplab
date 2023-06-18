#ifndef ASLAM_TRACK_MANAGER_H_
#define ASLAM_TRACK_MANAGER_H_

#include <mutex>
#include <unordered_set>

#include <aslam/matcher/match.h>
#include <glog/logging.h>

namespace aslam {
  class VisualNFrame;

  template<typename IdType>
  class ThreadSafeIdProvider {
   public:
    ThreadSafeIdProvider(IdType initial_id) : initial_id_(initial_id) {
      reset();
    }

    IdType getNewId() {
      std::lock_guard<std::mutex> lock(mutex_);
      return id_++;
    }

    void reset() {
      std::lock_guard<std::mutex> lock(mutex_);
      id_ = initial_id_;
    }

   private:
    std::mutex mutex_;
    IdType id_;
    IdType initial_id_;
  };

  /// \brief The Track manager assigns track ids to the given matches with different strategies.
  class TrackManager {
   public:
    TrackManager() {}
    virtual ~TrackManager() {};

    /// \brief Writes track ids for a list of matches into two given frames.
    ///
    /// @param[in]  matches_A_B   List of matches between the A and B frame.
    /// @param[in]  A_frame   Pointer to the A frame.
    /// @param[in]  B_frame  Pointer to the B frame.
    virtual void applyMatchesToFrames(
        const FrameToFrameMatches& matches_A_B,
        VisualFrame* A_frame, VisualFrame* B_frame) = 0;

    /// \brief Returns a pointer to the track id channel. If no track id channel is present for the
    ///        given frame, a new track id channel will be created with num_keypoints many track
    ///        ids, all set to to -1.
    ///
    /// @param[in]  frame   Pointer to the visual frame.
    /// @return             Pointer to the track id channel.
    static Eigen::VectorXi* createAndGetTrackIdChannel(VisualFrame* frame);

    static void resetIdProvider() {
      track_id_provider_.reset();
    }

   protected:
    static ThreadSafeIdProvider<size_t> track_id_provider_;
  };


  /// \brief Track manager simply writing track ids into the given frames for
  ///        the given matches.
  class SimpleTrackManager : public TrackManager {
   public:
    SimpleTrackManager() = default;
    virtual ~SimpleTrackManager() {};

    /// \brief Writes track ids into the given frames for the given matches.
    ///        If for a match, both track ids are < 0, a new track id is
    ///        generated and applied.
    ///        If any of the two track ids for a match is >= 0 the other one is
    ///        either expected to be identical (in which case no change is
    ///        applied) or < 0, in which case the valid id (>=0) is copied over.
    ///        Matches are expected to be exclusive.
    virtual void applyMatchesToFrames(
        const FrameToFrameMatches& matches,
        VisualFrame* A_frame, VisualFrame* B_frame);
  };

  inline void addToSetsAndCheckExclusiveness(
      int index_A, int index_B, std::unordered_set<int>* consumed_As,
      std::unordered_set<int>* consumed_Bs) {
    CHECK_NOTNULL(consumed_As);
    CHECK_NOTNULL(consumed_Bs);
    std::pair<std::unordered_set<int>::iterator, bool> ret_A =
        consumed_As->insert(index_A);
    CHECK(ret_A.second) << "The given matches don't seem to be exclusive."
        " Trying to assign A " << index_A << " more than once!";
    std::pair<std::unordered_set<int>::iterator, bool> ret_B =
        consumed_Bs->insert(index_B);
    CHECK(ret_B.second) << "The given matches don't seem to be "
        "exclusive. Trying to assign B " << index_B << " more than once!";
  }

}  // namespace aslam

#endif  // ASLAM_TRACK_MANAGER_H_
