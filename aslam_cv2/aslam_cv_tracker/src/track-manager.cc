#include <set>

#include <glog/logging.h>

#include <aslam/frames/visual-frame.h>
#include <aslam/matcher/match.h>

#include "aslam/tracker/track-manager.h"

namespace aslam {

  ThreadSafeIdProvider<size_t> TrackManager::track_id_provider_(0u);

  Eigen::VectorXi* TrackManager::createAndGetTrackIdChannel(VisualFrame* frame) {
    // Load (and create) track id channels.
    CHECK_NOTNULL(frame);
    size_t num_track_ids = frame->getNumKeypointMeasurements();
    if (!frame->hasTrackIds()) {
      frame->setTrackIds(Eigen::VectorXi::Constant(num_track_ids, -1));
    }
    return CHECK_NOTNULL(frame->getTrackIdsMutable());
  }

  void SimpleTrackManager::applyMatchesToFrames(
      const FrameToFrameMatches& matches_A_B, VisualFrame* A_frame,
      VisualFrame* B_frame) {
    CHECK_NOTNULL(A_frame);
    CHECK_NOTNULL(B_frame);

    // Get the track ID channels.
    Eigen::VectorXi& A_track_ids = *CHECK_NOTNULL(createAndGetTrackIdChannel(A_frame));
    Eigen::VectorXi& B_track_ids = *CHECK_NOTNULL(createAndGetTrackIdChannel(B_frame));

    size_t num_A_track_ids = static_cast<size_t>(A_track_ids.rows());
    size_t num_B_track_ids = static_cast<size_t>(B_track_ids.rows());

    std::unordered_set<int> consumed_As;
    std::unordered_set<int> consumed_Bs;

    for (const FrameToFrameMatch& match : matches_A_B) {
      int index_A = match.getKeypointIndexInFrameA();
      CHECK_LT(index_A, static_cast<int>(num_A_track_ids));
      CHECK_GE(index_A, 0);

      int index_B = match.getKeypointIndexInFrameB();
      CHECK_LT(index_B, static_cast<int>(num_B_track_ids));
      CHECK_GE(index_B, 0);

      addToSetsAndCheckExclusiveness(index_A,
                                     index_B,
                                     &consumed_As,
                                     &consumed_Bs);

      int track_id_A = A_track_ids(index_A);
      int track_id_B = B_track_ids(index_B);

      if ((track_id_A) < 0 && (track_id_B < 0)) {
        // Both track ids are < 0. Start a new track.
        int new_track_id = track_id_provider_.getNewId();
        A_track_ids(index_A) = new_track_id;
        B_track_ids(index_B) = new_track_id;
      } else {
        // Either one of the track ids is >= 0.
        if (track_id_A != track_id_B) {
          // The two track ids are not equal, so we need to copy one over to the
          // other.
          if (track_id_B >= 0) {
            CHECK_LT(track_id_A, 0) << "Both the A and the B track "
                "id are >= 0 but they are not equal!";

            A_track_ids(index_A) = track_id_B;
          } else {
            CHECK_LT(track_id_B, 0) << "Both the A and the B "
                "track id are >= 0 but they are not equal!";

            B_track_ids(index_B) = track_id_A;
          }
        }
      }
    }
  }

}
