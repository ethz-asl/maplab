#ifndef ASLAM_FEATURE_TRACK_INL_H_
#define ASLAM_FEATURE_TRACK_INL_H_

#include <iostream>

namespace aslam {
inline std::ostream& operator<<(std::ostream& out, const FeatureTrack& track) {
  out << "FeatureTrack:" << std::endl;
  out << "\ttrack id: " << track.getTrackId() << std::endl;
  out << "\tlength: " << track.getTrackLength() << std::endl;

  if (track.hasObservations()) {
    size_t keypoint_idx = 0;
    for (const KeypointIdentifier& keypoint : track.getKeypointIdentifiers()) {
      out << std::setiosflags(std::ios::fixed)
          << std::setprecision(3)
          << "\tobservation " << keypoint_idx++ << ":" << std::endl
          << "\t\tNFrameId: " << keypoint.getNFrameId() << std::endl
          << "\t\tcamera idx: " << keypoint.getFrameIndex() << std::endl
          << "\t\tkeypoint: " << keypoint.getKeypointMeasurement().transpose() << std::endl
          << std::endl;
    }
  } else {
    out << "\tNo observations!";
  }
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const FeatureTracks& tracks) {
  out << std::setiosflags(std::ios::fixed)
      << std::setfill (' ')
      << std::setw(10) << "track id"
      << std::setw(10) << "camera idx"
      << std::setw(10) << "length" << std::endl;

  if (!tracks.empty()) {
    for (const FeatureTrack& track : tracks) {
        out << std::setiosflags(std::ios::fixed)
            << std::setfill (' ')
            << std::setprecision(3)
            << std::setw(10) << track.getTrackId()
            << std::setw(10) << track.getFirstKeypointIdentifier().getFrameIndex()
            << std::setw(10) << track.getTrackLength() << std::endl;
    }
  } else {
    out << "No tracks!";
  }
  return out;
}

inline std::ostream& operator<<(std::ostream& out,
                                const std::vector<aslam::FeatureTracks>& rig_tracks) {
  size_t cam_idx = 0;
  for(const aslam::FeatureTracks& camera_tracks : rig_tracks) {
    out << "Camera: " << cam_idx++ << ":\n";
    out << camera_tracks << "\n";
  }
  return out;
}

}  // namespace aslam
#endif  // ASLAM_FEATURE_TRACK_INL_H_
