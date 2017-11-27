#ifndef LOOP_CLOSURE_HANDLER_LOOP_CLOSURE_CONSTRAINT_H_
#define LOOP_CLOSURE_HANDLER_LOOP_CLOSURE_CONSTRAINT_H_

#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <vi-map/vertex.h>

namespace loop_closure_handler {

struct FrameKeypointIndexPair {
  unsigned int frame_idx;
  unsigned int keypoint_idx;

  FrameKeypointIndexPair() : frame_idx(-1), keypoint_idx(-1) {}
  FrameKeypointIndexPair(unsigned int _frame_idx, unsigned int _keypoint_idx)
      : frame_idx(_frame_idx), keypoint_idx(_keypoint_idx) {}

  inline bool operator==(const FrameKeypointIndexPair& lhs) const {
    bool is_same = true;
    is_same &= frame_idx == lhs.frame_idx;
    is_same &= keypoint_idx == lhs.keypoint_idx;
    return is_same;
  }
};

}  // namespace loop_closure_handler

namespace std {
template <>
struct hash<loop_closure_handler::FrameKeypointIndexPair> {
 private:
  const hash<unsigned int> frame_hash;
  const hash<unsigned int> keypoint_hash;

 public:
  hash() : frame_hash(), keypoint_hash() {}
  inline size_t operator()(
      const loop_closure_handler::FrameKeypointIndexPair& p) const {
    size_t seed = frame_hash(p.frame_idx);
    // A way to combine hashes, taken from Boost library.
    // See hash_combine in: http://boost.cowic.de/rc/pdf/hash.pdf
    return keypoint_hash(p.keypoint_idx) + 0x9e3779b9 + (seed << 6) +
           (seed >> 2);
  }
};
}  // namespace std

#endif  // LOOP_CLOSURE_HANDLER_LOOP_CLOSURE_CONSTRAINT_H_
