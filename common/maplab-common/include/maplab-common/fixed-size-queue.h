#ifndef MAPLAB_COMMON_FIXED_SIZE_QUEUE_H_
#define MAPLAB_COMMON_FIXED_SIZE_QUEUE_H_

#include <deque>

#include <glog/logging.h>

#include <aslam/common/memory.h>

namespace common {

template <typename ElementType>
class FixedSizeQueue {
 public:
  typedef Aligned<std::deque, ElementType> BufferType;

  explicit FixedSizeQueue(size_t max_buffer_size)
      : max_buffer_size_(max_buffer_size) {
    CHECK_GT(max_buffer_size, 0u);
  }

  void insert(const ElementType& element) {
    buffer_.emplace_back(element);
    // Remove the oldest element if the buffer is full.
    if (size() > max_buffer_size_) {
      buffer_.pop_front();
    }
  }

  void clear() {
    buffer_.clear();
  }

  size_t size() const {
    return buffer_.size();
  }

  size_t isFull() const {
    return buffer_.size() >= max_buffer_size_;
  }

  const BufferType& buffer() const {
    return buffer_;
  }

 private:
  BufferType buffer_;
  const size_t max_buffer_size_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_FIXED_SIZE_QUEUE_H_
