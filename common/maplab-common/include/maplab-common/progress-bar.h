#ifndef MAPLAB_COMMON_PROGRESS_BAR_H_
#define MAPLAB_COMMON_PROGRESS_BAR_H_

#include <stdio.h>

namespace common {

/// \brief Prints out progress on a single line.
class ProgressBar {
 public:
  /// The number of elements defines the range of the progress bar,
  /// i.e. it corresponds to the number of elements to process.
  explicit ProgressBar(size_t num_elements);
  virtual ~ProgressBar() {}

  /// Updates the progress bar given the current number of processed elements.
  /// Expecting num_elements_processed <= num_elements.
  void update(size_t num_elements_processed);
  /// \brief Increases the number of processed elements by one.
  void increment();
  /// Resets the progress bar. Assigns the new range of elements with
  /// num_elements
  /// and resets the num_elements_processed to zero.
  void reset(size_t num_elements);

 private:
  void print();

  size_t num_elements_;
  size_t num_elements_processed_;
};

}  // namespace common

#endif  // MAPLAB_COMMON_PROGRESS_BAR_H_
