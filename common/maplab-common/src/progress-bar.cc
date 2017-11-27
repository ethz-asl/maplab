#include "maplab-common/progress-bar.h"

#include <glog/logging.h>
#include <iomanip>
#include <iostream>  //NOLINT
#include <math.h>
#include <string>

namespace common {

const std::string kPreText = "Progress: ";
const std::string kProgressSymbol = "#";
const std::string kFillSymbol = "-";
const std::string kPostText = "]";
const size_t kProgressBarWidth = 50u;
const double kProgressBarWidthFloatingPoint =
    static_cast<double>(kProgressBarWidth);

ProgressBar::ProgressBar(size_t num_elements)
    : num_elements_(static_cast<double>(num_elements)),
      num_elements_processed_(0u) {}

void ProgressBar::update(size_t num_elements_processed) {
  CHECK_LE(num_elements_processed, num_elements_)
      << "The given number of "
      << "processed elements (" << num_elements_processed << ") is greater than"
      << " the total number of elements to process (" << num_elements_ << "). "
      << "Use reset(new_total_number_of_elements) to reset the progress bar "
      << "to a new range (caution: this also resets the number of processed "
      << "elements to zero!).";
  num_elements_processed_ = num_elements_processed;
  print();
}

void ProgressBar::increment() {
  update(num_elements_processed_ + 1);
}

void ProgressBar::reset(size_t num_elements) {
  num_elements_ = num_elements;
  num_elements_processed_ = 0u;
  std::cout << std::endl;
}

void ProgressBar::print() {
  double num_progress_symbols = kProgressBarWidthFloatingPoint *
                                static_cast<double>(num_elements_processed_) /
                                static_cast<double>(num_elements_);
  double percentage =
      100.0 * static_cast<double>(num_elements_processed_) / num_elements_;

  size_t num_progress_symbols_integer = floor(num_progress_symbols);
  CHECK_LE(num_progress_symbols_integer, kProgressBarWidth)
      << "Number of "
      << "progress bar symbols is greater than the progress bar width. Make "
      << "sure the number of processed elements (" << num_elements_processed_
      << ") is at most equal to the total number of elements to be processed ("
      << num_elements_ << ")";

  std::cout << "\r" << kPreText << std::setfill(' ') << std::fixed
            << std::setw(5) << std::setprecision(1) << percentage << "% [";
  for (size_t idx = 0; idx < kProgressBarWidth; ++idx) {
    if (idx < num_progress_symbols_integer) {
      std::cout << kProgressSymbol;
    } else {
      std::cout << kFillSymbol;
    }
  }
  std::cout << kPostText << std::flush;

  if (num_elements_processed_ == num_elements_) {
    std::cout << std::endl << "Complete!" << std::endl;
  }
  std::cout << std::flush;
}

}  // namespace common
