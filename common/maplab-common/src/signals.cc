#include "maplab-common/signals.h"

#include <cmath>
#include <functional>

#include <glog/logging.h>

namespace common {
namespace signals {

void computeFFT(std::vector<double>* signal_) {
  CHECK_NOTNULL(signal_);
  std::vector<double>& signal = *signal_;
  CHECK_GT(signal.size(), 0u);

  // Check power of two.
  CHECK_EQ(signal.size() & (signal.size() - 1), 0u)
      << "This FFT implementation only handles signals of length 2^n.";
  // Check even number of elements.
  CHECK_EQ(signal.size() & 1, 0u);

  // Reverse binary reindexing.
  uint64_t i, j, n, m;

  n = signal.size() >> 1;
  j = 1;
  for (i = 1; i < signal.size(); i += 2) {
    if (j > i) {
      std::swap(signal[j - 1], signal[i - 1]);
      std::swap(signal[j], signal[i]);
    }
    m = n;
    while (m >= 2 && j > m) {
      j -= m;
      m >>= 1;
    }
    j += m;
  }

  // Danielson-Lanczos algorithm.
  uint64_t mmax, istep;
  double wtemp, wr, wpr, wpi, wi, theta, tempr, tempi;

  mmax = 2;
  while (signal.size() > mmax) {
    istep = mmax << 1;
    theta = -(2 * M_PI / mmax);
    wtemp = std::sin(0.5 * theta);
    wpr = -2.0 * wtemp * wtemp;
    wpi = std::sin(theta);
    wr = 1.0;
    wi = 0.0;
    for (m = 1; m < mmax; m += 2) {
      for (i = m; i < signal.size(); i += istep) {
        j = i + mmax;
        tempr = wr * signal[j - 1] - wi * signal[j];
        tempi = wr * signal[j] + wi * signal[j - 1];

        signal[j - 1] = signal[i - 1] - tempr;
        signal[j] = signal[i] - tempi;
        signal[i - 1] += tempr;
        signal[i] += tempi;
      }
      wtemp = wr;
      wr += wr * wpr - wi * wpi;
      wi += wi * wpr + wtemp * wpi;
    }
    mmax = istep;
  }
}

void computeFFT(const std::vector<double>& signal, std::vector<double>* fft) {
  CHECK_NOTNULL(fft);
  *fft = signal;
  computeFFT(fft);
}

void computeIFFT(std::vector<double>* signal_) {
  CHECK_NOTNULL(signal_);
  std::vector<double>& signal = *signal_;

  for (size_t i = 1; i < signal.size(); i += 2) {
    signal[i] *= -1.0;
  }

  computeFFT(&signal);

  for (size_t i = 1; i < signal.size(); i += 2) {
    signal[i] *= -1.0;
  }

  double N = signal.size() / 2.0;
  for (size_t i = 0; i < signal.size(); ++i) {
    signal[i] /= N;
  }
}

void computeIFFT(const std::vector<double>& fft, std::vector<double>* signal) {
  CHECK_NOTNULL(signal);
  *signal = fft;
  computeIFFT(signal);
}

void toComplexSignal(
    const std::vector<double>& signal, std::vector<double>* result) {
  CHECK_NOTNULL(result);
  size_t real_size = signal.size();
  result->resize(2 * real_size);

  for (int64_t i = real_size - 1; i >= 0; --i) {
    (*result)[i << 1] = signal[i];
    (*result)[(i << 1) + 1] = 0;
  }
}

void toRealSignal(
    const std::vector<double>& signal, std::vector<double>* result) {
  CHECK_NOTNULL(result);
  size_t complex_size = signal.size();
  result->resize(complex_size / 2);

  for (size_t i = 0; i < complex_size; i += 2) {
    (*result)[i >> 1] = signal[i];
  }
}

void convolve(
    const std::vector<double>& A, const std::vector<double>& B,
    std::vector<double>* result) {
  CHECK_NOTNULL(result);

  size_t min_size, pow2_size;
  min_size = A.size() + B.size() - 1;

  // Find minimum required FFT length.
  pow2_size = 1;
  while (pow2_size < min_size) {
    pow2_size *= 2;
  }

  std::vector<double> A_fft, B_fft;
  toComplexSignal(A, &A_fft);
  toComplexSignal(B, &B_fft);
  A_fft.resize(2 * pow2_size, 0);
  B_fft.resize(2 * pow2_size, 0);

  computeFFT(&A_fft);
  computeFFT(&B_fft);

  std::vector<double> result_fft(2 * pow2_size);
  for (size_t i = 1; i < result_fft.size(); i += 2) {
    result_fft[i] = A_fft[i] * B_fft[i - 1] + A_fft[i - 1] * B_fft[i];
    result_fft[i - 1] = A_fft[i - 1] * B_fft[i - 1] - A_fft[i] * B_fft[i];
  }

  computeIFFT(result_fft, result);

  toRealSignal(*result, result);
  result->resize(min_size);
}

void correlate(
    const std::vector<double>& A, const std::vector<double>& B,
    std::vector<double>* result) {
  CHECK_NOTNULL(result);

  // Flip B so we can use convolutions.
  std::vector<double> B_flip(B.size());
  for (size_t i = 0; i < B.size(); ++i) {
    B_flip[B.size() - i - 1] = B[i];
  }

  convolve(A, B_flip, result);
}

void filter(
    const std::vector<double>& signal, const std::vector<double>& filter,
    std::vector<double>* result, Padding padding) {
  convolve(signal, filter, result);

  if (padding == VALID) {
    // For a filter size of 1 the result is already in place
    // If the filter is bigger than the signal no values are valid
    if (filter.size() > 1 && signal.size() > filter.size()) {
      std::copy(
          result->begin() + filter.size() - 1,
          result->begin() + signal.size() + 1, result->begin());
    }

    // Resize to correct dimensions
    result->resize(std::max(size_t(0), signal.size() - filter.size() + 1));
  } else if (padding == SAME) {
    if (filter.size() > 1) {
      std::copy(
          result->begin() + filter.size() / 2,
          result->begin() + filter.size() / 2 + signal.size() + 1,
          result->begin());
    }

    result->resize(signal.size());
  }
}

}  // namespace signals
}  // namespace common
