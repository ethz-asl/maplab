#ifndef MAPLAB_COMMON_SIGNALS_H_
#define MAPLAB_COMMON_SIGNALS_H_

#include <cstdint>
#include <vector>

namespace common {
namespace signals {

// Compute FFT of a complex signal inplace
// Implementation from S.A. Teukolsky, et al. in Numerical Recipes in C++.
//
// Signal length must be a power of 2. Format assumes each even element of
// signal contains the real part of the signal and correspondingly each uneven
// element contains the complex part.
void computeFFT(std::vector<double>* signal_);

// Same as above, but store the resulting transform in fft.
void computeFFT(const std::vector<double>& signal, std::vector<double>* fft);

// Compute the inverse FFT using the computeFFT. Therefore same requirements on
// signal format and length as computeFFT.
void computeIFFT(std::vector<double>* signal_);
void computeIFFT(const std::vector<double>& fft, std::vector<double>* signal);

// Convert a real signal to the format required by computeFFT.
// Can be done inplace by passing a pointer to the original signal for result.
void toComplexSignal(
    const std::vector<double>& signal, std::vector<double>* result);

// From a complex signal in the format used by computeFFT extract the real part
// Can be done inplace by passing a pointer to the original signal for result
void toRealSignal(
    const std::vector<double>& signal, std::vector<double>* result);

// Perform linear convolution to between two array of real data using FFT
void convolve(
    const std::vector<double>& A, const std::vector<double>& B,
    std::vector<double>* result);

// Perform linear cross correlation between real signals using FFT
void correlate(
    const std::vector<double>& A, const std::vector<double>& B,
    std::vector<double>* result);

// Which part of the filtered signal that has been zero padded to return
enum Padding { FULL, SAME, VALID };

// Same as convolution, with option to specify output size
void filter(
    const std::vector<double>& signal, const std::vector<double>& filter,
    std::vector<double>* result, Padding padding = FULL);

}  // namespace signals
}  // namespace common

#endif  // MAPLAB_COMMON_SIGNALS_H_
