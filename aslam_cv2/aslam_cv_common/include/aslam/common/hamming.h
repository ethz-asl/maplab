/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ASLAM_COMMON_HAMMING_H_
#define ASLAM_COMMON_HAMMING_H_

#ifdef __ARM_NEON
#include <arm_neon.h>
#else
#include <emmintrin.h>
#include <tmmintrin.h>
#endif  // __ARM_NEON

namespace aslam {
namespace common {
// Faster Hamming distance functor - uses SSE
// bit count of A exclusive XOR'ed with B.
class  Hamming {
 public:
  Hamming() { }

#ifdef __ARM_NEON
  static __inline__ uint32_t NEONPopcntofXORed(const uint8x16_t* signature1,
                                               const uint8x16_t* signature2,
                                               const int numberOf128BitWords);
  static __inline__ uint32_t PopcntofXORed(const unsigned char* signature1,
                                           const unsigned char* signature2,
                                           const int numberOf128BitWords) {
    return NEONPopcntofXORed(reinterpret_cast<const uint8x16_t*>(signature1),
                             reinterpret_cast<const uint8x16_t*>(signature2),
                             numberOf128BitWords);
  }
  static __inline__ uint32_t PopcntofXORed(const uint8x16_t* signature1,
                                           const uint8x16_t* signature2,
                                           const int numberOf128BitWords) {
    return NEONPopcntofXORed(signature1, signature2, numberOf128BitWords);
  }
#else
  static __inline__ uint32_t SSSE3PopcntofXORed(const __m128i* signature1,
                                                const __m128i* signature2,
                                                const int numberOf128BitWords);
  static __inline__ uint32_t PopcntofXORed(const __m128i* signature1,
                                           const __m128i* signature2,
                                           const int numberOf128BitWords) {
    return SSSE3PopcntofXORed(signature1, signature2, numberOf128BitWords);
  }
  static __inline__ uint32_t PopcntofXORed(const unsigned char* signature1,
                                           const unsigned char* signature2,
                                           const int numberOf128BitWords) {
    return SSSE3PopcntofXORed(reinterpret_cast<const __m128i*>(signature1),
                              reinterpret_cast<const __m128i*>(signature2),
                              numberOf128BitWords);
  }
#endif  // __ARM_NEON

  typedef unsigned char ValueType;

  // Important that this is signed as weird behavior happens in BruteForce if
  // not.
  typedef int ResultType;

  static ResultType evaluate(const unsigned char* a,
                             const unsigned char* b,
                             const int size) {
#ifdef __ARM_NEON
    return NEONPopcntofXORed(reinterpret_cast<const uint8x16_t*>(a),
                             reinterpret_cast<const uint8x16_t*>(b),
                             size / 16);
#else
    return SSSE3PopcntofXORed(reinterpret_cast<const __m128i*>(a),
                              reinterpret_cast<const __m128i*>(b),
                              size / 16);
#endif  // __ARM_NEON
  }

  // This will count the bits in a ^ b.
  inline ResultType operator()(const unsigned char* a,
                               const unsigned char* b,
                               const int size) const {
    return evaluate(a, b, size);
  }
};
}  // namespace common
}  // namespace aslam

#include "./hamming-inl.h"

#endif  // ASLAM_COMMON_HAMMING_H_

