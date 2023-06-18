/*
 Copyright(C) 2013 The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision(ICCV2011).

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

#ifndef ASLAM_NEON_HELPERS_H_
#define ASLAM_NEON_HELPERS_H_
#ifdef __ARM_NEON
#include <arm_neon.h>

namespace brisk {
inline uint8x16_t shuffle_epi8_neon(const uint8x16_t& lhs,
                                    const uint8x16_t& mask) {
  uint8_t tmp_mask[16];
  uint8_t value[16];
  vst1q_u8(tmp_mask, mask);
  vst1q_u8(value, lhs);
  uint8_t temp2_upper_array[16];
  for (int shuffleidx = 0; shuffleidx < 16; ++shuffleidx) {
    temp2_upper_array[shuffleidx] =
        (tmp_mask[shuffleidx] & 0x80)
        ? 0 :
            value[tmp_mask[shuffleidx] & 0x0F];
  }
  return vld1q_u8(&temp2_upper_array[0]);
}
}  // namespace brisk

#endif  // __ARM_NEON
#endif  // ASLAM_NEON_HELPERS_H_
