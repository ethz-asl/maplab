#ifndef VOCABULARY_TREE_HAMMING_INL_H_
#define VOCABULARY_TREE_HAMMING_INL_H_

#if !defined(__ARM_NEON__)
#include <emmintrin.h>
#include <tmmintrin.h>
#endif  // __ARM_NEON__

#include <glog/logging.h>

namespace loop_closure {

#ifdef __GNUC__
static const char __attribute__((aligned(16)))
MASK_4bit[16] = {0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
                 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
static const uint8_t __attribute__((aligned(16)))
POPCOUNT_4bit[16] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
#endif  // __GNUC__
#ifdef _MSC_VER
__declspec(align(16)) static const
    char MASK_4bit[16] = {0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf,
                          0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf, 0xf};
__declspec(align(16)) static const uint8_t POPCOUNT_4bit[16] = {
    0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};
static const __m128i shiftval = _mm_set_epi32(0, 0, 0, 4);
#endif  // _MSC_VER

#if !defined(__ARM_NEON__)
template <int NUM_SSE_REGISTERS>
inline unsigned int SSSE3PopcntofXORed(
    const __m128i* signature1, const __m128i* signature2) {
  CHECK_NOTNULL(signature1);
  CHECK_NOTNULL(signature2);

  uint32_t result = 0;

  __m128i xmm0;
  __m128i xmm1;
  __m128i xmm2;
  __m128i xmm3;
  __m128i xmm4;
  __m128i xmm5;
  __m128i xmm6;
  __m128i xmm7;

  xmm7 = _mm_load_si128(reinterpret_cast<const __m128i*>(POPCOUNT_4bit));
  xmm6 = _mm_load_si128(reinterpret_cast<const __m128i*>(MASK_4bit));
  xmm5 = _mm_setzero_si128();  // xmm5 -- global accumulator.

  const __m128i* end = signature1 + NUM_SSE_REGISTERS;
  xmm4 = xmm5;  // xmm4 -- local accumulator.
  const __m128i shiftval = _mm_set_epi32(0, 0, 0, 4);
  do {
    xmm0 = _mm_xor_si128(*signature1++, *signature2++);
    xmm1 = xmm0;
    xmm1 = _mm_srl_epi16(xmm1, shiftval);
    xmm0 = _mm_and_si128(xmm0, xmm6);  // xmm0 := lower nibbles.
    xmm1 = _mm_and_si128(xmm1, xmm6);  // xmm1 := higher nibbles.
    xmm2 = xmm7;
    xmm3 = xmm7;                          // Get popcount.
    xmm2 = _mm_shuffle_epi8(xmm2, xmm0);  // For all nibbles.
    xmm3 = _mm_shuffle_epi8(xmm3, xmm1);  // Using PSHUFB.
    xmm4 = _mm_add_epi8(xmm4, xmm2);      // Update local.
    xmm4 = _mm_add_epi8(xmm4, xmm3);      // Accumulator.
  } while (signature1 < end);
  // Update global accumulator(two 32-bits counters).
  xmm4 = _mm_sad_epu8(xmm4, xmm5);
  xmm5 = _mm_add_epi32(xmm5, xmm4);
  // Finally add together 32-bits counters stored in global accumulator.
  // TODO(slynen): fix with appropriate intrinsic
  // __asm__ volatile(
  //  "movhlps  %%xmm5, %%xmm0 \n"
  xmm0 = _mm_cvtps_epi32(
      _mm_movehl_ps(_mm_cvtepi32_ps(xmm0), _mm_cvtepi32_ps(xmm5)));
  xmm0 = _mm_add_epi32(xmm0, xmm5);
  result = _mm_cvtsi128_si32(xmm0);
  return result;
}
#elif defined(__aarch64__)
__inline__ uint32_t NEONPopcntofXORed(
    const uint8x16_t* signature1, const uint8x16_t* signature2,
    const int numberOf128BitWords) {
  CHECK_NOTNULL(signature1);
  CHECK_NOTNULL(signature2);
  uint32_t result = 0;

  for (int i = 0; i < numberOf128BitWords; ++i) {
    uint8x16_t xor_result = veorq_u8(signature1[i], signature2[i]);
    uint8x16_t set_bits = vcntq_u8(xor_result);
    uint8_t result_popcnt[16];
    vst1q_u8(result_popcnt, set_bits);
    for (int j = 0; j < 16; ++j) {
      result += result_popcnt[j];
    }
  }
  return result;
}
#endif

// Hamming distance for 128 bits.
inline unsigned int HammingDistance128(
    const unsigned char d1[16], const unsigned char d2[16]) {
#ifdef __ARM_NEON__
#ifdef __aarch64__
  const uint8x16_t* a = reinterpret_cast<const uint8x16_t*>(d1);
  const uint8x16_t* b = reinterpret_cast<const uint8x16_t*>(d2);

  return NEONPopcntofXORed(a, b, 1);
#else   // __aarch64__
  unsigned int out_value;

  __asm__ volatile(
      // Load constants.
      "vmov.u8             q12, #0x55          \n\t"  // m1.
      "vmov.u8             q13, #0x33          \n\t"  // m2.
      "vmov.u8             q14, #0x0f          \n\t"  // m4.
      "vmov.u8             q15, #0x01          \n\t"  // h01.

      // Load d1.
      "vld1.32             {q0}, [%1]          \n\t"  // load the first half of
                                                      // d1 (16 bytes)  //
                                                      // NOLINT

      // Load d2.
      "vld1.32             {q1}, [%2]          \n\t"  // load the first half of
                                                      // d2 (16 bytes)  //
                                                      // NOLINT

      // xor.
      "veor.32             q0, q0, q1          \n\t"  // xor of first half.

      // x -= (x >> 1) & m1;
      "vshr.u32            q1, q0, #1           \n\t"  // Right shift by 1.
      "vand.32             q1, q1, q12         \n\t"   // and with m1.
      "vsub.u32            q0, q0, q1          \n\t"   // Subtract.

      // x = (x & m2) + ((x >> 2) & m2);
      "vand.32             q1, q0, q13         \n\t"
      "vshr.u32            q2, q0, #2           \n\t"
      "vand.32             q2, q2, q13         \n\t"
      "vadd.u32            q0, q1, q2          \n\t"

      // x = (x + (x >> 4)) & m4;
      "vshr.u32            q1, q0, #4           \n\t"
      "vadd.u32            q0, q0, q1          \n\t"
      "vand.32             q0, q0, q14         \n\t"

      // (x * h01) >> 24;
      "vmul.u32            q0, q0, q15         \n\t"
      "vshr.u32            q0, q0, #24          \n\t"

      // sum distances
      "vpadd.u32           d0, d0, d1          \n\t"
      "vpadd.u32           d0, d0, d0          \n\t"

      // Move distance to return register.
      "vmov.32             %0, d0[0]           \n\t"

      // Output.
      : "=r"(out_value), "+r"(d1), "+r"(d2)
      // input
      :
      // Clobber list.
      : "q0", "q1", "q2", "q12", "q13", "q14", "q15");

  return out_value;
#endif  // __aarch64__
#else   // __ARM_NEON__
  const __m128i* a = reinterpret_cast<const __m128i*>(d1);
  const __m128i* b = reinterpret_cast<const __m128i*>(d2);

  return SSSE3PopcntofXORed<1>(a, b);
#endif  // __ARM_NEON__
}

// Hamming distance for 256 bits.
inline unsigned int HammingDistance256(
    const unsigned char d1[32], const unsigned char d2[32]) {
#ifdef __ARM_NEON__
#ifdef __aarch64__
  const uint8x16_t* a = reinterpret_cast<const uint8x16_t*>(d1);
  const uint8x16_t* b = reinterpret_cast<const uint8x16_t*>(d2);

  return NEONPopcntofXORed(a, b, 2);
#else   // __aarch64__
  unsigned int out_value;

  __asm__ volatile(
      "pld                 [%1, #32]           \n\t"
      "pld                 [%2, #32]           \n\t"

      // Load constants.
      "vmov.u8             q12, #0x55          \n\t"  // m1.
      "vmov.u8             q13, #0x33          \n\t"  // m2.
      "vmov.u8             q14, #0x0f          \n\t"  // m4.
      "vmov.u8             q15, #0x01          \n\t"  // h01.

      // Load d1
      "vld1.32             {q0,q1}, [%1]       \n\t"  // load d1.

      // Load d2
      "vld1.32             {q2, q3}, [%2]      \n\t"  // load d2.

      // xor
      "veor.32             q0, q0, q2          \n\t"  // xor left side.
      "veor.32             q3, q1, q3          \n\t"  // xor right side.

      // x -= (x >> 1) & m1;
      "vshr.u32            q1, q0, #1           \n\t"
      "vshr.u32            q4, q3, #1           \n\t"
      "vand.32             q1, q1, q12         \n\t"
      "vand.32             q4, q4, q12         \n\t"
      "vsub.u32            q0, q0, q1          \n\t"
      "vsub.u32            q3, q3, q4          \n\t"

      // x = (x & m2) + ((x >> 2) & m2);
      "vand.32             q1, q0, q13         \n\t"
      "vand.32             q4, q3, q13         \n\t"
      "vshr.u32            q2, q0, #2           \n\t"
      "vshr.u32            q5, q3, #2           \n\t"
      "vand.32             q2, q2, q13         \n\t"
      "vand.32             q5, q5, q13         \n\t"
      "vadd.u32            q0, q1, q2          \n\t"
      "vadd.u32            q3, q4, q5          \n\t"

      // x = (x + (x >> 4)) & m4;
      "vshr.u32            q1, q0, #4           \n\t"
      "vshr.u32            q4, q3, #4           \n\t"
      "vadd.u32            q0, q0, q1          \n\t"
      "vadd.u32            q3, q3, q4          \n\t"
      "vand.32             q0, q0, q14         \n\t"
      "vand.32             q3, q3, q14         \n\t"

      // (x * h01) >> 24;
      "vmul.u32            q0, q0, q15         \n\t"
      "vmul.u32            q3, q3, q15         \n\t"
      "vshr.u32            q0, q0, #24          \n\t"
      "vshr.u32            q3, q3, #24          \n\t"

      // sum distances
      "vpadd.u32           d0, d0, d1          \n\t"
      "vpadd.u32           d6, d6, d7          \n\t"
      "vpadd.u32           d0, d0, d0          \n\t"
      "vpadd.u32           d6, d6, d6          \n\t"

      // add d0,d6.
      "vadd.u32            d0, d0, d6          \n\t"

      // Move distance to return register.
      "vmov.32             %0, d0[0]           \n\t"

      // Output.
      : "=r"(out_value), "+r"(d1), "+r"(d2)
      // input
      :
      // Clobber list.
      : "q0", "q1", "q2", "q3", "q4", "q5", "q12", "q13", "q14", "q15");

  return out_value;
#endif  // __aarch64__
#else   // __ARM_NEON__
  const __m128i* a = reinterpret_cast<const __m128i*>(d1);
  const __m128i* b = reinterpret_cast<const __m128i*>(d2);

  return SSSE3PopcntofXORed<2>(a, b);
#endif  // __ARM_NEON__
}

// Hamming distance for 512 bits.
inline unsigned int HammingDistance512(
    const unsigned char d1[64], const unsigned char d2[64]) {
#ifdef __ARM_NEON__
#ifdef __aarch64__
  const uint8x16_t* a = reinterpret_cast<const uint8x16_t*>(d1);
  const uint8x16_t* b = reinterpret_cast<const uint8x16_t*>(d2);

  return NEONPopcntofXORed(a, b, 4);
#else
  return HammingDistance256(d1, d2) + HammingDistance256(d1 + 32, d2 + 32);
#endif
#else
  const __m128i* a = reinterpret_cast<const __m128i*>(d1);
  const __m128i* b = reinterpret_cast<const __m128i*>(d2);

  return SSSE3PopcntofXORed<4>(a, b);
#endif
}

// Hamming distance for different vector sizes.
inline unsigned int HammingDistance(
    const unsigned char* d1, const unsigned char* d2, unsigned int numBits) {
  switch (numBits) {
    case 128:
      return HammingDistance128(d1, d2);
    case 256:
      return HammingDistance256(d1, d2);
    case 512:
      return HammingDistance512(d1, d2);
    default:
      LOG(FATAL) << "Unsupported number of bits in descriptors.";
      return 0;
  }
}
}  // namespace loop_closure
#endif  // VOCABULARY_TREE_HAMMING_INL_H_
