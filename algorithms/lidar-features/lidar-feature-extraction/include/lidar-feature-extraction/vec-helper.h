#ifndef LIDAR_FEATURE_EXTRACTION_VEC_HELPER_H_
#define LIDAR_FEATURE_EXTRACTION_VEC_HELPER_H_

#include <lidar-feature-extraction/ouster-configuration.h>

#include <emmintrin.h>
#include <pmmintrin.h>
#include <smmintrin.h>
#include <xmmintrin.h>

namespace LidarFeatureExtraction {

static const __m128 rad2degConv = _mm_div_ps(_mm_set_ps1(180.0f), _mm_set_ps1(M_PI));

const __m128 ang_bottom_ps = _mm_set_ps1(ang_bottom_);
const __m128 ang_res_x_ps = _mm_set_ps1(ang_res_x_);
const __m128 ang_res_y_ps = _mm_set_ps1(ang_res_y_);
const __m128 n_scan_ps = _mm_set_ps1(beam_size_);
const __m128 horizon_scan_ps = _mm_set_ps1(ring_size_);
const __m128 horizon_scan_half_ps = _mm_set_ps1(ring_size_ / 2.0f);
const __m128 degree90_ps = _mm_set_ps1(90.0f);

const __m128 zero_ps = _mm_set_ps1(0.0f);
const __m128 tenThousand_ps = _mm_set_ps1(10000.0f);
const __m128 sign_mask = _mm_set1_ps(-0.f);
const __m128 max_range = _mm_set1_ps(255.0f);

inline __m128 rad2deg_ps(__m128 radian) { return _mm_mul_ps(radian, rad2degConv); }

}

#endif
