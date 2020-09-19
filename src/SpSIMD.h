#ifndef SP_SIMD_HEADER
#define SP_SIMD_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

#ifdef AVX_ENABLED

	#define sp_vec3_sqrt_sse(vec3_sse) _mm_sqrt_ss(vec3_sse)

	#define sp_vec3_convert_simd(v) _mm_set_ps(ZERO_FLOAT, v.z, v.y, v.x)
	#define sp_vec3_convert_ref_simd(v) _mm_set_ps(ZERO_FLOAT, v->z, v->y, v->x)

	#define sp_vec3_add_simd(vec3_simd1, vec3_simd2) _mm_add_ps(vec3_simd1, vec3_simd2)

		// result = (vec3_simd.x + vec3_simd.y + vec3_simd.z)
	#define sp_vec3_add_vertical_simd(vec3_simd, vec3_simd_output) \
			vec3_simd_output = _mm_hadd_ps(vec3_simd, vec3_simd); \
			vec3_simd_output = _mm_hadd_ps(vec3_simd_output, vec3_simd_output)

	#define sp_vec3_sub_simd(vec3_simd1, vec3_simd2) _mm_sub_ps(vec3_simd1, vec3_simd2)

	#define sp_vec3_sqrt_simd(vec3_simd) _mm_sqrt_ps(vec3_simd)
	#define sp_vec3_rsqrt_simd(vec3_simd) _mm_rsqrt_ps(vec3_simd)

	#define sp_vec3_dot_simd(vec3_simd1, vec3_simd2) _mm_dp_ps(vec3_simd1, vec3_simd2, 0xff)

	#define sp_vec3_mult_simd(vec3_simd1, vec3_simd2) _mm_mul_ps(vec3_simd1, vec3_simd2)

	#define sp_vec3_shuflle_simd(vec3_simd1, vec3_simd2, mask1, mask2, mask3, mask4) _mm_shuffle_ps(vec3_simd1, vec3_simd2, _MM_SHUFFLE(mask1, mask2, mask3, mask4))

	#define sp_vec3_length_simd(vec3_simd_input, vec3_simd_output) \
			__m128 v1 = sp_vec3_convert_simd(vec3_simd_input); \
			v1 = sp_vec3_dot_simd(v1, v1); \
			vec3_simd_output = sp_vec3_sqrt_simd(v1)

	#define sp_vec3_cross_simd(vec3_simd1, vec3_simd2, vec3_simd_output) \
			const __m128 swp0 = sp_vec3_shuflle_simd(vec3_simd2, vec3_simd2, 3, 0, 2, 1); \
			const __m128 swp1 = sp_vec3_shuflle_simd(vec3_simd2, vec3_simd2, 3, 1, 0, 2); \
			const __m128 swp2 = sp_vec3_shuflle_simd(vec3_simd1, vec3_simd1, 3, 0, 2, 1); \
			const __m128 swp3 = sp_vec3_shuflle_simd(vec3_simd1, vec3_simd1, 3, 1, 0, 2); \
			const __m128 mul0 = sp_vec3_mult_simd(swp0, swp3); \
			const __m128 mul1 = sp_vec3_mult_simd(swp1, swp2); \
			vec3_simd_output = sp_vec3_sub_simd(mul0, mul1)

	#define sp_vec3_normalize_simd(vec3_simd, vec3_simd_output)     \
			const __m128 vDot = sp_vec3_dot_simd(vec3_simd, vec3_simd); \
			const __m128 vDotSquaredRoot = sp_vec3_rsqrt_simd(vDot);    \
			vec3_simd_output = sp_vec3_mult_simd(vec3_simd, vDotSquaredRoot)

	#define sp_vec3_normal_simd(vec3_simd1, vec3_simd2, vec3_simd3, vec3_simd_output) \
			const __m128 line1 = sp_vec3_sub_simd(vec3_simd3, vec3_simd2); \
			const __m128 line2 = sp_vec3_sub_simd(vec3_simd2, vec3_simd1); \
			sp_vec3_cross_simd(line1, line2, const __m128 crossedVec);     \
			sp_vec3_normalize_simd(crossedVec, vec3_simd_output)

	#define sp_vec3_distance_simd(vec3_simd1, vec3_simd2, vec3_simd_output) \
			__m128 tempDist = sp_vec3_sub_simd(vec3_simd1, vec3_simd2); \
			__m128 rDist = sp_vec3_mult_simd(tempDist, tempDist); \
			sp_vec3_add_vertical_simd(rDist, rDist); \
			vec3_simd_output = sp_vec3_sqrt_sse(rDist)


	#define sp_quat_convert_simd(quat) _mm_set_ps(quat.z, quat.y, quat.x, quat.w)

		const __m128 sp_quat_conjugated_mult = _mm_set_ps(-ONE_FLOAT, -ONE_FLOAT, -ONE_FLOAT, ONE_FLOAT);
		const __m128 sp_quat_not_w = _mm_set_ps(ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, -ONE_FLOAT);
		const __m128 sp_quat_not_x = _mm_set_ps(ONE_FLOAT, ONE_FLOAT, -ONE_FLOAT, ONE_FLOAT);
		const __m128 sp_quat_not_y = _mm_set_ps(ONE_FLOAT, -ONE_FLOAT, ONE_FLOAT, ONE_FLOAT);
		const __m128 sp_quat_not_z = _mm_set_ps(-ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT);

	#define sp_quat_conjugate_simd(quat_simd) _mm_mul_ps(quat_simd, quat_conjugated_mult)

	#define sp_quat_mult(quat_simd1, quat_simd2, quat_simd_output) \
		__m128 rowW = _mm_mul_ps(q1, q2);												\
		__m128 rowX = _mm_mul_ps(q1, _mm_shuffle_ps(q2, q2, _MM_SHUFFLE(2, 3, 0, 1)));  \
		__m128 rowY = _mm_mul_ps(q1, _mm_shuffle_ps(q2, q2, _MM_SHUFFLE(1, 0, 3, 2)));  \
		__m128 rowZ = _mm_mul_ps(q1, _mm_shuffle_ps(q2, q2, _MM_SHUFFLE(0, 2, 1, 3)));  \
		__m128 tempMultW = _mm_hsub_ps(rowW, rowW);    \
		tempMultW = _mm_hsub_ps(tempMultW, tempMultW); \
		__m128 tempMultX = _mm_hadd_ps(rowX, rowX);    \
		tempMultX = _mm_hsub_ps(tempMultX, tempMultX); \
		rowY = _mm_mul_ps(rowY, sp_quat_not_w);        \
		__m128 tempMultY = _mm_hadd_ps(rowY, rowY);    \
		tempMultY = _mm_hadd_ps(tempMultY, tempMultY); \
		rowZ = _mm_mul_ps(rowZ, sp_quat_not_y);        \
		__m128 tempMultZ = _mm_hadd_ps(rowZ, rowZ);    \
		tempMultZ = _mm_hadd_ps(tempMultZ, tempMultZ); \
		tempMultW = _mm_shuffle_ps(tempMultW, tempMultX, _MM_SHUFFLE(0, 1, 2, 3)); \
		tempMultW = _mm_permute_ps(tempMultW, _MM_SHUFFLE(0, 2, 1, 3));            \
		tempMultY = _mm_shuffle_ps(tempMultY, tempMultZ, _MM_SHUFFLE(0, 1, 2, 3)); \
		tempMultY = _mm_permute_ps(tempMultY, _MM_SHUFFLE(0, 2, 1, 3));            \
		quat_simd_output = _mm_shuffle_ps(tempMultW, tempMultY, _MM_SHUFFLE(0, 1, 2, 3))

#endif

}

#endif // SP_SIMD_HEADER