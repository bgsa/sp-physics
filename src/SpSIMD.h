#ifndef SP_SIMD_HEADER
#define SP_SIMD_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

#ifdef AVX_ENABLED

	#define sp_create_simd4f(value1, value2, value3, value4) _mm_set_ps(value4, value3, value2, value1)
	#define sp_vec4_create_simd1f(value) _mm_set_ps(value, value, value, value)
	#define sp_vec4_div_simd(vec4_simd1, vec4_simd2) _mm_div_ps(vec4_simd1, vec4_simd2)


	#define sp_vec3_sqrt_sse(vec3_sse) _mm_sqrt_ss(vec3_sse)

	#define sp_vec3_convert_simd(v) _mm_set_ps(ZERO_FLOAT, v.z, v.y, v.x)
	#define sp_vec3_convert_ref_simd(v) _mm_set_ps(ZERO_FLOAT, v->z, v->y, v->x)

	#define sp_vec3_add_simd(vec3_simd1, vec3_simd2) _mm_add_ps(vec3_simd1, vec3_simd2)

		// result = (vec3_simd.x + vec3_simd.y + vec3_simd.z)
	#define sp_vec3_add_vertical_simd(vec3_simd, vec3_simd_output) \
			vec3_simd_output = _mm_hadd_ps(vec3_simd, vec3_simd); \
			vec3_simd_output = _mm_hadd_ps(vec3_simd_output, vec3_simd_output)

	#define sp_vec3_sub_simd(vec3_simd1, vec3_simd2) _mm_sub_ps(vec3_simd1, vec3_simd2)

	#define sp_vec3_div_simd(vec3_simd1, vec3_simd2) _mm_div_ps(vec3_simd1, vec3_simd2)

	#define sp_vec3_sqrt_simd(vec3_simd) _mm_sqrt_ps(vec3_simd)
	#define sp_vec3_rsqrt_simd(vec3_simd) _mm_rsqrt_ps(vec3_simd)

	#define sp_vec3_dot_simd(vec3_simd1, vec3_simd2) _mm_dp_ps(vec3_simd1, vec3_simd2, 0xff)

	#define sp_vec3_mult_simd(vec3_simd1, vec3_simd2) _mm_mul_ps(vec3_simd1, vec3_simd2)

	#define sp_vec3_shuflle_simd(vec3_simd1, vec3_simd2, mask1, mask2, mask3, mask4) _mm_shuffle_ps(vec3_simd1, vec3_simd2, _MM_SHUFFLE(mask1, mask2, mask3, mask4))

	#define sp_vec3_length_simd(input_simd) \
			sp_vec3_sqrt_simd(sp_vec3_dot_simd(input_simd, input_simd))

	#define sp_vec3_cross_simd(vec3_simd1, vec3_simd2) \
			sp_vec3_sub_simd(sp_vec3_mult_simd(sp_vec3_shuflle_simd(vec3_simd2, vec3_simd2, 3, 0, 2, 1), sp_vec3_shuflle_simd(vec3_simd1, vec3_simd1, 3, 1, 0, 2)), sp_vec3_mult_simd(sp_vec3_shuflle_simd(vec3_simd2, vec3_simd2, 3, 1, 0, 2), sp_vec3_shuflle_simd(vec3_simd1, vec3_simd1, 3, 0, 2, 1)))

	#define sp_vec3_normalize_simd(vec3_simd) \
			sp_vec3_mult_simd(vec3_simd,  sp_vec3_rsqrt_simd(sp_vec3_dot_simd(vec3_simd, vec3_simd)))

	#define sp_vec3_normal_simd(vec3_simd1, vec3_simd2, vec3_simd3) \
			sp_vec3_normalize_simd(sp_vec3_cross_simd(sp_vec3_sub_simd(vec3_simd3, vec3_simd2), sp_vec3_sub_simd(vec3_simd2, vec3_simd1)))

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


	// Plane3D
	#define sp_plane3D_distance_simd(normal_simd, target_simd, output_simd) \
		const __m128 divisor_simd = sp_vec3_dot_simd(normal_simd, normal_simd);   \
		__m128 normalDotTarget_simd = sp_vec3_dot_simd(normal_simd, target_simd); \
		normalDotTarget_simd = sp_vec3_sub_simd(normalDotTarget_simd, sp_vec4_create_simd1f(distanceFromOrigin)); \
		output_simd = sp_vec3_div_simd(normalDotTarget_simd, divisor_simd);

	#define sp_plane3D_intersection_ray_simd(normal_simd, ray_point_simd, ray_direction_simd, contact_simd, hasIntersection) \
		const __m128 _angle = sp_vec3_dot_simd(normal_simd, ray_direction_simd); \
		if (isCloseEnough(_angle.m128_f32[0], ZERO_FLOAT)) \
				hasIntersection = false; \
		else { \
			hasIntersection = true;														\
			__m128 temp1 = sp_vec3_dot_simd(normal_simd, ray_point_simd);				\
			temp1 = sp_vec3_sub_simd(sp_vec4_create_simd1f(distanceFromOrigin), temp1); \
			temp1 = sp_vec3_div_simd(temp1, _angle);									\
			temp1 = sp_vec3_mult_simd(ray_direction_simd, temp1);						\
			contact_simd = sp_vec3_add_simd(ray_point_simd, temp1);						\
		}

	#define sp_plane3D_intersection_line(normal_simd, d_simd, line_point1_simd, line_point2_simd, contact_simd, hasIntersection) \
		const __m128 line_direction_simd = sp_vec3_sub_simd(line_point2_simd, line_point1_simd); \
		const __m128 _angle = sp_vec3_dot_simd(normal_simd, line_direction_simd);				 \
		hasIntersection = false;																 \
	\
		if (!isCloseEnough(_angle.m128_f32[0], ZERO_FLOAT))										 \
		{																						 \
			__m128 temp1 = sp_vec3_dot_simd(normal_simd, line_point1_simd);						 \
			temp1 = sp_vec3_sub_simd(d_simd, temp1);	 										 \
			temp1 = sp_vec3_div_simd(temp1, _angle);											 \
	\
			const sp_float t = temp1.m128_f32[0];												 \
			if (t >= -_epsilon && t <= ONE_FLOAT + _epsilon)									 \
			{																					 \
				temp1 = sp_vec3_mult_simd(line_direction_simd, temp1);							 \
				contact_simd = sp_vec3_add_simd(line_point1_simd, temp1);						 \
				hasIntersection = true;															 \
			}																					 \
		}

	// Triangle3D
	#define sp_triangle3D_area_simd(a_simd, b_simd, c_simd, output_simd)  \
		output_simd = sp_vec3_length_simd(sp_vec3_cross_simd(sp_vec3_sub_simd(b_simd, a_simd), sp_vec3_sub_simd(c_simd, a_simd))); \
		output_simd = sp_vec3_mult_simd(output_simd, sp_vec4_create_simd1f(HALF_FLOAT));

	#define sp_triangle3D_isInside_simd(point1_simd, point2_simd, point3_simd, target_simd, output) \
		const __m128 half_simd = sp_vec4_create_simd1f(HALF_FLOAT);					\
		__m128 ab_simd = sp_vec3_sub_simd(point2_simd, point1_simd);				\
		__m128 ac_simd = sp_vec3_sub_simd(point3_simd, point1_simd);				\
		__m128 temp = sp_vec3_length_simd(sp_vec3_cross_simd(ab_simd, ac_simd));	\
		__m128 total_simd = sp_vec3_mult_simd(temp, half_simd);						\
		\
		ab_simd = sp_vec3_sub_simd(point2_simd, point1_simd);						\
		ac_simd = sp_vec3_sub_simd(target_simd, point1_simd);						\
		temp = sp_vec3_length_simd(sp_vec3_cross_simd(ab_simd, ac_simd));			\
		__m128 sum_area = sp_vec3_mult_simd(temp, half_simd);						\
		\
		ab_simd = sp_vec3_sub_simd(point3_simd, point2_simd);						\
		ac_simd = sp_vec3_sub_simd(target_simd, point2_simd);						\
		temp = sp_vec3_length_simd(sp_vec3_cross_simd(ab_simd, ac_simd));			\
		sum_area = sp_vec3_add_simd(sum_area, sp_vec3_mult_simd(temp, half_simd));	\
		\
		ab_simd = sp_vec3_sub_simd(point1_simd, point3_simd);						\
		ac_simd = sp_vec3_sub_simd(target_simd, point3_simd);						\
		temp = sp_vec3_length_simd(sp_vec3_cross_simd(ab_simd, ac_simd));			\
		sum_area = sp_vec3_add_simd(sum_area, sp_vec3_mult_simd(temp, half_simd));	\
		\
		output = isCloseEnough(total_simd.m128_f32[0], sum_area.m128_f32[0], total_simd.m128_f32[0] * _epsilon);


	// Line3D
	#define sp_line3D_isPerpendicular_simd(line1_point1, line1_point2, line2_point1, line2_point2) \
		isCloseEnough( \
			sp_vec3_dot_simd( \
				sp_vec3_sub_simd(sp_vec3_convert_simd(line1_point2), sp_vec3_convert_simd(line1_point1)), \
				sp_vec3_sub_simd(sp_vec3_convert_simd(line2_point2), sp_vec3_convert_simd(line2_point1))  \
			).m128_f32[0] \
			, ZERO_FLOAT, _epsilon)

	#define sp_line3D_isPerpendicular2_simd(line_point1, line_point2, direction) \
		isCloseEnough( \
			sp_vec3_dot_simd( \
				sp_vec3_sub_simd(sp_vec3_convert_simd(line_point2), sp_vec3_convert_simd(line_point1)), \
				direction																				\
			).m128_f32[0] \
			, ZERO_FLOAT, _epsilon)

#endif

}

#endif // SP_SIMD_HEADER