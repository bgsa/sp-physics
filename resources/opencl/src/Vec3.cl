#include "OpenCLBase.cl"

#ifndef VEC3_HEADER
#define VEC3_HEADER

typedef struct Vec3
{
	sp_float x, y, z;
} Vec3;

inline void vec3_abs(const Vec3 vec, Vec3* result)
{
	result->x = fabs(vec.x);
	result->y = fabs(vec.y);
	result->z = fabs(vec.z);
}

inline void vec3_multiply_float(const Vec3 vec, const sp_float value, Vec3* result)
{
	result->x = vec.x * value;
	result->y = vec.y * value;
	result->z = vec.z * value;
}

inline void vec3_multiply_vec3(const Vec3 vec1, const Vec3 vec2, Vec3* result)
{
	result->x = vec1.x * vec2.x;
	result->y = vec1.y * vec2.y;
	result->z = vec1.z * vec2.z;
}

inline void vec3_minus_vec3(const Vec3 vec1, const Vec3 vec2, Vec3* result)
{
	result->x = vec1.x - vec2.x;
	result->y = vec1.y - vec2.y;
	result->z = vec1.z - vec2.z;
}

inline sp_bool vec3_lesserThanOrEqual_float(const Vec3 vec, const sp_float value)
{
	return
		vec.x <= value &&
		vec.y <= value &&
		vec.z <= value;
}

inline sp_bool vec3_graterThanOrEqual_float(const Vec3 vec, const sp_float value)
{
	return
		vec.x >= value &&
		vec.y >= value &&
		vec.z >= value;
}

inline sp_bool vec3_isCloseEnough_vec3(const Vec3 vec1, const Vec3 vec2, const sp_float epsilon)
{
	return 
		isCloseEnough(vec1.x, vec2.x, epsilon) &&
		isCloseEnough(vec1.y, vec2.y, epsilon) &&
		isCloseEnough(vec1.z, vec2.z, epsilon);
}

#endif // VEC3_HEADER