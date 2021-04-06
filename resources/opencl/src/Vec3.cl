#include "OpenCLBase.cl"

#ifndef VEC3_HEADER
#define VEC3_HEADER

typedef float3 Vec3;

#define vec3_up(vec) \
	vec.x =  0.0f;   \
	vec.y =  1.0f;   \
	vec.z =  0.0f;

#define vec3_down(vec) \
	vec.x =  0.0f;     \
	vec.y = -1.0f;     \
	vec.z =  0.0f;

#define vec3_left(vec) \
	vec.x = -1.0f;     \
	vec.y =  0.0f;     \
	vec.z =  0.0f;

#define vec3_right(vec) \
	vec.x =  1.0f;      \
	vec.y =  0.0f;      \
	vec.z =  0.0f;

#define vec3_depth(vec) \
	vec.x =  0.0f;      \
	vec.y =  0.0f;      \
	vec.z = -1.0f;

#define vec3_front(vec) \
	vec.x =  0.0f;      \
	vec.y =  0.0f;      \
	vec.z =  1.0f;

#define vec3_up_left(vec) \
	vec.x = -1.0f; \
	vec.y = 1.0f;  \
	vec.z = 0.0f;

#define vec3_right_down(vec) \
	vec.x = 1.0f;  \
	vec.y = -1.0f; \
	vec.z = 0.0f;

#define vec3_up_right(vec) \
	vec.x = 0.707f;  \
	vec.y = 0.707f; \
	vec.z = 0.0f;

#define vec3_down_left(vec) \
	vec.x = -0.707f;  \
	vec.y = -0.707f; \
	vec.z = 0.0f;

#define vec3_up_front(vec) \
	vec.x = 0.0f;   \
	vec.y = 0.707f; \
	vec.z = 0.707f;

#define vec3_down_depth(vec) \
	vec.x = 0.0f;    \
	vec.y = -0.707f; \
	vec.z = -0.707f;

#define vec3_up_depth(vec) \
	vec.x = 0.0f; \
	vec.y = 1.0f; \
	vec.z = -1.0f;

#define vec3_down_front(vec) \
	vec.x = 0.0f;  \
	vec.y = -1.0f; \
	vec.z = 1.0f;

#define vec3_left_depth(vec) \
	vec.x = -0.707f;  \
	vec.y = 0.0f; \
	vec.z = -0.707f;

#define vec3_right_front(vec) \
	vec.x = 0.707f;  \
	vec.y = 0.0f; \
	vec.z = 0.707f;

#define vec3_right_depth(vec) \
	vec.x = 1.0f;  \
	vec.y = 0.0f;  \
	vec.z = -1.0f;

#define vec3_left_front(vec) \
	vec.x = -1.0f;  \
	vec.y = 0.0f;  \
	vec.z = 1.0f;


inline void vec3_abs(const Vec3 vec, Vec3* result)
{
	result->x = fabs(vec.x);
	result->y = fabs(vec.y);
	result->z = fabs(vec.z);
}

#define vec3_dot_vec3(vector1, vector2) \
	(vector1.x * vector2.x + vector1.y * vector2.y + vector1.z * vector2.z)


#define squared_distance(vector1, vector2) \
	(vector2.x - vector1.x) * (vector2.x - vector1.x) +   \
	(vector2.y - vector1.y) * (vector2.y - vector1.y) +   \
	(vector2.z - vector1.z) * (vector2.z - vector1.z)

	
#define distance(vector1, vector2) \
	sqrt( squared_distance(vector1, vector2) )

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

#define vec3_minus_vec3(vec1, vec2, result) \
	result.x = vec1.x - vec2.x; \
	result.y = vec1.y - vec2.y; \
	result.z = vec1.z - vec2.z

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