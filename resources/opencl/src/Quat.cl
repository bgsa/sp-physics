#include "OpenCLBase.cl"

#ifndef QUAT_HEADER
#define QUAT_HEADER

typedef struct Quat
{
	sp_float w, x, y, z;
} Quat;

inline void quat_conjugate(Quat input, Quat* output)
{
	output->w =  input.w;
	output->x = -input.x;
	output->y = -input.y;
	output->z = -input.z;
}

inline void quat_multiply_vec3(Vec3 vertex, Quat input, Quat* output)
{
	output->w = input.w - (vertex.x * input.x) - (vertex.y * input.y) - (vertex.z * input.z);
	output->x = input.x + (vertex.x * input.w) - (vertex.y * input.z) + (vertex.z * input.y);
	output->y = input.y + (vertex.x * input.z) + (vertex.y * input.w) - (vertex.z * input.x);
	output->z = input.z - (vertex.x * input.y) + (vertex.y * input.x) + (vertex.z * input.w);
}

inline void quat_multiply_sum_quat(Quat quat1, Quat quat2, Vec3 translation, Vec3* output)
{
	output->x = (quat1.w * quat2.x) + (quat1.x * quat2.w) - (quat1.y * quat2.z) + (quat1.z * quat2.y) + translation.x;
	output->y = (quat1.w * quat2.y) + (quat1.x * quat2.z) + (quat1.y * quat2.w) - (quat1.z * quat2.x) + translation.y;
	output->z = (quat1.w * quat2.z) - (quat1.x * quat2.y) + (quat1.y * quat2.x) + (quat1.z * quat2.w) + translation.z;
}

inline sp_bool quat_isCloseEnough_quat(const Quat quat1, const Quat quat2, const sp_float epsilon)
{
	return 
		isCloseEnough(quat1.w, quat2.w, epsilon) &&
		isCloseEnough(quat1.x, quat2.x, epsilon) &&
		isCloseEnough(quat1.y, quat2.y, epsilon) &&
		isCloseEnough(quat1.z, quat2.z, epsilon);
}

#endif // QUAT_HEADER