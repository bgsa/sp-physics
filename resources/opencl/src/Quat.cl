#include "OpenCLBase.cl"

#ifndef QUAT_HEADER
#define QUAT_HEADER

typedef struct Quat
{
	sp_float w, x, y, z;
} Quat;

inline sp_bool quat_isCloseEnough_quat(const Quat quat1, const Quat quat2, const sp_float epsilon)
{
	return 
		isCloseEnough(quat1.w, quat2.w, epsilon) &&
		isCloseEnough(quat1.x, quat2.x, epsilon) &&
		isCloseEnough(quat1.y, quat2.y, epsilon) &&
		isCloseEnough(quat1.z, quat2.z, epsilon);
}

#endif // QUAT_HEADER