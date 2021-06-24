#ifndef QUAT_HEADER
#define QUAT_HEADER

#include "OpenCLBase.cl"

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

inline void quat_conjugate(Quat input, Quat* output)
{
	output->w =  input.w;
	output->x = -input.x;
	output->y = -input.y;
	output->z = -input.z;
}

inline void quat_rotate_vec3(Quat rotation, Vec3 point, Vec3* output)
{
	Quat qc;
	quat_conjugate(rotation, &qc);

	Quat q1;
	q1.w = rotation.w - (point.x * rotation.x) - (point.y * rotation.y) - (point.z * rotation.z);
	q1.x = rotation.x + (point.x * rotation.w) - (point.y * rotation.z) + (point.z * rotation.y);
	q1.y = rotation.y + (point.x * rotation.z) + (point.y * rotation.w) - (point.z * rotation.x);
	q1.z = rotation.z - (point.x * rotation.y) + (point.y * rotation.x) + (point.z * rotation.w);

	output->x = (qc.w * q1.x) + (qc.x * q1.w) - (qc.y * q1.z) + (qc.z * q1.y);
	output->y = (qc.w * q1.y) + (qc.x * q1.z) + (qc.y * q1.w) - (qc.z * q1.x);
	output->z = (qc.w * q1.z) - (qc.x * q1.y) + (qc.y * q1.x) + (qc.z * q1.w);
}


#endif // QUAT_HEADER