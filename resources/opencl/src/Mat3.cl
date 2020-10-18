#include "OpenCLBase.cl"

#ifndef MAT3_HEADER
#define MAT3_HEADER

typedef struct Mat3
{
	sp_float
		m00, m01, m02,
		m10, m11, m12,
		m20, m21, m22;
} Mat3;

#endif // MAT3_HEADER