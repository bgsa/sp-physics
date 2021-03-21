#include "OpenCLBase.cl"
#include "Vec3.cl"

#ifndef SPHERE_OPENCL_HEADER
#define SPHERE_OPENCL_HEADER

#define SPHERE_STRIDE (4)

typedef struct Sphere
{
	Vec3 center;
	sp_float ray;
} Sphere;


#endif // SPHERE_OPENCL_HEADER