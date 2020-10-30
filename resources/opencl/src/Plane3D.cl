#ifndef PLANE3D_HEADER
#define PLANE3D_HEADER

#include "OpenCLBase.cl"
#include "Vec3.cl"

typedef struct Plane3D
{
	Vec3 point;
	Vec3 normalVector;
	sp_float distanceFromOrigin;
} Plane3D;

#define plane3D_distanceFromOrigin(plane) \
	vec3_dot_vec3(plane.point, plane.normalVector)

#define plane3D_distance(plane, target) \
	(vec3_dot_vec3(plane.normalVector, target) - plane.distanceFromOrigin) / vec3_dot_vec3(plane.normalVector, plane.normalVector);

#endif // PLANE3D_HEADER