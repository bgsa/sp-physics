#include "Triangle3D.h"

namespace NAMESPACE_PHYSICS
{
	void Triangle3D::project(const Vec3& target, Vec3* output) const
	{
		Plane3D plane(point1, point2, point3);
		Ray ray(target, -plane.normalVector);

		plane.intersection(ray, output);
	}

	void Triangle3D::edges(Line3D* lines) const
	{
		lines[0] = Line3D(point1, point2);
		lines[1] = Line3D(point2, point3);
		lines[2] = Line3D(point3, point1);
	}

}
