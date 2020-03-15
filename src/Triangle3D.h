#ifndef TRIANGLE3D_HEADER
#define TRIANGLE3D_HEADER

#include "OpenML.h"
#include "Line3D.h"

namespace NAMESPACE_PHYSICS
{

	class Triangle3D
	{
	public:
		Vec3f point1;
		Vec3f point2;
		Vec3f point3;

		API_INTERFACE Triangle3D();
		API_INTERFACE Triangle3D(const Vec3f& point1, const Vec3f& point2, const Vec3f& point3);
		API_INTERFACE Triangle3D(float* point1, float* point2, float* point3);

		API_INTERFACE Line3D* getLines() const;

		API_INTERFACE Vec3f barycentric(const Vec3f& point) const;

	};

}

#endif // TRIANGLE3D_HEADER