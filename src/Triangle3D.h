#ifndef TRIANGLE3D_HEADER
#define TRIANGLE3D_HEADER

#include "SpectrumPhysics.h"
#include "Line3D.h"

namespace NAMESPACE_PHYSICS
{

	class Triangle3D
	{
	public:
		Vec3 point1;
		Vec3 point2;
		Vec3 point3;

		API_INTERFACE Triangle3D();
		API_INTERFACE Triangle3D(const Vec3& point1, const Vec3& point2, const Vec3& point3);
		API_INTERFACE Triangle3D(float* point1, float* point2, float* point3);

		API_INTERFACE Line3D* getLines() const;

		API_INTERFACE Vec3 barycentric(const Vec3& point) const;

	};

}

#endif // TRIANGLE3D_HEADER