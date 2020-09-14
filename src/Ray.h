#ifndef RAY_HEADER
#define RAY_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class Ray
	{
	public:
		Vec3 point;
		Vec3 direction;

		API_INTERFACE Ray() { }

		API_INTERFACE Ray(const Vec3& point, const Vec3& direction)
		{
			this->point = point;
			this->direction = direction;
		}

		API_INTERFACE void closestPoint(const Ray& ray, Vec3* closestPointRay1, Vec3* closestPointRay2, sp_float* sqDistance, const sp_float _epsilon = DefaultErrorMargin) const;

		API_INTERFACE sp_bool intersection(const Ray& ray, Vec3* contact, const sp_float _epsilon = DefaultErrorMargin) const;

	};

}

#endif // RAY_HEADER
