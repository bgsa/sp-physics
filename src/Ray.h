#ifndef RAY_HEADER
#define RAY_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "Plane.h"

namespace NAMESPACE_PHYSICS
{
	class Ray
	{
	public:
		Vec3 point;
		Vec3 direction;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE inline Ray() { }

		/// <summary>
		/// Constructor with parameters
		/// </summary>
		/// <param name="point">Reference point</param>
		/// <param name="direction">Ray direction</param>
		/// <returns></returns>
		API_INTERFACE inline Ray(const Vec3& point, const Vec3& direction)
		{
			this->point = point;
			this->direction = direction;
		}

		API_INTERFACE void closestPoint(const Ray& ray, Vec3* closestPointRay1, Vec3* closestPointRay2, sp_float* sqDistance, const sp_float _epsilon = DefaultErrorMargin) const;

		API_INTERFACE sp_bool intersection(const Ray& ray, Vec3* contact, const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Check if the ray intersect the plane
		/// </summary>
		API_INTERFACE inline sp_bool intersection(const Plane& p, Vec3& contact) const;

		API_INTERFACE sp_bool intersection(const AABB& aabb, Vec3 contact[2], sp_float& distanceToFirstContact, sp_float& distanceToSecondContact, const sp_float _epsilon = DefaultErrorMargin) const;

	};

}

#endif // RAY_HEADER
