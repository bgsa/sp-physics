#ifndef AABB_HEADER
#define AABB_HEADER

#include "SpectrumPhysics.h"
#include "DetailedCollisionStatus.h"
#include <algorithm>
#include "BoundingVolume.h"
#include "Plane.h"
#include "Sphere.h"

namespace NAMESPACE_PHYSICS
{
#define AABB_STRIDER     (6)
#define AABB_OFFSET      (2)
#define AABB_ORIENTATION (3)

	class AABB
	{
	public:
		Vec3 minPoint;
		Vec3 maxPoint;

		///<summary>
		///Default constructur - build a unit AABB with the center in the origin
		///</summary>
		API_INTERFACE inline AABB()
		{
			minPoint = Vec3(-HALF_FLOAT, -HALF_FLOAT, -HALF_FLOAT);
			maxPoint = -minPoint;
		}

		///<summary>
		///Constructor using min and max points
		///</summary>
		API_INTERFACE inline AABB(Vec3 minPoint, Vec3 maxPoint)
		{
			this->minPoint = minPoint;
			this->maxPoint = maxPoint;
		}

		///<summary>
		///Constructur using min points and distances from this point in the axis
		///</summary>
		API_INTERFACE inline AABB(Vec3 minPoint, sp_float width, sp_float height, sp_float depth)
		{
			this->minPoint = minPoint;

			maxPoint = Vec3(
				minPoint.x + width,
				minPoint.y + height,
				minPoint.z + depth
			);
		}

		///<summary>
		///Get the center of AABB
		///</summary>
		API_INTERFACE Vec3 center() const
		{
			return (maxPoint + minPoint) * 0.5f;
		}

		///<summary>
		///Get the SQUARED distance from a point and AABB
		///</summary>
		API_INTERFACE inline sp_float squaredDistance(const Vec3& target) const
		{
			sp_float result = ZERO_FLOAT;

			// For each axis count any excess distance outside box extents 
			for (sp_int axis = 0; axis < 3; axis++)
			{
				sp_float v = target[axis];

				if (v < minPoint[axis])
					result += (minPoint[axis] - v) * (minPoint[axis] - v);

				if (v > maxPoint[axis])
					result += (v - maxPoint[axis]) * (v - maxPoint[axis]);
			}

			return result;
		}

		///<summary>
		///Get the distance from a point and AABB
		///</summary>
		API_INTERFACE inline sp_float distance(const Vec3& target) const
		{
			return (sp_float)sqrtf(squaredDistance(target));
		}

		/// <summary>
		/// Translate the bounding volume
		/// </summary>
		API_INTERFACE void translate(const Vec3& translation)
		{
			minPoint += translation;
			maxPoint += translation;
		}

		/// <summary>
		/// Scale the bounding volume (only in X, Y and Z)
		/// </summary>
		API_INTERFACE inline void scale(const Vec3& factor)
		{
			minPoint.x *= factor.x;
			minPoint.y *= factor.y;
			minPoint.z *= factor.z;

			maxPoint.x *= factor.x;
			maxPoint.y *= factor.y;
			maxPoint.z *= factor.z;
		}

		///<summary>
		///Check whether the AABBs are in contact each other
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const AABB& aabb) const
		{
			if (maxPoint.x < aabb.minPoint.x || minPoint.x > aabb.maxPoint.x)
				return CollisionStatus::OUTSIDE;

			if (maxPoint.y < aabb.minPoint.y || minPoint.y > aabb.maxPoint.y)
				return CollisionStatus::OUTSIDE;

			if (maxPoint.z < aabb.minPoint.z || minPoint.z > aabb.maxPoint.z)
				return CollisionStatus::OUTSIDE;

			return CollisionStatus::INSIDE;
		}

		///<summary>
		///Check whether the AABB intersect the plane
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const Plane& plane);

		///<summary>
		///Check whether the AABB intersect the sphere
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const Sphere& sphere);
		
		///<summary>
		///Given a point, find the closest point in AABB
		///</summary>
		API_INTERFACE Vec3 closestPointInAABB(const Vec3& target);

		///<summary>
		///Given a point, find the closest point in AABB
		///</summary>
		API_INTERFACE Vec3 closestPointInAABB(const Sphere& sphgere);

		///<summary>
		///Given a list of point (mesh), build the AABB
		///</summary>
		API_INTERFACE static AABB buildFrom(const Vec3List& pointList);

		///<summary>
		///Given a sphere, build the AABB to enclose the sphere
		///</summary>
		API_INTERFACE static AABB buildFrom(const Sphere& sphere);

		///<summary>
		///Ecnlose/add a new bounding volume with AABB in parameter
		///</summary>
		API_INTERFACE AABB enclose(const AABB& aabb);

		///<summary>
		///Ecnlose/add a new bounding volume with Sphere in parameter
		///</summary>
		API_INTERFACE AABB enclose(const Sphere& sphere);

		/// <summary>
		///Compare this AABB to another one. Compare each minPoint and maxPoint
		/// </summary>
		API_INTERFACE sp_bool operator==(const AABB& aabb) const;

		/// <summary>
		///Compare this AABB to another one. Compare each minPoint and maxPoint
		/// </summary>
		API_INTERFACE sp_bool operator!=(const AABB& aabb) const;

		/// <summary>
		///Comparator function
		/// </summary>
		API_INTERFACE sp_bool operator<(const AABB& aabb) const;

		/// <summary>
		///Comparator function
		/// </summary>
		API_INTERFACE sp_bool operator>(const AABB& aabb) const;

		/// <summary>
		/// Hash code function
		/// </summary>
		API_INTERFACE inline sp_size operator()(const AABB& aabb) const
		{
			sp_float hash = 1.0f;
			const sp_float constant = 3.0f;

			hash = constant * hash + aabb.minPoint.x;
			hash = constant * hash + aabb.minPoint.y;
			hash = constant * hash + aabb.minPoint.z;
			hash = constant * hash + aabb.maxPoint.x;
			hash = constant * hash + aabb.maxPoint.y;
			hash = constant * hash + aabb.maxPoint.z;

			return size_t(hash);
		}
		
		/// <summary>
		/// Equals function
		/// </summary>
		API_INTERFACE inline sp_bool operator()(const AABB& aabb1, const AABB& aabb2) const
		{
			return aabb1 == aabb2;
		}

	};

}

#endif // AABB_HEADER