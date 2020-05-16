#ifndef AABB_HEADER
#define AABB_HEADER

#include <algorithm>
#include "Vec3.h"
#include "Vec3List.h"
#include "BoundingVolume.h"
#include "Plane3D.h"
#include "Sphere.h"

namespace NAMESPACE_PHYSICS
{
#define AABB_STRIDER 8
#define AABB_OFFSET 2

	class AABB
		: public BoundingVolume
	{
	public:
		Vec3f minPoint;
		Vec3f maxPoint;

		///<summary>
		///Default constructur - build a unit AABB with the center in the origin
		///</summary>
		API_INTERFACE AABB();

		///<summary>
		///Constructor using min and max points
		///</summary>
		API_INTERFACE AABB(Vec3f minPoint, Vec3f maxPoint);

		///<summary>
		///Constructur using min points and distances from this point in the axis
		///</summary>
		API_INTERFACE AABB(Vec3f minPoint, float width, float height, float depth);

		///<summary>
		///Get the center of AABB
		///</summary>
		API_INTERFACE Vec3f center() const;

		///<summary>
		///Get the center of bounding volumne (AABB)
		///</summary>
		API_INTERFACE Vec3f centerOfBoundingVolume() const override;

		///<summary>
		///Get the SQUARED distance from a point and AABB
		///</summary>
		API_INTERFACE float squaredDistance(const Vec3f& target);

		///<summary>
		///Get the distance from a point and AABB
		///</summary>
		API_INTERFACE float distance(const Vec3f& target);

		///<summary>
		///Check whether the AABBs are in contact each other
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const AABB& aabb);

		///<summary>
		///Check whether the AABB intersect the plane
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const Plane3D& plane);

		///<summary>
		///Check whether the AABB intersect the sphere
		///</summary>
		API_INTERFACE CollisionStatus collisionStatus(const Sphere& sphere);
		
		///<summary>
		///Given a point, find the closest point in AABB
		///</summary>
		API_INTERFACE Vec3f closestPointInAABB(const Vec3f& target);

		///<summary>
		///Given a point, find the closest point in AABB
		///</summary>
		API_INTERFACE Vec3f closestPointInAABB(const Sphere& sphgere);

		///<summary>
		///Given a list of point (mesh), build the AABB
		///</summary>
		API_INTERFACE static AABB buildFrom(const Vec3List<float>& pointList);

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
		API_INTERFACE bool operator==(const AABB& aabb) const;

		/// <summary>
		///Compare this AABB to another one. Compare each minPoint and maxPoint
		/// </summary>
		API_INTERFACE bool operator!=(const AABB& aabb) const;

		/// <summary>
		///Comparator function
		/// </summary>
		API_INTERFACE bool operator<(const AABB& aabb) const;

		/// <summary>
		///Comparator function
		/// </summary>
		API_INTERFACE bool operator>(const AABB& aabb) const;

		/// <summary>
		///Hash code function
		/// </summary>
		API_INTERFACE size_t operator()(const AABB& aabb) const;
		
		/// <summary>
		///Equals function
		/// </summary>
		API_INTERFACE bool operator()(const AABB& aabb1, const AABB& aabb2) const;

		/// <summary>
		/// Returns the type of bounding volume
		/// </summary>
		API_INTERFACE BoundingVolumeType type() const override;

		/// <summary>
		/// Releases all allocated resouces
		/// </summary>
		API_INTERFACE virtual void dispose() override
		{
		}

		API_INTERFACE virtual const sp_char* toString() override
		{
			return "AABB";
		}
	};

}

#endif // AABB_HEADER