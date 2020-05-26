#ifndef AABB_HEADER
#define AABB_HEADER

#include "SpectrumPhysics.h"
#include "DetailedCollisionStatus.h"
#include <algorithm>
#include "BoundingVolume.h"
#include "Plane3D.h"
#include "Sphere.h"

namespace NAMESPACE_PHYSICS
{
#define AABB_STRIDER     (8)
#define AABB_OFFSET      (2)
#define AABB_ORIENTATION (3)

	class AABB
		: public BoundingVolume
	{
	private:

		/// <summary>
		/// This method is not applied to k-DOPs
		/// </summary>
		API_INTERFACE void rotate(const Vec3& angles) override;

	public:
		Vec3 minPoint;
		Vec3 maxPoint;

		///<summary>
		///Default constructur - build a unit AABB with the center in the origin
		///</summary>
		API_INTERFACE AABB();

		///<summary>
		///Constructor using min and max points
		///</summary>
		API_INTERFACE AABB(Vec3 minPoint, Vec3 maxPoint);

		///<summary>
		///Constructur using min points and distances from this point in the axis
		///</summary>
		API_INTERFACE AABB(Vec3 minPoint, sp_float width, sp_float height, sp_float depth);

		///<summary>
		///Get the center of AABB
		///</summary>
		API_INTERFACE Vec3 center() const;

		///<summary>
		///Get the center of bounding volumne (AABB)
		///</summary>
		API_INTERFACE Vec3 centerOfBoundingVolume() const override;

		///<summary>
		///Get the SQUARED distance from a point and AABB
		///</summary>
		API_INTERFACE sp_float squaredDistance(const Vec3& target);

		///<summary>
		///Get the distance from a point and AABB
		///</summary>
		API_INTERFACE sp_float distance(const Vec3& target);

		/// <summary>
		/// Translate the bounding volume
		/// </summary>
		API_INTERFACE void translate(const Vec3& translation) override;

		/// <summary>
		/// Scale the bounding volume (only in X, Y and Z)
		/// </summary>
		API_INTERFACE void scale(const Vec3& factor) override;

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