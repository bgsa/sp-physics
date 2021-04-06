#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "Mat4.h"
#include "Plane.h"
#include "DetailedCollisionStatus.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
#define SPHERE_STRIDE (4)

	class Sphere
	{
	public:
		Vec3 center;
		sp_float ray;

		/// <summary>
		/// Default construct - unit sphere with the center in origin
		/// </summary>
		API_INTERFACE Sphere();

		/// <summary>
		/// Construct with the center point and a ray
		/// </summary>
		API_INTERFACE Sphere(const Vec3 &center, sp_float ray);

		/// <summary>
		/// Build the sphere from 1 point (center) and ray = 1
		/// </summary>
		API_INTERFACE Sphere(const Vec3 &point1);

		/// <summary>
		/// Build the sphere from 2 (support) points
		/// </summary>
		API_INTERFACE Sphere(const Vec3 &point1, const Vec3 &point2);

		/// <summary>
		/// Build the sphere from 3 (support) points
		/// </summary>
		API_INTERFACE Sphere(const Vec3 &point1, const Vec3 &point2, const Vec3 &point3);

		/// <summary>
		/// Build the sphere from 4 points
		/// </summary>
		API_INTERFACE Sphere(const Vec3 &point1, const Vec3 &point2, const Vec3 &point3, const Vec3 &point4);

		/// <summary>
		/// Translate the bounding volume
		/// </summary>
		API_INTERFACE void translate(const Vec3& translation);

		/// <summary>
		/// Scale the bounding volume (only in X, Y and Z)
		/// </summary>
		API_INTERFACE void scale(const Vec3& factor);

		/// <summary>
		/// Check the status of collision against the point
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Vec3 &point) const;

		/// <summary>
		/// Check the status of collision against the plane
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Plane& plane) const;

		/// <summary>
		/// Check the status of collision against the plane
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Sphere& sphere) const;

		/// <summary>
		/// Build a enclosing sphere from an AABB
		/// </summary>
		API_INTERFACE static Sphere buildFrom(const AABB &aabb);

		/// <summary>
		/// Build a enclosing sphere from an AABB
		/// </summary>
		API_INTERFACE static Sphere buildFrom(const Vec3List& pointList);

		/// <summary>
		/// Enclose/add the sphere in another one
		/// </summary>
		API_INTERFACE Sphere enclose(const Sphere& sphere);

		/// <summary>
		/// Enclose/add the sphere in AABB
		/// </summary>
		API_INTERFACE Sphere enclose(const AABB& aabb);

		/// <summary>
		/// Returns the type og bounding volume
		/// </summary>
		API_INTERFACE BoundingVolumeType type() const;

	};

}

#endif // SPHERE_HEADER