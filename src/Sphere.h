#ifndef SPHERE_HEADER
#define SPHERE_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"
#include "Mat4.h"
#include "Plane3D.h"
#include "CollisionStatus.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{

	class Sphere
		: public BoundingVolume
	{
	public:
		Vec3f center;
		sp_float ray;

		/// <summary>
		/// Default construct - unit sphere with the center in origin
		/// </summary>
		API_INTERFACE Sphere();

		/// <summary>
		/// Construct with the center point and a ray
		/// </summary>
		API_INTERFACE Sphere(const Vec3f &center, sp_float ray);

		/// <summary>
		/// Build the sphere from 1 point (center) and ray = 1
		/// </summary>
		API_INTERFACE Sphere(const Vec3f &point1);

		/// <summary>
		/// Build the sphere from 2 (support) points
		/// </summary>
		API_INTERFACE Sphere(const Vec3f &point1, const Vec3f &point2);

		/// <summary>
		/// Build the sphere from 3 (support) points
		/// </summary>
		API_INTERFACE Sphere(const Vec3f &point1, const Vec3f &point2, const Vec3f &point3);

		/// <summary>
		/// Build the sphere from 4 points
		/// </summary>
		API_INTERFACE Sphere(const Vec3f &point1, const Vec3f &point2, const Vec3f &point3, const Vec3f &point4);

		/// <summary>
		/// Get the center of sphere
		/// </summary>
		API_INTERFACE inline Vec3f centerOfBoundingVolume() const override {
			return center;
		}

		/// <summary>
		/// Check the status of collision against the point
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Vec3f &point) const;

		/// <summary>
		/// Check the status of collision against the plane
		/// </summary>
		API_INTERFACE CollisionStatus collisionStatus(const Plane3D &plane) const;

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
		API_INTERFACE static Sphere buildFrom(const Vec3List<float>& pointList);

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

		/// <summary>
		/// Releases all allocated resouces
		/// </summary>
		API_INTERFACE virtual void dispose() override
		{
		}

		API_INTERFACE virtual const sp_char* toString() override
		{
			return "Sphere Bounding Volume";
		}

	};

}

#endif // SPHERE_HEADER