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
		: public BoundingVolumeSphere
	{
	private:
		void initParticleSystem();

	public:
		Vec3f center;
		float ray;

		/// <summary>
		/// Default construct - unit sphere with the center in origin
		/// </summary>
		API_INTERFACE Sphere();

		/// <summary>
		/// Construct with the center point and a ray
		/// </summary>
		API_INTERFACE Sphere(const Vec3f &center, float ray);

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
		/// Translate the sphere
		/// </summary>
		API_INTERFACE Sphere* translate(float xAxis, float yAxis, float zAxis) override;

		/// <summary>
		/// Bounding volume of Sphere do not rotate.
		/// </summary>
		API_INTERFACE Sphere* rotate(float angleInRadians, float xAxis, float yAxis, float zAxis) override;

		/// <summary>
		/// Get model view of Sphere
		/// </summary>
		API_INTERFACE Mat3f modelView() override;

		/// <summary>
		/// Scale the sphere
		/// </summary>
		API_INTERFACE Sphere* scale(float xAxis, float yAxis, float zAxis) override;

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

	};

}

#endif // SPHERE_HEADER