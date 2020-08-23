#ifndef PLANE3D_HEADER
#define PLANE3D_HEADER

#include "SpectrumPhysics.h"
#include "Ray.h"
#include "Line3D.h"
#include "Orientation.h"
#include "Triangle3D.h"

namespace NAMESPACE_PHYSICS
{
	class Plane3D
	{

	public:
		Vec3 point;
		Vec3 normalVector;

		API_INTERFACE Plane3D()
		{
			point = Vec3(0.0f);
			normalVector = Vec3(0.0f, 1.0f, 0.0f);
		}

		/// <summary>
		/// Build a plane from a point and normal vector (NORMALIZED!)
		/// </summary>
		API_INTERFACE Plane3D(const Vec3& point, const Vec3& normal)
		{
			this->point = point;
			this->normalVector = normal;
		}

		/// <summary>
		/// Build a plane from 3 points (right-hand)
		/// </summary>
		API_INTERFACE Plane3D(const Vec3& point1, const Vec3& point2, const Vec3& point3);

		/// <summary>
		/// Build a plane from equation
		/// </summary>
		API_INTERFACE Plane3D(sp_float a, sp_float b, sp_float c, sp_float d);

		/// <summary>
		/// Build a plane from triangle
		/// </summary>
		API_INTERFACE Plane3D(const Triangle3D& triangle);

		/// <summary>
		/// Get "D" components from plane equation: ax + by + cz + D = 0
		/// </summary>
		API_INTERFACE inline sp_float getDcomponent() const
		{
			return -normalVector.dot(point);
		}

		/// <summary>
		/// Get the equation of the plane
		/// </summary>
		API_INTERFACE inline Vec4 equation() const
		{
			return Vec4(
				normalVector[0],
				normalVector[1],
				normalVector[2],
				getDcomponent()
			);
		}

		/// <summary>
		/// Test if the ray cross the plane
		/// </summary>
		API_INTERFACE inline void intersection(const Ray& ray, Vec3* contactPoint) const
		{
			sp_float angle = normalVector.dot(ray.direction);

			if (isCloseEnough(angle, 0.0f))
				return;

			Vec4 planeEquation = equation();

			sp_float numerator = -(planeEquation[0] * ray.point[0] + planeEquation[1] * ray.point[1] + planeEquation[2] * ray.point[2] + planeEquation[3]);
			sp_float denominator = planeEquation[0] * ray.direction[0] + planeEquation[1] * ray.direction[1] + planeEquation[2] * ray.direction[2];

			sp_float t = numerator / denominator;

			contactPoint->x = ray.point[0] + ray.direction[0] * t;
			contactPoint->y = ray.point[1] + ray.direction[1] * t;
			contactPoint->z = ray.point[2] + ray.direction[2] * t;
		}

		/// <summary>
		/// Test if the line cross the plane
		/// </summary>
		API_INTERFACE void intersection(const Line3D& line, Vec3* contactPoint) const;

		/// <summary>
		/// Get the line intersection between the planes
		/// </summary>
		API_INTERFACE void intersection(const Plane3D& plane, Line3D* line) const;
		
		/// <summary>
		/// Get the angle of two planes
		/// </summary>
		API_INTERFACE sp_float angle(const Plane3D& plane) const;

		/// <summary>
		/// Get the distance from the plane to the point
		/// </summary>
		API_INTERFACE sp_float distance(const Vec3& point) const;

		/// <summary>
		/// Get the distance from the plane to the other one
		/// </summary>
		API_INTERFACE sp_float distance(const Plane3D& plane) const;

		/// <summary>
		/// Indicate whether the point is on the left, right fo the plane OR the point relies on the plane
		/// </summary>
		/// <param name="point">Arbitrary point</param>
		/// <returns>0 if the point relies on the plane; 
		/// Greater than zero if the point is left of the plane; 
		/// Lesser than zero if the point is right of the plane 
		/// </returns>
		API_INTERFACE sp_float orientation(const Vec3& point) const
		{
			return distance(point);
		}

		/// <summary>
		/// Check if the planes are parallel each other
		/// </summary>
		API_INTERFACE inline sp_bool isParallel(const Plane3D& plane, const sp_float _epsilon = DefaultErrorMargin) const
		{
			return normalVector.cross(plane.normalVector).isCloseEnough(Vec3(), _epsilon);
		}

		/// <summary>
		/// Check if the planes are perpendicular each other
		/// </summary>
		API_INTERFACE inline sp_bool isPerpendicular(const Plane3D& plane, const sp_float _epsilon = DefaultErrorMargin) const
		{
			return isCloseEnough(normalVector.dot(plane.normalVector), ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Given an arbitrary point, find the closest point on the plane
		/// </summary>
		API_INTERFACE Vec3 closestPointOnThePlane(const Vec3 &target) const;

	};

}

#endif // PLANE3D_HEADER