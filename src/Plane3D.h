#ifndef PLANE3D_HEADER
#define PLANE3D_HEADER

#include "SpectrumPhysics.h"
#include "Line3D.h"
#include "Orientation.h"

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
		/// Build a plane from 3 points, making a face
		/// </summary>
		API_INTERFACE Plane3D(const Vec3& point1, const Vec3& point2, const Vec3& point3);

		/// <summary>
		/// Build a plane from equation
		/// </summary>
		API_INTERFACE Plane3D(sp_float a, sp_float b, sp_float c, sp_float d);

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
		/// Indicate whether the point is on the left, right fo the plane OR the point lies on the plane
		/// </summary>
		API_INTERFACE Orientation orientation(const Vec3& point) const;

		/// <summary>
		/// Check if the planes are parallel each other
		/// </summary>
		API_INTERFACE inline sp_bool isParallel(const Plane3D& plane) const
		{
			return normalVector.cross(plane.normalVector) == ZERO_FLOAT;
		}

		/// <summary>
		/// Check if the planes are perpendicular each other
		/// </summary>
		API_INTERFACE inline sp_bool isPerpendicular(const Plane3D& plane) const
		{
			return normalVector.dot(plane.normalVector) == ZERO_FLOAT;
		}

		/// <summary>
		/// Given an arbitrary point, find the closest point on the plane
		/// </summary>
		API_INTERFACE Vec3 closestPointOnThePlane(const Vec3 &target) const;

	};

}

#endif // PLANE3D_HEADER