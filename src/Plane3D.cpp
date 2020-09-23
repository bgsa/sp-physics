#include "Plane3D.h"

namespace NAMESPACE_PHYSICS
{

	Plane3D::Plane3D(const Vec3& point1, const Vec3& point2, const Vec3& point3)
	{
		point = point1;
		normal(point1, point2, point3, &normalVector);
		distanceFromOrigin = normalVector.dot(point);
	}

	Plane3D::Plane3D(const Triangle3D& triangle)
	{
		point = triangle.point1;
		triangle.normalFace(&normalVector);
		distanceFromOrigin = normalVector.dot(point);
	}

	Plane3D::Plane3D(sp_float a, sp_float b, sp_float c, sp_float d)
	{
		point = Vec3(ZERO_FLOAT, ZERO_FLOAT, -d / c);

		normalize(Vec3(a, b, c), &normalVector);
		distanceFromOrigin = normalVector.dot(point);
	}

	sp_bool Plane3D::intersection(const Line3D& line, Vec3* contactPoint, const sp_float _epsilon) const
	{
		const Vec3 lineAsVector = line.point2 - line.point1;
		const sp_float angle = normalVector.dot(lineAsVector);

		if (isCloseEnough(angle, ZERO_FLOAT))
			return false;

		const sp_float t = (distanceFromOrigin - normalVector.dot(line.point1)) / angle;

		if (t >= -_epsilon && t <= ONE_FLOAT + _epsilon)
		{
			contactPoint[0] = line.point1 + lineAsVector * t;
			return true;
		}

		return false;
	}

	void Plane3D::closestPoint(const Line3D& line, Vec3* closest) const
	{
		const Vec3 lineAsVector = line.point2 - line.point1;

		const sp_float t
			= -(normalVector[0] * line.point1[0] + normalVector[1] * line.point1[1] + normalVector[2] * line.point1[2] + distanceFromOrigin)
			/ normalVector[0] * lineAsVector[0] + normalVector[1] * lineAsVector[1] + normalVector[2] * lineAsVector[2];

		closest->x = line.point1[0] + lineAsVector[0] * t;
		closest->y = line.point1[1] + lineAsVector[1] * t;
		closest->z = line.point1[2] + lineAsVector[2] * t;
	}

	sp_bool Plane3D::intersection(const Plane3D& plane, Line3D* line) const
	{
		// Compute direction of intersection line  
		Vec3 lineDirection;
		cross(plane.normalVector, normalVector, &lineDirection);

		// If d is (near) zero, the planes are parallel (and separated)  
		// or coincident, so they’re not considered intersecting  
		const sp_float denom = lineDirection.dot(lineDirection);

		if (denom < DefaultErrorMargin)
			return false;

		// Compute point on intersection line  
		cross(plane.normalVector * distanceFromOrigin - distanceFromOrigin * plane.distanceFromOrigin, lineDirection, &line->point1);
		line->point1 /= denom;
		line->point2 = line->point1 + lineDirection;

		return true;
	}

	sp_bool Plane3D::intersection(const Plane3D& plane, Ray* ray) const
	{
		// Compute direction of intersection line  
		cross(normalVector, plane.normalVector, &ray->direction);

		// If d is (near) zero, the planes are parallel (and separated)  
		// or coincident, so they’re not considered intersecting  
		const sp_float denom = ray->direction.dot(ray->direction);

		if (denom < DefaultErrorMargin)
			return false;

		// Compute point on intersection line  
		cross(plane.normalVector * distanceFromOrigin - distanceFromOrigin * plane.distanceFromOrigin, ray->direction, &ray->point);
		ray->point /= denom;
		
		return true;
	}

	sp_float Plane3D::distance(const Vec3& target) const
	{
		return (normalVector.dot(target) - distanceFromOrigin) / normalVector.dot(normalVector);
	}

	sp_float Plane3D::distance(const Plane3D& plane) const
	{
		Vec3 projectedPoint;
		project(plane.point, &projectedPoint);

		return NAMESPACE_PHYSICS::distance(plane.point, projectedPoint);
		
		/* It does not work when (distanceFromOrigin - plane.distanceFromOrigin) = 0
		return std::fabsf(distanceFromOrigin - plane.distanceFromOrigin)
				/ std::sqrtf(point.x * point.x + point.y * point.y + point.z * point.z);
		*/
	}

	Vec3 Plane3D::closestPointOnThePlane(const Vec3 &target) const
	{
		// t = ((n . p) - d) / (n.n)
		sp_float t = (normalVector.dot(target) - getDcomponent()) / normalVector.dot(normalVector); 

		return target - (normalVector * t); //result = point - tn
	}

	sp_float Plane3D::angle(const Plane3D& plane) const
	{
		sp_float angle = normalVector.dot(plane.normalVector);
		sp_float length = NAMESPACE_PHYSICS::length(normalVector) * NAMESPACE_PHYSICS::length(plane.normalVector);

		return angle / length;
	}

	void Plane3D::project(const Vec3& target, Vec3* output) const
	{
		Ray ray(target, -normalVector);
		intersection(ray, output);
	}

}