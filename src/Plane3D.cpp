#include "Plane3D.h"

namespace NAMESPACE_PHYSICS
{
	Plane3D::Plane3D() {
		this->point = Vec3(0.0f);
	};

	Plane3D::Plane3D(const Vec3& point, const Vec3& normal)
	{
		this->point = point;
		this->normalVector = normal;
	}

	Plane3D::Plane3D(const Vec3& point1, const Vec3& point2, const Vec3& point3)
	{
		Vec3 ab = point1 - point2;
		Vec3 ac = point1 - point3;

		point = point2;

		normalVector = Vec3{
			ab[1] * ac[2] - (ab[2] * ac[1]),
			ab[2] * ac[0] - (ab[0] * ac[2]),
			ab[0] * ac[1] - (ab[1] * ac[0])
		}.normalize();
	}

	Plane3D::Plane3D(sp_float a, sp_float b, sp_float c, sp_float d)
	{
		point = Vec3(
			0.0f,
			0.0f,
			-d / c
			);

		normalVector = Vec3(a, b, c).normalize();
	}

	sp_float Plane3D::getDcomponent() const
	{
		return -normalVector.dot(point);
		//return normalVector.dot(point);
	}

	Vec4 Plane3D::getEquation() const
	{
		return Vec4(
			normalVector[0],
			normalVector[1],
			normalVector[2],
			getDcomponent()
			);
	}

	Vec3* Plane3D::findIntersection(const Line3D& line) const
	{
		Vec3 lineAsVector = line.point2 - line.point1;

		sp_float angle = normalVector.dot(lineAsVector);

		if (angle == 0.0f)
			return nullptr;

		Vec4 planeEquation = getEquation();

		sp_float numerator = -(planeEquation[0] * line.point1[0] + planeEquation[1] * line.point1[1] + planeEquation[2] * line.point1[2] + planeEquation[3]);
		sp_float denominator = planeEquation[0] * lineAsVector[0] + planeEquation[1] * lineAsVector[1] + planeEquation[2] * lineAsVector[2];

		sp_float t = numerator / denominator;

		Vec3* intersection = ALLOC_NEW(Vec3)(
			line.point1[0] + lineAsVector[0] * t,
			line.point1[1] + lineAsVector[1] * t,
			line.point1[2] + lineAsVector[2] * t
			);

		return intersection;
	}

	Line3D* Plane3D::findIntersection(const Plane3D& plane) const
	{
		if (isParallel(plane))
			return nullptr;

		Vec3 lineDirection = normalVector.cross(plane.normalVector);
		
		sp_float d1 = getDcomponent();
		sp_float d2 = plane.getDcomponent();

		// find a point on the line, which is also on both planes
		sp_float dot = lineDirection.dot(lineDirection);					// V dot V
		Vec3 u1 = normalVector * d2;								// d2 * normalVector
		Vec3 u2 = plane.normalVector * -d1;					    //-d1 * plane.normalVector
		Vec3 point1 = (u1 + u2).cross(lineDirection) / dot;      // (d2*N1-d1*N2) X V / V dot V

		// find another point on the line
		Vec3 point2 = point1 + lineDirection;

		return ALLOC_NEW(Line3D)(point1, point2);
	}

	sp_float Plane3D::distance(const Vec3& target) const
	{
		Vec3 rayToTarget = target - point;

		sp_float numerator = normalVector.dot(rayToTarget);
		sp_float length = normalVector.length();

		return numerator / length;
	}

	Vec3 Plane3D::closestPointOnThePlane(const Vec3 &target) const
	{
		sp_float d = getDcomponent();

		sp_float t = (normalVector.dot(target) - d) / normalVector.dot(normalVector); //t = ((n . p) - d) / (n.n)

		return target - (normalVector * t); //result = point - tn
	}

	sp_float Plane3D::angle(const Plane3D& plane) const
	{
		sp_float angle = normalVector.dot(plane.normalVector);
		sp_float length = normalVector.length() * plane.normalVector.length();

		return angle / length;
	}

	Orientation Plane3D::orientation(const Vec3& point) const
	{
		sp_float distanceToPoint =  distance(point);

		if (distanceToPoint == 0.0f)
			return Orientation::NONE;
		else if (distanceToPoint > 0.0f)
			return Orientation::LEFT;
		
		return Orientation::RIGHT;
	}

	sp_bool Plane3D::isParallel(const Plane3D& plane) const
	{
		return normalVector.cross(plane.normalVector) == 0.0f;
	}

	sp_bool Plane3D::isPerpendicular(const Plane3D& plane) const
	{
		return normalVector.dot(plane.normalVector) == 0.0f;
	}
}