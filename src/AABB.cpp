#include "AABB.h"

namespace NAMESPACE_PHYSICS
{
	AABB::AABB()
	{
		this->minPoint = Vec3(-0.5f, -0.5f, -0.5f);
		this->maxPoint = -this->minPoint;
	}

	AABB::AABB(Vec3 minPoint, Vec3 maxPoint)
	{
		this->minPoint = minPoint;
		this->maxPoint = maxPoint;
	}

	AABB::AABB(Vec3 minPoint, sp_float width, sp_float height, sp_float depth)
	{
		this->minPoint = minPoint;

		maxPoint = Vec3(
			minPoint.x + width,
			minPoint.y + height,
			minPoint.z + depth
			);
	}

	Vec3 AABB::center() const
	{
		return (maxPoint + minPoint) * 0.5f;
	}

	Vec3 AABB::centerOfBoundingVolume() const
	{
		return center();
	}

	sp_float AABB::squaredDistance(const Vec3& target)
	{
		sp_float result = 0.0f;

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

	sp_float AABB::distance(const Vec3& target)
	{
		return float(sqrt(squaredDistance(target)));
	}

	void AABB::translate(const Vec3& translation)
	{
		minPoint += translation;
		maxPoint += translation;
	}

	void AABB::scale(const Vec3& factor)
	{
		minPoint.x *= factor.x;
		minPoint.y *= factor.y;
		minPoint.z *= factor.z;

		maxPoint.x *= factor.x;
		maxPoint.y *= factor.y;
		maxPoint.z *= factor.z;
	}

	void AABB::rotate(const Vec3& factor) { }

	CollisionStatus AABB::collisionStatus(const AABB& aabb) 
	{
		if (maxPoint.x < aabb.minPoint.x || minPoint.x > aabb.maxPoint.x)
			return CollisionStatus::OUTSIDE;

		if (maxPoint.y < aabb.minPoint.y || minPoint.y > aabb.maxPoint.y)
			return CollisionStatus::OUTSIDE;

		if (maxPoint.z < aabb.minPoint.z || minPoint.z > aabb.maxPoint.z)
			return CollisionStatus::OUTSIDE;

		return CollisionStatus::INSIDE;
	}

	CollisionStatus AABB::collisionStatus(const Plane3D& plane)
	{
		Vec3 centerPoint = center(); 
		sp_float d = plane.getDcomponent();
		Vec3 halfDistanceFromCenter = maxPoint - centerPoint;

		// Compute the projection interval radius of AABB onto L(t) = center + t * normalPlane
		sp_double r =
			halfDistanceFromCenter.x * std::abs(plane.normalVector.x)
			+ halfDistanceFromCenter.y * std::abs(plane.normalVector.y)
			+ halfDistanceFromCenter.z * std::abs(plane.normalVector.z);
		
		sp_double distanceFromAABB2Plane = std::abs(plane.normalVector.dot(centerPoint) + d);
		
		if (NAMESPACE_FOUNDATION::isCloseEnough(distanceFromAABB2Plane, r))
			return CollisionStatus::INLINE;

		// it has intersection when distance falls within [-r,+r] interval 	

		if (distanceFromAABB2Plane <= r)  
			return CollisionStatus::INSIDE;

		return CollisionStatus::OUTSIDE;
	}

	CollisionStatus AABB::collisionStatus(const Sphere& sphere)
	{
		sp_float distanceToSphere = squaredDistance(sphere.center);
		sp_float squaredRay = sphere.ray * sphere.ray;
		
		if (NAMESPACE_FOUNDATION::isCloseEnough(distanceToSphere, squaredRay))
			return CollisionStatus::INLINE;

		if (distanceToSphere < squaredRay)
			return CollisionStatus::INSIDE;

		return CollisionStatus::OUTSIDE;
	}

	Vec3 AABB::closestPointInAABB(const Vec3& target)
	{
		Vec3 result;

		for (sp_int axis = 0; axis < 3; axis++)
		{
			sp_float v = target[axis];
					
			v = std::max(v, minPoint[axis]);
			v = std::min(v, maxPoint[axis]);
			
			result[axis] = v;
		}

		return result;
	}

	Vec3 AABB::closestPointInAABB(const Sphere& sphere)
	{
		return closestPointInAABB(sphere.center);
	}

	AABB AABB::buildFrom(const Vec3List& pointList)
	{
		sp_int* indexes = pointList.findExtremePointsAlongAxisXYZ();

		return AABB(
			Vec3(
				pointList.points[indexes[0]].x,
				pointList.points[indexes[2]].y,
				pointList.points[indexes[4]].z
				), 
			Vec3(
				pointList.points[indexes[1]].x,
				pointList.points[indexes[3]].y,
				pointList.points[indexes[5]].z
			)
		);
	}

	AABB AABB::buildFrom(const Sphere& sphere)
	{	
		return AABB(
			Vec3(
				sphere.center.x - sphere.ray,
				sphere.center.y - sphere.ray,
				sphere.center.z - sphere.ray
				),
			Vec3(
				sphere.center.x + sphere.ray,
				sphere.center.y + sphere.ray,
				sphere.center.z + sphere.ray
				)
			);
	}

	AABB AABB::enclose(const AABB& aabb)
	{
		return AABB(
			Vec3(
				std::min(this->minPoint.x, aabb.minPoint.x),
				std::min(this->minPoint.y, aabb.minPoint.y),
				std::min(this->minPoint[2], aabb.minPoint.z)
			),
			Vec3(
				std::max(this->maxPoint.x, aabb.maxPoint.x),
				std::max(this->maxPoint.y, aabb.maxPoint.y),
				std::max(this->maxPoint.z, aabb.maxPoint.z)
			)
		);
	}

	AABB AABB::enclose(const Sphere& sphere)
	{
		return enclose(AABB::buildFrom(sphere));
	}

	sp_bool AABB::operator==(const AABB& aabb) const
	{
		return this->minPoint == aabb.minPoint 
			&& this->maxPoint == aabb.maxPoint;
	}

	sp_bool AABB::operator!=(const AABB& aabb) const
	{
		return ! (*this == aabb);
	}

	sp_bool AABB::operator<(const AABB& aabb) const
	{
		if (this->minPoint.x < aabb.minPoint.x)
			return true;

		if (this->minPoint.y < aabb.minPoint.y)
			return true;

		if (this->minPoint.z < aabb.minPoint.z)
			return true;

		return false;
	}

	sp_bool AABB::operator>(const AABB& aabb) const
	{
		if (this->maxPoint.x > aabb.maxPoint.x)
			return true;

		if (this->maxPoint.y > aabb.maxPoint.y)
			return true;

		if (this->maxPoint.z > aabb.maxPoint.z)
			return true;

		return false;
	}

	sp_size AABB::operator()(const AABB& aabb) const
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

	sp_bool AABB::operator()(const AABB& aabb1, const AABB& aabb2) const
	{
		return aabb1 == aabb2;
	}

	BoundingVolumeType AABB::type() const
	{
		return BoundingVolumeType::AABB;
	}

}