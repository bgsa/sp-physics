#include "AABB.h"

namespace NAMESPACE_PHYSICS
{

	CollisionStatus AABB::collisionStatus(const Plane& plane)
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

}