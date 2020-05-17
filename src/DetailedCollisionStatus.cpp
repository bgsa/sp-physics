#include "DetailedCollisionStatus.h"

namespace NAMESPACE_PHYSICS
{

	DetailedCollisionStatus::DetailedCollisionStatus(CollisionStatus status)
	{
		this->status = status;
	}

	DetailedCollisionStatus::DetailedCollisionStatus(CollisionStatus status, Vec3 point1)
	{
		this->status = status;

		points = ALLOC_ARRAY(Vec3, 1);
		points[0] = point1;

		pointsCount = 1;
	}

	DetailedCollisionStatus::DetailedCollisionStatus(CollisionStatus status, Vec3 point1, Vec3 point2)
	{
		this->status = status;

		points = ALLOC_ARRAY(Vec3, 2);
		points[0] = point1;
		points[1] = point2;

		pointsCount = 2;
	}

	DetailedCollisionStatus::DetailedCollisionStatus(CollisionStatus status, Vec3* points, sp_int pointsCount)
	{
		this->status = status;
		this->points = points;
		this->pointsCount = pointsCount;
	}

}