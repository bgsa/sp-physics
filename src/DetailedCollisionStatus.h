#ifndef DETAILED_COLLISION_STATUS_HEADER
#define DETAILED_COLLISION_STATUS_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"

namespace NAMESPACE_PHYSICS
{
	class DetailedCollisionStatus
	{
	public:
		sp_uint pointsCount = 0;
		Vec3* points = nullptr;
		CollisionStatus status;

		API_INTERFACE DetailedCollisionStatus(CollisionStatus status);

		API_INTERFACE DetailedCollisionStatus(CollisionStatus status, Vec3 point1);

		API_INTERFACE DetailedCollisionStatus(CollisionStatus status, Vec3 point1, Vec3 point2);

		API_INTERFACE DetailedCollisionStatus(CollisionStatus status, Vec3* points, sp_int pointsCount);

	};
}

#endif // DETAILED_COLLISION_STATUS_HEADER