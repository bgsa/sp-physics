#ifndef SP_COLLISION_DETAILS_HEADER
#define SP_COLLISION_DETAILS_HEADER

#include "SpectrumPhysics.h"
#include "Plane3D.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionDetails
	{
	public:
		sp_uint objIndex1;
		sp_uint objIndex2;
		sp_uint objectIndexPlane1;
		sp_uint objectIndexPlane2;
		sp_float timeOfCollision;
		sp_float timeStep;
		Vec3 contactPoint;
		sp_bool ignoreCollision;

		SpCollisionDetails()
		{
			ignoreCollision = false;
		}
	};

}

#endif // SP_COLLISION_DETAILS_HEADER