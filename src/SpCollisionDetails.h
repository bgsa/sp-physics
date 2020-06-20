#ifndef SP_COLLISION_DETAILS_HEADER
#define SP_COLLISION_DETAILS_HEADER

#include "SpectrumPhysics.h"
#include "Plane3D.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionDetails
	{
	public:
		Vec3 contactPoint;
		sp_float timeOfCollision;
		sp_float timeStep;
		sp_uint objectIndexPlane1;
		sp_uint objectIndexPlane2;
	};

}

#endif // SP_COLLISION_DETAILS_HEADER