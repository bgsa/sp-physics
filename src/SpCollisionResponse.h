#ifndef SP_COLLISION_RESPONSE_HEADER
#define SP_COLLISION_RESPONSE_HEADER

#include "SpectrumPhysics.h"
#include "SpPhysicProperties.h"
#include "SpCollisionDetails.h"
#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionResponse
	{
	private:

		void addFriction(SpPhysicProperties* obj1Properties, SpPhysicProperties* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j, SpCollisionDetails* details);

	public:

		API_INTERFACE void handleCollisionResponse(SpCollisionDetails* details);
		
	};

}

#endif // SP_COLLISION_RESPONSE_HEADER