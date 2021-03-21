#ifndef SP_COLLISION_RESPONSE_HEADER
#define SP_COLLISION_RESPONSE_HEADER

#include "SpectrumPhysics.h"
#include "SpRigidBody3D.h"
#include "SpCollisionDetails.h"
#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionResponse
	{
	private:

		void addFriction(SpRigidBody3D* obj1Properties, SpRigidBody3D* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const sp_bool obj2IsPositiveNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j, SpCollisionDetails* details);

	public:

		API_INTERFACE void handleCollisionResponse(SpCollisionDetails* details);
		
	};

}

#endif // SP_COLLISION_RESPONSE_HEADER