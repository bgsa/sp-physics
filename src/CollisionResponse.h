#ifndef COLLISION_RESPONSE
#define COLLISION_RESPONSE

#include "SpectrumPhysics.h"
#include "Sphere.h"

namespace NAMESPACE_PHYSICS
{

	class CollisionResponse
	{
	private:
		CollisionResponse(const Vec3& contactPoint);

	public:
		Vec3 object1Impulse;
		Vec3 object2Impulse;
		Vec3 contactPoint;

		CollisionResponse();

		API_INTERFACE static CollisionResponse* handle(Sphere* sphere1, Sphere* sphere2);

		API_INTERFACE static void separeteSpheres(Sphere* sphere1, Sphere* sphere2);

	};

}

#endif // !COLLISION_RESPONSE