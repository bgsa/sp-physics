#ifndef SP_PHYSIC_SIMULATOR_HEADER
#define SP_PHYSIC_SIMULATOR_HEADER

#include "SpectrumPhysics.h"
#include "SpVector.h"
#include "SpPhysicObject.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:

		SpVector<SpPhysicObject*> _objects;

		SpPhysicSimulator();

	public:

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE inline void add(SpPhysicObject* object);

		API_INTERFACE void run();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER