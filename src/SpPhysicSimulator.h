#ifndef SP_PHYSIC_SIMULATOR_HEADER
#define SP_PHYSIC_SIMULATOR_HEADER

#include "SpectrumPhysics.h"
#include "SpVector.h"
#include "SpPhysicObject.h"
#include "SpPhysicObjectList.h"
#include "GpuContext.h"
#include "DOP18.h"
#include "SweepAndPrune.h"
#include "SpEventDispatcher.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		sp_int gpuBuffer = -1;

		SpVector<SpPhysicObject*> _objects;
		SpVector<SpPhysicObjectList*> _objectsList;

		GpuDevice* gpu;
		SweepAndPrune sap;

		SpPhysicSimulator() { }

	public:

		API_INTERFACE static void init();

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE inline void add(SpPhysicObject* object);
		API_INTERFACE inline void add(SpPhysicObjectList* object);

		API_INTERFACE void run();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER