#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{

	SpPhysicSimulator::SpPhysicSimulator()
	{
	}

	SpPhysicSimulator* SpPhysicSimulator::instance()
	{
		static SpPhysicSimulator* _instance = sp_mem_new(SpPhysicSimulator)();
		return _instance;
	}

	void SpPhysicSimulator::add(SpPhysicObject* object)
	{
		_objects.add(object);
	}

	void SpPhysicSimulator::run()
	{

	}

}