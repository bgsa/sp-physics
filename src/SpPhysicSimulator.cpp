#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	static SpPhysicSimulator* _instance;
	
	void SpPhysicSimulator::init()
	{
		_instance = sp_mem_new(SpPhysicSimulator)();
		_instance->gpu = GpuContext::instance()->defaultDevice();
	}

	SpPhysicSimulator* SpPhysicSimulator::instance()
	{
		return _instance;
	}

	void SpPhysicSimulator::add(SpPhysicObject* object)
	{
		_objects.add(object);
	}

	void SpPhysicSimulator::add(SpPhysicObjectList* objectsList)
	{
		/*
		DOP18* boundingVolumes = (DOP18*)objectsList->boundingVolumes();
		sp_uint boundingVolumesLength = objectsList->length();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << boundingVolumesLength
					<< " -DINPUT_STRIDE=" << DOP18_STRIDER
					<< " -DINPUT_OFFSET=" << DOP18_OFFSET;

		SweepAndPrune* sap = ALLOC_NEW(SweepAndPrune)();
		sap->init(gpu, buildOptions.str().c_str());
		sap->setParameters((sp_float*)boundingVolumes, boundingVolumesLength, DOP18_STRIDER, DOP18_OFFSET, 0, 1);
		*/

		 _objectsList.add(objectsList);
	}

	void SpPhysicSimulator::run()
	{
		/*
		sp_uint length = _objectsList.begin()->value()->length();
		DOP18* kdops = (DOP18*) _objectsList.begin()->value()->boundingVolumes();

		SweepAndPruneResultCpu result = sap.findCollisions(kdops, length);
		std::cout << result.length << END_OF_LINE;
		ALLOC_RELEASE(result.indexes);
		*/

		/*
		const sp_uint indexesLength = _objectsList.length() * 2;
		const sp_uint indexesSize = indexesLength * SIZEOF_UINT;

		cl_mem result = sap.execute();

		sp_uint* indexes = ALLOC_NEW_ARRAY(sp_uint, indexesLength);

		gpu->commandManager->executeReadBuffer(result, indexesSize, indexes, true);
		
		//SpEventDispatcher::instance()->push( collisionEvent );

		ALLOC_RELEASE(indexes);
		*/
	}

}