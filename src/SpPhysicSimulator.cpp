#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	static SpPhysicSimulator* _instance;
	
	SpPhysicSimulator* SpPhysicSimulator::instance()
	{
		return _instance;
	}

	void SpPhysicSimulator::init(sp_uint objectsLength)
	{
		_instance = sp_mem_new(SpPhysicSimulator)();
		_instance->gpu = GpuContext::instance()->defaultDevice();

		_instance->objectsLength = ZERO_UINT;
		_instance->objectsLengthAllocated = objectsLength;
		_instance->_boundingVolumes = sp_mem_new_array(DOP18, objectsLength);
		_instance->boundingVolumeBuffer = _instance->gpu->createBuffer(DOP18_SIZE*objectsLength, CL_MEM_READ_WRITE);

		_instance->_physicProperties = sp_mem_new_array(SpPhysicProperties, objectsLength);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << _instance->objectsLengthAllocated
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << DOP18_OFFSET
			<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

		_instance->sap = ALLOC_NEW(SweepAndPrune)();
		_instance->sap->init(_instance->gpu, buildOptions.str().c_str());

		_instance->sap->setParameters(_instance->boundingVolumeBuffer, objectsLength,
			DOP18_STRIDER, DOP18_OFFSET, DOP18_ORIENTATIONS);

		//cl_mem glMem = clCreateFromGLBuffer(context, CL_MEM_READ_WRITE, glBufId, null);

		//int error = CL10GL.clEnqueueAcquireGLObjects(queue, glMem, null, null);
		//error = CL10GL.clEnqueueReleaseGLObjects(queue, glMem, null, null);

		// dispose: CL10.clReleaseMemObject(glMem);
	}

	sp_uint SpPhysicSimulator::alloc(sp_uint length)
	{
		sp_uint allocated = objectsLength;		
		objectsLength += length;
		return allocated;
	}

	void SpPhysicSimulator::run(const Timer& timer)
	{
		SweepAndPruneResultCpu resultCpu;
		resultCpu.indexes = ALLOC_ARRAY(sp_uint, multiplyBy4(objectsLength));

		sap->findCollisions(_boundingVolumes, objectsLength, &resultCpu);

		sp_uint indexesLength = resultCpu.length;
		sp_uint* indexes = resultCpu.indexes;
		
		/*
		cl_event lastEvent = gpu->commandManager->updateBuffer(boundingVolumeBuffer, DOP18_SIZE * objectsLengthAllocated, _boundingVolumes);

		cl_mem result = sap->execute(ONE_UINT, &lastEvent);

		const sp_uint indexesLength = multiplyBy2(sap->fetchCollisionLength());

		sp_uint* indexes = ALLOC_NEW_ARRAY(sp_uint, indexesLength);
		gpu->commandManager->readBuffer(result, indexesLength * SIZEOF_UINT, indexes);
		*/

		const sp_float elapsedTime = timer.elapsedTime();

		for (sp_uint i = 0; i < indexesLength; i+=2u)
		{
			SpCollisionEvent* evt = sp_mem_new(SpCollisionEvent)();
			evt->indexBody1 = indexes[i];
			evt->indexBody2 = indexes[i+1];

			SpEventDispatcher::instance()->push(evt);

			handleCollision(evt->indexBody1, evt->indexBody2, elapsedTime);
		}

		//ALLOC_RELEASE(indexes);
		ALLOC_RELEASE(resultCpu.indexes);
	}

	void SpPhysicSimulator::dispose()
	{
		if (_boundingVolumes != nullptr)
		{
			sp_mem_release(_boundingVolumes);
			_boundingVolumes = nullptr;
		}

		if (_physicProperties != nullptr)
		{
			sp_mem_release(_physicProperties);
			_physicProperties = nullptr;
		}

		if (boundingVolumeBuffer != nullptr)
		{
			gpu->releaseBuffer(boundingVolumeBuffer);
			boundingVolumeBuffer = nullptr;
		}
	}

	SpPhysicSimulator::~SpPhysicSimulator()
	{
		dispose();
	}

}