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

		_instance->boundingVolumesLength = ZERO_UINT;
		_instance->boundingVolumesLengthAllocated = objectsLength;
		_instance->boundingVolumes = sp_mem_new_array(DOP18, objectsLength);
		_instance->boundingVolumeBuffer = _instance->gpu->createBuffer(DOP18_SIZE*objectsLength, CL_MEM_READ_WRITE);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << _instance->boundingVolumesLengthAllocated
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

	BoundingVolume* SpPhysicSimulator::alloc(sp_uint length)
	{
		BoundingVolume* bvAllocated = &boundingVolumes[boundingVolumesLength];

		boundingVolumesLength += length;

		return bvAllocated;
	}

	void SpPhysicSimulator::run()
	{
		/*
		SweepAndPruneResultCpu resultCpu = sap->findCollisions(boundingVolumes, boundingVolumesLength);
		std::cout << "EXPECTED: " << resultCpu.length << END_OF_LINE;
		ALLOC_RELEASE(resultCpu.indexes);
		*/
		
		cl_event lastEvent = gpu->commandManager->updateBuffer(boundingVolumeBuffer, DOP18_SIZE * boundingVolumesLengthAllocated, boundingVolumes);

		cl_mem result = sap->execute(ONE_UINT, &lastEvent);

		const sp_uint collisionsLength = sap->fetchCollisionLength();
		const sp_uint indexesLength = collisionsLength * 2u;
		const sp_uint indexesSize = indexesLength * SIZEOF_UINT;

		sp_uint* indexes = ALLOC_NEW_ARRAY(sp_uint, indexesLength);
		gpu->commandManager->readBuffer(result, indexesSize, indexes);

		//SpEventDispatcher::instance()->push( collisionEvent );

		ALLOC_RELEASE(indexes);
	}

	void SpPhysicSimulator::dispose()
	{
		if (boundingVolumes != nullptr)
		{
			sp_mem_release(boundingVolumes);
			boundingVolumes = nullptr;
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