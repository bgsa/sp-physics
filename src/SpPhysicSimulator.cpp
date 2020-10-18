#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	static SpPhysicSimulator* _instance;
	
	SpPhysicSimulator* SpPhysicSimulator::instance()
	{
		return _instance;
	}

	SpPhysicSimulator::SpPhysicSimulator(sp_uint objectsLength)
	{
		sp_assert(instanceGpuRendering != nullptr, "NullPointerException");

		timerToPhysic.start();
		lastEvent = nullptr;
		_boundingVolumesGPU = nullptr;
		_physicPropertiesGPU = nullptr;
		integrator = sp_mem_new(SpPhysicIntegratorVelocityVerlet)();

		_objectsLength = ZERO_UINT;
		_objectsLengthAllocated = objectsLength;
		_physicProperties = sp_mem_new_array(SpPhysicProperties, objectsLength);
		_boundingVolumes = sp_mem_new_array(DOP18, objectsLength);
		_transforms = sp_mem_new_array(SpTransform, objectsLength);
		_collisionFeatures = sp_mem_new_array(SpCollisionFeatures, objectsLength);
		_meshes = sp_mem_new(SpArray<SpMesh*>)(objectsLength);

		gpu = GpuContext::instance()->defaultDevice();
		
		_transformsGPUBuffer = instanceGpuRendering->createTextureBuffer();
		_transformsGPUBuffer
			->use()
			->updateData(sizeof(SpTransform) * objectsLength, _transforms);

		_transformsGPU = gpu->createBufferFromOpenGL(_transformsGPUBuffer);

		const sp_uint outputIndexSize = multiplyBy2(objectsLength) * SP_SAP_MAX_COLLISION_PER_OBJECT * SIZEOF_UINT;

		_boundingVolumesGPU = gpu->createBuffer(_boundingVolumes, sizeof(DOP18) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		_physicPropertiesGPU = gpu->createBuffer(_physicProperties, sizeof(SpPhysicProperties) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, false);
		_collisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_collisionIndexesLengthGPU = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		_sapCollisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_sapCollisionIndexesLengthGPU = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << _objectsLengthAllocated
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0
			<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

		sap = sp_mem_new(SweepAndPrune)();
		sap->init(gpu, buildOptions.str().c_str());
		sap->setParameters(_boundingVolumesGPU, objectsLength,
			DOP18_STRIDER, 0, DOP18_ORIENTATIONS, _physicPropertiesGPU, sizeof(SpPhysicProperties), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU);

		collisionResponseGPU = sp_mem_new(SpCollisionResponseGPU);
		collisionResponseGPU->init(gpu, nullptr);
		collisionResponseGPU->setParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU, objectsLength, _boundingVolumesGPU, _physicPropertiesGPU,_collisionIndexesGPU, _collisionIndexesLengthGPU, outputIndexSize);
	}

	SpPhysicSimulator* SpPhysicSimulator::init(sp_uint objectsLength)
	{
		_instance = sp_mem_new(SpPhysicSimulator)(objectsLength);

		// Share OpenCL OpenGL Buffer
		//int error = CL10GL.clEnqueueAcquireGLObjects(queue, glMem, null, null);
		//error = CL10GL.clEnqueueReleaseGLObjects(queue, glMem, null, null);

		// dispose: CL10.clReleaseMemObject(glMem);
		return _instance;
	}

	void SpPhysicSimulator::handleCollisionCPU(void* threadParameter)
	{
		SpCollisionDetails* details = (SpCollisionDetails*)threadParameter;
		sp_assert(details != nullptr, "InvalidArgumentException");
		sp_assert(details->objIndex1 != details->objIndex2, "InvalidArgumentException");

		SpCollisionDetector collisionDetector;
		collisionDetector.filterCollision(details);
		if (details->ignoreCollision)
			return;

		SpCollisionResponse collisionResponse;
		collisionDetector.collisionDetails(details);

		if (details->ignoreCollision)
			return;

		sp_assert(details->type != SpCollisionType::None, "InvalidOperationException");
		sp_assert(details->contactPointsLength > 0u, "InvalidOperationException");

		collisionResponse.handleCollisionResponse(details);
	}

	void SpPhysicSimulator::handleCollisionGPU(void* threadParameter)
	{
		SpCollisionDetails* details = (SpCollisionDetails*)threadParameter;
		sp_assert(details != nullptr, "InvalidArgumentException");
		sp_assert(details->objIndex1 != details->objIndex2, "InvalidArgumentException");

		SpCollisionDetector collisionDetector;
		collisionDetector.collisionDetails(details);

		if (details->ignoreCollision)
			return;

		sp_assert(details->type != SpCollisionType::None, "InvalidOperationException");
		sp_assert(details->contactPointsLength > 0u, "InvalidOperationException");

		SpCollisionResponse collisionResponse;
		collisionResponse.handleCollisionResponse(details);
	}

	void SpPhysicSimulator::findCollisionsCpu(SweepAndPruneResult* result)
	{
		sp_uint* sortedIndexes = ALLOC_ARRAY(sp_uint, _objectsLength);
		for (sp_uint i = ZERO_UINT; i < _objectsLength; i++)
			sortedIndexes[i] = i;

		sap->findCollisions(_boundingVolumes, sortedIndexes, _objectsLength, result);

		ALLOC_RELEASE(sortedIndexes);
	}

	void SpPhysicSimulator::findCollisionsGpu(SweepAndPruneResult* result)
	{
		sap->execute(ONE_UINT, &sap->lastEvent);

		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU, _boundingVolumes, _physicProperties);

		collisionResponseGPU->execute(ONE_UINT, &sap->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
	}

	void SpPhysicSimulator::run()
	{
		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy2(_objectsLength) * SP_SAP_MAX_COLLISION_PER_OBJECT);

		//findCollisionsCpu(&sapResult);

		updateDataOnGPU();
		findCollisionsGpu(&sapResult);
		updateDataOnCPU();

		SpCollisionDetails* detailsArray = ALLOC_NEW_ARRAY(SpCollisionDetails, sapResult.length);
		SpThreadTask* tasks = ALLOC_NEW_ARRAY(SpThreadTask, sapResult.length);
		SpThreadPool* threadPool = SpThreadPool::instance();
		const sp_float elapsedTime = Timer::physicTimer()->elapsedTime();
		
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			detailsArray[i].objIndex1 = sapResult.indexes[multiplyBy2(i)];
			detailsArray[i].objIndex2 = sapResult.indexes[multiplyBy2(i) + 1];
			detailsArray[i].timeStep = elapsedTime;
			detailsArray[i].cacheObj1 = ALLOC_NEW(SpMeshCache)(mesh(collisionFeatures(detailsArray[i].objIndex1)->meshIndex)->vertexesMesh->length());
			detailsArray[i].cacheObj2 = ALLOC_NEW(SpMeshCache)(mesh(collisionFeatures(detailsArray[i].objIndex2)->meshIndex)->vertexesMesh->length());

			//tasks[i].func = &SpPhysicSimulator::handleCollisionCPU;
			tasks[i].func = &SpPhysicSimulator::handleCollisionGPU;
			tasks[i].parameter = &detailsArray[i];
			
			threadPool->schedule(&tasks[i]);
			//handleCollisionCPU(&detailsArray[i]);
		}
		SpThreadPool::instance()->waitToFinish();
		
		/* dispatch collision events
		for (sp_uint i = 0; i < sapResult.length; i++)
			if (!detailsArray[i].ignoreCollision)
				dispatchEvent(&detailsArray[i]);
		*/

		ALLOC_RELEASE(sapResult.indexes);
		sapResult.indexes = nullptr;
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

		if (_boundingVolumesGPU != nullptr)
		{
			gpu->releaseBuffer(_boundingVolumesGPU);
			_boundingVolumesGPU = nullptr;
		}

		if (_physicPropertiesGPU != nullptr)
		{
			gpu->releaseBuffer(_physicPropertiesGPU);
			_physicPropertiesGPU = nullptr;
		}

		if (_collisionIndexesGPU != nullptr)
		{
			gpu->releaseBuffer(_collisionIndexesGPU);
			_collisionIndexesGPU = nullptr;
		}

		if (_collisionIndexesLengthGPU != nullptr)
		{
			gpu->releaseBuffer(_collisionIndexesLengthGPU);
			_collisionIndexesLengthGPU = nullptr;
		}

		if (sap != nullptr)
		{
			sp_mem_delete(sap, SweepAndPrune);
			sap = nullptr;
		}

		if (collisionResponseGPU != nullptr)
		{
			sp_mem_delete(collisionResponseGPU, SpCollisionResponseGPU);
			collisionResponseGPU = nullptr;
		}
	}

	SpPhysicSimulator::~SpPhysicSimulator()
	{
		dispose();
	}

}