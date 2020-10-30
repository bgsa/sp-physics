#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	static SpPhysicSimulator* _instance;
	
	SpPhysicSimulator* SpPhysicSimulator::instance()
	{
		return _instance;
	}

	void SpPhysicSimulator::initMeshCache()
	{
		sp_uint vertexCounter = ZERO_UINT;
		sp_uint* meshCacheVertexesLength = ALLOC_NEW_ARRAY(sp_uint, _objectsLength);

		PoolMemoryAllocator::main()->enableMemoryAlignment();

		_meshesCache = sp_mem_new(SpArray<SpMeshCache*>)(_objectsLength, _objectsLength);

		for (sp_uint i = 0; i < _objectsLength; i++)
		{
			const sp_uint verteLength = mesh(collisionFeatures(i)->meshIndex)->vertexesMesh->length();

			_meshesCache->data()[i] = sp_mem_new(SpMeshCache)(verteLength);
		
			vertexCounter += verteLength;
			meshCacheVertexesLength[i] = verteLength;
		}
		
		PoolMemoryAllocator::main()->disableMemoryAlignment();

#ifdef OPENCL_ENABLED
		const sp_uint size = (_objectsLength * 3u) * vertexCounter * VEC3_SIZE;
		_meshCacheGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshCacheGPU->init(size);

		_meshCacheIndexesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshCacheIndexesGPU->init(_objectsLength * SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		_meshCacheVertexesLengthGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshCacheVertexesLengthGPU->init(_objectsLength * SIZEOF_UINT, meshCacheVertexesLength, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);

		_inputLengthGPU->update(&_objectsLength);

		dop18Factory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _boundingVolumesGPU);
#endif

		ALLOC_RELEASE(meshCacheVertexesLength);
	}

	void SpPhysicSimulator::updateMeshCache()
	{
		sp_uint* meshCacheIndexes = ALLOC_NEW_ARRAY(sp_uint, _objectsLength);
		meshCacheIndexes[0] = 3u;

		SpMesh* mesh = this->mesh(collisionFeatures(0u)->meshIndex);
		SpTransform* transformation = transforms(0u);
		_meshesCache->get(0u)->update(mesh, transformation);

		sp_uint previousVertexLength = mesh->vertexesMesh->length();

		for (sp_uint i = 1u; i < _objectsLength; i++)
		{
			mesh = this->mesh(collisionFeatures(i)->meshIndex);
			transformation = transforms(i);

			_meshesCache->get(i)->update(mesh, transformation);

			meshCacheIndexes[i] = meshCacheIndexes[i - 1] + previousVertexLength * 3u + 3u;
			previousVertexLength = mesh->vertexesMesh->length();
		}

		_meshCacheIndexesGPU->update(meshCacheIndexes);
		_meshCacheGPU->update(_meshesCache->data());
	}

	void SpPhysicSimulator::buildDOP18() const
	{
		for (sp_uint i = 0; i < _objectsLength; i++)
		{
			SpMesh* mesh = this->mesh(collisionFeatures(i)->meshIndex);
			SpMeshCache* cache = _meshesCache->get(i);
			
			dop18Factory.build(mesh, cache, transforms(i)->position, &_boundingVolumes[i]);
		}
	}

	SpPhysicSimulator::SpPhysicSimulator(sp_uint objectsLength)
	{
		sp_assert(instanceGpuRendering != nullptr, "NullPointerException");

		timerToPhysic.start();
		lastEvent = nullptr;
		integrator = sp_mem_new(SpPhysicIntegratorVelocityVerlet)();

		_objectsLength = ZERO_UINT;
		_objectsLengthAllocated = objectsLength;
		_physicProperties = sp_mem_new_array(SpPhysicProperties, objectsLength);
		_boundingVolumes = sp_mem_new_array(DOP18, objectsLength);
		_transforms = sp_mem_new_array(SpTransform, objectsLength);
		_collisionFeatures = sp_mem_new_array(SpCollisionFeatures, objectsLength);
		_meshes = sp_mem_new(SpArray<SpMesh*>)(objectsLength, objectsLength);
		
		gpu = GpuContext::instance()->defaultDevice();

		_transformsGPUBuffer = instanceGpuRendering->createTextureBuffer();
		_transformsGPUBuffer
			->use()
			->updateData(sizeof(SpTransform) * objectsLength, _transforms);

		_transformsGPU = gpu->createBufferFromOpenGL(_transformsGPUBuffer);

		const sp_uint outputIndexSize = multiplyBy2(objectsLength) * SP_SAP_MAX_COLLISION_PER_OBJECT * SIZEOF_UINT;

		_inputLengthGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_inputLengthGPU->init(SIZEOF_UINT, &_objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

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

	void SpPhysicSimulator::groupCollisions(const SweepAndPruneResult& sapResult, SpCollisionGroups* collisionGroups)
	{
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			sp_uint id = sapResult.indexes[multiplyBy2(i)];

			// TODO: REMOVE !! ignora o indice 0 pq eh o chao!
			//if (id == 0) continue;

			sp_uint groupId = collisionGroups->mapper[id];

			if (groupId == SP_UINT_MAX) // if does not has allocated group, create
			{
				groupId = collisionGroups->groupLength++;
				collisionGroups->mapper[id] = groupId;
				collisionGroups->groups[groupId].id = id;
			}

			collisionGroups->groups[groupId].addElement(sapResult.indexes[multiplyBy2(i) + 1]);
		}
	}

	sp_uint pairsLeaf(const SpCollisionGroups& groups, const SpCollisionGroup& group, sp_uint* output)
	{
		sp_uint pairsLength = ZERO_UINT;

		for (sp_uint i = 0; i < group.elementsLength; i++)
		{
			sp_uint element = group.elements[i];
			sp_uint index = groups.mapper[element];

			if (index == SP_UINT_MAX)
			{
				output[pairsLength++] = group.id;
				output[pairsLength++] = element;
			}
			else
				pairsLength += pairsLeaf(groups, groups.groups[index], &output[pairsLength]);
		}

		return pairsLength;
	}

	void SpPhysicSimulator::run()
	{
		updateDataOnGPU();
		
		updateMeshCache();

		// TODO: FAZER !!!
		//updateMeshCacheGPU();

		dop18Factory.buildGPU(gpu, _transformsGPU);

		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy2(_objectsLength) * SP_SAP_MAX_COLLISION_PER_OBJECT);

		// CPU way
		//buildDOP18();
		//findCollisionsCpu(&sapResult);

		findCollisionsGpu(&sapResult);
		updateDataOnCPU();

		SpThreadPool* threadPool = SpThreadPool::instance();
		const sp_float elapsedTime = Timer::physicTimer()->elapsedTime();

		/*
		SpCollisionGroups groups(_objectsLength, multiplyBy2(sapResult.length));
		groupCollisions(sapResult, &groups);

		sp_uint taskIndex = ZERO_UINT;
		SpCollisionDetails* detailsArray = ALLOC_NEW_ARRAY(SpCollisionDetails, multiplyBy4(_objectsLength));
		SpThreadTask* tasks = ALLOC_NEW_ARRAY(SpThreadTask, multiplyBy4(_objectsLength));

		for (sp_uint i = 0; i < groups.groupLength; i++)
		{
			sp_uint* pairs = ALLOC_ARRAY(sp_uint, multiplyBy2(50));
			sp_uint pairsLength = pairsLeaf(groups, groups.groups[i], pairs);

			for (sp_uint j = 0; j < pairsLength; j+=2)
			{
				detailsArray[taskIndex].objIndex1 = pairs[j];
				detailsArray[taskIndex].objIndex2 = pairs[j + 1];
				detailsArray[taskIndex].timeStep = elapsedTime;
				detailsArray[taskIndex].cacheObj1 = ALLOC_NEW(SpMeshCache)(mesh(collisionFeatures(detailsArray[taskIndex].objIndex1)->meshIndex)->vertexesMesh->length());
				detailsArray[taskIndex].cacheObj2 = ALLOC_NEW(SpMeshCache)(mesh(collisionFeatures(detailsArray[taskIndex].objIndex2)->meshIndex)->vertexesMesh->length());

				tasks[taskIndex].func = &SpPhysicSimulator::handleCollisionCPU;
				//tasks[taskIndex].func = &SpPhysicSimulator::handleCollisionGPU;
				tasks[taskIndex].parameter = &detailsArray[taskIndex];

				threadPool->schedule(&tasks[taskIndex]);
				//handleCollisionCPU(&detailsArray[taskIndex]);
				taskIndex++;
			}

			ALLOC_RELEASE(pairs);
		}
		*/

		SpCollisionDetails* detailsArray = ALLOC_NEW_ARRAY(SpCollisionDetails, sapResult.length);
		SpThreadTask* tasks = ALLOC_NEW_ARRAY(SpThreadTask, sapResult.length);
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			detailsArray[i].objIndex1 = sapResult.indexes[multiplyBy2(i)];
			detailsArray[i].objIndex2 = sapResult.indexes[multiplyBy2(i) + 1];
			detailsArray[i].timeStep = elapsedTime;
			detailsArray[i].cacheObj1 = ALLOC_NEW(SpMeshCache)(mesh(collisionFeatures(detailsArray[i].objIndex1)->meshIndex)->vertexesMesh->length());
			detailsArray[i].cacheObj2 = ALLOC_NEW(SpMeshCache)(mesh(collisionFeatures(detailsArray[i].objIndex2)->meshIndex)->vertexesMesh->length());

			tasks[i].func = &SpPhysicSimulator::handleCollisionCPU;
			//tasks[i].func = &SpPhysicSimulator::handleCollisionGPU;
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