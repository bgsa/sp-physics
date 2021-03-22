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
		sp_uint* meshCacheIndexes = ALLOC_NEW_ARRAY(sp_uint, _objectsLength);
		sp_uint* meshCacheVertexesLength = ALLOC_NEW_ARRAY(sp_uint, _objectsLength);
		sp_uint* meshesIndexes = ALLOC_NEW_ARRAY(sp_uint, _objectsLength * 3u);
		meshCacheIndexes[0] = ZERO_UINT;

		SpMesh* m = mesh(collisionFeatures(0u)->meshIndex);
		meshCacheVertexesLength[0] = m->vertexesMesh->length();

		sp_uint vertexCounter = meshCacheVertexesLength[0];

		const sp_size initialMemoryIndex = (sp_size)_meshes->data()[0];
		sp_size vertexMemoryIndex = (sp_size)_meshes->data()[0]->vertexesMesh->data()[0];
		sp_size faceMemoryIndex = (sp_size)_meshes->data()[0]->faces->data()[0];
		sp_size edgeMemoryIndex = (sp_size)_meshes->data()[0]->edges->data()[0];

		meshesIndexes[0] = divideBy4(vertexMemoryIndex - initialMemoryIndex);
		meshesIndexes[1] = divideBy4(faceMemoryIndex - initialMemoryIndex);
		meshesIndexes[2] = divideBy4(edgeMemoryIndex - initialMemoryIndex);

		SpPoolMemoryAllocator::main()->enableMemoryAlignment();

		_meshesCache = sp_mem_new(SpArray<SpMeshCache*>)(_objectsLength, _objectsLength);
		_meshesCache->data()[0] = sp_mem_new(SpMeshCache)(m->vertexesMesh->length());

		for (sp_uint i = 1; i < _objectsLength; i++)
		{
			SpMesh* m = mesh(collisionFeatures(i)->meshIndex);
			
			const sp_uint vertexLength = m->vertexesMesh->length();

			_meshesCache->data()[i] = sp_mem_new(SpMeshCache)(vertexLength);

			meshCacheIndexes[i] = meshCacheIndexes[i - 1] + meshCacheVertexesLength[i - 1] * 3u;
			meshCacheVertexesLength[i] = vertexLength;
			vertexCounter += vertexLength;

			vertexMemoryIndex = (sp_size)m->vertexesMesh->data()[0];
			faceMemoryIndex = (sp_size)m->faces->data()[0];
			edgeMemoryIndex = (sp_size)m->edges->data()[0];

			const sp_uint idx = i * 3u;
			meshesIndexes[idx     ] = divideBy4(vertexMemoryIndex - initialMemoryIndex);
			meshesIndexes[idx + 1u] = divideBy4(faceMemoryIndex - initialMemoryIndex);
			meshesIndexes[idx + 2u] = divideBy4(edgeMemoryIndex - initialMemoryIndex);
		}
		
		SpPoolMemoryAllocator::main()->disableMemoryAlignment();

#ifdef OPENCL_ENABLED
		_objectMapperGPU->update(_objectMapper);

		SpMesh* lastMesh = _meshes->data()[collisionFeatures(_objectsLength - 1u)->meshIndex];
		SpEdgeMesh* lastEdge = lastMesh->edges->data()[lastMesh->edges->length() - 1u];
		sp_size lastMemoryAddress = (sp_size) &lastEdge->faces.data()[lastEdge->faces.length() - 1u];

		_meshesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshesGPU->init(lastMemoryAddress - initialMemoryIndex, _meshes->data()[0], CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

		_meshesIndexesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshesIndexesGPU->init(_objectsLength * 3u * SIZEOF_UINT, meshesIndexes, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

		_meshCacheGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshCacheGPU->init(vertexCounter * VEC3_SIZE);

		_meshCacheIndexesGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshCacheIndexesGPU->init(_objectsLength * SIZEOF_UINT, meshCacheIndexes, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

		_meshCacheVertexesLengthGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_meshCacheVertexesLengthGPU->init(_objectsLength * SIZEOF_UINT, meshCacheVertexesLength, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR);

		_inputLengthGPU->update(&_objectsLength);

		_meshCacheUpdater.init(gpu);
		_meshCacheUpdater.setParameters(_inputLengthGPU, _objectMapperGPU, _meshesGPU, _meshesIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _meshCacheIndexesGPU, _meshCacheGPU, _objectsLength);

		dop18Factory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _boundingVolumesGPU);
		aabbFactory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _boundingVolumesGPU);
		sphereFactory.init(gpu, _inputLengthGPU, _objectsLength, _meshCacheGPU, _meshCacheIndexesGPU, _meshCacheVertexesLengthGPU, _transformsGPU, _boundingVolumesGPU);
#endif

		ALLOC_RELEASE(meshCacheIndexes);
	}

	void SpPhysicSimulator::updateMeshCache()
	{
		for (sp_uint i = 0u; i < _objectsLength; i++)
			_meshesCache->get(i)->update(mesh(collisionFeatures(i)->meshIndex), transforms(i));
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

	void SpPhysicSimulator::buildAABB() const
	{
		for (sp_uint i = 0; i < _objectsLength; i++)
		{
			SpMesh* mesh = this->mesh(collisionFeatures(i)->meshIndex);
			SpMeshCache* cache = _meshesCache->get(i);

			aabbFactory.build(mesh, cache, transforms(i)->position, &_boundingVolumes[i]);
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
		_rigidBodies3D = sp_mem_new_array(SpRigidBody3D, objectsLength);
		_boundingVolumes = sp_mem_new_array(DOP18, objectsLength);
		_transforms = sp_mem_new_array(SpTransform, objectsLength);
		_objectMapper = sp_mem_new_array(SpCollisionFeatures, objectsLength);
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

		_objectMapperGPU = sp_mem_new(GpuBufferOpenCL)(gpu);
		_objectMapperGPU->init(sizeof(SpCollisionFeatures) * _objectsLengthAllocated, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		_boundingVolumesGPU = gpu->createBuffer(_boundingVolumes, sizeof(DOP18) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		_rigidBodies3DGPU = gpu->createBuffer(_rigidBodies3D, sizeof(SpRigidBody3D) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, false);
		_collisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_collisionIndexesLengthGPU = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		
		_sapCollisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_sapCollisionIndexesLengthGPU = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << _objectsLengthAllocated
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0
			<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

		sapDOP18 = sp_mem_new(SweepAndPrune)();
		sapDOP18->init(gpu, buildOptions.str().c_str());
		sapDOP18->setParameters(_boundingVolumesGPU, objectsLength,
			DOP18_STRIDER, 0, DOP18_ORIENTATIONS, _rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU, "sweepAndPruneSingleAxis");

		sapAABB = sp_mem_new(SweepAndPrune)();
		sapAABB->init(gpu, buildOptions.str().c_str());
		sapAABB->setParameters(_boundingVolumesGPU, objectsLength,
			DOP18_STRIDER, 0, DOP18_ORIENTATIONS, _rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU, "sweepAndPruneSingleAxisAABB");

		sapSphere = sp_mem_new(SweepAndPrune)();
		sapSphere->init(gpu, buildOptions.str().c_str());
		sapSphere->setParameters(_boundingVolumesGPU, objectsLength,
			DOP18_STRIDER, 0, DOP18_ORIENTATIONS, _rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU, "sweepAndPruneSingleAxisSphere");

		collisionResponseGPU = sp_mem_new(SpCollisionResponseGPU);
		collisionResponseGPU->init(gpu, nullptr);
		collisionResponseGPU->setParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU, objectsLength, _rigidBodies3DGPU,_collisionIndexesGPU, _collisionIndexesLengthGPU, outputIndexSize);
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
		//Timer timeDebug;
		//timeDebug.start();

		SpCollisionDetails* details = (SpCollisionDetails*)threadParameter;
		sp_assert(details != nullptr, "InvalidArgumentException");
		sp_assert(details->objIndex1 != details->objIndex2, "InvalidArgumentException");

		SpCollisionDetector collisionDetector;
		collisionDetector.collisionDetails(details);
		//sp_log_debug1sfnl("Collision Details: ", timeDebug.elapsedTime());

		if (details->ignoreCollision)
			return;

		sp_assert(details->type != SpCollisionType::None, "InvalidOperationException");
		sp_assert(details->contactPointsLength > 0u, "InvalidOperationException");

		//timeDebug.update();
		SpCollisionResponse collisionResponse;
		collisionResponse.handleCollisionResponse(details);
		//sp_log_debug1sfnl("Collision Response: ", timeDebug.elapsedTime());

		//sp_log_debug1sfnl("TASK END: ", timeDebug.elapsedTime());
	}

	void SpPhysicSimulator::findCollisionsCpu(SweepAndPruneResult* result)
	{
		sp_uint* sortedIndexes = ALLOC_ARRAY(sp_uint, _objectsLength);
		for (sp_uint i = ZERO_UINT; i < _objectsLength; i++)
			sortedIndexes[i] = i;

		sapDOP18->findCollisions(_boundingVolumes, sortedIndexes, _objectsLength, result);

		ALLOC_RELEASE(sortedIndexes);
	}

	void SpPhysicSimulator::findCollisionsGpuDOP18(SweepAndPruneResult* result)
	{
		sapDOP18->execute(ONE_UINT, &sapDOP18->lastEvent);
		
		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapDOP18->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
	}
	void SpPhysicSimulator::findCollisionsGpuAABB(SweepAndPruneResult* result)
	{
		sapAABB->execute(ONE_UINT, &sapAABB->lastEvent);

		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapAABB->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
	}
	void SpPhysicSimulator::findCollisionsGpuSphere(SweepAndPruneResult& result)
	{
		sapSphere->execute(ONE_UINT, &sapSphere->lastEvent);

		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapSphere->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result.length);
		collisionResponseGPU->fetchCollisions(result.indexes);
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

	Timer tt;
	void SpPhysicSimulator::run(const sp_float elapsedTime)
	{
		SpPhysicSettings* physicSettings = SpPhysicSettings::instance();
		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy2(_objectsLength) * SP_SAP_MAX_COLLISION_PER_OBJECT);

		// get GPU Buffer access shared OpenCL and OpenGL 
		gpu->commandManager->acquireGLObjects(_transformsGPU);

		updateDataOnGPU();

		// update mesh cache vertexes
		_meshCacheUpdater.execute();

		// build bounding volumes Sphere
		sphereFactory.buildGPU();

		// find collisions pair on GPU using Bounding Volume
		tt.update();
		findCollisionsGpuSphere(sapResult);
		const sp_float timeBroadPhaseSphere = tt.elapsedTime();
		const sp_uint paresBroadPhaseSphere = sapResult.length;

		// build bounding volumes AABB
		aabbFactory.buildGPU();

		// find collisions pair on GPU using Bounding Volume
		tt.update(); 
		findCollisionsGpuAABB(&sapResult);
		const sp_float timeBroadPhaseAABB = tt.elapsedTime();
		const sp_uint paresBroadPhaseAABB = sapResult.length;

		// build bounding volumes 18-DOP
		dop18Factory.buildGPU();

		// find collisions pair on GPU using Bounding Volume
		tt.update();
		findCollisionsGpuDOP18(&sapResult);
		const sp_float timeBroadPhaseDOP18 = tt.elapsedTime();
		const sp_float paresBroadPhaseDOP18 = sapResult.length;


		sp_log_debug1sfnl("Pares DOP18: ", (sp_float) paresBroadPhaseDOP18);
		sp_log_debug1sfnl("Tempo DOP18: ", timeBroadPhaseDOP18);
		sp_log_debug1sfnl("Pares AABB: ", (sp_float)paresBroadPhaseAABB);
		sp_log_debug1sfnl("Tempo AABB: ", timeBroadPhaseAABB); 
		sp_log_debug1sfnl("Pares Sphere: ", (sp_float)paresBroadPhaseSphere);
		sp_log_debug1sfnl("Tempo Sphere: ", timeBroadPhaseSphere);

		//const sp_uint collisionsWith18DOP = sapResult.length;
		//sp_log_info1s("Collisions: "); sp_log_info1u(sapResult.length); sp_log_newline();

		// release GPU shared buffer OpenCL and OpenGL
		gpu->commandManager->releaseGLObjects(_transformsGPU);

		// Run on CPU way
		//updateMeshCache();
		//buildDOP18();
		//buildAABB();
		//findCollisionsCpu(&sapResult);

		updateDataOnCPU();

		//SpThreadPool* threadPool = SpThreadPool::instance();

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


		tt.update();

		SpCollisionDetails* detailsArray = ALLOC_NEW_ARRAY(SpCollisionDetails, sapResult.length);
		//SpThreadTask* tasks = ALLOC_NEW_ARRAY(SpThreadTask, sapResult.length);
		//std::thread** threads = ALLOC_ARRAY(std::thread*, sapResult.length);

		SpCollisionResponseShapeMatching shapeMatching;

		SpMeshCache** caches = ALLOC_NEW_ARRAY(SpMeshCache*, _objectsLength);
		std::memset(caches, ZERO_INT, SIZEOF_WORD * _objectsLength);

		SpRigidBodyShapeMatch** shapes = ALLOC_NEW_ARRAY(SpRigidBodyShapeMatch*, _objectsLength);
		std::memset(shapes, ZERO_INT, SIZEOF_INT * _objectsLength);

		// init shapes
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			const sp_uint obj1 = sapResult.indexes[multiplyBy2(i)];
			const sp_uint obj2 = sapResult.indexes[multiplyBy2(i) + 1u];

			if (shapes[obj1] == nullptr)
			{
				shapes[obj1] = ALLOC_NEW(SpRigidBodyShapeMatch)();
				shapeMatching.initShape(obj1, shapes[obj1]);
			}

			if (shapes[obj2] == nullptr)
			{
				shapes[obj2] = ALLOC_NEW(SpRigidBodyShapeMatch)();
				shapeMatching.initShape(obj2, shapes[obj2]);
			}
		}
		// many shape match iterations
		for (sp_uint iterations = 0u; iterations < 10u; iterations++)
		{
			for (sp_uint i = 0u; i < sapResult.length; i++)
				shapeMatching.solve(
					shapes[sapResult.indexes[multiplyBy2(i)]], 
					shapes[sapResult.indexes[multiplyBy2(i) + 1u]]
				);
		}

		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			sp_uint obj1 = sapResult.indexes[multiplyBy2(i)];
			sp_uint obj2 = sapResult.indexes[multiplyBy2(i) + 1];

			if (shapes[obj1]->isDirty && rigidBody3D(obj1)->isDynamic())
			{
				shapeMatching.updateFromShape(obj1, &detailsArray[i], shapes[obj1]);
				shapes[obj1]->isDirty = false;
			}

			if (shapes[obj2]->isDirty && rigidBody3D(obj2)->isDynamic())
			{
				shapeMatching.updateFromShape(obj2, &detailsArray[i], shapes[obj2]);
				shapes[obj2]->isDirty = false;
			}
		}

		/* dispatch collision events
		for (sp_uint i = 0; i < sapResult.length; i++)
			if (!detailsArray[i].ignoreCollision)
				dispatchEvent(&detailsArray[i]);
		*/

		sp_float et = tt.elapsedTime();
		sp_log_debug1sfnl("TIME: ", et);

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

		if (_rigidBodies3D != nullptr)
		{
			sp_mem_release(_rigidBodies3D);
			_rigidBodies3D = nullptr;
		}

		if (_boundingVolumesGPU != nullptr)
		{
			gpu->releaseBuffer(_boundingVolumesGPU);
			_boundingVolumesGPU = nullptr;
		}

		if (_rigidBodies3DGPU != nullptr)
		{
			gpu->releaseBuffer(_rigidBodies3DGPU);
			_rigidBodies3DGPU = nullptr;
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

		if (_objectMapperGPU != nullptr)
		{
			sp_mem_delete(_objectMapperGPU, GpuBufferOpenCL);
			_objectMapperGPU = nullptr;
		}

		if (sapDOP18 != nullptr)
		{
			sp_mem_delete(sapDOP18, SweepAndPrune);
			sapDOP18 = nullptr;
		}

		if (sapAABB != nullptr)
		{
			sp_mem_delete(sapAABB, SweepAndPrune);
			sapAABB = nullptr;
		}

		if (sapSphere != nullptr)
		{
			sp_mem_delete(sapSphere, SweepAndPrune);
			sapSphere = nullptr;
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