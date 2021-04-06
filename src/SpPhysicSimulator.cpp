#include "SpPhysicSimulator.h"
#include "SpWorldManager.h"

namespace NAMESPACE_PHYSICS
{

	void SpPhysicSimulator::init()
	{
		gpu = GpuContext::instance()->defaultDevice();

		// Share OpenCL OpenGL Buffer
		//int error = CL10GL.clEnqueueAcquireGLObjects(queue, glMem, null, null);
		//error = CL10GL.clEnqueueReleaseGLObjects(queue, glMem, null, null);

		// dispose: CL10.clReleaseMemObject(glMem);

		SpWorld* world = SpWorldManagerInstance->current();

		timerToPhysic.start();
		lastEvent = nullptr;
		integrator = sp_mem_new(SpPhysicIntegratorVelocityVerlet)();

		const sp_uint outputIndexSize = multiplyBy2(world->objectsLengthAllocated()) * SP_SAP_MAX_COLLISION_PER_OBJECT * SIZEOF_UINT;

		_collisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_collisionIndexesLengthGPU = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		
		_sapCollisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_sapCollisionIndexesLengthGPU = gpu->createBuffer(SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sapDOP18 = sp_mem_new(SweepAndPrune)();
		sapDOP18->init(gpu, nullptr);
		sapDOP18->setParameters(world->dop18Factory.boundingVolumeGPU(), world->objectsLengthAllocated(), BoundingVolumeType::DOP18,
			DOP18_STRIDER, world->_rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU, "sweepAndPruneSingleAxis");

		sapAABB = sp_mem_new(SweepAndPrune)();
		sapAABB->init(gpu, nullptr);
		sapAABB->setParameters(world->dop18Factory.boundingVolumeGPU(), world->objectsLengthAllocated(), BoundingVolumeType::AABB,
			AABB_STRIDER, world->_rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU, "sweepAndPruneSingleAxisAABB");

		sapSphere = sp_mem_new(SweepAndPrune)();
		sapSphere->init(gpu, nullptr);
		sapSphere->setParameters(world->dop18Factory.boundingVolumeGPU(), world->objectsLengthAllocated(), BoundingVolumeType::Sphere,
			SPHERE_STRIDE, world->_rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU, "sweepAndPruneSingleAxisSphere");

		collisionResponseGPU = sp_mem_new(SpCollisionResponseGPU);
		collisionResponseGPU->init(gpu, nullptr);
		collisionResponseGPU->setParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU, world->objectsLengthAllocated(), world->_rigidBodies3DGPU,_collisionIndexesGPU, _collisionIndexesLengthGPU, outputIndexSize);
	}

	void SpPhysicSimulator::backToTime(const sp_uint index)
	{
		SpWorld* world = SpWorldManagerInstance->current();

		SpRigidBody3D* element = &world->_rigidBodies3D[index];

		Vec3 translation;
		diff(element->previousState.position(), element->currentState.position(), translation);

		element->rollbackState();

		world->_transforms[index].position = element->currentState.position();
		world->_transforms[index].orientation = element->currentState.orientation();

		world->_boundingVolumes[index].translate(translation);
	}

	void SpPhysicSimulator::moveAwayDynamicObjects()
	{
		SpWorld* world = SpWorldManagerInstance->current();

		for (sp_uint i = 0; i < world->objectsLength(); i++)
		{
			if (world->_rigidBodies3D[i].isStatic())
				continue;

			DOP18 bv1 = world->_boundingVolumes[i];

			for (sp_uint j = i + 1u; j < world->objectsLength(); j++)
			{
				if (world->_rigidBodies3D[j].isStatic())
					continue;

				if (bv1.collisionStatus(world->_boundingVolumes[j]) != CollisionStatus::OUTSIDE)
					world->translate(j, Vec3(0.0f, world->_boundingVolumes[j].height() + 0.1f, 0.0f));
			}
		}
	}

	void SpPhysicSimulator::updateDataOnGPU()
	{
		SpWorld* world = SpWorldManagerInstance->current();

		sapDOP18->updatePhysicProperties(world->_rigidBodies3D);
		sapAABB->updatePhysicProperties(world->_rigidBodies3D);
		sapSphere->updatePhysicProperties(world->_rigidBodies3D);
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
		SpWorld* world = SpWorldManagerInstance->current();

		sp_uint* sortedIndexes = ALLOC_ARRAY(sp_uint, world->objectsLength());
		for (sp_uint i = ZERO_UINT; i < world->objectsLength(); i++)
			sortedIndexes[i] = i;

		sapDOP18->findCollisions(world->_boundingVolumes, sortedIndexes, world->objectsLength(), result);

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
		SpWorld* world = SpWorldManagerInstance->current();

		SpPhysicSettings* physicSettings = SpPhysicSettings::instance();
		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy2(world->objectsLength()) * SP_SAP_MAX_COLLISION_PER_OBJECT);

		// get GPU Buffer access shared OpenCL and OpenGL 
		gpu->commandManager->acquireGLObjects(world->_transformsGPU);

		world->updateDataOnGPU();
		updateDataOnGPU();

		// update mesh cache vertexes
		world->_meshCacheUpdater.execute();

		// build bounding volumes Sphere
		world->sphereFactory.execute();

		// find collisions pair on GPU using Bounding Volume
		tt.update();
		findCollisionsGpuSphere(sapResult);
		const sp_float timeBroadPhaseSphere = tt.elapsedTime();
		const sp_uint paresBroadPhaseSphere = sapResult.length;

		// build bounding volumes AABB
		world->aabbFactory.execute();


		// find collisions pair on GPU using Bounding Volume
		tt.update(); 
		findCollisionsGpuAABB(&sapResult);
		const sp_float timeBroadPhaseAABB = tt.elapsedTime();
		const sp_uint paresBroadPhaseAABB = sapResult.length;

		// build bounding volumes 18-DOP
		world->dop18Factory.execute();

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
		gpu->commandManager->releaseGLObjects(world->_transformsGPU);

		// Run on CPU way
		//updateMeshCache();
		//buildDOP18();
		//buildAABB();
		//findCollisionsCpu(&sapResult);

		world->updateDataOnCPU();

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

		SpMeshCache** caches = ALLOC_NEW_ARRAY(SpMeshCache*, world->objectsLength());
		std::memset(caches, ZERO_INT, SIZEOF_WORD * world->objectsLength());

		SpRigidBodyShapeMatch** shapes = ALLOC_NEW_ARRAY(SpRigidBodyShapeMatch*, world->objectsLength());
		std::memset(shapes, ZERO_INT, SIZEOF_INT * world->objectsLength());

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

			if (shapes[obj1]->isDirty && world->rigidBody3D(obj1)->isDynamic())
			{
				shapeMatching.updateFromShape(obj1, &detailsArray[i], shapes[obj1]);
				shapes[obj1]->isDirty = false;
			}

			if (shapes[obj2]->isDirty && world->rigidBody3D(obj2)->isDynamic())
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