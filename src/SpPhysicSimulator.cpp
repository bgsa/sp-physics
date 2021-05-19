#include "SpPhysicSimulator.h"
#include "SpWorldManager.h"
#include "SpString.h"

namespace NAMESPACE_PHYSICS
{
	sp_uint pcaFrameCounter = SP_UINT_MAX;

	void SpPhysicSimulator::init()
	{
		gpu = GpuContext::instance()->defaultDevice();

		// Share OpenCL OpenGL Buffer
		//int error = CL10GL.clEnqueueAcquireGLObjects(queue, glMem, null, null);
		//error = CL10GL.clEnqueueReleaseGLObjects(queue, glMem, null, null);

		// dispose: CL10.clReleaseMemObject(glMem);

		SpWorld* world = SpWorldManagerInstance->current();

		timerToPhysic.start();
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

		sp_assert(details->type != SP_COLLISION_TYPE_NONE, "InvalidOperationException");
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

		sp_assert(details->type != SP_COLLISION_TYPE_NONE, "InvalidOperationException");
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

	void SpPhysicSimulator::findCollisionsGpuDOP18(SweepAndPruneResult& result, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvent)
	{
		cl_event pEvent, lastEvent;

		sapDOP18->execute(previousEventsLength, previousEvents, &pEvent);
		
		result.length = sapDOP18->fetchCollisionLength(ONE_UINT, &pEvent, &lastEvent);
		gpu->releaseEvent(pEvent);

		sapDOP18->fetchCollisionIndexes(result.indexes, ONE_UINT, &lastEvent, currentEvent);
		gpu->releaseEvent(lastEvent);

		/*
		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapDOP18->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
		*/
	}
	void SpPhysicSimulator::findCollisionsGpuAABB(SweepAndPruneResult& result, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvent)
	{
		cl_event evt;
		sapAABB->execute(previousEventsLength, previousEvents, &evt);

		result.length = sapAABB->fetchCollisionLength(ONE_UINT, &evt, currentEvent);
		gpu->releaseEvent(evt);

		/*
		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapAABB->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
		*/
	}
	void SpPhysicSimulator::findCollisionsGpuSphere(SweepAndPruneResult& result, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvent)
	{
		cl_event evt;
		sapSphere->execute(previousEventsLength, previousEvents, &evt);

		result.length = sapSphere->fetchCollisionLength(ONE_UINT, &evt, currentEvent);
		gpu->releaseEvent(evt);

		/*
		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapSphere->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result.length);
		collisionResponseGPU->fetchCollisions(result.indexes);
		*/
	}

	void SpPhysicSimulator::groupCollisions(const SweepAndPruneResult& sapResult, SpCollisionGroups* collisionGroups)
	{
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			sp_uint id = sapResult.indexes[multiplyBy2(i)];

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

	void SpPhysicSimulator::run(const sp_float elapsedTime)
	{
		SpWorld* world = SpWorldManagerInstance->current();

		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy2(world->objectsLength()) * SP_SAP_MAX_COLLISION_PER_OBJECT);

		// get GPU Buffer access shared OpenCL and OpenGL 
		cl_event previousEvent, evt;
		gpu->commandManager->acquireGLObjects(world->_transformsGPU, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		world->updateDataOnGPU(&evt);
		gpu->releaseEvent(evt);

		updateDataOnGPU();

		// update mesh cache vertexes
		world->_meshCacheUpdater.execute(ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);
		
		if (pcaFrameCounter > SpPhysicSettings::instance()->pcaExecutionPerFrame())
		{
			Vec3 axis;

			Timer t;
			t.start();
			if (sapDOP18->pca(axis, 50u))
			{
				((sp_float*)SpGlobalPropertiesInscance->get(ID_pcaTime))[0] = t.elapsedTime();
				((sp_uint*)SpGlobalPropertiesInscance->get(ID_qtdAlteracoesPCA))[0]++;
				((Vec3*)SpGlobalPropertiesInscance->get(ID_eixoPCA))[0] = axis;

				sapDOP18->axis = sapDOP18->axisIdForDOP18(axis);
				sapAABB->axis = sapAABB->axisIdForAABB(axis);
				sapSphere->axis = sapDOP18->axis;
			}
			pcaFrameCounter = ZERO_UINT;
		}
		pcaFrameCounter++;
		
		// build bounding volumes Sphere
		world->sphereFactory.execute(ZERO_UINT, NULL, &previousEvent);
		const sp_float timeBuildSpheres = (sp_float) world->sphereFactory.timeOfLastExecution(previousEvent);
		((sp_float*)SpGlobalPropertiesInscance->get(ID_buildSphereTime))[0] = timeBuildSpheres;

		// find collisions pair on GPU using Bounding Volume
		findCollisionsGpuSphere(sapResult, ONE_UINT, &previousEvent, &evt);
		((sp_float*)SpGlobalPropertiesInscance->get(ID_buildElementsSphereTime))[0] = ((sp_float*)SpGlobalPropertiesInscance->get(ID_buildElementsDOP18Time))[0];
		((sp_float*)SpGlobalPropertiesInscance->get(ID_sapSphereTime))[0] = ((sp_float*)SpGlobalPropertiesInscance->get(ID_sapDOP18Time))[0];
		((sp_uint*)SpGlobalPropertiesInscance->get(ID_paresSphere))[0] = sapResult.length;
		gpu->releaseEvent(previousEvent);
		gpu->releaseEvent(evt);

		// build bounding volumes AABB
		world->aabbFactory.execute(ZERO_UINT, NULL, &previousEvent);
		const sp_float timeBuildAABBs = (sp_float)world->aabbFactory.timeOfLastExecution(previousEvent);
		((sp_float*)SpGlobalPropertiesInscance->get(ID_buildAABBTime))[0] = timeBuildAABBs;
		
		// find collisions pair on GPU using Bounding Volume
		findCollisionsGpuAABB(sapResult, ONE_UINT, &previousEvent, &evt);
		((sp_float*)SpGlobalPropertiesInscance->get(ID_buildElementsAABBTime))[0] = ((sp_float*)SpGlobalPropertiesInscance->get(ID_buildElementsDOP18Time))[0];
		((sp_float*)SpGlobalPropertiesInscance->get(ID_sapAABBTime))[0] = ((sp_float*)SpGlobalPropertiesInscance->get(ID_sapDOP18Time))[0];
		((sp_uint*)SpGlobalPropertiesInscance->get(ID_paresAABB))[0] = sapResult.length;
		//sp_uint paresAABBLen = sapResult.length;
		//sp_uint* paresAABB = ALLOC_ARRAY(sp_uint, paresAABBLen * 2);
		//std::memcpy(paresAABB, sapResult.indexes, sizeof(sp_uint) * paresAABBLen * 2);
		//AABB* aabbs = ALLOC_ARRAY(AABB, world->objectsLength());
		//gpu->commandManager->readBuffer(world->aabbFactory.boundingVolumeGPU(), sizeof(AABB) * world->objectsLength(), aabbs);
		gpu->releaseEvent(previousEvent);
		gpu->releaseEvent(evt);

		// build bounding volumes 18-DOP
		world->dop18Factory.execute(ZERO_UINT, NULL, &previousEvent);
		const sp_float timeBuildDOP18 = (sp_float)world->dop18Factory.timeOfLastExecution(previousEvent);
		((sp_float*)SpGlobalPropertiesInscance->get(ID_buildDOP18Time))[0] = timeBuildDOP18;

		// find collisions pair on GPU using Bounding Volume
		findCollisionsGpuDOP18(sapResult, ONE_UINT, &previousEvent, &evt);
		((sp_uint*)SpGlobalPropertiesInscance->get(ID_paresDOP18))[0] = sapResult.length;
		//sp_uint paresKDOPLen = sapResult.length;
		//DOP18* kdops = ALLOC_ARRAY(DOP18, world->objectsLength());
		//gpu->commandManager->readBuffer(world->dop18Factory.boundingVolumeGPU(), sizeof(DOP18) * world->objectsLength(), kdops);
		gpu->releaseEvent(previousEvent);
		gpu->releaseEvent(evt);

		// release GPU shared buffer OpenCL and OpenGL
		gpu->commandManager->releaseGLObjects(world->_transformsGPU, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		/* Test kdop x aabb
		if (paresKDOPLen != paresAABBLen)
			if (paresKDOPLen > paresAABBLen)
			{

				// encontrar o par diferente
				for (sp_uint i = 0; i < paresKDOPLen; i++)
				{
					const sp_uint obj1 = sapResult.indexes[multiplyBy2(i)];
					const sp_uint obj2 = sapResult.indexes[multiplyBy2(i) + 1u];
					sp_uint obj1AABB, obj2AABB;
					sp_bool found = false;

					for (sp_uint j = 0; j < paresAABBLen; j++)
					{
						obj1AABB = paresAABB[multiplyBy2(j)];
						obj2AABB = paresAABB[multiplyBy2(j) + 1u];
				
						if ((obj1 == obj1AABB && obj2 == obj2AABB) 
							|| (obj1 == obj1AABB && obj2 == obj1AABB))
						{
							found = true;
							break;
						}
					}

					if (!found)
					{
						// pegar o bounding volumes desse par
						DOP18 kdop1 = kdops[obj1];
						DOP18 kdop2 = kdops[obj2];
						AABB aabb1 = aabbs[obj1AABB];
						AABB aabb2 = aabbs[obj2AABB];

						// comparar
						int a = 1;
					}
				}
			}
		*/

		// Run on CPU way
		//updateMeshCache();
		//buildDOP18();
		//buildAABB();
		//findCollisionsCpu(&sapResult);

		evt = world->updateDataOnCPU();
		gpu->waitEvents(ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		SpCollisionResponseShapeMatching shapeMatching;

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
				shapeMatching.updateFromShape(shapes[obj1]);
				shapes[obj1]->isDirty = false;
			}

			if (shapes[obj2]->isDirty && world->rigidBody3D(obj2)->isDynamic())
			{
				shapeMatching.updateFromShape(shapes[obj2]);
				shapes[obj2]->isDirty = false;
			}
		}

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