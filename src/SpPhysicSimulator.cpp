#include "SpPhysicSimulator.h"
#include "SpWorldManager.h"
#include "SpString.h"
#include "SpCollisionResponseShapeMatching.h"

namespace NAMESPACE_PHYSICS
{
	sp_uint pcaFrameCounter = SP_UINT_MAX;

	void SpPhysicSimulator::init()
	{
		gpu = GpuContextInstance->defaultDevice();

		// Share OpenCL OpenGL Buffer
		//int error = CL10GL.clEnqueueAcquireGLObjects(queue, glMem, null, null);
		//error = CL10GL.clEnqueueReleaseGLObjects(queue, glMem, null, null);

		// dispose: CL10.clReleaseMemObject(glMem);

		collisionResponse = sp_mem_new(SpCollisionResponseShapeMatching);

		SpWorld* world = SpWorldManagerInstance->current();

		timerToPhysic.start();
		integrator = sp_mem_new(SpPhysicIntegratorVelocityVerlet)();

		const sp_uint outputIndexSize = multiplyBy2(world->objectsLengthAllocated()) * SP_SAP_MAX_COLLISION_PER_OBJECT * sizeof(sp_uint);

		_collisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_collisionIndexesLengthGPU = gpu->createBuffer(sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		
		_sapCollisionIndexesGPU = gpu->createBuffer(outputIndexSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		_sapCollisionIndexesLengthGPU = gpu->createBuffer(sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sap = sp_mem_new(SweepAndPrune)();
		sap->init(gpu, nullptr);
		sap->setParameters(world->boundingVolumeFactory->boundingVolumeGPU(), world->objectsLengthAllocated(), SpPhysicSettings::instance()->boundingVolumeType(),
			DOP18_STRIDER, world->_rigidBodies3DGPU, sizeof(SpRigidBody3D), _sapCollisionIndexesLengthGPU, _sapCollisionIndexesGPU);

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
		sap->updatePhysicProperties(SpWorldManagerInstance->current()->_rigidBodies3D);
	}

	void SpPhysicSimulator::handleCollisionCPU(void* threadParameter)
	{
		/*
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
		*/
	}

	void SpPhysicSimulator::handleCollisionGPU(void* threadParameter)
	{
		/*
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
		*/
	}

	void SpPhysicSimulator::findCollisionsCpu(SweepAndPruneResult* result)
	{
		SpWorld* world = SpWorldManagerInstance->current();

		sp_uint* sortedIndexes = ALLOC_ARRAY(sp_uint, world->objectsLength());
		for (sp_uint i = ZERO_UINT; i < world->objectsLength(); i++)
			sortedIndexes[i] = i;

		sap->findCollisions(world->_boundingVolumes, sortedIndexes, world->objectsLength(), result);

		ALLOC_RELEASE(sortedIndexes);
	}

	void SpPhysicSimulator::findCollisionsGpu(SweepAndPruneResult& result, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvent)
	{
		cl_event pEvent, lastEvent;

		sap->execute(previousEventsLength, previousEvents, &pEvent);
		
		result.length = sap->fetchCollisionLength(ONE_UINT, &pEvent, &lastEvent);
		gpu->releaseEvent(pEvent);

		sap->fetchCollisionIndexes(result.indexes, ONE_UINT, &lastEvent, currentEvent);
		gpu->releaseEvent(lastEvent);

		/*
		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU);

		collisionResponseGPU->execute(ONE_UINT, &sapDOP18->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
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
		world->_meshVertexCacheUpdater.execute(ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		/*
		// TODO: REMOVE !!!
		Vec3* vertexes = ALLOC_ARRAY(Vec3, 24 * 512);
		gpu->commandManager->readBuffer(world->_meshVertexCacheGPU->buffer(), sizeof(Vec3) * 24 * 512, vertexes);
		Vec3 firstVertex = vertexes[2601];

		SpTransform* trans = ALLOC_ARRAY(SpTransform, 512);
		gpu->commandManager->readBuffer(world->_transformsGPU, sizeof(SpTransform) * 512, trans);

		Vec3 v;
		world->mesh(1)->vertex(0, &v);

		Vec3 scale = world->transforms(1)->scaleVector;
		Vec3 vs(v.x * scale.x, v.y * scale.y, v.z * scale.z);

		Vec3 vt;
		world->mesh(1)->vertex(0, *world->transforms(1), vt);

		Vec3 angles;
		eulerAnglesXYZ(world->transforms(1)->orientation, angles);
		std::cout << "NEW OBJ 0" << std::endl;
		std::cout << "X: " << degree(angles.x) << std::endl;
		std::cout << "Y: " << degree(angles.y) << std::endl;
		std::cout << "Z: " << degree(angles.z) << std::endl;
		*/
		
		if (pcaFrameCounter > SpPhysicSettings::instance()->pcaExecutionPerFrame())
		{
			Vec3 axis;

			Timer t;
			t.start();
			if (sap->pca(axis, 50u))
			{
				((sp_float*)SpGlobalPropertiesInscance->get(ID_pcaTime))[0] = t.elapsedTime();
				((sp_uint*)SpGlobalPropertiesInscance->get(ID_qtdAlteracoesPCA))[0]++;
				((Vec3*)SpGlobalPropertiesInscance->get(ID_eixoPCA))[0] = axis;

				sap->updateAxis(axis);
			}
			pcaFrameCounter = ZERO_UINT;
		}
		pcaFrameCounter++;
		
		// build bounding volumes
		world->boundingVolumeFactory->execute(ZERO_UINT, NULL, &previousEvent);
		const sp_float timeBuildDOP18 = (sp_float)world->boundingVolumeFactory->timeOfLastExecution(previousEvent);
		((sp_float*)SpGlobalPropertiesInscance->get(ID_buildVolumeTime))[0] = timeBuildDOP18;

		/*
		// TODO: REMOVE
		DOP18 bvs[512];
		gpu->commandManager->readBuffer(world->boundingVolumeFactory->boundingVolumeGPU(), sizeof(DOP18) * 512, bvs);
		*/

		// find collisions pair on GPU using Bounding Volume
		findCollisionsGpu(sapResult, ONE_UINT, &previousEvent, &evt);
		((sp_uint*)SpGlobalPropertiesInscance->get(ID_paresSAP))[0] = sapResult.length;
		gpu->releaseEvent(previousEvent);
		gpu->releaseEvent(evt);

		sp_assert(sapResult.length <= world->objectsLength() * SP_SAP_MAX_COLLISION_PER_OBJECT, "ApplicationException");

		// release GPU shared buffer OpenCL and OpenGL
		gpu->commandManager->releaseGLObjects(world->_transformsGPU, ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		// Run on CPU way
		//updateMeshCache();
		//buildDOP18();
		//buildAABB();
		//findCollisionsCpu(&sapResult);

		evt = world->updateDataOnCPU();
		gpu->waitEvents(ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		collisionResponse->run(sapResult);

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