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
		_instance->_objectsLength = ZERO_UINT;
		_instance->_objectsLengthAllocated = objectsLength;
		_instance->_physicProperties = sp_mem_new_array(SpPhysicProperties, objectsLength);
		_instance->_boundingVolumes = sp_mem_new_array(DOP18, objectsLength);

		_instance->gpu = GpuContext::instance()->defaultDevice();  // TODO: ENABLE!
		_instance->boundingVolumeBuffer = _instance->gpu->createBuffer(DOP18_SIZE*objectsLength, CL_MEM_READ_WRITE); // TODO: ENABLE!

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << _instance->_objectsLengthAllocated
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << DOP18_OFFSET
			<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

		_instance->sap = ALLOC_NEW(SweepAndPrune)(); // TODO: ENABLE!
		_instance->sap->init(_instance->gpu, buildOptions.str().c_str()); // TODO: ENABLE!

		_instance->sap->setParameters(_instance->boundingVolumeBuffer, objectsLength,
			DOP18_STRIDER, DOP18_OFFSET, DOP18_ORIENTATIONS);

		// Share OpenCL OpenGL Buffer
		//cl_mem glMem = clCreateFromGLBuffer(context, CL_MEM_READ_WRITE, glBufId, null);

		//int error = CL10GL.clEnqueueAcquireGLObjects(queue, glMem, null, null);
		//error = CL10GL.clEnqueueReleaseGLObjects(queue, glMem, null, null);

		// dispose: CL10.clReleaseMemObject(glMem);
	}

	void SpPhysicSimulator::timeOfCollision(SpCollisionDetails* details)
	{
		SpPhysicProperties* obj1Properties = &_physicProperties[details->objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[details->objIndex2];

		CollisionStatus status = CollisionStatus::INLINE;
		const sp_float _epsilon = 0.09f;
		const sp_float minElapsedTime = 0.01f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float diff = TEN_FLOAT;
		sp_float elapsedTime = previousElapsedTime * HALF_FLOAT;

		while (diff > _epsilon || (status == CollisionStatus::INSIDE && diff > minElapsedTime))
		{
			backToTime(details->objIndex1);
			backToTime(details->objIndex2);

			integrate(details->objIndex1, elapsedTime);
			integrate(details->objIndex2, elapsedTime);

			status = _boundingVolumes[details->objIndex1].collisionStatus(_boundingVolumes[details->objIndex2]);

			diff = std::fabsf(previousElapsedTime - elapsedTime);
			previousElapsedTime = elapsedTime;

			if (status == CollisionStatus::OUTSIDE)
				elapsedTime += (diff * HALF_FLOAT);
			else
				elapsedTime -= (diff * HALF_FLOAT);
		}

		details->timeOfCollision = elapsedTime < minElapsedTime ? ZERO_FLOAT : elapsedTime;
	}

	void SpPhysicSimulator::handleCollisionResponse(SpCollisionDetails* details)
	{
		SpPhysicProperties* obj1Properties = &_physicProperties[details->objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[details->objIndex2];

		const DOP18 bv1 = _boundingVolumes[details->objIndex1];
		const DOP18 bv2 = _boundingVolumes[details->objIndex2];

		const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());
		const sp_float cof = std::max(obj1Properties->coeficientOfFriction(), obj2Properties->coeficientOfFriction());

		if (obj1Properties->isStatic())
		{
			const Vec3 normal = bv1.normal(details->objectIndexPlane1);
			const Vec3 rayToContact = (details->contactPoint - bv2.centerOfBoundingVolume());
			const Vec3 pointVelocity = obj2Properties->velocity() + obj2Properties->torque().cross(rayToContact);
			const Vec3 velocity = pointVelocity.dot(normal);

			const Vec3 parenthesis = rayToContact.cross(obj2Properties->inertialTensorInverse() * (rayToContact.cross(normal)));
			const sp_float denominator = normal.dot(parenthesis) + obj2Properties->massInverse();

			const Vec3 impulse = (velocity * -(ONE_FLOAT + cor)) / denominator;
			
			obj2Properties->_velocity = impulse * rayToContact;
			obj2Properties->_torque = impulse * rayToContact.cross(normal);
			obj2Properties->_torque *= obj2Properties->angularDamping();

			obj2Properties->_acceleration = ZERO_FLOAT;
		}
		else
		{
			if (obj2Properties->isStatic())
			{
				const Vec3 normal = bv2.normal(details->objectIndexPlane2);
				const Vec3 rayToContact = (details->contactPoint - bv1.centerOfBoundingVolume());
				const Vec3 pointVelocity = obj1Properties->velocity() + obj1Properties->torque().cross(rayToContact);
				const Vec3 velocity = pointVelocity.dot(normal);

				const Vec3 parenthesis = obj1Properties->inertialTensorInverse() * rayToContact.cross(rayToContact.cross(normal));
				const sp_float denominator = normal.dot(parenthesis) + obj1Properties->massInverse();

				const Vec3 impulse = (velocity * -(ONE_FLOAT + cor)) / denominator;

				obj1Properties->_velocity = impulse * rayToContact;
				obj1Properties->_torque = impulse * rayToContact.cross(normal);
				obj1Properties->_torque *= obj1Properties->angularDamping();

				obj1Properties->_acceleration = ZERO_FLOAT;
			}
			else // both are dynamic
			{
				const Vec3 normalObj1 = bv1.normal(details->objectIndexPlane1);
				const Vec3 normalObj2 = bv2.normal(details->objectIndexPlane2);

				const Vec3 normal = normalObj1;
				const Vec3 rayToContactObj1 = (details->contactPoint - bv1.centerOfBoundingVolume());
				const Vec3 rayToContactObj2 = (details->contactPoint - bv2.centerOfBoundingVolume());
				const Vec3 pointVelocityObj1 = obj1Properties->velocity() + obj1Properties->torque().cross(rayToContactObj1);
				const Vec3 pointVelocityObj2 = obj2Properties->velocity() + obj2Properties->torque().cross(rayToContactObj2);
				const Vec3 relativeVelocity = normal.dot(pointVelocityObj1 - pointVelocityObj2);

				const Vec3 parenthesisObj1 = obj1Properties->inertialTensorInverse() 
					* rayToContactObj1.cross(rayToContactObj1.cross(normal));

				const Vec3 parenthesisObj2 = obj2Properties->inertialTensorInverse()
					* rayToContactObj2.cross(rayToContactObj2.cross(normal));

				const sp_float denominator = obj1Properties->massInverse() + obj2Properties->massInverse()
					+ normal.dot(parenthesisObj1 + parenthesisObj2);

				const Vec3 impulse = (relativeVelocity * -(ONE_FLOAT + cor)) / denominator;

				obj1Properties->_velocity = -impulse * normal;
				obj1Properties->_torque = -impulse * rayToContactObj1.cross(normal);
				obj1Properties->_torque *= obj1Properties->angularDamping();
				obj1Properties->_acceleration = ZERO_FLOAT;

				obj2Properties->_velocity = impulse * normal;
				obj2Properties->_torque = impulse * rayToContactObj2.cross(normal);
				obj2Properties->_torque *= obj2Properties->angularDamping();
				obj2Properties->_acceleration = ZERO_FLOAT;

				integrate(details->objIndex1, details->timeStep);
				integrate(details->objIndex2, details->timeStep);
			}
		}
	}

	void SpPhysicSimulator::handleCollision(void* threadParameter)
	{
		SpCollisionDetails* details = (SpCollisionDetails*)threadParameter;
		
		sp_assert(details->objIndex1 != details->objIndex2, "InvalidArgumentException");

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpPhysicProperties* obj1Properties = &simulator->_physicProperties[details->objIndex1];
		SpPhysicProperties* obj2Properties = &simulator->_physicProperties[details->objIndex2];

		const sp_bool isObj1Static = obj1Properties->isStatic();
		const sp_bool isObj2Static = obj2Properties->isStatic();

		if (isObj1Static && isObj2Static)
		{
			details->ignoreCollision = true;
			return;
		}

		const sp_bool isObj1Resting = obj1Properties->isResting();
		const sp_bool isObj2Resting = obj2Properties->isResting();

		if (isObj1Resting && isObj2Resting)
		{
			obj1Properties->_position = obj1Properties->_previousPosition;
			obj1Properties->_velocity = Vec3(ZERO_FLOAT);
			obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
			
			obj2Properties->_position = obj2Properties->_previousPosition;
			obj2Properties->_velocity = Vec3(ZERO_FLOAT);
			obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
			
			details->ignoreCollision = true;
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj2Properties->_velocity = Vec3(ZERO_FLOAT);
			
			details->ignoreCollision = true;
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj1Properties->_velocity = Vec3(ZERO_FLOAT);
			
			details->ignoreCollision = true;
			return;
		}

		if (simulator->areMovingAway(details->objIndex1, details->objIndex2)) // if the objects are getting distant
		{
			details->ignoreCollision = true;
			return;
		}

		simulator->collisionDetails(details);

		simulator->handleCollisionResponse(details);
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
		cl_event lastEvent = gpu->commandManager->updateBuffer(boundingVolumeBuffer, sizeof(DOP18) * _objectsLengthAllocated, _boundingVolumes);
		
		cl_mem sapBuffer = sap->execute(ONE_UINT, &lastEvent);
		
		lastEvent = sap->lastEvent;

		result->length = sap->fetchCollisionLength();

		gpu->commandManager->readBuffer(sapBuffer, multiplyBy2(result->length) * SIZEOF_UINT, result->indexes, ONE_UINT, &lastEvent);
	}

	void SpPhysicSimulator::run(const Timer& timer)
	{
		/*
		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy4(_objectsLength));

		findCollisionsCpu(&sapResult);

		const sp_float elapsedTime = timer.elapsedTime();

		for (sp_uint i = 0; i < multiplyBy2(sapResult.length); i += 2u)
			handleCollision({ sapResult.indexes[i], sapResult.indexes[i + 1], elapsedTime });
	
		ALLOC_RELEASE(sapResult.indexes);
		sapResult.indexes = nullptr;
		*/

		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy4(_objectsLength));

		//Timer tt; tt.start();

		findCollisionsGpu(&sapResult);
		
		//sp_float t = tt.elapsedTime(); // 3-4 ms
		//std::cout << t << END_OF_LINE;

		const sp_float elapsedTime = timer.elapsedTime();

		//Timer tt; tt.start();

		SpCollisionDetails* detailsArray = ALLOC_NEW_ARRAY(SpCollisionDetails, sapResult.length);
		SpThreadTask* tasks = ALLOC_NEW_ARRAY(SpThreadTask, sapResult.length);
		
		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			detailsArray[i].objIndex1 = sapResult.indexes[multiplyBy2(i)];
			detailsArray[i].objIndex2 = sapResult.indexes[multiplyBy2(i) + 1];
			detailsArray[i].timeStep = elapsedTime;

			tasks[i].func = &SpPhysicSimulator::handleCollision;
			tasks[i]. parameter = &detailsArray[i];

			SpThreadPool::instance()->schedule(&tasks[i]);
		}

		SpThreadPool::instance()->waitToFinish();
		
		for (sp_uint i = 0; i < sapResult.length; i++)
			if (!detailsArray[i].ignoreCollision)
				dispatchEvent(&detailsArray[i]);

		//std::cout << tt.elapsedTime() << END_OF_LINE;

		ALLOC_RELEASE(detailsArray);
		ALLOC_RELEASE(sapResult.indexes);
		sapResult.indexes = nullptr;
	}

	void SpPhysicSimulator::collisionDetailsPlanesRightDepthAndLeftFront(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{	
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];
		Plane3D planeRightDepth = bv1.planeRightDepth();

		Vec3 vertexes[4] = {
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(-1.0f, 0.0f, 0.0f));
		planeRightDepth.intersection(ray, vertexes);
		vertexes[1].x = vertexes[0].x;

		ray = Ray(vertexes[2], Vec3(0.0f, 0.0f, -1.0f));
		planeRightDepth.intersection(ray, &vertexes[2]);
		vertexes[3].z = vertexes[2].z;
		// plane right-depth built

		// left vertexes
		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(0.5f, 0.0f, -0.5f));
		bv2.planeRight().intersection(ray, &temp);
		vertexes[0].x = std::min(temp.x, vertexes[0].x);
		vertexes[0].z = std::max(temp.z, vertexes[0].z);
		vertexes[1].x = vertexes[0].x;
		vertexes[1].z = vertexes[0].z;

		// right vertexes
		ray = Ray(vertexes[2], Vec3(-0.5f, 0.0f, 0.5f));
		bv2.planeDepth().intersection(ray, &temp);
		vertexes[2].x = std::max(temp.x, vertexes[2].x);
		vertexes[2].z = std::min(temp.z, vertexes[2].z);
		vertexes[3].x = vertexes[2].x;
		vertexes[3].z = vertexes[2].z;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesRightFrontAndLeftDepth(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];
		Plane3D planeRightFront = bv1.planeRightFront();

		Vec3 vertexes[4] = {
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(-1.0f, 0.0f, 0.0f));
		planeRightFront.intersection(ray, vertexes);
		vertexes[3].x = vertexes[0].x;

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, -1.0f));
		planeRightFront.intersection(ray, &vertexes[1]);
		vertexes[2].z = vertexes[1].z;
		// plane right-front built

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(0.5f, 0.0f, -0.5f));
		bv2.planeLeft().intersection(ray, &temp);
		vertexes[0].x = std::max(temp.x, vertexes[0].x);
		vertexes[0].z = std::min(temp.z, vertexes[0].z);
		vertexes[3].x = vertexes[0].x;
		vertexes[3].z = vertexes[0].z;

		ray = Ray(vertexes[1], Vec3(-0.5f, 0.0f, 0.5f));
		bv2.planeDepth().intersection(ray, &temp);
		vertexes[1].x = std::min(temp.x, vertexes[1].x);
		vertexes[1].z = std::max(temp.z, vertexes[1].z);
		vertexes[2].x = vertexes[1].x;
		vertexes[2].z = vertexes[1].z;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesDownUp(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];

		Vec3 vertexes[4] = {
			Vec3(bv1.min[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv1.planeDownLeft().intersection(ray, vertexes);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv1.planeDownDepth().intersection(ray, &vertexes[1]);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv1.planeDownRight().intersection(ray, &vertexes[2]);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv1.planeDownFront().intersection(ray, &vertexes[3]);

		vertexes[0].z = vertexes[1].z;
		vertexes[1].x = vertexes[2].x;
		vertexes[2].z = vertexes[3].z;
		vertexes[3].x = vertexes[0].x;

		vertexes[0].y = bv2.max[DOP18_AXIS_Y];
		vertexes[1].y = bv2.max[DOP18_AXIS_Y];
		vertexes[2].y = bv2.max[DOP18_AXIS_Y];
		vertexes[3].y = bv2.max[DOP18_AXIS_Y];

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv2.planeUpLeft().intersection(ray, &temp);
		vertexes[0].x = std::max(temp.x, vertexes[0].x);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv2.planeUpDepth().intersection(ray, &temp);
		vertexes[1].z = std::max(temp.z, vertexes[1].z);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv2.planeUpRight().intersection(ray, &temp);
		vertexes[2].x = std::min(temp.x, vertexes[2].x);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv2.planeUpFront().intersection(ray, &temp);
		vertexes[3].z = std::min(temp.z, vertexes[3].z);

		vertexes[0].z = vertexes[1].z;
		vertexes[1].x = vertexes[2].x;
		vertexes[2].z = vertexes[3].z;
		vertexes[3].x = vertexes[0].x;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesRightLeft(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];

		Vec3 vertexes[4] = { // pseudo-vertexes of plane right
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(-1.0f, 0.0f, 0.0f));
		bv1.planeRightFront().intersection(ray, vertexes);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv1.planeDownRight().intersection(ray, &vertexes[1]);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv1.planeRightDepth().intersection(ray, &vertexes[2]);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv1.planeUpRight().intersection(ray, &vertexes[3]);

		vertexes[0].y = vertexes[1].y;
		vertexes[1].z = vertexes[2].z;
		vertexes[2].y = vertexes[3].y;
		vertexes[3].z = vertexes[0].z;

		vertexes[0].x = bv1.max[DOP18_AXIS_X];
		vertexes[1].x = bv1.max[DOP18_AXIS_X];
		vertexes[2].x = bv1.max[DOP18_AXIS_X];
		vertexes[3].x = bv1.max[DOP18_AXIS_X]; // plane right built ...

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv2.planeLeftFront().intersection(ray, &temp);
		vertexes[0].z = std::max(temp.z, vertexes[0].z);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv2.planeDownLeft().intersection(ray, &temp);
		vertexes[1].y = std::max(temp.y, vertexes[1].y);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv2.planeLeftDepth().intersection(ray, &temp);
		vertexes[2].z = std::min(temp.z, vertexes[2].z);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv2.planeUpLeft().intersection(ray, &temp);
		vertexes[3].y = std::min(temp.y, vertexes[3].y);

		vertexes[0].y = vertexes[1].y;
		vertexes[1].z = vertexes[2].z;
		vertexes[2].y = vertexes[3].y;
		vertexes[3].z = vertexes[0].z;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesDepthFront(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];

		Vec3 vertexes[4] = { // pseudo-vertexes of plane front
			Vec3(bv2.min[DOP18_AXIS_X], bv2.min[DOP18_AXIS_Y], bv2.max[DOP18_AXIS_Z]),
			Vec3(bv2.max[DOP18_AXIS_X], bv2.min[DOP18_AXIS_Y], bv2.max[DOP18_AXIS_Z]),
			Vec3(bv2.max[DOP18_AXIS_X], bv2.max[DOP18_AXIS_Y], bv2.max[DOP18_AXIS_Z]),
			Vec3(bv2.min[DOP18_AXIS_X], bv2.max[DOP18_AXIS_Y], bv2.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv2.planeLeftFront().intersection(ray, vertexes);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, -1.0f));
		bv2.planeDownFront().intersection(ray, &vertexes[1]);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv2.planeRightFront().intersection(ray, &vertexes[2]);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv2.planeUpFront().intersection(ray, &vertexes[3]);

		vertexes[0].y = vertexes[1].y;
		vertexes[1].x = vertexes[2].x;
		vertexes[2].y = vertexes[3].y;
		vertexes[3].x = vertexes[0].x;

		vertexes[0].z = bv2.max[DOP18_AXIS_Z];
		vertexes[1].z = bv2.max[DOP18_AXIS_Z];
		vertexes[2].z = bv2.max[DOP18_AXIS_Z];
		vertexes[3].z = bv2.max[DOP18_AXIS_Z]; // plane front built !

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv1.planeLeftDepth().intersection(ray, &temp);
		vertexes[0].x = std::max(temp.x, vertexes[0].x);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv1.planeDownDepth().intersection(ray, &temp);
		vertexes[1].y = std::max(temp.y, vertexes[1].y);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv1.planeRightDepth().intersection(ray, &temp);
		vertexes[2].x = std::min(temp.x, vertexes[2].x);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, 1.0f));
		bv1.planeUpDepth().intersection(ray, &temp);
		vertexes[3].y = std::min(temp.y, vertexes[3].y);

		vertexes[0].y = vertexes[1].y;
		vertexes[1].x = vertexes[2].x;
		vertexes[2].y = vertexes[3].y;
		vertexes[3].x = vertexes[0].x;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesUpLeftAndDownRight(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];
		Plane3D planeUpLeft = bv1.planeUpLeft();

		Vec3 vertexes[4] = {
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(0.0f, -1.0f, 0.0f));
		planeUpLeft.intersection(ray, vertexes);
		vertexes[1].y = vertexes[0].y;

		ray = Ray(vertexes[2], Vec3(1.0f, 0.0f, 0.0f));
		planeUpLeft.intersection(ray, &vertexes[2]);
		vertexes[3].x = vertexes[2].x;
		// plane up-left built

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(-0.5f, -0.5f, 0.0f));
		bv2.planeDown().intersection(ray, &temp);
		vertexes[0].x = std::max(temp.x, vertexes[0].x);
		vertexes[0].y = std::max(temp.y, vertexes[0].y);
		vertexes[1].x = vertexes[0].x;
		vertexes[1].y = vertexes[0].y;

		ray = Ray(vertexes[2], Vec3(0.5f, 0.5f, 0.0f));
		bv2.planeRight().intersection(ray, &temp);
		vertexes[2].x = std::min(temp.x, vertexes[2].x);
		vertexes[2].y = std::min(temp.y, vertexes[2].y);
		vertexes[3].x = vertexes[2].x;
		vertexes[3].y = vertexes[2].y;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesUpRightAndDownLeft(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];
		Plane3D planeUpRight = bv1.planeUpRight();

		Vec3 vertexes[4] = {
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(0.0f, -1.0f, 0.0f));
		planeUpRight.intersection(ray, vertexes);
		vertexes[1].y = vertexes[0].y;

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		planeUpRight.intersection(ray, &vertexes[2]);
		vertexes[3].x = vertexes[2].x;
		// plane up-right built

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(0.5f, -0.5f, 0.0f));
		bv2.planeDown().intersection(ray, &temp);
		vertexes[0].x = std::min(temp.x, vertexes[0].x);
		vertexes[0].y = std::max(temp.y, vertexes[0].y);
		vertexes[1].x = vertexes[0].x;
		vertexes[1].y = vertexes[0].y;

		ray = Ray(vertexes[2], Vec3(-0.5f, 0.5f, 0.0f));
		bv2.planeLeft().intersection(ray, &temp);
		vertexes[2].x = std::max(temp.x, vertexes[2].x);
		vertexes[2].y = std::min(temp.y, vertexes[2].y);
		vertexes[3].x = vertexes[2].x;
		vertexes[3].y = vertexes[2].y;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesUpFrontAndDownDepth(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];
		Plane3D planeUpFront = bv1.planeUpFront();

		Vec3 vertexes[4] = {
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(0.0f, -1.0f, 0.0f));
		planeUpFront.intersection(ray, vertexes);
		vertexes[1].y = vertexes[0].y;

		ray = Ray(vertexes[2], Vec3(0.0f, 0.0f, -1.0f));
		planeUpFront.intersection(ray, &vertexes[2]);
		vertexes[3].z = vertexes[2].z;
		// plane up-front built

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(0.0f, -0.5f, 0.5f));
		bv2.planeDown().intersection(ray, &temp);
		vertexes[0].y = std::max(temp.y, vertexes[0].y);
		vertexes[0].z = std::min(temp.z, vertexes[0].z);
		vertexes[1].y = vertexes[0].y;
		vertexes[1].z = vertexes[0].z;

		ray = Ray(vertexes[2], Vec3(0.0f, 0.5f, -0.5f));
		bv2.planeDepth().intersection(ray, &temp);
		vertexes[2].y = std::min(temp.y, vertexes[2].y);
		vertexes[2].z = std::max(temp.z, vertexes[2].z);
		vertexes[3].y = vertexes[2].y;
		vertexes[3].z = vertexes[2].z;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesUpDepthAndDownFront(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[details->objIndex1];
		DOP18 bv2 = _boundingVolumes[details->objIndex2];
		Plane3D planeUpDepth = bv1.planeUpDepth();

		Vec3 vertexes[4] = {
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(0.0f, -1.0f, 0.0f));
		planeUpDepth.intersection(ray, vertexes);
		vertexes[1].y = vertexes[0].y;

		ray = Ray(vertexes[2], Vec3(0.0f, 0.0f, 1.0f));
		planeUpDepth.intersection(ray, &vertexes[2]);
		vertexes[3].z = vertexes[2].z;
		// plane up-depth built

		Vec3 temp;
		ray = Ray(vertexes[0], Vec3(0.0f, -0.5f, -0.5f));
		bv2.planeDown().intersection(ray, &temp);
		vertexes[0].y = std::max(temp.y, vertexes[0].y);
		vertexes[0].z = std::max(temp.z, vertexes[0].z);
		vertexes[1].y = vertexes[0].y;
		vertexes[1].z = vertexes[0].z;

		ray = Ray(vertexes[2], Vec3(0.0f, 0.5f, 0.5f));
		bv2.planeFront().intersection(ray, &temp);
		vertexes[2].y = std::min(temp.y, vertexes[2].y);
		vertexes[2].z = std::min(temp.z, vertexes[2].z);
		vertexes[3].y = vertexes[2].y;
		vertexes[3].z = vertexes[2].z;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}
	
	void SpPhysicSimulator::collisionDetails(SpCollisionDetails* details)
	{
		sp_assert(details->objIndex1 >= ZERO_UINT && details->objIndex1 < _objectsLength, "IndexOutOfRangeException");
		sp_assert(details->objIndex2 >= ZERO_UINT && details->objIndex2 < _objectsLength, "IndexOutOfRangeException");

		timeOfCollision(details);

		const SpPhysicProperties* obj1Properties = &_physicProperties[details->objIndex1];
		const SpPhysicProperties* obj2Properties = &_physicProperties[details->objIndex2];

		const DOP18 bv1 = _boundingVolumes[details->objIndex1];
		const DOP18 bv2 = _boundingVolumes[details->objIndex2];

		const Vec3 centerObj1 = obj1Properties->position();
		const Vec3 centerObj2 = obj2Properties->position();

		const sp_bool isUp = Plane3D(centerObj1, Vec3(0.0f, 1.0f, 0.0f)).orientation(centerObj2) == Orientation::LEFT;
		const sp_bool isRight = Plane3D(centerObj1, Vec3(1.0f, 0.0f, 0.0f)).orientation(centerObj2) == Orientation::LEFT;
		const sp_bool isFront = Plane3D(centerObj1, Vec3(0.0f, 0.0f, 1.0f)).orientation(centerObj2) == Orientation::LEFT;

   		if (isUp) // it the collision happend up ...
		{
			if (isRight) // it the collision happend right ...
			{
				if (isFront) // it the collision happend depth ...
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = std::fabsf(bv1.max[DOP18_AXIS_X] - bv2.min[DOP18_AXIS_X]); // = bv1.planeRight().distance(bv2.planeLeft());

					collisionDetailsPlanesRightLeft(details->objIndex1, details->objIndex2, details);

					sp_float newDistace = std::fabsf(bv1.max[DOP18_AXIS_Z] - bv2.min[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_Y] - bv2.min[DOP18_AXIS_Y]); // = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightFrontAndLeftDepth(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = std::fabsf(bv1.max[DOP18_AXIS_X] - bv2.min[DOP18_AXIS_X]); // = bv1.planeRight().distance(bv2.planeLeft());

					collisionDetailsPlanesRightLeft(details->objIndex1, details->objIndex2, details);

					sp_float newDistace = std::fabsf(bv1.min[DOP18_AXIS_Z] - bv2.max[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_Y] - bv2.min[DOP18_AXIS_Y]); // = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_RIGHT] - bv2.min[DOP18_AXIS_UP_RIGHT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpRightAndDownLeft(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
			}
			else
			{
				if (isFront) // it the collision happend depth ...
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
					sp_float smallestDistace = std::fabsf(bv1.min[DOP18_AXIS_X] - bv2.max[DOP18_AXIS_X]); // = bv1.planeLeft().distance(bv2.planeRight());

					collisionDetailsPlanesRightLeft(details->objIndex2, details->objIndex1, details);

					sp_float newDistace = std::fabsf(bv1.max[DOP18_AXIS_Z] - bv2.min[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_Y] - bv2.min[DOP18_AXIS_Y]); // = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
					sp_float smallestDistace = std::fabsf(bv1.min[DOP18_AXIS_X] - bv2.max[DOP18_AXIS_X]); // = bv1.planeLeft().distance(bv2.planeRight());

					collisionDetailsPlanesRightLeft(details->objIndex2, details->objIndex1, details);

					sp_float newDistace = std::fabsf(bv1.min[DOP18_AXIS_Z] - bv2.max[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_Y] - bv2.min[DOP18_AXIS_Y]); // = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_DEPTH] - bv2.min[DOP18_AXIS_UP_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpDepthAndDownFront(details->objIndex1, details->objIndex2, details);
					}
				}
			}
		}
		else
		{
			if (isRight) // it the collision happend right ...
			{
				if (isFront) // it the collision happend depth ...
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = std::fabsf(bv1.max[DOP18_AXIS_X] - bv2.min[DOP18_AXIS_X]); // = bv1.planeRight().distance(bv2.planeLeft());

					collisionDetailsPlanesRightLeft(details->objIndex1, details->objIndex2, details);

					sp_float newDistace = std::fabsf(bv1.max[DOP18_AXIS_Z] - bv2.min[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex2, details->objIndex1, details);
					}

					 newDistace = std::fabsf(bv1.min[DOP18_AXIS_Y] - bv2.max[DOP18_AXIS_Y]); // = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightFrontAndLeftDepth(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = std::fabsf(bv2.min[DOP18_AXIS_X] - bv1.max[DOP18_AXIS_X]); // = bv2.planeLeft().distance(bv1.planeRight());

					collisionDetailsPlanesRightLeft(details->objIndex1, details->objIndex2, details);

					sp_float newDistace = std::fabsf(bv1.min[DOP18_AXIS_Z] - bv2.max[DOP18_AXIS_Z]); // = bv1.planeFront().distance(bv2.planeDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;

 						collisionDetailsPlanesDepthFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.min[DOP18_AXIS_Y] - bv2.max[DOP18_AXIS_Y]); // = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightFrontAndLeftDepth(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT]- bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
			}
			else
			{
				if (isFront)
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = std::fabsf(bv1.max[DOP18_AXIS_X] - bv2.min[DOP18_AXIS_X]); // = bv1.planeLeft().distance(bv2.planeRight());

					collisionDetailsPlanesRightLeft(details->objIndex1, details->objIndex2, details);

					sp_float newDistace = std::fabsf(bv1.min[DOP18_AXIS_Z] - bv2.max[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.min[DOP18_AXIS_Y] - bv2.max[DOP18_AXIS_Y]); // = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						smallestDistace = newDistace;

						sp_assert(false, "TODO");
						collisionDetailsPlanesRightFrontAndLeftDepth(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						sp_assert(false, "TODO");
						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
 					sp_float smallestDistace = std::fabsf(bv2.min[DOP18_AXIS_X] - bv1.max[DOP18_AXIS_X]);  // = bv1.planeLeft().distance(bv2.planeRight());

					collisionDetailsPlanesRightLeft(details->objIndex2, details->objIndex1, details);

					sp_float newDistace = std::fabsf(bv1.min[DOP18_AXIS_Z] - bv2.max[DOP18_AXIS_Z]); // = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDepthFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.min[DOP18_AXIS_Y] - bv2.max[DOP18_AXIS_Y]);  // = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_LEFT_DEPTH] - bv2.min[DOP18_AXIS_LEFT_DEPTH]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_DEPTH_INDEX;
						smallestDistace = newDistace;

						sp_assert(false, "TODO");
						collisionDetailsPlanesRightFrontAndLeftDepth(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_RIGHT_DEPTH] - bv2.min[DOP18_AXIS_RIGHT_DEPTH]); // = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;

						sp_assert(false, "TODO");
						collisionDetailsPlanesRightDepthAndLeftFront(details->objIndex1, details->objIndex2, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_LEFT] - bv2.min[DOP18_AXIS_UP_LEFT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;

						sp_assert(false, "TODO");
						collisionDetailsPlanesUpLeftAndDownRight(details->objIndex2, details->objIndex1, details);
					}

					newDistace = std::fabsf(bv1.max[DOP18_AXIS_UP_FRONT] - bv2.min[DOP18_AXIS_UP_FRONT]);
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesUpFrontAndDownDepth(details->objIndex1, details->objIndex2, details);
					}
				}
			}
		}

		sp_assert(details->objectIndexPlane1 >= 0 && details->objectIndexPlane1 < 18, "Invalid Plane Index");
		sp_assert(details->objectIndexPlane2 >= 0 && details->objectIndexPlane2 < 18, "Invalid Plane Index");
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