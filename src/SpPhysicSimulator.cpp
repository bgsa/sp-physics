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

	sp_float SpPhysicSimulator::timeOfCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime)
	{
		SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

		sp_float previousElapsedTime = elapsedTime;
		sp_float factor = 0.25f;
		elapsedTime *= 0.5f;
		CollisionStatus status;
		do
		{
			backToTime(objIndex1);
			backToTime(objIndex2);

			integrate(objIndex1, elapsedTime);
			integrate(objIndex2, elapsedTime);

			status = _boundingVolumes[objIndex1].collisionStatus(_boundingVolumes[objIndex2]);

			previousElapsedTime = elapsedTime;
			if (status == CollisionStatus::OUTSIDE)
				elapsedTime += elapsedTime * factor;
			else
				elapsedTime -= elapsedTime * factor;
			factor *= 0.65f;
		} while (std::abs(elapsedTime - previousElapsedTime) > 0.9f);

		return elapsedTime;
	}

	void SpPhysicSimulator::handleCollisionResponse(sp_uint objIndex1, sp_uint objIndex2, const SpCollisionDetails& details)
	{
		SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

		const sp_float sumMass = obj1Properties->massInverse() + obj2Properties->massInverse();
		const Vec3 relativeVelocity = obj2Properties->velocity() - obj1Properties->velocity();
		const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());

		if (obj1Properties->isDynamic())
		{
			const Line3D lineOfAction(obj1Properties->position(), details.contactPoint);

			if (obj2Properties->isDynamic())
			{
				sp_float factor1
					= (obj1Properties->massInverse() - cor * obj2Properties->massInverse())
					/ sumMass;

				sp_float factor2
					= ((1.0f + cor) * obj2Properties->massInverse())
					/ sumMass;

				const Vec3 newForceObj1
					= (obj1Properties->velocity() * factor1)
					+ (obj2Properties->velocity() * factor2);

				obj1Properties->_velocity = lineOfAction.direction() * newForceObj1;
			}
			else
				obj1Properties->_velocity = (lineOfAction.direction()  * obj1Properties->velocity()) * cor;

			obj1Properties->_acceleration = ZERO_FLOAT;
		}

		if (obj2Properties->isDynamic())
		{
			//const Line3D lineOfAction(obj2Properties->position(), details.contactPoint);
			const Line3D lineOfAction(details.contactPoint, obj2Properties->position());

			if (obj1Properties->isDynamic())
			{
				sp_float factor1 =
					((1.0f + cor) * obj1Properties->massInverse())
					/ sumMass;

				sp_float factor2 =
					(obj2Properties->massInverse() - cor * obj1Properties->massInverse())
					/ sumMass;

				const Vec3 newForceObj2
					= (relativeVelocity * factor1)
					+ (relativeVelocity * factor2);

				obj2Properties->_velocity = lineOfAction.direction() * newForceObj2;
			}
			else
				obj2Properties->_velocity = (lineOfAction.direction() * relativeVelocity) * cor;

			obj2Properties->_acceleration = ZERO_FLOAT;
		}
	}

	void SpPhysicSimulator::handleCollision(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime)
	{
		sp_assert(objIndex1 != objIndex2, "InvalidArgumentException");

		SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

		const sp_bool isObj1Static = obj1Properties->isStatic();
		const sp_bool isObj2Static = obj2Properties->isStatic();

		if (isObj1Static && isObj2Static)
			return;

		const sp_bool isObj1Resting = obj1Properties->isResting();
		const sp_bool isObj2Resting = obj2Properties->isResting();

		if (isObj1Resting && isObj2Resting)
		{
			obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj1Properties->_velocity = Vec3(ZERO_FLOAT);
			obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj2Properties->_velocity = Vec3(ZERO_FLOAT);
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj2Properties->_velocity = Vec3(ZERO_FLOAT);
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj1Properties->_velocity = Vec3(ZERO_FLOAT);
			return;
		}

		SpCollisionDetails details;
		collisionDetails(objIndex1, objIndex2, elapsedTime, &details);

		handleCollisionResponse(objIndex1, objIndex2, details);

		dispatchEvent(objIndex1, objIndex2);
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

		sp_uint* sortedIndexes = ALLOC_ARRAY(sp_uint, objectsLength);
		for (sp_uint i = ZERO_UINT; i < objectsLength; i++)
			sortedIndexes[i] = i;

		sap->findCollisions(_boundingVolumes, sortedIndexes, objectsLength, &resultCpu);

		ALLOC_RELEASE(sortedIndexes);

		sp_uint indexesLength = multiplyBy2(resultCpu.length);
		sp_uint* indexes = resultCpu.indexes;
		
		/*
		cl_event lastEvent = gpu->commandManager->updateBuffer(boundingVolumeBuffer, DOP18_SIZE * objectsLengthAllocated, _boundingVolumes);

		cl_mem result = sap->execute(ONE_UINT, &lastEvent);

		const sp_uint indexesLength = multiplyBy2(sap->fetchCollisionLength());

		sp_uint* indexes = ALLOC_NEW_ARRAY(sp_uint, indexesLength);
		gpu->commandManager->readBuffer(result, indexesLength * SIZEOF_UINT, indexes);
		//ALLOC_RELEASE(indexes);
		*/

		const sp_float elapsedTime = timer.elapsedTime();

		for (sp_uint i = 0; i < indexesLength; i+=2u)
			handleCollision(indexes[i], indexes[i+1], elapsedTime);

		ALLOC_RELEASE(resultCpu.indexes);
	}

	void SpPhysicSimulator::collisionDetailsPlanesDownUp(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[objIndex1];
		DOP18 bv2 = _boundingVolumes[objIndex2];

		Vec3 vertexes[4] = {
			Vec3(bv1.min[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.min[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv1.planeDownLeft().intersection(ray, vertexes);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv1.planeDownFront().intersection(ray, &vertexes[1]);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv1.planeDownRight().intersection(ray, &vertexes[2]);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv1.planeDownDepth().intersection(ray, &vertexes[3]);

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
		bv2.planeUpFront().intersection(ray, &temp);
		vertexes[1].z = std::max(temp.z, vertexes[1].z);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv2.planeUpRight().intersection(ray, &temp);
		vertexes[2].x = std::min(temp.x, vertexes[2].x);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv2.planeUpDepth().intersection(ray, &temp);
		vertexes[3].z = std::min(temp.z, vertexes[3].z);

		vertexes[0].z = vertexes[1].z;
		vertexes[1].x = vertexes[2].x;
		vertexes[2].z = vertexes[3].z;
		vertexes[3].x = vertexes[0].x;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
	}

	void SpPhysicSimulator::collisionDetailsPlanesLeftRight(const sp_uint objIndex1, const sp_uint objIndex2, SpCollisionDetails* details)
	{
		DOP18 bv1 = _boundingVolumes[objIndex1];
		DOP18 bv2 = _boundingVolumes[objIndex2];

		Vec3 vertexes[4] = { // pseudo-vertexes of plane right
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.min[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.max[DOP18_AXIS_Z]),
			Vec3(bv1.max[DOP18_AXIS_X], bv1.max[DOP18_AXIS_Y], bv1.min[DOP18_AXIS_Z])
		};

		Ray ray(vertexes[0], Vec3(1.0f, 0.0f, 0.0f));
		bv1.planeRightFront().intersection(ray, vertexes);

		ray = Ray(vertexes[1], Vec3(0.0f, 0.0f, 1.0f));
		bv1.planeDownRight().intersection(ray, &vertexes[1]);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv1.planeRightDepth().intersection(ray, &vertexes[2]);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv1.planeUpRight().intersection(ray, &vertexes[3]);

		/*
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
		bv2.planeUpFront().intersection(ray, &temp);
		vertexes[1].z = std::max(temp.z, vertexes[1].z);

		ray = Ray(vertexes[2], Vec3(-1.0f, 0.0f, 0.0f));
		bv2.planeUpRight().intersection(ray, &temp);
		vertexes[2].x = std::min(temp.x, vertexes[2].x);

		ray = Ray(vertexes[3], Vec3(0.0f, 0.0f, -1.0f));
		bv2.planeUpDepth().intersection(ray, &temp);
		vertexes[3].z = std::min(temp.z, vertexes[3].z);

		vertexes[0].z = vertexes[1].z;
		vertexes[1].x = vertexes[2].x;
		vertexes[2].z = vertexes[3].z;
		vertexes[3].x = vertexes[0].x;

		details->contactPoint = (vertexes[0] + vertexes[1] + vertexes[2] + vertexes[3]) * 0.25f;
		*/
	}

	void SpPhysicSimulator::collisionDetails(const sp_uint objIndex1, const sp_uint objIndex2, sp_float elapsedTime, SpCollisionDetails* details)
	{
		details->timeOfCollision = timeOfCollision(objIndex1, objIndex2, elapsedTime);

		SpPhysicProperties* obj1Properties = &_physicProperties[objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[objIndex2];

		DOP18 bv1 = _boundingVolumes[objIndex1];
		DOP18 bv2 = _boundingVolumes[objIndex2];

		Vec3 centerObj1 = obj1Properties->position();
		Vec3 centerObj2 = obj2Properties->position();

		Line3D lineOfAction(centerObj2, centerObj1);
		Vec3 direction = lineOfAction.direction().normalize();

		sp_float angleRight = direction.dot({ 1.0f, 0.0f, 0.0f });
		sp_float angleUp = direction.dot({ 0.0f, 1.0f, 0.0f });
		sp_float angleDepth = direction.dot({ 0.0f, 0.0f, 1.0f });

		Vec3 contactPoint;

		if (angleUp >= HALF_PI && angleUp <= HALF_PI) // it the collision happend up ...
		{
			if (angleRight >= HALF_PI && angleRight <= HALF_PI) // it the collision happend right ...
			{
				if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

					sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUpRight().distance(bv2.planeDownLeft());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_LEFT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUpDepth().distance(bv2.planeDownFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_FRONT_INDEX;
						smallestDistace = newDistace;
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

					sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUpRight().distance(bv2.planeDownLeft());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_LEFT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeRightFront().distance(bv2.planeLeftDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUpFront().distance(bv2.planeDownDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;
					}
				}
			}
			else
			{
				if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
					sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

					sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUpLeft().distance(bv2.planeDownRight());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeLeftDepth().distance(bv2.planeRightFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeLeftDepth().distance(bv2.planeRightFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_FRONT_INDEX;
						smallestDistace = newDistace;
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

					sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUp().distance(bv2.planeDown());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(objIndex2, objIndex1, details);
					}

					newDistace = bv1.planeUpLeft().distance(bv2.planeDownRight());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeLeftFront().distance(bv2.planeRightDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeUpFront().distance(bv2.planeDownDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_UP_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						smallestDistace = newDistace;
					}
				}
			}
		}
		else
		{
			if (angleRight >= HALF_PI && angleRight <= HALF_PI) // it the collision happend right ...
			{
				if (angleDepth >= HALF_PI && angleDepth <= HALF_PI) // it the collision happend depth ...
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

					sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDownRight().distance(bv2.planeUpLeft());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeRightDepth().distance(bv2.planeLeftFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDownDepth().distance(bv2.planeUpFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_FRONT_INDEX;
						smallestDistace = newDistace;
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_RIGHT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_LEFT_INDEX;
					sp_float smallestDistace = bv1.planeRight().distance(bv2.planeLeft());

					sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDownRight().distance(bv2.planeUpLeft());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_RIGHT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_LEFT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeRightFront().distance(bv2.planeLeftDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeRightFront().distance(bv2.planeLeftDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_DEPTH_INDEX;
						smallestDistace = newDistace;
					}
				}
			}
			else
			{
				if (angleDepth >= HALF_PI && angleDepth <= HALF_PI)
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
					sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

					sp_float newDistace = bv1.planeDepth().distance(bv2.planeFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDownLeft().distance(bv2.planeUpRight());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_RIGHT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeLeftDepth().distance(bv2.planeRightFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_FRONT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDownDepth().distance(bv2.planeUpFront());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_DEPTH_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_FRONT_INDEX;
						smallestDistace = newDistace;
					}
				}
				else
				{
					details->objectIndexPlane1 = DOP18_PLANES_LEFT_INDEX;
					details->objectIndexPlane2 = DOP18_PLANES_RIGHT_INDEX;
					sp_float smallestDistace = bv1.planeLeft().distance(bv2.planeRight());

					sp_float newDistace = bv1.planeFront().distance(bv2.planeDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDown().distance(bv2.planeUp());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_INDEX;
						smallestDistace = newDistace;

						collisionDetailsPlanesDownUp(objIndex1, objIndex2, details);
					}

					newDistace = bv1.planeDownLeft().distance(bv2.planeUpRight());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_LEFT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_RIGHT_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeLeftFront().distance(bv2.planeRightDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_LEFT_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_RIGHT_DEPTH_INDEX;
						smallestDistace = newDistace;
					}

					newDistace = bv1.planeDownFront().distance(bv2.planeUpDepth());
					if (newDistace < smallestDistace)
					{
						details->objectIndexPlane1 = DOP18_PLANES_DOWN_FRONT_INDEX;
						details->objectIndexPlane2 = DOP18_PLANES_UP_DEPTH_INDEX;
						smallestDistace = newDistace;
					}
				}
			}
		}
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