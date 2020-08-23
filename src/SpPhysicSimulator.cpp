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
		timerToPhysic.start();
		lastEvent = nullptr;
		_boundingVolumesGPU = nullptr;
		_physicPropertiesGPU = nullptr;

		_objectsLength = ZERO_UINT;
		_objectsLengthAllocated = objectsLength;
		_collisionFeatureLength = ZERO_UINT;
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

		const sp_uint outputIndexSize = multiplyBy4(objectsLength) * SIZEOF_UINT;

		_transformsGPU = gpu->createBufferFromOpenGL(_transformsGPUBuffer);
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

	void SpPhysicSimulator::handleCollisionResponseOLD(SpCollisionDetails* details)
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
			const Vec3 rayToContact = (details->contactPoints[0] - bv2.centerOfBoundingVolume());
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
				const Vec3 rayToContact = (details->contactPoints[0] - bv1.centerOfBoundingVolume());
				const Vec3 pointVelocity = obj1Properties->velocity() + obj1Properties->torque().cross(rayToContact);
				const Vec3 velocity = pointVelocity.dot(normal);

				const Vec3 raCrossN = rayToContact.cross(normal);

				const Vec3 parenthesis = obj1Properties->inertialTensorInverse() * raCrossN.cross(rayToContact);
				const sp_float denominator = obj1Properties->massInverse() + normal.dot(parenthesis);

				const Vec3 impulse = (velocity * -(ONE_FLOAT + cor)) / denominator;

				obj1Properties->_velocity = -impulse * rayToContact;
				obj1Properties->_torque = impulse * rayToContact.cross(normal);
				obj1Properties->_torque *= obj1Properties->angularDamping();

				obj1Properties->_acceleration = ZERO_FLOAT;
			}
			else // both are dynamic
			{
				const Vec3 normalObj1 = bv1.normal(details->objectIndexPlane1);
				const Vec3 normalObj2 = bv2.normal(details->objectIndexPlane2);

				const Vec3 normal = normalObj1;
				const Vec3 rayToContactObj1 = (details->contactPoints[0] - bv1.centerOfBoundingVolume());
				const Vec3 rayToContactObj2 = (details->contactPoints[0] - bv2.centerOfBoundingVolume());
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

	void SpPhysicSimulator::handleCollisionResponseWithStatic(SpCollisionDetails* details, SpPhysicProperties* objProperties, const Vec3& center, const sp_float cor)
	{
		Vec3 rayToContact;
		
		switch (details->type)
		{
		case SpCollisionType::PointFace:
			rayToContact = details->contactPoints[0] - center;
			break;

		case SpCollisionType::EdgeFace:
			rayToContact
				= (details->contactPoints[0] + details->contactPoints[1]) * HALF_FLOAT;
			break;

		case SpCollisionType::FaceFace:
			for (sp_uint i = 0u; i < details->contactPointsLength; i++)
				rayToContact += details->contactPoints[i];
			rayToContact /= details->contactPointsLength;
			break;

		default:
			sp_assert(false, "NotImplementedException");
		}
		
		sp_float numerator = (-(1.0f + cor) * (-objProperties->velocity()).dot(details->collisionNormal));

		Vec3 d2 = (objProperties->inertialTensorInverse() * rayToContact.cross(details->collisionNormal))
					.cross(rayToContact);

		sp_float denominator = objProperties->massInverse() + details->collisionNormal.dot(d2);

		const sp_float j = denominator == ZERO_FLOAT ? ZERO_FLOAT
			: numerator / denominator;

		const Vec3 impulse = details->collisionNormal * j;

		objProperties->_velocity = objProperties->velocity() - impulse * objProperties->massInverse();
		objProperties->_angularVelocity = objProperties->angularVelocity() - objProperties->inertialTensorInverse() * rayToContact.cross(impulse);
		
		/*
			// calculate friction ...
			Vec3 tangent = relativeVel - (collisionNormal * relativeVel.dot(collisionNormal));

			if (isCloseEnough((tangent.x + tangent.y + tangent.z), ZERO_FLOAT))
				return;

			tangent = tangent.normalize();
			numerator = -relativeVel.dot(tangent);
			d2 = (obj1Properties->inertialTensorInverse() * tangent.cross(rayToContact)).cross(rayToContact);
			denominator = obj1Properties->massInverse() + tangent.dot(d2);

			if (denominator == 0.0f)
				return;

			sp_float jt = numerator / denominator;

			if (isCloseEnough(jt, 0.0f))
				return;

			sp_float friction = sqrtf(obj1Properties->coeficientOfFriction() * obj2Properties->coeficientOfFriction());
			if (jt > j * friction) {
				jt = j * friction;
			}
			else if (jt < -j * friction) {
				jt = -j * friction;
			}

			Vec3 tangentImpuse = tangent * jt;

			obj1Properties->_velocity = obj1Properties->velocity() - tangentImpuse * obj1Properties->massInverse();
			obj1Properties->_angularVelocity = obj1Properties->angularVelocity() - obj1Properties->inertialTensorInverse() * rayToContact.cross(tangentImpuse);
			*/

		objProperties->_acceleration = ZERO_FLOAT;
		objProperties->_force = ZERO_FLOAT;
		objProperties->_torque = ZERO_FLOAT;
		//integrate(details->objIndex2, details->timeStep);
	}

	void SpPhysicSimulator::collisionDetailsObj1ToObj2(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details) 
	{
		SpMesh* mesh1 = mesh(collisionFeatures(obj1Index)->meshIndex);
		SpTransform* transformObj1 = transforms(obj1Index);
		Vec3* vertexesObj1 = mesh1->vertexes->data();
		SpPoint3<sp_uint>* facesObj1 = mesh1->facesIndexes->data();
		SpPoint2<sp_uint>* edgesObj1 = mesh1->edgesIndexes->data();
		const sp_uint facesIndexLengthObj1 = mesh1->facesIndexes->length();
		const sp_uint edgesLengthObj1 = mesh1->edgesIndexes->length();
		SpVertexEdges* vertexEdgesObj1 = mesh1->vertexEdges()[0];

		SpMesh* mesh2 = mesh(collisionFeatures(obj2Index)->meshIndex);
		SpTransform* transformObj2 = transforms(obj2Index);
		Vec3* vertexesObj2 = mesh2->vertexes->data();
		SpPoint3<sp_uint>* facesObj2 = mesh2->facesIndexes->data();
		SpPoint2<sp_uint>* _edgesObj2 = mesh2->edgesIndexes->data();
		const sp_uint facesIndexLengthObj2 = mesh2->facesIndexes->length();
		const sp_uint edgesLengthObj2 = mesh2->edgesIndexes->length();
		SpVertexEdges* vertexEdgesObj2 = mesh2->vertexEdges()[0];

		// for each edge from mesh 1
		for (sp_uint i = 0; i < edgesLengthObj1; i++)
		{
			Vec3 _edgePoint1;
			Vec3 _edgePoint2;
			transformObj1->transform(vertexesObj1[edgesObj1[i].x], &_edgePoint1);
			transformObj1->transform(vertexesObj1[edgesObj1[i].y], &_edgePoint2);

			Line3D _edge(_edgePoint1, _edgePoint2);

			// for each face from mesh 2
			for (sp_uint j = 0; j < facesIndexLengthObj2; j++)
			{
				Vec3 contactPoint, p1, p2, p3;
				sp_bool hasIntersection;
				transformObj2->transform(vertexesObj2[facesObj2[j].x], &p1);
				transformObj2->transform(vertexesObj2[facesObj2[j].y], &p2);
				transformObj2->transform(vertexesObj2[facesObj2[j].z], &p3);

				const Triangle3D _face(p1, p2, p3);
				_edge.intersection(_face, &contactPoint, &hasIntersection);

				if (!hasIntersection)
					continue;

				// check which point of the line crossed the face (back-face)
				Plane3D faceAsPlane(_face);

				// if point is back-face, ...
				sp_uint contactPointIndex;
				if (faceAsPlane.distance(_edge.point1) <= ZERO_FLOAT)
				{
					contactPoint = _edge.point1;
					contactPointIndex = edgesObj1[i].x;
				}
				else
				{
					contactPoint = _edge.point2;
					contactPointIndex = edgesObj1[i].y;
				}

				// pega os ponto paralelos normal a face que colidiu (se houver)
				SpVertexEdges* parallelPoints[16];
				sp_uint parallelPointsLength = ZERO_UINT;
				SpVertexEdges* vertexEdge = vertexEdgesObj1->find(contactPointIndex);
				vertexEdge->findParallelVertexes(faceAsPlane, *transformObj1, parallelPoints, &parallelPointsLength, contactPointIndex);

				details->extremeVertexObj1 = vertexEdge;
				details->collisionNormal = faceAsPlane.normalVector;
				details->contactPoints[0] = contactPoint;
				details->contactPointsLength = parallelPointsLength;

				if (parallelPointsLength == ZERO_UINT) // point-face contact
				{
					details->contactPoints[0] = contactPoint;
					details->contactPointsLength = ONE_UINT;
					details->type = SpCollisionType::PointFace;
					return;
				}

				if (parallelPointsLength == ONE_UINT) // point-face contact
				{
					details->contactPoints[0] = vertexesObj1[parallelPoints[0]->vertexIndex()];
					details->type = SpCollisionType::PointFace;
					return;
				}

				if (parallelPointsLength == TWO_UINT) // edge-face contact
				{
					details->contactPoints[0] = vertexesObj1[parallelPoints[0]->vertexIndex()];
					details->contactPoints[1] = vertexesObj1[parallelPoints[1]->vertexIndex()];
					details->type = SpCollisionType::EdgeFace;
					return;
				}

				if (parallelPointsLength > TWO_UINT) // face-face contact
				{
					details->type = SpCollisionType::FaceFace;
					for (sp_uint i = 0; i < parallelPointsLength; i++)
						details->contactPoints[i] = vertexesObj1[parallelPoints[i]->vertexIndex()];
					return;
				}
			}
		}
		
	}

	void SpPhysicSimulator::addFriction(SpPhysicProperties* obj1Properties, SpPhysicProperties* obj2Properties, const Vec3& relativeVel, const Vec3& collisionNormal, const Vec3& rayToContactObj1, const Vec3& rayToContactObj2, const sp_float& j)
	{
		const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();

		Vec3 tangent = relativeVel - (collisionNormal * relativeVel.dot(collisionNormal));

		if (isCloseEnough((tangent.x + tangent.y + tangent.z), ZERO_FLOAT))
			return;

		tangent = tangent.normalize();
		sp_float numerator = -relativeVel.dot(tangent);
		Vec3 d2 = (obj1Properties->inertialTensorInverse() * tangent.cross(rayToContactObj1)).cross(rayToContactObj1);
		Vec3 d3 = (obj2Properties->inertialTensorInverse() * tangent.cross(rayToContactObj2)).cross(rayToContactObj2);
		sp_float denominator = invMassSum + tangent.dot(d2 + d3);

		if (denominator == 0.0f)
			return;

		sp_float jt = numerator / denominator;

		if (isCloseEnough(jt, 0.0f))
			return;

		const sp_float friction = sqrtf(obj1Properties->coeficientOfFriction() * obj2Properties->coeficientOfFriction());
		if (jt > j * friction) {
			jt = j * friction;
		}
		else if (jt < -j * friction) {
			jt = -j * friction;
		}

		const Vec3 tangentImpuse = tangent * jt;

		obj1Properties->_velocity = obj1Properties->velocity() - tangentImpuse * obj1Properties->massInverse();
		obj1Properties->_angularVelocity = obj1Properties->angularVelocity() - obj1Properties->inertialTensorInverse() * rayToContactObj1.cross(tangentImpuse);

		obj2Properties->_velocity = obj2Properties->velocity() + tangentImpuse * obj2Properties->massInverse();
		obj2Properties->_angularVelocity = obj2Properties->angularVelocity() + obj2Properties->inertialTensorInverse() * rayToContactObj2.cross(tangentImpuse);
	}

	void SpPhysicSimulator::handleCollisionResponse(SpCollisionDetails* details)
	{
		SpPhysicProperties* obj1Properties = &_physicProperties[details->objIndex1];
		SpPhysicProperties* obj2Properties = &_physicProperties[details->objIndex2];

		const sp_float cor = std::min(obj1Properties->coeficientOfRestitution(), obj2Properties->coeficientOfRestitution());

		if (obj1Properties->isStatic())
		{
			const Vec3 center = _transforms[details->objIndex2].position;
			handleCollisionResponseWithStatic(details, obj2Properties, center, cor);
		}
		else if (obj2Properties->isStatic())
		{
			const Vec3 center = _transforms[details->objIndex1].position;
			handleCollisionResponseWithStatic(details, obj1Properties, center, cor);
		}
		else
		{
			const sp_float invMassSum = obj1Properties->massInverse() + obj2Properties->massInverse();
			
			//const Vec3 centerObj1 = _boundingVolumes[details->objIndex1].centerOfBoundingVolume();
			//const Vec3 centerObj2 = _boundingVolumes[details->objIndex2].centerOfBoundingVolume();
			const Vec3 centerObj1 = _transforms[details->objIndex1].position;
			const Vec3 centerObj2 = _transforms[details->objIndex2].position;

			const Vec3 rayToContactObj1 = (details->contactPoints[0] - centerObj1);
			const Vec3 rayToContactObj2 = (details->contactPoints[0] - centerObj2);
			
			const Vec3 collisionNormal = (details->contactPoints[0] - centerObj1).normalize();

			const Vec3 relativeVel = (obj2Properties->velocity() + obj2Properties->angularVelocity().cross(rayToContactObj2))
									- (obj1Properties->velocity() + obj2Properties->angularVelocity().cross(rayToContactObj1));

			sp_float numerator = (-(1.0f + cor) * relativeVel.dot(collisionNormal));
			
			Vec3 d2 = (obj1Properties->inertialTensorInverse() * rayToContactObj1.cross(collisionNormal))
									.cross(rayToContactObj1);
			
			Vec3 d3 = (obj2Properties->inertialTensorInverse() * rayToContactObj2.cross(collisionNormal))
									.cross(rayToContactObj2);
			
			sp_float denominator = invMassSum + collisionNormal.dot(d2 + d3);

			sp_float j = denominator == ZERO_FLOAT ? ZERO_FLOAT 
								: numerator / denominator;

			if (details->contactPointsLength > ZERO_FLOAT && j != ZERO_FLOAT)
				j /= (sp_float)details->contactPointsLength;

			const Vec3 impulse = collisionNormal * j;

			obj1Properties->_velocity = obj1Properties->velocity() - impulse * obj1Properties->massInverse();
			obj1Properties->_angularVelocity = obj1Properties->angularVelocity() - obj1Properties->inertialTensorInverse() * rayToContactObj1.cross(impulse);
			
			obj2Properties->_velocity = obj2Properties->velocity() + impulse * obj2Properties->massInverse();
			obj2Properties->_angularVelocity = obj2Properties->angularVelocity() + obj2Properties->inertialTensorInverse() * rayToContactObj2.cross(impulse);
			
			//addFriction(obj1Properties, obj2Properties, relativeVel, collisionNormal , rayToContactObj1, rayToContactObj2, j);
			
			obj1Properties->_acceleration = ZERO_FLOAT;
			obj1Properties->_torque = ZERO_FLOAT;

			obj2Properties->_acceleration = ZERO_FLOAT;
			obj2Properties->_torque = ZERO_FLOAT;
		}
	}

	void SpPhysicSimulator::handleCollisionCPU(void* threadParameter)
	{
		SpCollisionDetails* details = (SpCollisionDetails*)threadParameter;
		
		sp_assert(details->objIndex1 != details->objIndex2, "InvalidArgumentException");

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpPhysicProperties* obj1Properties = &simulator->_physicProperties[details->objIndex1];
		SpPhysicProperties* obj2Properties = &simulator->_physicProperties[details->objIndex2];

		const sp_bool isObj1Static = obj1Properties->isStatic();
		const sp_bool isObj2Static = obj2Properties->isStatic();

		if (isObj1Static && isObj2Static) // if both are static, ...
		{
			details->ignoreCollision = true;
			return;
		}

		const sp_bool isObj1Resting = obj1Properties->isResting();
		const sp_bool isObj2Resting = obj2Properties->isResting();

		if (isObj1Resting && isObj2Resting) // both are resting
		{
			if (!isObj1Static && !isObj2Static) // both are dynamic, but both are resting, return
				return;

			simulator->translate(details->objIndex1, obj1Properties->_previousPosition - obj1Properties->_position);
			obj1Properties->_position = obj1Properties->_previousPosition;
			obj1Properties->_velocity = Vec3(ZERO_FLOAT);
			obj1Properties->_angularVelocity = Vec3(ZERO_FLOAT);
			obj1Properties->_acceleration = Vec3(ZERO_FLOAT);

			simulator->translate(details->objIndex2, obj2Properties->_previousPosition - obj2Properties->_position);
			obj2Properties->_position = obj2Properties->_previousPosition;
			obj2Properties->_velocity = Vec3(ZERO_FLOAT);
			obj2Properties->_angularVelocity = Vec3(ZERO_FLOAT);
			obj2Properties->_acceleration = Vec3(ZERO_FLOAT);

			details->ignoreCollision = true;
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			simulator->translate(details->objIndex2, obj2Properties->_previousPosition - obj2Properties->_position);
			obj2Properties->_position = obj2Properties->_previousPosition;
			obj2Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj2Properties->_angularVelocity = Vec3(ZERO_FLOAT);
			obj2Properties->_velocity = Vec3(ZERO_FLOAT);
			
			details->ignoreCollision = true;
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			simulator->translate(details->objIndex1, obj1Properties->_previousPosition - obj1Properties->_position);
			obj1Properties->_position = obj1Properties->_previousPosition;
			obj1Properties->_acceleration = Vec3(ZERO_FLOAT);
			obj1Properties->_angularVelocity = Vec3(ZERO_FLOAT);
			obj1Properties->_velocity = Vec3(ZERO_FLOAT);
			
			details->ignoreCollision = true;
			return;
		}

		if (simulator->areMovingAway(details->objIndex1, details->objIndex2)) // if the objects are getting distant
		{
			details->ignoreCollision = true;
			return;
		}

		//simulator->filterCollisions(details);

		if (isObj1Static || isObj1Resting)
		{
			simulator->findTimeOfCollisionWithMeshAndObj1Static(details->objIndex1, details->objIndex2, details);
			
			//if (details->ignoreCollision)
				//simulator->findTimeOfCollisionWithMesh(details->objIndex2, details->objIndex1, details);
		}
		else
		{
			if (isObj2Static || isObj2Resting)
			{
				simulator->findTimeOfCollisionWithMeshAndObj1Static(details->objIndex2, details->objIndex1, details);

				//if (details->ignoreCollision)
					//simulator->findTimeOfCollisionWithMesh(details->objIndex1, details->objIndex2, details);
			}
			else
				simulator->findTimeOfCollisionWithMesh(details->objIndex2, details->objIndex1, details);
		}

		if (details->ignoreCollision) // if they are not colliding in geometry
			return;

		// The objects are not static and not resting and not moving away and colliding, handle response...
		if (isObj1Static || isObj1Resting)
			simulator->collisionDetailsObj1ToObj2(details->objIndex2, details->objIndex1, details);
		else
			if (isObj2Static || isObj2Resting)
				simulator->collisionDetailsObj1ToObj2(details->objIndex1, details->objIndex2, details);
			//else
				//simulator->collisionDetails(details);
		// TODO: Descomentar !!!!!

		if (details->type == SpCollisionType::None || details->contactPointsLength == 0u)
			char* arr = "ERRO";

		simulator->handleCollisionResponse(details);
	}

	void SpPhysicSimulator::handleCollisionGPU(void* threadParameter)
	{
		SpCollisionDetails* details = (SpCollisionDetails*)threadParameter;

		sp_assert(details != nullptr, "InvalidArgumentException");
		sp_assert(details->objIndex1 != details->objIndex2, "InvalidArgumentException");

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		
		//simulator->filterCollisions(details);
		// TODO: fazer;

		if (details->ignoreCollision)
			return;

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

	void SpPhysicSimulator::findCollisionsGpuOLD(SweepAndPruneResult* result)
	{
		sap->execute(ONE_UINT, &lastEvent);
		lastEvent = sap->lastEvent;

		result->length = sap->fetchCollisionLength();

		gpu->commandManager->readBuffer(_sapCollisionIndexesGPU, SIZEOF_TWO_UINT * result->length, result->indexes, 1u, &lastEvent);
	}

	void SpPhysicSimulator::findCollisionsGpu(SweepAndPruneResult* result)
	{
		//timerToPhysic.update();
		sap->execute(ONE_UINT, &sap->lastEvent);

		collisionResponseGPU->updateParameters(_sapCollisionIndexesGPU, _sapCollisionIndexesLengthGPU, _boundingVolumes, _physicProperties);

		collisionResponseGPU->execute(ONE_UINT, &sap->lastEvent);
		lastEvent = collisionResponseGPU->lastEvent;

		collisionResponseGPU->fetchCollisionLength(&result->length);
		collisionResponseGPU->fetchCollisions(result->indexes);
		//std::cout << timerToPhysic.elapsedTime() << END_OF_LINE;

		//std::cout << result->length << END_OF_LINE;
	}

	void SpPhysicSimulator::run()
	{
		SweepAndPruneResult sapResult;
		sapResult.indexes = ALLOC_ARRAY(sp_uint, multiplyBy4(_objectsLength));
		
		findCollisionsCpu(&sapResult);

		updateDataOnGPU();
		//findCollisionsGpu(&sapResult);
		updateDataOnCPU();

		SpCollisionDetails* detailsArray = ALLOC_NEW_ARRAY(SpCollisionDetails, sapResult.length);
		SpThreadTask* tasks = ALLOC_NEW_ARRAY(SpThreadTask, sapResult.length);
		SpThreadPool* threadPool = SpThreadPool::instance();
		const sp_float elapsedTime = Timer::physicTimer()->elapsedTime();

		for (sp_uint i = 0; i < sapResult.length; i++)
		{
			sp_assert(sp_isHeapInitialized(sapResult.indexes[multiplyBy2(i)]), "MemoryNotInitializedExeption");
			sp_assert(sp_isHeapInitialized(sapResult.indexes[multiplyBy2(i)+1]), "MemoryNotInitializedExeption");

			detailsArray[i].objIndex1 = sapResult.indexes[multiplyBy2(i)];
			detailsArray[i].objIndex2 = sapResult.indexes[multiplyBy2(i) + 1];
			detailsArray[i].timeStep = elapsedTime;

			//tasks[i].func = &SpPhysicSimulator::handleCollisionGPU;
			tasks[i].func = &SpPhysicSimulator::handleCollisionCPU;
			tasks[i].parameter = &detailsArray[i];

			threadPool->schedule(&tasks[i]);
		}
		SpThreadPool::instance()->waitToFinish();

		for (sp_uint i = 0; i < sapResult.length; i++)
			if (!detailsArray[i].ignoreCollision)
				dispatchEvent(&detailsArray[i]);

		ALLOC_RELEASE(sapResult.indexes);
		sapResult.indexes = nullptr;
	}

	void SpPhysicSimulator::collisionDetails(SpCollisionDetails* details)
	{
		sp_assert(false, "NotImplementedException");
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