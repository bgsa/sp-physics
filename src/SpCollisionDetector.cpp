#include "SpCollisionDetector.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionDetector::filterCollision(SpCollisionDetails* details) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpPhysicProperties* obj1Properties = simulator->physicProperties(details->objIndex1);
		SpPhysicProperties* obj2Properties = simulator->physicProperties(details->objIndex2);

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
			{
				details->ignoreCollision = true;
				return;
			}

			simulator->translate(details->objIndex1, obj1Properties->previousState.position() - obj1Properties->currentState.position());
			obj1Properties->currentState.position(obj1Properties->previousState.position());
			obj1Properties->currentState.velocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.acceleration(Vec3(ZERO_FLOAT));

			simulator->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.velocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.acceleration(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			simulator->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.velocity(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			simulator->translate(details->objIndex1, obj1Properties->previousState.position() - obj1Properties->currentState.position());
			obj1Properties->currentState.position(obj1Properties->previousState.position());
			obj1Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.velocity(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}
	}

	sp_bool SpCollisionDetector::findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, sp_uint* vertexIndexObj1, Vec3* contactPoint, SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(obj1Index)->meshIndex);
		SpEdgeMesh** edges = mesh1->edges->data();
		SpTransform transformObj1 = *simulator->transforms(obj1Index);
		const sp_uint edgesLengthObj1 = mesh1->edges->length();

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(obj2Index)->meshIndex);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpTransform transformObj2 = *simulator->transforms(obj2Index);
		const sp_uint facesLengthObj2 = mesh2->faces->length();

		// for each edge from mesh 1
		for (sp_uint i = 0; i < edgesLengthObj1; i++)
		{
			if (!edges[i]->isBoundaryEdge())// check it is a boundary edge. if false, ignore
				continue;

			Line3D edge;
			edges[i]->convert(&edge, transformObj1);

			// for each face from mesh 2
			for (sp_uint j = 0; j < facesLengthObj2; j++)
			{
				Triangle3D face;
				allFacesObj2[j]->convert(&face, transformObj2);
			
				if (!edge.intersection(face, contactPoint))
					continue;
				
				// check which point crossed the face
				Plane3D plane(face);

				if (plane.isBackFace(edge.point1))
					vertexIndexObj1[0] = edges[i]->vertexIndex1;
				else
					vertexIndexObj1[0] = edges[i]->vertexIndex2;

				return true;
			}
		}

		return false;
	}

	CollisionStatus SpCollisionDetector::collisionStatus(Vec3* contactPoint, sp_bool* searchOnObj1, SpCollisionDetails* details)
	{
		sp_bool hasCollision = findCollisionEdgeFace(details->objIndex1, details->objIndex2, &details->vertexIndexObj1, contactPoint, details);
		
		if (hasCollision)
		{
			searchOnObj1[0] = false;
			return CollisionStatus::INSIDE;
		}

		hasCollision = findCollisionEdgeFace(details->objIndex2, details->objIndex1, &details->vertexIndexObj2, contactPoint, details);

		if (hasCollision)
		{
			searchOnObj1[0] = true;
			return CollisionStatus::INSIDE;
		}

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpTransform* transformObj1 = simulator->transforms(details->objIndex1);
		SpTransform* transformObj2 = simulator->transforms(details->objIndex2);

		hasCollision = mesh1->isInside(mesh2, *transformObj1, *transformObj2);

		if (hasCollision)
			return CollisionStatus::INSIDE;

		hasCollision = mesh2->isInside(mesh1, *transformObj2, *transformObj1);

		return hasCollision
			? CollisionStatus::INSIDE
			: CollisionStatus::OUTSIDE;
	}

	void SpCollisionDetector::timeOfCollision(SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		CollisionStatus status = CollisionStatus::OUTSIDE;
		const sp_float _epsilon = 0.1f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float diff = TEN_FLOAT;
		sp_float elapsedTime = details->timeStep * HALF_FLOAT;
		sp_float wasInsideAt = ZERO_FLOAT;
		Vec3 contactPoint;
		sp_bool searchOnObj1;
		
		while ((status != CollisionStatus::INSIDE || diff > _epsilon) && diff > 0.01f)
		{
			simulator->backToTime(details->objIndex1);
			simulator->integrator->execute(details->objIndex1, elapsedTime);

			simulator->backToTime(details->objIndex2);
			simulator->integrator->execute(details->objIndex2, elapsedTime);

			status = collisionStatus(&contactPoint, &searchOnObj1, details);

			diff = std::fabsf(previousElapsedTime - elapsedTime);
			previousElapsedTime = elapsedTime;

			if (status == CollisionStatus::OUTSIDE)
				elapsedTime += (diff * HALF_FLOAT);
			else
			{
				wasInsideAt = elapsedTime;
				elapsedTime -= (diff * HALF_FLOAT);
			}
		}

		if (status == CollisionStatus::OUTSIDE && wasInsideAt == ZERO_FLOAT)
		{
			details->ignoreCollision = true;
			return;
		}

		if (status == CollisionStatus::OUTSIDE && wasInsideAt != ZERO_FLOAT)
		{
			elapsedTime = wasInsideAt;

			simulator->backToTime(details->objIndex1);
			simulator->integrator->execute(details->objIndex1, elapsedTime);

			simulator->backToTime(details->objIndex2);
			simulator->integrator->execute(details->objIndex2, elapsedTime);

			collisionStatus(&contactPoint, &searchOnObj1, details);
		}

		if (searchOnObj1)
		{
			const sp_uint meshIndex = simulator->collisionFeatures(details->objIndex1)->meshIndex;
			SpMesh* mesh = simulator->mesh(meshIndex);
			details->vertexIndexObj1 = mesh->findClosest(contactPoint, *simulator->transforms(details->objIndex1))->index();
		}
		else
		{
			const sp_uint meshIndex = simulator->collisionFeatures(details->objIndex2)->meshIndex;
			SpMesh* mesh = simulator->mesh(meshIndex);
			details->vertexIndexObj2 = mesh->findClosest(contactPoint, *simulator->transforms(details->objIndex2))->index();
		}

		simulator->physicProperties(details->objIndex1)->integratedTime(elapsedTime);
		simulator->physicProperties(details->objIndex2)->integratedTime(elapsedTime);
		details->timeOfCollision = elapsedTime;
	}

	void SpCollisionDetector::collisionDetails(SpCollisionDetails* details)
	{
		timeOfCollision(details);

		if (details->ignoreCollision) // if they are not colliding in geometry
			return;

		fillCollisionDetails(details);
	}

	sp_bool SpCollisionDetector::fillCollisionDetailsEdgeEdge(const Line3D& edge, SpVertexMesh* vertex, const SpTransform& vertexTransform, sp_uint* edgeIndexOutput, const sp_float _epsilon)
	{
		// for each edge, if the closest point is contact point, edge-edge
		Vec3 closestPointOnEdge1, closestPointOnEdge2;
		sp_float smallerDistance = SP_FLOAT_MAX;

		sp_bool isPoint1FrontFace = false;
		sp_bool isPoint2FrontFace = false;
		SpFaceMesh* facePoint1Collision = nullptr;
		SpFaceMesh* facePoint2Collision = nullptr;

		for (sp_uint i = 0; i < vertex->faceIndexLength(); i++)
		{
			SpFaceMesh* face = vertex->face(i);
		
			if (face->isFrontFace(edge.point1, vertexTransform))
			{
				isPoint1FrontFace = true;

				if (facePoint1Collision == nullptr)
				{
					Vec3 contact;
					if (face->intersection(edge, &contact, vertexTransform))
						facePoint1Collision = face;
				}
			}

			if (face->isFrontFace(edge.point2, vertexTransform))
			{
				isPoint2FrontFace = true;
			
				if (facePoint2Collision == nullptr)
				{
					Vec3 contact;
					if (face->intersection(edge, &contact, vertexTransform))
						facePoint2Collision = face;
				}
			}
		}

		// if the edge points are front-face, the collision is edge
		if (isPoint1FrontFace && isPoint2FrontFace)
		{ 
			facePoint1Collision->intersection(facePoint2Collision, vertexTransform, edgeIndexOutput);
			return true;
		}

		/*
		for (sp_uint i = 0; i < vertex->_edges2->length(); i++)
		{
			SpEdgeMesh* edgeMesh = vertex->mesh()->edges->data()[i];
		
			if (!edgeMesh->isBoundaryEdge())
				continue;

			Line3D edge2; 
			edgeMesh->convert(&edge2, vertexTransform);

			Vec3 closesPointOnEdge1, closesPointOnEdge2;
			sp_float distance;
			edge.closestPoint(edge2, &closesPointOnEdge1, &closesPointOnEdge2, &distance);

			if (distance < smallerDistance)
			{
				// se essa edge cortar alguma face do outro, ok... é valida

				smallerDistance = distance;
				edgeIndexOutput[0] = i;
			}
		}

		if (smallerDistance < _epsilon)
			return true;
			*/
			
		return false;
	}

	void SpCollisionDetector::fillCollisionDetails(SpCollisionDetails* details)
	{
		if (isFaceFaceCollision(details))
			return;

		if (isEdgeFaceCollisionObj1(details))
			return;

		if (isEdgeFaceCollisionObj2(details))
			return;

		if (isEdgeEdgeCollision(details))
			return;

		isVertexFaceCollision(details);
	
		sp_assert(details->ignoreCollision == false, "ApplicationException");
		sp_assert(details->type != SpCollisionType::None, "ApplicationException");
	}

	sp_bool SpCollisionDetector::areMovingAway(sp_uint objIndex1, sp_uint objIndex2) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		const SpPhysicProperties* obj1Properties = simulator->physicProperties(objIndex1);
		const SpPhysicProperties* obj2Properties = simulator->physicProperties(objIndex2);

		Vec3 lineOfAction = obj2Properties->currentState.position() - obj1Properties->currentState.position();
		const Vec3 velocityToObject2 = obj1Properties->currentState.velocity() * lineOfAction;

		lineOfAction = obj1Properties->currentState.position() - obj2Properties->currentState.position();
		const Vec3 velocityToObject1 = obj2Properties->currentState.velocity() * lineOfAction;

		return velocityToObject2 <= ZERO_FLOAT && velocityToObject1 <= ZERO_FLOAT;
	}

	sp_bool SpCollisionDetector::isFaceFaceCollision(SpCollisionDetails* details) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		
		const SpTransform transform1 = *simulator->transforms(details->objIndex1);
		const SpTransform transform2 = *simulator->transforms(details->objIndex2);

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpVertexMesh* vertex1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		SpFaceMesh** allFacesObj1 = vertex1->mesh()->faces->data();
		SpEdgeMesh** allEdgesObj1 = mesh1->edges->data();

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpVertexMesh* vertex2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];
		SpFaceMesh** allFacesObj2 = vertex2->mesh()->faces->data();
		SpEdgeMesh** allEdgesObj2 = mesh2->edges->data();

		Vec3 intersectionPoints[10];
		sp_uint intersectionPointsLength = ZERO_UINT;

		sp_uint facesIndexesMesh1[10];
		sp_uint facesIndexesMesh1Length = ZERO_UINT;
		sp_uint facesIndexesMesh2[10];
		sp_uint facesIndexesMesh2Length = ZERO_UINT;

		vertex1->findParallelFaces(vertex2, transform1, transform2, facesIndexesMesh1, &facesIndexesMesh1Length , facesIndexesMesh2, &facesIndexesMesh2Length, 0.2f);

		if (facesIndexesMesh1Length == ZERO_UINT || facesIndexesMesh2Length == ZERO_UINT)
			return false;

		for (sp_uint i = 0; i < facesIndexesMesh1Length; i++) // for each parallel face from mesh 1
		{
			sp_uint* edgesIndexesObj1 = allFacesObj1[facesIndexesMesh1[i]]->edgesIndexes;
			
			for (sp_uint z = 0; z < 3u; z++) // for each edge from face "i"
			{
				SpEdgeMesh* edge1 = allEdgesObj1[edgesIndexesObj1[z]];

				if (!edge1->isBoundaryEdge())
					continue;

				for (sp_uint j = 0; j < facesIndexesMesh2Length; j++) // for each parallel face from mesh 3
				{
					sp_uint* edgesIndexesObj2 = allFacesObj2[facesIndexesMesh2[j]]->edgesIndexes;

					for (sp_uint w = 0; w < 3u; w++) // for each edge from face "j"
					{
						SpEdgeMesh* edge2 = allEdgesObj2[edgesIndexesObj2[w]];

						if (!edge2->isBoundaryEdge())
							continue;

						Vec3 contact;

						if (edge1->intersection(edge2, &contact, transform1, transform2, 0.2f))
						{
							// add intersection point if not exists and is inside the faces
							if (!contains(intersectionPoints, intersectionPointsLength, contact, 0.009f)
								&& allFacesObj1[facesIndexesMesh1[i]]->isInside(contact, transform1)
								&& allFacesObj2[facesIndexesMesh2[j]]->isInside(contact, transform2))
								intersectionPoints[intersectionPointsLength++] = contact;

							mesh1->vertex(edge1->vertexIndex1, transform1, &contact);

							// check if vertex points from edge 1 is inside the face from mesh2
							if (!contains(intersectionPoints, intersectionPointsLength, contact, 0.009f)
								&& allFacesObj2[facesIndexesMesh2[j]]->isInside(contact, transform2))
								intersectionPoints[intersectionPointsLength++] = contact;
							else
							{
								mesh1->vertex(edge1->vertexIndex2, transform1, &contact);

								if (!contains(intersectionPoints, intersectionPointsLength, contact, 0.009f)
									&& allFacesObj2[facesIndexesMesh2[j]]->isInside(contact, transform2))
									intersectionPoints[intersectionPointsLength++] = contact;
							}

							mesh2->vertex(edge2->vertexIndex1, transform2, &contact);

							// check if vertex points from edge 2 is inside the face from mesh1
							if (allFacesObj1[facesIndexesMesh1[i]]->isInside(contact, transform1)
								&& !contains(intersectionPoints, intersectionPointsLength, contact, 0.009f))
								intersectionPoints[intersectionPointsLength++] = contact;
							else
							{
								mesh2->vertex(edge2->vertexIndex2, transform2, &contact);

								if (allFacesObj1[facesIndexesMesh1[i]]->isInside(contact, transform1)
									&& !contains(intersectionPoints, intersectionPointsLength, contact, 0.009f))
									intersectionPoints[intersectionPointsLength++] = contact;
							}
						}
					}
				}
			}
		}

		sp_assert(intersectionPointsLength > 2u, "InvalidOperationException");

		details->type = SpCollisionType::FaceFace;
		//allFacesObj1[facesIndexesMesh1[0]]->normalVector(&details->collisionNormalObj1);
		//details->collisionNormalObj1 = allFacesObj1[facesIndexesMesh1[0]]->faceNormal;
		rotate(transform1.orientation, allFacesObj1[facesIndexesMesh1[0]]->faceNormal, &details->collisionNormalObj1);

		details->collisionNormalObj2 = -details->collisionNormalObj1;

		for (sp_uint i = 0; i < intersectionPointsLength; i++)
		{
			details->centerContactPoint += intersectionPoints[i];
			details->contactPointsObj1[i] = intersectionPoints[i];
			details->contactPointsObj2[i] = intersectionPoints[i];
		}
		details->contactPointsLengthObj1 = intersectionPointsLength;
		details->contactPointsLengthObj2 = intersectionPointsLength;
		details->centerContactPoint /= (sp_float)intersectionPointsLength;

		return true;
	}

	sp_bool SpCollisionDetector::isEdgeFaceCollision(SpVertexMesh* vertex1, SpVertexMesh* vertex2, const SpTransform& transform1, const SpTransform& transform2, sp_uint* faceIndexObj1, sp_uint* vertexIndexObj2) const
	{
		SpMesh* mesh2 = vertex2->mesh();
	
		Line3D edge2;
		transform2.transform(vertex2->value(), &edge2.point1);

		for (sp_uint i = 0; i < vertex1->faceIndexLength(); i++)
		{
			Plane3D face1;
			vertex1->face(i)->convert(&face1, transform1);

			for (sp_uint j = 0; j < vertex2->edgeVertexIndexLength(); j++)
			{
				mesh2->vertex(vertex2->edgeVertexIndex(j), transform2, &edge2.point2);

				// the edge and face normal should be perpendicular for edge-face collision
				if (!edge2.isPerpendicular(face1.normalVector, 0.1f))
					continue;

				const sp_float distancePoint1 = face1.distance(edge2.point1);
				
				if (!isCloseEnough(std::fabsf(distancePoint1), ZERO_FLOAT, 0.1f))
					continue;

				const sp_float distancePoint2 = face1.distance(edge2.point2);

				if (isCloseEnough(distancePoint1, distancePoint2, 0.1f))
				{
					faceIndexObj1[0] = vertex1->faceIndex(i);
					vertexIndexObj2[0] = vertex2->edgeVertexIndex(j);
					return true;
				}
			}
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeEdgeCollision(SpCollisionDetails* details) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		const SpTransform transform1 = *simulator->transforms(details->objIndex1);
		const SpTransform transform2 = *simulator->transforms(details->objIndex2);

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		const sp_uint edgesLengthObj1 = vertexMeshObj1->edgeVertexIndexLength();

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];
		const sp_uint edgesLengthObj2 = vertexMeshObj2->edgeVertexIndexLength();

		sp_float smallestDistance = SP_FLOAT_MAX;

		Line3D line1;
		transform1.transform(vertexMeshObj1->value(), &line1.point1);

		Line3D line2;
		transform2.transform(vertexMeshObj2->value(), &line2.point1);

		for (sp_uint i = 0; i < edgesLengthObj1; i++)
		{
			mesh1->vertex(vertexMeshObj1->edgeVertexIndex(i), transform1, &line1.point2);

			for (sp_uint j = 0; j < edgesLengthObj2; j++)
			{
				mesh2->vertex(vertexMeshObj2->edgeVertexIndex(j), transform2, &line2.point2);
				
				Vec3 p1, p2;
				sp_float sqDistance;
				line1.closestPoint(line2, &p1, &p2, &sqDistance);

				// if the distance is greater or the contact is far away, discard
				if (sqDistance >= smallestDistance || 
					!isCloseEnough(std::fabsf(sqDistance), ZERO_FLOAT, 0.1f))
					continue;

				smallestDistance = sqDistance;
				
				if (isCloseEnough(smallestDistance, ZERO_FLOAT))
				{
					details->type = SpCollisionType::EdgeEdge;

					details->contactPointsLengthObj1 = 2u;
					details->contactPointsObj1[0] = line1.point1;
					details->contactPointsObj1[1] = line1.point2;

					details->contactPointsLengthObj2 = 2u;
					details->contactPointsObj2[0] = line2.point1;
					details->contactPointsObj2[1] = line2.point2;

					details->centerContactPoint = p1;

					line1.cross(line2, &details->collisionNormalObj1);
					normalize(&details->collisionNormalObj1);

					details->collisionNormalObj2 = -details->collisionNormalObj1;

					return true;
				}
			}
		}

		return false;
	}

	sp_bool SpCollisionDetector::isVertexFaceCollision(SpCollisionDetails* details) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		const SpTransform transformObj1 = *simulator->transforms(details->objIndex1);
		const SpTransform transformObj2 = *simulator->transforms(details->objIndex2);

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];

		SpFaceMesh** facesObj1 = vertexMeshObj1->mesh()->faces->data();
		SpFaceMesh** facesObj2 = vertexMeshObj2->mesh()->faces->data();

		for (sp_uint i = 0; i < vertexMeshObj1->faceIndexLength(); i++)
		{
			Triangle3D triangle;
			vertexMeshObj1->face(i)->convert(&triangle, transformObj1);

			for (sp_uint j = 0; j < vertexMeshObj2->edgeVertexIndexLength(); j++)
			{
				Line3D line;
				transformObj2.transform(vertexMeshObj2->value(), &line.point1);
				mesh2->vertex(vertexMeshObj2->edgeVertexIndex(j), transformObj2, &line.point2);
			
				if (line.intersection(triangle, &details->centerContactPoint))
				{
					Plane3D faceContact(triangle);
					
					details->contactPointsLengthObj2 = details->contactPointsLengthObj1 = ONE_UINT;
					if (faceContact.isBackFace(line.point1))
						details->contactPointsObj2[0] = details->contactPointsObj1[0] = line.point1;
					else
						details->contactPointsObj2[0] = details->contactPointsObj1[0] = line.point2;

					details->type = SpCollisionType::PointFace;
					details->collisionNormalObj1 = faceContact.normalVector;
					details->collisionNormalObj2 = -faceContact.normalVector;

					return true;
				}
			}
		}

		for (sp_uint i = 0; i < vertexMeshObj2->faceIndexLength(); i++)
		{
			Triangle3D triangle;
			vertexMeshObj2->face(i)->convert(&triangle, transformObj2);

			for (sp_uint j = 0; j < vertexMeshObj1->edgeVertexIndexLength(); j++)
			{
				Line3D line;
				transformObj1.transform(vertexMeshObj1->value(), &line.point1);
				mesh1->vertex(vertexMeshObj1->edgeVertexIndex(j), transformObj1, &line.point2);
				
				if (line.intersection(triangle, &details->centerContactPoint))
				{
					Plane3D faceContact(triangle);
					
					details->contactPointsLengthObj1 = details->contactPointsLengthObj2 = ONE_UINT;
					if (faceContact.isBackFace(line.point1))
						details->contactPointsObj1[0] = details->contactPointsObj2[0] = line.point1;
					else
						details->contactPointsObj1[0] = details->contactPointsObj2[0] = line.point2;

					details->type = SpCollisionType::PointFace;
					details->collisionNormalObj1 = -faceContact.normalVector;
					details->collisionNormalObj2 = faceContact.normalVector;

					return true;
				}
			}
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeFaceCollisionObj1(SpCollisionDetails* details) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		const SpTransform transformObj1 = *simulator->transforms(details->objIndex1);
		SpFaceMesh** allFacesObj1 = mesh1->faces->data();
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		const SpTransform transformObj2 = *simulator->transforms(details->objIndex2);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];

		sp_uint faceIndexCollisionObj1;
		sp_uint edgeVertexIndexObj2;

		if (isEdgeFaceCollision(vertexMeshObj1, vertexMeshObj2, transformObj1, transformObj2, &faceIndexCollisionObj1, &edgeVertexIndexObj2))
		{
#define MAX_PARALLEL_FACES 10u
#define MAX_CONTACTS 2u
			SpFaceMesh* faceCollisionObj1 = mesh1->faces->data()[faceIndexCollisionObj1];

			Line3D edge;
			transformObj2.transform(vertexMeshObj2->value(), &edge.point1);
			//mesh2->vertex(vertexMeshObj2->edgeVertexIndex(edgeVertexIndexObj2), transformObj2, &edge.point2);
			mesh2->vertex(edgeVertexIndexObj2, transformObj2, &edge.point2);

			Plane3D faceAsPlane;
			faceCollisionObj1->convert(&faceAsPlane, transformObj1);


			sp_uint parallelFacesIndexes[MAX_PARALLEL_FACES];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			vertexMeshObj1->findParallelFaces(faceAsPlane, transformObj1, parallelFacesIndexes, &parallelFacesIndexesLength, 0.2f);

			sp_assert(parallelFacesIndexesLength <= MAX_PARALLEL_FACES, "IndexOutOfRangeException");

			Vec3 contacts[MAX_CONTACTS];
			sp_uint contactsLength = ZERO_UINT;

			// find which faces this edge cross and get the two contact points
			for (sp_uint i = 0; i < parallelFacesIndexesLength; i++)
			{
				SpFaceMesh* face = allFacesObj1[parallelFacesIndexes[i]];

				for (sp_uint j = 0; j < 3u; j++)
				{
					SpEdgeMesh* edgeMesh = face->edges(j);

					if (!edgeMesh->isBoundaryEdge())
						continue;

					Line3D line;
					edgeMesh->convert(&line, transformObj1);

					if (line.intersection(edge, &contacts[contactsLength], 0.2f))
					{
						contactsLength++;
						sp_assert(contactsLength <= MAX_CONTACTS, "IndexOutOfRangeException");

						if (face->isInside(edge.point1, transformObj1))
						{
							contacts[contactsLength++] = edge.point1;
							goto break_loops1;
						}
						if (face->isInside(edge.point2, transformObj1))
						{
							contacts[contactsLength++] = edge.point2;
							goto break_loops1;
						}
					}
				}
			}

		break_loops1:
			details->type = SpCollisionType::EdgeFace;

			details->contactPointsLengthObj1 = 2u;
			details->contactPointsObj1[0] = contacts[0];
			details->contactPointsObj1[1] = contacts[1];

			details->contactPointsLengthObj2 = 2u;
			details->contactPointsObj2[0] = contacts[0];
			details->contactPointsObj2[1] = contacts[1];

			details->centerContactPoint = (contacts[0] + contacts[1]) * HALF_FLOAT;

			details->collisionNormalObj1 = faceAsPlane.normalVector;
			details->collisionNormalObj2 = -faceAsPlane.normalVector;

			return true;
#undef MAX_PARALLEL_FACES
#undef MAX_CONTACTS
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeFaceCollisionObj2(SpCollisionDetails* details) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		const SpTransform transformObj1 = *simulator->transforms(details->objIndex1);
		SpFaceMesh** allFacesObj1 = mesh1->faces->data();
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		const SpTransform transformObj2 = *simulator->transforms(details->objIndex2);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];

		sp_uint faceIndexCollisionObj2;
		sp_uint edgeVertexIndexObj1;

		if (isEdgeFaceCollision(vertexMeshObj2, vertexMeshObj1, transformObj2, transformObj1, &faceIndexCollisionObj2, &edgeVertexIndexObj1))
		{
			SpFaceMesh* faceCollisionObj2 = mesh2->faces->data()[faceIndexCollisionObj2];

			Line3D edge;
			transformObj1.transform(vertexMeshObj1->value(), &edge.point1);
			//mesh1->vertex(vertexMeshObj1->edgeVertexIndex(edgeVertexIndexObj1), transformObj1, &edge.point2);
			mesh1->vertex(edgeVertexIndexObj1, transformObj1, &edge.point2);

			Plane3D faceAsPlane;
			faceCollisionObj2->convert(&faceAsPlane, transformObj2);

			sp_uint parallelFacesIndexes[10];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			// find all parallel faces
			vertexMeshObj2->findParallelFaces(faceAsPlane, transformObj2, parallelFacesIndexes, &parallelFacesIndexesLength, 0.2f);

			Vec3 contacts[2];
			sp_uint contactsLength = ZERO_UINT;

			// find which faces this edge cross and get the two contact points
			for (sp_uint i = 0; i < parallelFacesIndexesLength; i++)
			{
				SpFaceMesh* face = allFacesObj2[parallelFacesIndexes[i]];

				for (sp_uint j = 0; j < 3u; j++)
				{
					SpEdgeMesh* edgeMesh = face->edges(j);

					if (!edgeMesh->isBoundaryEdge())
						continue;

					Line3D line;
					edgeMesh->convert(&line, transformObj1);

					if (line.intersection(edge, &contacts[contactsLength]))
					{
						contactsLength++;

						if (face->isInside(edge.point1, transformObj2))
						{
							contacts[contactsLength++] = edge.point1;
							goto break_loops2;
						}
						if (face->isInside(edge.point2, transformObj2))
						{
							contacts[contactsLength++] = edge.point2;
							goto break_loops2;
						}
					}
				}
			}

		break_loops2:
			details->type = SpCollisionType::EdgeFace;

			details->contactPointsLengthObj1 = 2u;
			details->contactPointsObj1[0] = contacts[0];
			details->contactPointsObj1[1] = contacts[1];

			details->contactPointsLengthObj2 = 2u;
			details->contactPointsObj2[0] = contacts[0];
			details->contactPointsObj2[1] = contacts[1];

			details->centerContactPoint = (contacts[0] + contacts[1]) * HALF_FLOAT;

			details->collisionNormalObj1 = -faceAsPlane.normalVector;
			details->collisionNormalObj2 = faceAsPlane.normalVector;

			return true;
		}

		return false;
	}

}
