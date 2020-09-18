#include "SpCollisionDetector.h"
#include "SpMapleExporter.h"

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
			obj1Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.orientation(Quat());
			obj1Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.torque(Vec3(ZERO_FLOAT));

			simulator->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.velocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.orientation(Quat());
			obj2Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.torque(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			simulator->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.velocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.orientation(Quat());
			obj2Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.torque(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			simulator->translate(details->objIndex1, obj1Properties->previousState.position() - obj1Properties->currentState.position());
			obj1Properties->currentState.position(obj1Properties->previousState.position());
			obj1Properties->currentState.velocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.orientation(Quat());
			obj1Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.torque(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}
	}

	sp_bool SpCollisionDetector::findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, sp_uint* vertexIndexObj1, Vec3* contactPoint, sp_uint* edgeIndex, sp_uint* faceIndex, SpCollisionDetails* details)
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

				if (!edge.intersection(face, contactPoint, 0.02f))
					continue;

				// check which point crossed the face
				Plane3D plane(face);
				if (plane.isBackFace(edge.point1))
					vertexIndexObj1[0] = edges[i]->vertexIndex1;
				else
					vertexIndexObj1[0] = edges[i]->vertexIndex2;

				edgeIndex[0] = i;
				faceIndex[0] = j;

				return true;
			}
		}

		return false;
	}
	
	sp_bool SpCollisionDetector::collisionStatusCache(SpCollisionDetectorCache* cache, Vec3* contactPoint, SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);

		if (cache->searchOnObj1)
			return mesh2->edges->get(cache->edgeIndex)->intersection(
				*mesh1->faces->get(cache->faceIndex),
				contactPoint,
				*simulator->transforms(details->objIndex2),
				*simulator->transforms(details->objIndex1)
			);

		return mesh1->edges->get(cache->edgeIndex)->intersection(
			*mesh2->faces->get(cache->faceIndex),
			contactPoint,
			*simulator->transforms(details->objIndex1),
			*simulator->transforms(details->objIndex2)
		);
	}

	CollisionStatus SpCollisionDetector::collisionStatus(Vec3* contactPoint, SpCollisionDetectorCache* cache, SpCollisionDetails* details)
	{
		// check if the edge-face keep in contact,
		// if false, search on general collisionStatus
		if (cache->hasCache())
			if (collisionStatusCache(cache, contactPoint, details))
				return CollisionStatus::INSIDE;

		sp_bool hasCollision = findCollisionEdgeFace(details->objIndex1, details->objIndex2, &details->vertexIndexObj1, contactPoint, &cache->edgeIndex, &cache->faceIndex, details);
		
		if (hasCollision)
		{
			cache->searchOnObj1 = false;
			return CollisionStatus::INSIDE;
		}

		hasCollision = findCollisionEdgeFace(details->objIndex2, details->objIndex1, &details->vertexIndexObj2, contactPoint, &cache->edgeIndex, &cache->faceIndex, details);

		if (hasCollision)
		{
			cache->searchOnObj1 = true;
			return CollisionStatus::INSIDE;
		}

		/* check object is totally inside the other
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpTransform* transformObj1 = simulator->transforms(details->objIndex1);
		SpTransform* transformObj2 = simulator->transforms(details->objIndex2);

		hasCollision = mesh1->isInside(mesh2, *transformObj1, *transformObj2);

		if (hasCollision)
			return CollisionStatus::INSIDE;

		hasCollision = mesh2->isInside(mesh1, *transformObj2, *transformObj1);
		*/

		return hasCollision
			? CollisionStatus::INSIDE
			: CollisionStatus::OUTSIDE;
	}

	void SpCollisionDetector::timeOfCollision(SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		sp_assert(details->objIndex1 < simulator->objectsLength(), "InvalidArgumentException");
		sp_assert(details->objIndex2 < simulator->objectsLength(), "InvalidArgumentException");

		CollisionStatus status = CollisionStatus::OUTSIDE;
		const sp_float _epsilon = 0.1f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float diff = TEN_FLOAT;
		sp_float elapsedTime = details->timeStep * HALF_FLOAT;
		sp_float wasInsideAt = ZERO_FLOAT;
		Vec3 contactPoint;
		SpCollisionDetectorCache cache;

		while ((status != CollisionStatus::INSIDE || diff > _epsilon) && diff > 0.01f)
		{
			simulator->backToTime(details->objIndex1);
			simulator->integrator->execute(details->objIndex1, elapsedTime);

			simulator->backToTime(details->objIndex2);
			simulator->integrator->execute(details->objIndex2, elapsedTime);

			status = collisionStatus(&contactPoint, &cache, details);

			// TODO: REMOVER!
			if (details->objIndex1 > simulator->objectsLength())
				int a = 1;
			if (details->objIndex2 > simulator->objectsLength())
				int a = 1;
			

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

			collisionStatus(&contactPoint, &cache, details);
		}

		if (cache.searchOnObj1)
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

		sp_assert(details->objIndex1 < simulator->objectsLength(), "InvalidArgumentException");
		sp_assert(details->objIndex2 < simulator->objectsLength(), "InvalidArgumentException");
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
				// se essa edge cortar alguma face do outro, ok... � valida

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

		if (isVertexFaceCollision(details))
			return;
	
		// TODO: REMOVER!!
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		sp_char text1[8000];
		Maple::convert(*simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex), *simulator->transforms(details->objIndex1), "mesh1", "red", text1);
		sp_char text2[8000];
		Maple::convert(*simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex), *simulator->transforms(details->objIndex2), "mesh2", "blue", text2);
		sp_char display[300];
		sp_uint v = ZERO_UINT;
		Maple::display("mesh1", "mesh2", display, &v);

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
#define MAX_INTERSECTION_POINTS 12
#define MIN_INTERSECTION_POINTS 2
#define MAX_PARALLEL_FACES 10
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		
		const SpTransform transform1 = *simulator->transforms(details->objIndex1);
		const SpTransform transform2 = *simulator->transforms(details->objIndex2);

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpVertexMesh* vertex1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpVertexMesh* vertex2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];
		
		sp_uint facesIndexesMesh1[MAX_PARALLEL_FACES];
		sp_uint facesIndexesMesh1Length = ZERO_UINT;
		sp_uint facesIndexesMesh2[MAX_PARALLEL_FACES];
		sp_uint facesIndexesMesh2Length = ZERO_UINT;

		vertex1->findParallelFaces(vertex2, transform1, transform2, facesIndexesMesh1, &facesIndexesMesh1Length , facesIndexesMesh2, &facesIndexesMesh2Length, 0.02f);

		sp_assert(facesIndexesMesh1Length < MAX_PARALLEL_FACES, "IndexOutOfRangeException");
		sp_assert(facesIndexesMesh2Length < MAX_PARALLEL_FACES, "IndexOutOfRangeException");

		if (facesIndexesMesh1Length == ZERO_UINT || facesIndexesMesh2Length == ZERO_UINT)
			return false;

		SpFaceMesh** allFacesObj1 = vertex1->mesh()->faces->data();
		SpEdgeMesh** allEdgesObj1 = mesh1->edges->data();
		
		SpFaceMesh** allFacesObj2 = vertex2->mesh()->faces->data();
		SpEdgeMesh** allEdgesObj2 = mesh2->edges->data();

		Vec3 intersectionPoints[MAX_INTERSECTION_POINTS];
		sp_uint intersectionPointsLength = ZERO_UINT;

		sp_uint vertexIndexesObj1[MAX_INTERSECTION_POINTS];
		sp_uint vertexIndexesObj1Length = ZERO_UINT;
		sp_uint vertexIndexesObj2[MAX_INTERSECTION_POINTS];
		sp_uint vertexIndexesObj2Length = ZERO_UINT;

		// find points that is inside the face
		for (sp_uint i = 0; i < facesIndexesMesh1Length; i++)
		{
			SpFaceMesh* face1 = allFacesObj1[facesIndexesMesh1[i]];
			Triangle3D triangleFace1;
			face1->convert(&triangleFace1, transform1);

			for (sp_uint j = 0; j < facesIndexesMesh2Length; j++)
			{
				SpFaceMesh* face2 = allFacesObj2[facesIndexesMesh2[j]];
				Triangle3D triangleFace2;
				face2->convert(&triangleFace2, transform2);

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace2.point1, ERROR_MARGIN_PHYSIC)
					&& triangleFace1.isInside(triangleFace2.point1, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace2.point1;
					vertexIndexesObj2[vertexIndexesObj2Length++] = face2->vertexesIndexes[0];
				}


				// TODO: REMOVER!
				if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
					int aad = 1;


				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace2.point2, ERROR_MARGIN_PHYSIC)
					&& triangleFace1.isInside(triangleFace2.point2, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace2.point2;
					vertexIndexesObj2[vertexIndexesObj2Length++] = face2->vertexesIndexes[1];
				}

				// TODO: REMOVER!
				if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
					int aad = 1;

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace2.point3, ERROR_MARGIN_PHYSIC)
					&& triangleFace1.isInside(triangleFace2.point3, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace2.point3;
					vertexIndexesObj2[vertexIndexesObj2Length++] = face2->vertexesIndexes[2];
				}


				// TODO: REMOVER!
				if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
					int aad = 1;


				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace1.point1, ERROR_MARGIN_PHYSIC)
					&& triangleFace2.isInside(triangleFace1.point1, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace1.point1;
					vertexIndexesObj1[vertexIndexesObj1Length++] = face1->vertexesIndexes[0];
				}


				// TODO: REMOVER!
				if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
					int aad = 1;


				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace1.point2, ERROR_MARGIN_PHYSIC)
					&& triangleFace2.isInside(triangleFace1.point2, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace1.point2;
					vertexIndexesObj1[vertexIndexesObj1Length++] = face1->vertexesIndexes[1];
				}


				// TODO: REMOVER!
				if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
					int aad = 1;


				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace1.point3, ERROR_MARGIN_PHYSIC)
					&& triangleFace2.isInside(triangleFace1.point3, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace1.point3;
					vertexIndexesObj1[vertexIndexesObj1Length++] = face1->vertexesIndexes[2];
				}

				// TODO: REMOVER!
				if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
					int aad = 1;


			}
		}

		sp_assert(intersectionPointsLength< MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");
		sp_assert(vertexIndexesObj1Length < MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");
		sp_assert(vertexIndexesObj2Length < MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");

		// for all edge from inside vertexes ...
		for (sp_uint i = 0; i < vertexIndexesObj1Length; i++)
		{
			const SpVertexMesh* vertexI = mesh1->vertexesMesh->get(vertexIndexesObj1[i]);
			Ray ray1;
			transform1.transform(vertexI->value(), &ray1.point);

			for (sp_uint i2 = 0; i2 < vertexI->edgeLength(); i2++)
			{
				const SpVertexMesh* vertexI2 = mesh1->vertexesMesh->get(vertexI->edgeVertexIndex(i2));
				
				sp_uint eindex1 = mesh1->findEdge(vertexI->index(), vertexI->edgeVertexIndex(i2));
				SpEdgeMesh* e1 = mesh1->edges->get(eindex1);
				if (!e1->isBoundaryEdge())
					continue;
				
				transform1.transform(vertexI2->value(), &ray1.direction);
				diff(ray1.direction, ray1.point, &ray1.direction);
				normalize(&ray1.direction);

				for (sp_uint j = 0; j < vertexIndexesObj2Length; j++)
				{
					const SpVertexMesh* vertexJ = mesh2->vertexesMesh->get(vertexIndexesObj2[j]);
					Ray ray2;
					transform2.transform(vertexJ->value(), &ray2.point);

					for (sp_uint j2 = 0; j2 < vertexJ->edgeLength(); j2++)
					{
						const SpVertexMesh* vertexJ2 = mesh2->vertexesMesh->get(vertexJ->edgeVertexIndex(j2));
					
						sp_uint eindex = mesh2->findEdge(vertexJ->index(), vertexJ->edgeVertexIndex(j2));
						SpEdgeMesh* e = mesh2->edges->get(eindex);
						if (!e->isBoundaryEdge())
							continue;
					
						transform2.transform(vertexJ2->value(), &ray2.direction);
						diff(ray2.direction, ray2.point, &ray2.direction);
						normalize(&ray2.direction);

						// if the edge intersect on the plane, get the intersection point
						Vec3 contact;
						if (ray1.intersection(ray2, &contact, ERROR_MARGIN_PHYSIC))
							if (!contains(intersectionPoints, intersectionPointsLength, contact, ERROR_MARGIN_PHYSIC))
								intersectionPoints[intersectionPointsLength++] = contact;


						// TODO: REMOVER!
						if (intersectionPointsLength > MAX_INTERSECTION_POINTS)
							int aad = 1;

					}
				}
			}
		}

		/*
		for (sp_uint i = 0; i < facesIndexesMesh1Length; i++) // for each parallel face from mesh 1
		{
			Triangle3D face1;
			allFacesObj1[facesIndexesMesh1[i]]->convert(&face1, transform1);

			sp_uint* edgesIndexesObj1 = allFacesObj1[facesIndexesMesh1[i]]->edgesIndexes;
			
			for (sp_uint z = 0; z < 3u; z++) // for each edge from face "i"
			{
				SpEdgeMesh* edge1 = allEdgesObj1[edgesIndexesObj1[z]];

				if (!edge1->isBoundaryEdge())
					continue;

				Line3D line1;
				edge1->convert(&line1, transform1);

				for (sp_uint j = 0; j < facesIndexesMesh2Length; j++) // for each parallel face from mesh 3
				{
					Triangle3D face2;
					allFacesObj2[facesIndexesMesh2[j]]->convert(&face2, transform2);

					sp_uint* edgesIndexesObj2 = allFacesObj2[facesIndexesMesh2[j]]->edgesIndexes;

					for (sp_uint w = 0; w < 3u; w++) // for each edge from face "j"
					{
						SpEdgeMesh* edge2 = allEdgesObj2[edgesIndexesObj2[w]];

						if (!edge2->isBoundaryEdge())
							continue;

						Line3D line2;
						edge2->convert(&line2, transform2);

						Vec3 contact;
						if (line1.intersection(line2, &contact, 0.4f))
						{
							// add intersection point if not exists and is inside the faces
							if (!contains(intersectionPoints, intersectionPointsLength, contact, 0.009f)
								&& face1.isInside(contact) && face2.isInside(contact))
								intersectionPoints[intersectionPointsLength++] = contact;

							// check if vertex points from edge 1 is inside the face from mesh2
							if (!contains(intersectionPoints, intersectionPointsLength, line1.point1, 0.009f)
								&& face2.isInside(line1.point1))
								intersectionPoints[intersectionPointsLength++] = line1.point1;
							else
								if (!contains(intersectionPoints, intersectionPointsLength, line1.point2, 0.009f)
									&& face2.isInside(line1.point2))
									intersectionPoints[intersectionPointsLength++] = line1.point2;

							// check if vertex points from edge 2 is inside the face from mesh1
							if (face1.isInside(line2.point1)
								&& !contains(intersectionPoints, intersectionPointsLength, line2.point1, 0.009f))
								intersectionPoints[intersectionPointsLength++] = line2.point1;
							else
								if (face1.isInside(line2.point2)
									&& !contains(intersectionPoints, intersectionPointsLength, line2.point2, 0.009f))
									intersectionPoints[intersectionPointsLength++] = line2.point2;
						}
					}
				}
			}
		}
		*/
		sp_assert(intersectionPointsLength > MIN_INTERSECTION_POINTS, "InvalidOperationException");
		sp_assert(intersectionPointsLength < MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");

		details->type = SpCollisionType::FaceFace;
		rotate(transform1.orientation, allFacesObj1[facesIndexesMesh1[0]]->faceNormal, &details->collisionNormalObj1);
		details->collisionNormalObj2 = -details->collisionNormalObj1;

		for (sp_uint i = 0; i < intersectionPointsLength; i++)
		{
			details->centerContactPoint += intersectionPoints[i];
			details->contactPoints[i] = intersectionPoints[i];
		}
		details->contactPointsLength = intersectionPointsLength;
		details->centerContactPoint /= (sp_float)intersectionPointsLength;

		return true;
#undef MAX_INTERSECTION_POINTS
#undef MIN_INTERSECTION_POINTS
#undef MAX_PARALLEL_FACES
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

			for (sp_uint j = 0; j < vertex2->edgeLength(); j++)
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
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->get(details->vertexIndexObj1);
		const sp_uint edgesLengthObj1 = vertexMeshObj1->edgeLength();

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->get(details->vertexIndexObj2);
		const sp_uint edgesLengthObj2 = vertexMeshObj2->edgeLength();

		Vec3 contact;
		sp_float smallestDistance = SP_FLOAT_MAX;

		for (sp_uint i = 0; i < edgesLengthObj1; i++)
		{
			if (!vertexMeshObj1->edges(i)->isBoundaryEdge())
				continue;

			Line3D line1;
			vertexMeshObj1->edges(i)->convert(&line1, transform1);

			for (sp_uint j = 0; j < edgesLengthObj2; j++)
			{
				if (!vertexMeshObj2->edges(j)->isBoundaryEdge())
					continue;

				Line3D line2;
				vertexMeshObj2->edges(j)->convert(&line2, transform2);

				Vec3 p1, p2;
				sp_float sqDistance;
				line1.closestPoint(line2, &p1, &p2, &sqDistance);

				// if the distance is greater or the contact is far away, discard
				if (sqDistance >= smallestDistance || 
					!isCloseEnough(std::fabsf(sqDistance), ZERO_FLOAT, 0.4f))
					continue;

				// if the contact is at extreme of edge, it is vertex-edge collison...
				if (isCloseEnough(line2.point1, p2, ERROR_MARGIN_PHYSIC) || isCloseEnough(line2.point2, p2, ERROR_MARGIN_PHYSIC))
					continue;
				if (isCloseEnough(line1.point1, p1, ERROR_MARGIN_PHYSIC) || isCloseEnough(line1.point2, p1, ERROR_MARGIN_PHYSIC))
					continue;

				smallestDistance = sqDistance;
				contact = p1;
				line1.cross(line2, &details->collisionNormalObj1);
			}
		}

		if (isCloseEnough(smallestDistance, ZERO_FLOAT, 0.009f))
		{
			details->type = SpCollisionType::EdgeEdge;
			details->contactPointsLength = 1u;
			details->contactPoints[0] = contact;
			details->centerContactPoint = contact;

			normalize(&details->collisionNormalObj1);
			details->collisionNormalObj2 = -details->collisionNormalObj1;

			return true;
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

			for (sp_uint j = 0; j < vertexMeshObj2->edgeLength(); j++)
			{
				Line3D line;
				transformObj2.transform(vertexMeshObj2->value(), &line.point1);
				mesh2->vertex(vertexMeshObj2->edgeVertexIndex(j), transformObj2, &line.point2);
			
				if (line.intersection(triangle, &details->centerContactPoint))
				{
					Plane3D faceContact(triangle);
					
					details->contactPointsLength = ONE_UINT;
					if (faceContact.isBackFace(line.point1))
						details->contactPoints[0] = line.point1;
					else
						details->contactPoints[0] = line.point2;

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

			for (sp_uint j = 0; j < vertexMeshObj1->edgeLength(); j++)
			{
				Line3D line;
				transformObj1.transform(vertexMeshObj1->value(), &line.point1);
				mesh1->vertex(vertexMeshObj1->edgeVertexIndex(j), transformObj1, &line.point2);
				
				if (line.intersection(triangle, &details->centerContactPoint))
				{
					Plane3D faceContact(triangle);
					
					details->contactPointsLength = ONE_UINT;
					if (faceContact.isBackFace(line.point1))
						details->contactPoints[0] = line.point1;
					else
						details->contactPoints[0] = line.point2;

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

			vertexMeshObj1->findParallelFaces(faceAsPlane, transformObj1, parallelFacesIndexes, &parallelFacesIndexesLength, 0.1f);

			sp_assert(parallelFacesIndexesLength < MAX_PARALLEL_FACES, "IndexOutOfRangeException");

			Vec3 contacts[MAX_CONTACTS];
			sp_uint contactsLength = ZERO_UINT;

			// find which faces this edge cross and get the two contact points
			for (sp_uint i = 0; i < parallelFacesIndexesLength; i++)
			{
				SpFaceMesh* face = allFacesObj1[parallelFacesIndexes[i]];
				Triangle3D faceAsTriangle;
				face->convert(&faceAsTriangle, transformObj1);

				Line3D lines[3];
				faceAsTriangle.convert(lines);

				for (sp_uint j = 0; j < 3u; j++)
				{
					SpEdgeMesh* edgeMesh = face->edges(j);

					if (!edgeMesh->isBoundaryEdge())
						continue;

					if (lines[j].intersection(edge, &contacts[contactsLength], 0.2f))
						contactsLength++;

					if (faceAsTriangle.isInside(edge.point1, ERROR_MARGIN_PHYSIC))
						contacts[contactsLength++] = edge.point1;

					if (faceAsTriangle.isInside(edge.point2))
						contacts[contactsLength++] = edge.point2;
					
					sp_assert(contactsLength <= MAX_CONTACTS, "IndexOutOfRangeException");

					if (contactsLength == 2u)
						goto break_loops1;
				}
			}
			
		break_loops1:
			sp_assert(contactsLength <= MAX_CONTACTS, "IndexOutOfRangeException");
			
			details->type = SpCollisionType::EdgeFace;

			details->contactPointsLength = 2u;
			details->contactPoints[0] = contacts[0];
			details->contactPoints[1] = contacts[1];

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
#define MAX_PARALLEL_FACES 10u
#define MAX_CONTACTS 2u
			SpFaceMesh* faceCollisionObj2 = mesh2->faces->data()[faceIndexCollisionObj2];

			Line3D edge;
			transformObj1.transform(vertexMeshObj1->value(), &edge.point1);
			//mesh1->vertex(vertexMeshObj1->edgeVertexIndex(edgeVertexIndexObj1), transformObj1, &edge.point2);
			mesh1->vertex(edgeVertexIndexObj1, transformObj1, &edge.point2);

			Plane3D faceAsPlane;
			faceCollisionObj2->convert(&faceAsPlane, transformObj2);

			sp_uint parallelFacesIndexes[MAX_PARALLEL_FACES];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			// find all parallel faces
			vertexMeshObj2->findParallelFaces(faceAsPlane, transformObj2, parallelFacesIndexes, &parallelFacesIndexesLength, 0.1f);

			Vec3 contacts[MAX_CONTACTS];
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
			sp_assert(contactsLength <= MAX_CONTACTS, "IndexOutOfRangeException");

			details->type = SpCollisionType::EdgeFace;

			details->contactPointsLength = 2u;
			details->contactPoints[0] = contacts[0];
			details->contactPoints[1] = contacts[1];

			details->centerContactPoint = (contacts[0] + contacts[1]) * HALF_FLOAT;

			details->collisionNormalObj1 = -faceAsPlane.normalVector;
			details->collisionNormalObj2 = faceAsPlane.normalVector;

			return true;
#undef MAX_PARALLEL_FACES
#undef MAX_CONTACTS
		}

		return false;
	}

}
