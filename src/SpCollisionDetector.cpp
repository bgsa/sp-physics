#include "SpCollisionDetector.h"

namespace NAMESPACE_PHYSICS
{

	sp_bool SpCollisionDetector::hasPlaneCollision(sp_uint objIndex1, sp_uint objIndex2, SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		SpWorld* world = SpWorldManagerInstance->current();
		const SpMesh* mesh1 = world->mesh(world->collisionFeatures(objIndex1)->meshIndex);
		const SpMesh* mesh2 = world->mesh(world->collisionFeatures(objIndex2)->meshIndex);
		const Vec3 position2 = world->transforms(objIndex2)->position;
		SpVertexMesh* startingFrom = nullptr;
		
		if (details->vertexIndexObj2 != SP_UINT_MAX) // if the previous vertex was vertexIndexObj2, start searching from that vertex
			startingFrom = mesh2->vertexesMesh->get(details->vertexIndexObj2);

		SpVertexMesh* extremeVertex = mesh2->support(Vec3Down, details->cacheObj2->vertexes, startingFrom);

		details->vertexIndexObj2 = extremeVertex->index();

		const Vec3 vertex = details->cacheObj2->vertexes[extremeVertex->index()];

		if (vertex.y > ZERO_FLOAT)
			return false;

		cache->edgeIndexOnObj1 = false;
		cache->faceIndex = 0u; // take whaever face from plane

		// find penetrated edge to fill cache
		sp_float offset = ZERO_FLOAT;
		do_again:
		for (sp_uint i = 0; i < extremeVertex->edgeLength(); i++)
		{
			SpEdgeMesh* e = extremeVertex->edges(i);
			const Vec3 v1 = details->cacheObj2->vertexes[e->vertexIndex1];
			const Vec3 v2 = details->cacheObj2->vertexes[e->vertexIndex2];

			// check this edge cross the plane
			if ((v1.y + offset <= ZERO_FLOAT && v2.y + offset > ZERO_FLOAT)
				|| (v2.y + offset <= ZERO_FLOAT && v1.y + offset > ZERO_FLOAT))
			{
				cache->edgeIndex = e->index();
				details->depth = -(v1.y < ZERO_FLOAT ? v1.y : v2.y);
				break;
			}
		}

		if (cache->edgeIndex == SP_UINT_MAX)
		{
			offset += HALF_FLOAT;
			goto do_again;
		}

		details->collisionNormal = Vec3Up;

		sp_assert(cache->edgeIndex != SP_UINT_MAX, "ApplicationException");

		return true;
	}

	sp_bool SpCollisionDetector::hasCollisionCache(const SpMesh* mesh1, const SpMesh* mesh2, SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		if (cache->edgeIndexOnObj1)
		{
			SpEdgeMesh* edge = mesh1->edges->get(cache->edgeIndex);
			Line3D line;
			line.point1 = details->cacheObj1->vertexes[edge->vertexIndex1];
			line.point2 = details->cacheObj1->vertexes[edge->vertexIndex2];

			sp_uint* indexes = mesh2->faces->get(cache->faceIndex)->vertexesIndexes;
			Triangle3D face(
				details->cacheObj2->vertexes[indexes[0]],
				details->cacheObj2->vertexes[indexes[1]],
				details->cacheObj2->vertexes[indexes[2]]
			);

			Vec3 contact;
			if (line.intersection(face, &contact))
				return true;
		}
		else
		{
			SpEdgeMesh* edge = mesh2->edges->get(cache->edgeIndex);
			Line3D line;
			line.point1 = details->cacheObj1->vertexes[edge->vertexIndex1];
			line.point2 = details->cacheObj1->vertexes[edge->vertexIndex2];

			sp_uint* indexes = mesh1->faces->get(cache->faceIndex)->vertexesIndexes;
			Triangle3D face(
				details->cacheObj2->vertexes[indexes[0]],
				details->cacheObj2->vertexes[indexes[1]],
				details->cacheObj2->vertexes[indexes[2]]
			);

			Vec3 contact;
			if (line.intersection(face, &contact))
				return true;
		}

		return false;
	}

	sp_bool SpCollisionDetector::hasCollision(sp_uint objIndex1, sp_uint objIndex2, SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		if (objIndex1 == ZERO_UINT)
		{
			cache->searchOnObj1 = true;
			return hasPlaneCollision(objIndex1, objIndex2, details, cache);
		}

		SpWorld* world = SpWorldManagerInstance->current();
		const SpMesh* mesh1 = world->mesh(world->collisionFeatures(objIndex1)->meshIndex);
		const SpMesh* mesh2 = world->mesh(world->collisionFeatures(objIndex2)->meshIndex);
		
		if (cache->hasCache()) // check if cache is available
			hasCollisionCache(mesh1, mesh2, details, cache);

		const Vec3 position1 = world->transforms(objIndex1)->position;
		const Vec3 position2 = world->transforms(objIndex2)->position;

		Vec3 _direction;
		direction(position1, position2, _direction);
		
		SpVertexMesh* extremeVertex1 = mesh1->support(_direction, details->cacheObj1->vertexes, mesh1->vertexesMesh->get(0));
		SpVertexMesh* extremeVertex2 = mesh2->support(-_direction, details->cacheObj2->vertexes, mesh2->vertexesMesh->get(0));
		//SpVertexMesh* extremeVertex1 = mesh1->findExtremeVertexPoint(position2, details->cacheObj1, position1, mesh1->vertexesMesh->get(0));
		//SpVertexMesh* extremeVertex2 = mesh2->findExtremeVertexPoint(position1, details->cacheObj2, position2, mesh2->vertexesMesh->get(0));

		details->vertexIndexObj1 = extremeVertex1->index();
		details->vertexIndexObj2 = extremeVertex2->index();

		Vec3 closestPoint1 = details->cacheObj1->vertexes[extremeVertex1->index()];
		Vec3 closestPoint2 = details->cacheObj2->vertexes[extremeVertex2->index()];

		// check distance of vertexes
		Plane plane1(closestPoint1, _direction);
		Plane plane2(closestPoint2, -_direction);
		if (plane1.distance(closestPoint2) > ZERO_FLOAT && plane2.distance(closestPoint1) > ZERO_FLOAT)
			return false;

		// check edges from meshe's 1 vertex has collision with faces of mesh 2
		for (sp_uint i = 0; i < extremeVertex1->edgeLength(); i++)
		{
			SpEdgeMesh* edgeMesh1 = extremeVertex1->edges(i);
			Line3D line(
				details->cacheObj1->vertexes[edgeMesh1->vertexIndex1],
				details->cacheObj1->vertexes[edgeMesh1->vertexIndex2]
			);

			for (sp_uint j = 0; j < extremeVertex2->faceIndexLength(); j++)
			{
				SpFaceMesh* faceMesh2 = extremeVertex2->face(j);
				sp_uint* indexes = faceMesh2->vertexesIndexes;
			
				Triangle3D face(
					details->cacheObj2->vertexes[indexes[0]],
					details->cacheObj2->vertexes[indexes[1]],
					details->cacheObj2->vertexes[indexes[2]]
				);

				// check if has intersection
				Vec3 contact;
				if (line.intersection(face, &contact)) 
				{
					// store the result on cache
					cache->edgeIndexOnObj1 = true;
					cache->edgeIndex = edgeMesh1->index();
					cache->faceIndex = faceMesh2->index();

					face.normalFace(details->collisionNormal);

					Plane plane(face.point1, details->collisionNormal);
					details->depth = plane.distance(line.point1);
					if (details->depth > ZERO_FLOAT)
						details->depth = plane.distance(line.point2);

					return true;
				}	
			}
		}

		// check edges from meshe's 2 vertex has collision with faces of mesh 1
		for (sp_uint i = 0; i < extremeVertex2->edgeLength(); i++)
		{
			SpEdgeMesh* edgeMesh2 = extremeVertex2->edges(i);
			Line3D line(
				details->cacheObj2->vertexes[edgeMesh2->vertexIndex1],
				details->cacheObj2->vertexes[edgeMesh2->vertexIndex2]
			);

			for (sp_uint j = 0; j < extremeVertex1->faceIndexLength(); j++)
			{
				SpFaceMesh* faceMesh1 = extremeVertex1->face(j);
				sp_uint* indexes = faceMesh1->vertexesIndexes;

				Triangle3D face(
					details->cacheObj1->vertexes[indexes[0]],
					details->cacheObj1->vertexes[indexes[1]],
					details->cacheObj1->vertexes[indexes[2]]
				);

				// check if has intersection
				Vec3 contact;
				if (line.intersection(face, &contact))
				{
					// store the result on cache
					cache->edgeIndexOnObj1 = false;
					cache->edgeIndex = edgeMesh2->index();
					cache->faceIndex = faceMesh1->index();

					face.normalFace(details->collisionNormal);

					Plane plane(face.point1, details->collisionNormal);
					details->depth = plane.distance(line.point1);
					if (details->depth > ZERO_FLOAT)
						details->depth = plane.distance(line.point2);

					return true;
				}	
			}
		}

		return false;
	}

	void SpCollisionDetector::filterCollision(SpCollisionDetails* details) const
	{
		SpWorld* world = SpWorldManagerInstance->current();
		SpRigidBody3D* obj1Properties = world->rigidBody3D(details->objIndex1);
		SpRigidBody3D* obj2Properties = world->rigidBody3D(details->objIndex2);

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

			world->translate(details->objIndex1, obj1Properties->previousState.position() - obj1Properties->currentState.position());
			obj1Properties->currentState.position(obj1Properties->previousState.position());
			obj1Properties->currentState.velocity(Vec3Zeros);
			obj1Properties->currentState.acceleration(Vec3Zeros);
			obj1Properties->currentState.orientation(obj1Properties->previousState.orientation());
			obj1Properties->currentState.angularVelocity(Vec3Zeros);
			obj1Properties->currentState.torque(Vec3Zeros);

			world->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.velocity(Vec3Zeros);
			obj2Properties->currentState.acceleration(Vec3Zeros);
			obj2Properties->currentState.orientation(obj2Properties->previousState.orientation());
			obj2Properties->currentState.angularVelocity(Vec3Zeros);
			obj2Properties->currentState.torque(Vec3Zeros);

			details->ignoreCollision = true;
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			world->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.velocity(Vec3Zeros);
			obj2Properties->currentState.acceleration(Vec3Zeros);
			obj2Properties->currentState.orientation(obj2Properties->previousState.orientation());
			obj2Properties->currentState.angularVelocity(Vec3Zeros);
			obj2Properties->currentState.torque(Vec3Zeros);

			details->ignoreCollision = true;
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			world->translate(details->objIndex1, obj1Properties->previousState.position() - obj1Properties->currentState.position());
			obj1Properties->currentState.position(obj1Properties->previousState.position());
			obj1Properties->currentState.velocity(Vec3Zeros);
			obj1Properties->currentState.acceleration(Vec3Zeros);
			obj1Properties->currentState.orientation(obj1Properties->previousState.orientation());
			obj1Properties->currentState.angularVelocity(Vec3Zeros);
			obj1Properties->currentState.torque(Vec3Zeros);

			details->ignoreCollision = true;
			return;
		}
	}

	sp_bool SpCollisionDetector::findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, sp_uint* vertexIndexObj1, Vec3* contactPoint, SpCollisionDetectorCache* cache, SpMeshCache* cacheMesh1, SpMeshCache* cacheMesh2, SpCollisionDetails* details)
	{
		SpWorld* world = SpWorldManagerInstance->current();
		SpMesh* mesh1 = world->mesh(world->collisionFeatures(obj1Index)->meshIndex);
		SpEdgeMesh** edges = mesh1->edges->data();
		const sp_uint edgesLengthObj1 = mesh1->edges->length();

		SpMesh* mesh2 = world->mesh(world->collisionFeatures(obj2Index)->meshIndex);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		const sp_uint facesLengthObj2 = mesh2->faces->length();

		// for each edge from mesh 1
		for (sp_uint i = 0; i < edgesLengthObj1; i++)
		{
			if (!edges[i]->isBoundaryEdge())// check it is a boundary edge. if false, ignore
				continue;

			Line3D edge(
				cacheMesh1->vertexes[edges[i]->vertexIndex1], 
				cacheMesh1->vertexes[edges[i]->vertexIndex2]);
			
			// for each face from mesh 2
			for (sp_uint j = 0; j < facesLengthObj2; j++)
			{
				Triangle3D face(
					cacheMesh2->vertexes[allFacesObj2[j]->vertexesIndexes[0]],
					cacheMesh2->vertexes[allFacesObj2[j]->vertexesIndexes[1]],
					cacheMesh2->vertexes[allFacesObj2[j]->vertexesIndexes[2]]);

				if (!edge.intersection(face, contactPoint, DefaultErrorMargin))
					continue;

				// check which point crossed the face
				Plane plane(face);
				cache->distance = plane.distance(edge.point1);

				if (cache->distance < ZERO_FLOAT || NAMESPACE_FOUNDATION::isCloseEnough(cache->distance, ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
					vertexIndexObj1[0] = edges[i]->vertexIndex1;
				else
				{
					cache->distance = plane.distance(edge.point2);
					vertexIndexObj1[0] = edges[i]->vertexIndex2;
				}

				cache->edgeIndex = i;
				cache->faceIndex = j;

				return true;
			}
		}

		return false;
	}
	
	sp_bool SpCollisionDetector::collisionStatusCache(SpCollisionDetectorCache* cache, Vec3* contactPoint, SpCollisionDetails* details)
	{
		SpWorld* world = SpWorldManagerInstance->current();
		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);

		if (cache->searchOnObj1)
		{
			Line3D line;
			mesh2->edges->get(cache->edgeIndex)->convert(&line, *world->transforms(details->objIndex2));

			Triangle3D triangle;
			mesh1->faces->get(cache->faceIndex)->convert(&triangle, *world->transforms(details->objIndex1));

			sp_bool hasIntersection = line.intersection(triangle, contactPoint, DefaultErrorMargin);

			if (hasIntersection)
			{
				Plane p(triangle);
				cache->distance = p.distance(line.point1);

				if (cache->distance > ZERO_FLOAT)
					cache->distance = p.distance(line.point2);

				return true;
			}

			return false;
		}

		Line3D line;
		mesh1->edges->get(cache->edgeIndex)->convert(&line, *world->transforms(details->objIndex1));
		
		Triangle3D triangle;
		mesh2->faces->get(cache->faceIndex)->convert(&triangle, *world->transforms(details->objIndex2));
		
		sp_bool hasIntersection = line.intersection(triangle, contactPoint, DefaultErrorMargin);

		if (hasIntersection)
		{
			Plane p(triangle);
			cache->distance = p.distance(line.point1);

			if (cache->distance > ZERO_FLOAT)
				cache->distance = p.distance(line.point2);

			return true;
		}

		return false;

	}

	CollisionStatus SpCollisionDetector::collisionStatus(Vec3* contactPoint, SpCollisionDetectorCache* cache, SpCollisionDetails* details)
	{
		// check if the edge-face keep in contact,
		// if false, search on general collisionStatus
		if (cache->hasCache())
			if (collisionStatusCache(cache, contactPoint, details))
				return CollisionStatus::INSIDE;

		sp_bool hasCollision = findCollisionEdgeFace(details->objIndex1, details->objIndex2, &details->vertexIndexObj1, contactPoint, cache, details->cacheObj1, details->cacheObj2, details);
		
		if (hasCollision)
		{
			cache->searchOnObj1 = false;
			return CollisionStatus::INSIDE;
		}

		hasCollision = findCollisionEdgeFace(details->objIndex2, details->objIndex1, &details->vertexIndexObj2, contactPoint, cache, details->cacheObj2, details->cacheObj1, details);

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
		SpWorld* world = SpWorldManagerInstance->current();

		sp_assert(details->objIndex1 < world->objectsLength(), "InvalidArgumentException");
		sp_assert(details->objIndex2 < world->objectsLength(), "InvalidArgumentException");

		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);

		SpRigidBody3D* physicObj1 = world->rigidBody3D(details->objIndex1);
		SpRigidBody3D* physicObj2 = world->rigidBody3D(details->objIndex2);

		sp_bool isObj1Resting = physicObj1->isResting();
		sp_bool isObj2Resting = physicObj2->isResting();

		SpTransform* transformation1 = world->transforms(details->objIndex1);
		SpTransform* transformation2 = world->transforms(details->objIndex2);

		sp_bool hasIntersection = false;
		const sp_float _epsilon = 0.1f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float _diff = TEN_FLOAT;
		sp_float elapsedTime = details->timeStep * HALF_FLOAT;
		sp_float wasInsideAt = ZERO_FLOAT;
		
		SpCollisionDetectorCache cache;
		Vec3 contact;

		if (isObj1Resting && details->objIndex1 != ZERO_UINT) // ignore if obj1 is the plane
			details->cacheObj1->update(mesh1, transformation1);

		if (isObj2Resting)
			details->cacheObj2->update(mesh2, transformation2);

		while ((!hasIntersection || _diff > _epsilon) && _diff > 0.01f)
		{
			if (!isObj1Resting)
			{
				world->physicSimulator->backToTime(details->objIndex1);
				world->physicSimulator->integrator->execute(details->objIndex1, elapsedTime);
				details->cacheObj1->update(mesh1, transformation1);
			}

			if (!isObj2Resting)
			{
				world->physicSimulator->backToTime(details->objIndex2);
				world->physicSimulator->integrator->execute(details->objIndex2, elapsedTime);
				details->cacheObj2->update(mesh2, transformation2);
			}

			hasIntersection = hasCollision(details->objIndex1, details->objIndex2, details, &cache);

			_diff = sp_abs(previousElapsedTime - elapsedTime);
			previousElapsedTime = elapsedTime;

			if (!hasIntersection)
				elapsedTime += (_diff * HALF_FLOAT);
			else
			{
				wasInsideAt = elapsedTime;
				elapsedTime -= (_diff * HALF_FLOAT);
			}
		}

		if (!hasIntersection)
		{
			if (wasInsideAt == ZERO_FLOAT) 
			{
				details->ignoreCollision = true;
				return;
			}

			elapsedTime = wasInsideAt;

			if (!isObj1Resting)
			{
				world->physicSimulator->backToTime(details->objIndex1);
				world->physicSimulator->integrator->execute(details->objIndex1, elapsedTime);
				details->cacheObj1->update(mesh1, transformation1);
			}

			if (!isObj2Resting)
			{
				world->physicSimulator->backToTime(details->objIndex2);
				world->physicSimulator->integrator->execute(details->objIndex2, elapsedTime);
				details->cacheObj2->update(mesh2, transformation2);
			}

			SpVertexMesh* startingFrom;
			details->vertexIndexObj1 == SP_UINT_MAX
				? startingFrom = mesh1->vertexesMesh->get(0)
				: startingFrom = mesh1->vertexesMesh->get(details->vertexIndexObj1);

			details->vertexIndexObj1 = mesh1->findExtremeVertexPoint(transformation2->position, details->cacheObj1, transformation1->position, startingFrom)->index();
			
			cache.searchOnObj1 = false;
		}

		if (cache.searchOnObj1)
		{
			SpVertexMesh* startingFrom;
			details->vertexIndexObj1 == SP_UINT_MAX
				? startingFrom = mesh1->vertexesMesh->get(0)
				: startingFrom = mesh1->vertexesMesh->get(details->vertexIndexObj1);

			if (details->objIndex2 == ZERO_UINT) // if plane...
				details->vertexIndexObj1 = mesh1->support(Vec3Down, details->cacheObj1->vertexes, startingFrom)->index();
			else
				details->vertexIndexObj1 = mesh1->findExtremeVertexPoint(transformation2->position, details->cacheObj1, transformation1->position, startingFrom)->index();
		}
		else
		{
			SpVertexMesh* startingFrom;
			details->vertexIndexObj2 == SP_UINT_MAX
				? startingFrom = mesh2->vertexesMesh->get(0)
				: startingFrom = mesh2->vertexesMesh->get(details->vertexIndexObj2);

			if (details->objIndex1 == ZERO_UINT) // if plane...
				details->vertexIndexObj2 = mesh2->support(Vec3Down, details->cacheObj2->vertexes, startingFrom)->index();
			else
				details->vertexIndexObj2 = mesh2->findExtremeVertexPoint(transformation1->position, details->cacheObj2, transformation2->position, startingFrom)->index();
		}

		details->timeOfCollision = elapsedTime;

		sp_assert(details->objIndex1 < world->objectsLength(), "InvalidArgumentException");
		sp_assert(details->objIndex2 < world->objectsLength(), "InvalidArgumentException");
	}

	void SpCollisionDetector::collisionDetails(SpCollisionDetails* details)
	{
		SpWorld* world = SpWorldManagerInstance->current();
		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);

		//Timer timeDebug;
		//timeDebug.start();

		timeOfCollision(details);

		//sp_log_debug1sfnl("Time of Collision: ", timeDebug.elapsedTime());

		if (details->ignoreCollision) // if they are not colliding in geometry
			return;

		//timeDebug.update();

		fillCollisionDetails(details);

		//sp_log_debug1sfnl("Collision Details: ", timeDebug.elapsedTime());
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
#define MAX_PARALLEL_FACES 10
		// if obj 0 is plane
		if (details->objIndex1 == ZERO_UINT)
		{
			Plane surface(Vec3Zeros, Vec3Up);
			details->collisionNormal = Vec3Up;

			SpWorld* world = SpWorldManagerInstance->current();
			SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);
			SpVertexMesh* vertexMesh2 = mesh2->vertexesMesh->get(details->vertexIndexObj2);

			sp_uint facesIndexesMesh2[MAX_PARALLEL_FACES];
			sp_uint facesIndexesMesh2Length = ZERO_UINT;
			vertexMesh2->findParallelFaces(surface, details->cacheObj2, facesIndexesMesh2, &facesIndexesMesh2Length, ERROR_MARGIN_PHYSIC);
			sp_assert(facesIndexesMesh2Length < MAX_PARALLEL_FACES, "IndexOutOfRangeException");

			// check if face-face contact
			if (facesIndexesMesh2Length != ZERO_UINT)
			{
				details->type = SP_COLLISION_TYPE_FACE_FACE;
	
				// get the contact vertex and contact center
				for (sp_uint i = 0; i < facesIndexesMesh2Length; i++)
				{
					const SpFaceMesh* face = mesh2->faces->get(facesIndexesMesh2[i]);
					const sp_uint* indexes = face->vertexesIndexes;

					for (sp_uint j = 0; j < 3u; j++)
					{
						const Vec3 vertex = details->cacheObj2->vertexes[indexes[j]];

						if (!contains(details->contactPoints, details->contactPointsLength, vertex))
						{
							details->contactPoints[details->contactPointsLength++] = vertex;
							details->centerContactPoint += vertex;
						}
					}
				}
				details->centerContactPoint /= (sp_float)details->contactPointsLength;
				return;
			}

			const Vec3 vertex = details->cacheObj2->vertexes[vertexMesh2->index()];

			// check edge-face collision
			for (sp_uint i = 0; i < vertexMesh2->edgeLength(); i++)
			{
				SpEdgeMesh* vm = vertexMesh2->edges(i);
				Vec3 vertex2;

				if (vertexMesh2->index() == vm->vertexIndex1)
					vertex2 = details->cacheObj2->vertexes[vm->vertexIndex2];
				else
					vertex2 = details->cacheObj2->vertexes[vm->vertexIndex1];
				
				if (NAMESPACE_FOUNDATION::isCloseEnough(surface.distance(vertex2), ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
				{
					details->type = SP_COLLISION_TYPE_EDGE_FACE;
					details->contactPointsLength = 2u;
					details->contactPoints[0] = vertex;
					details->contactPoints[1] = vertex2;
					details->centerContactPoint = (vertex + vertex2) * HALF_FLOAT;
					return;
				}
			}

			// no face-face, no edge-face, so vertex-face
			details->type = SP_COLLISION_TYPE_VERTEX_FACE;
			details->contactPointsLength = 1u;
			details->contactPoints[0] = vertex;
			details->centerContactPoint = vertex;
			return;
		}

		// if obj 1 is plane
		if (details->objIndex2 == ZERO_UINT)
		{
			//TODO: FAZER
			//sp_assert(false, "FAZER!!");
		}
#undef MAX_PARALLEL_FACES

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
		details->ignoreCollision = true;
		return;
		
		/*
		SpWorld* world = SpWorldManagerInstance->current();
		Wavefront::SpWavefrontExporter exporter;
		exporter.write(*world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex), *world->transforms(details->objIndex1), "mesh1", "red");
		exporter.write(*world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex), *world->transforms(details->objIndex2), "mesh2", "blue");
		exporter.save("temp.obj");
		*/

		sp_assert(details->ignoreCollision == false, "ApplicationException");
		sp_assert(details->type != SP_COLLISION_TYPE_NONE, "ApplicationException");
	}

	sp_bool SpCollisionDetector::areMovingAway(sp_uint objIndex1, sp_uint objIndex2) const
	{
		SpWorld* world = SpWorldManagerInstance->current();
		const SpRigidBody3D* obj1Properties = world->rigidBody3D(objIndex1);
		const SpRigidBody3D* obj2Properties = world->rigidBody3D(objIndex2);

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
		SpWorld* world = SpWorldManagerInstance->current();
		
		const SpTransform transform1 = *world->transforms(details->objIndex1);
		const SpTransform transform2 = *world->transforms(details->objIndex2);

		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpVertexMesh* vertex1 = mesh1->vertexesMesh->get(details->vertexIndexObj1);
		
		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);
		SpVertexMesh* vertex2 = mesh2->vertexesMesh->get(details->vertexIndexObj2);
		
		sp_uint facesIndexesMesh1[MAX_PARALLEL_FACES];
		sp_uint facesIndexesMesh1Length = ZERO_UINT;
		sp_uint facesIndexesMesh2[MAX_PARALLEL_FACES];
		sp_uint facesIndexesMesh2Length = ZERO_UINT;

		vertex1->findParallelFaces(vertex2, details->cacheObj1, details->cacheObj2, facesIndexesMesh1, &facesIndexesMesh1Length , facesIndexesMesh2, &facesIndexesMesh2Length, ERROR_MARGIN_PHYSIC);

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
			Triangle3D triangleFace1(
				details->cacheObj1->vertexes[face1->vertexesIndexes[0]],
				details->cacheObj1->vertexes[face1->vertexesIndexes[1]],
				details->cacheObj1->vertexes[face1->vertexesIndexes[2]]
			);
			
			for (sp_uint j = 0; j < facesIndexesMesh2Length; j++)
			{
				SpFaceMesh* face2 = allFacesObj2[facesIndexesMesh2[j]];
				Triangle3D triangleFace2(
					details->cacheObj2->vertexes[face2->vertexesIndexes[0]],
					details->cacheObj2->vertexes[face2->vertexesIndexes[1]],
					details->cacheObj2->vertexes[face2->vertexesIndexes[2]]
				);

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace2.point1, ERROR_MARGIN_PHYSIC)
					&& triangleFace1.isInside(triangleFace2.point1, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace2.point1;
					vertexIndexesObj2[vertexIndexesObj2Length++] = face2->vertexesIndexes[0];
				}

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace2.point2, ERROR_MARGIN_PHYSIC)
					&& triangleFace1.isInside(triangleFace2.point2, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace2.point2;
					vertexIndexesObj2[vertexIndexesObj2Length++] = face2->vertexesIndexes[1];
				}

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace2.point3, ERROR_MARGIN_PHYSIC)
					&& triangleFace1.isInside(triangleFace2.point3, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace2.point3;
					vertexIndexesObj2[vertexIndexesObj2Length++] = face2->vertexesIndexes[2];
				}

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace1.point1, ERROR_MARGIN_PHYSIC)
					&& triangleFace2.isInside(triangleFace1.point1, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace1.point1;
					vertexIndexesObj1[vertexIndexesObj1Length++] = face1->vertexesIndexes[0];
				}

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace1.point2, ERROR_MARGIN_PHYSIC)
					&& triangleFace2.isInside(triangleFace1.point2, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace1.point2;
					vertexIndexesObj1[vertexIndexesObj1Length++] = face1->vertexesIndexes[1];
				}

				if (!contains(intersectionPoints, intersectionPointsLength, triangleFace1.point3, ERROR_MARGIN_PHYSIC)
					&& triangleFace2.isInside(triangleFace1.point3, ERROR_MARGIN_PHYSIC))
				{
					intersectionPoints[intersectionPointsLength++] = triangleFace1.point3;
					vertexIndexesObj1[vertexIndexesObj1Length++] = face1->vertexesIndexes[2];
				}
			}
		}

		sp_assert(intersectionPointsLength < MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");
		sp_assert(vertexIndexesObj1Length  < MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");
		sp_assert(vertexIndexesObj2Length  < MAX_INTERSECTION_POINTS, "IndexOutOfRangeException");

		// there is no vertex on face, try edges-edge intersections with faces
		// get all vertexes from the faces to compare edge-edge intersection to build face-face intersection
		if (vertexIndexesObj1Length == ZERO_UINT && vertexIndexesObj2Length == ZERO_UINT)
		{
			for (sp_uint i = 0; i < facesIndexesMesh1Length; i++)
			{
				sp_uint* vertexesFaceIndexes = allFacesObj1[facesIndexesMesh1[i]]->vertexesIndexes;
			
				if (!NAMESPACE_FOUNDATION::contains(vertexIndexesObj1, vertexIndexesObj1Length, vertexesFaceIndexes[0]))
					vertexIndexesObj1[vertexIndexesObj1Length++] = vertexesFaceIndexes[0];

				if (!NAMESPACE_FOUNDATION::contains(vertexIndexesObj1, vertexIndexesObj1Length, vertexesFaceIndexes[1]))
					vertexIndexesObj1[vertexIndexesObj1Length++] = vertexesFaceIndexes[1];

				if (!NAMESPACE_FOUNDATION::contains(vertexIndexesObj1, vertexIndexesObj1Length, vertexesFaceIndexes[2]))
					vertexIndexesObj1[vertexIndexesObj1Length++] = vertexesFaceIndexes[2];
			}

			for (sp_uint i = 0; i < facesIndexesMesh2Length; i++)
			{
				sp_uint* vertexesFaceIndexes = allFacesObj2[facesIndexesMesh2[i]]->vertexesIndexes;

				if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, vertexIndexesObj2Length, vertexesFaceIndexes[0]))
					vertexIndexesObj2[vertexIndexesObj2Length++] = vertexesFaceIndexes[0];

				if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, vertexIndexesObj2Length, vertexesFaceIndexes[1]))
					vertexIndexesObj2[vertexIndexesObj2Length++] = vertexesFaceIndexes[1];

				if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, vertexIndexesObj2Length, vertexesFaceIndexes[2]))
					vertexIndexesObj2[vertexIndexesObj2Length++] = vertexesFaceIndexes[2];
			}
		}
		else
		{
			// if only one vertex is inside face, ...
			if (vertexIndexesObj1Length == ZERO_UINT && vertexIndexesObj2Length == ONE_UINT)
			{
				for (sp_uint i = 0; i < facesIndexesMesh1Length; i++)
				{
					sp_uint* vertexesFaceIndexes = allFacesObj1[facesIndexesMesh1[i]]->vertexesIndexes;

					if (!NAMESPACE_FOUNDATION::contains(vertexIndexesObj1, vertexIndexesObj1Length, vertexesFaceIndexes[0]))
						vertexIndexesObj1[vertexIndexesObj1Length++] = vertexesFaceIndexes[0];

					if (!NAMESPACE_FOUNDATION::contains(vertexIndexesObj1, vertexIndexesObj1Length, vertexesFaceIndexes[1]))
						vertexIndexesObj1[vertexIndexesObj1Length++] = vertexesFaceIndexes[1];

					if (!NAMESPACE_FOUNDATION::contains(vertexIndexesObj1, vertexIndexesObj1Length, vertexesFaceIndexes[2]))
						vertexIndexesObj1[vertexIndexesObj1Length++] = vertexesFaceIndexes[2];
				}
			}
			else // if only one vertex is inside face, ...
			if (vertexIndexesObj1Length == ONE_UINT && vertexIndexesObj2Length == ZERO_UINT)
			{
				for (sp_uint i = 0; i < facesIndexesMesh2Length; i++)
				{
					sp_uint* vertexesFaceIndexes = allFacesObj2[facesIndexesMesh2[i]]->vertexesIndexes;

					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, vertexIndexesObj2Length, vertexesFaceIndexes[0]))
						vertexIndexesObj2[vertexIndexesObj2Length++] = vertexesFaceIndexes[0];

					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, vertexIndexesObj2Length, vertexesFaceIndexes[1]))
						vertexIndexesObj2[vertexIndexesObj2Length++] = vertexesFaceIndexes[1];

					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, vertexIndexesObj2Length, vertexesFaceIndexes[2]))
						vertexIndexesObj2[vertexIndexesObj2Length++] = vertexesFaceIndexes[2];
				}
			}
		}

		// for all edge from inside vertexes ...
		for (sp_uint i = 0; i < vertexIndexesObj1Length; i++)
		{
			const SpVertexMesh* vertexI = mesh1->vertexesMesh->get(vertexIndexesObj1[i]);
			
			Line3D line1(details->cacheObj1->vertexes[vertexI->index()], Vec3());
			
			for (sp_uint i2 = 0; i2 < vertexI->edgeLength(); i2++)
			{
				const SpVertexMesh* vertexI2 = mesh1->vertexesMesh->get(vertexI->edgeVertexIndex(i2));
				
				sp_uint eindex1 = mesh1->findEdge(vertexI->index(), vertexI->edgeVertexIndex(i2));
				SpEdgeMesh* e1 = mesh1->edges->get(eindex1);
				if (!e1->isBoundaryEdge())
					continue;
				
				line1.point2 = details->cacheObj1->vertexes[vertexI2->index()];

				for (sp_uint j = 0; j < vertexIndexesObj2Length; j++)
				{
					const SpVertexMesh* vertexJ = mesh2->vertexesMesh->get(vertexIndexesObj2[j]);
					
					Line3D line2(details->cacheObj1->vertexes[vertexJ->index()], Vec3());

					for (sp_uint j2 = 0; j2 < vertexJ->edgeLength(); j2++)
					{
						const SpVertexMesh* vertexJ2 = mesh2->vertexesMesh->get(vertexJ->edgeVertexIndex(j2));
					
						sp_uint eindex = mesh2->findEdge(vertexJ->index(), vertexJ->edgeVertexIndex(j2));
						SpEdgeMesh* e = mesh2->edges->get(eindex);
						if (!e->isBoundaryEdge())
							continue;
					
						line2.point2 = details->cacheObj2->vertexes[vertexJ2->index()];

						// if the edge intersect on the plane, get the intersection point
						Vec3 contact;
						if (line1.intersection(line2, &contact, ERROR_MARGIN_PHYSIC))
							if (!contains(intersectionPoints, intersectionPointsLength, contact, 0.09f))
								intersectionPoints[intersectionPointsLength++] = contact;
					}
				}
			}
		}

		if (intersectionPointsLength < 2u)
			return false;

		// all facesIndexesMesh1 are parallel, so get the first one
		rotate(transform1.orientation, allFacesObj1[facesIndexesMesh1[0]]->faceNormal, details->collisionNormal);

		if (intersectionPointsLength == 2u)
		{
			details->type = SP_COLLISION_TYPE_EDGE_FACE;
			details->contactPointsLength = 2u;
			details->contactPoints[0u] = intersectionPoints[0u];
			details->contactPoints[1u] = intersectionPoints[1u];
			details->centerContactPoint = (intersectionPoints[0u] + intersectionPoints[1u]) * HALF_FLOAT;
			return true;
		}

		sp_assert(intersectionPointsLength > MIN_INTERSECTION_POINTS, "InvalidOperationException");

		details->type = SP_COLLISION_TYPE_FACE_FACE;
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

	sp_bool SpCollisionDetector::isEdgeFaceCollision(SpVertexMesh* vertex1, SpVertexMesh* vertex2, const SpMeshCache* cacheMesh1, const SpMeshCache* cacheMesh2, sp_uint* faceIndexObj1, sp_uint* vertexIndexObj2) const
	{
		SpMesh* mesh2 = vertex2->mesh();
	
		Line3D edge2(cacheMesh2->vertexes[vertex2->index()], Vec3());

		for (sp_uint i = 0; i < vertex1->faceIndexLength(); i++)
		{
			sp_uint* vertexesIndexFaceObj1 = vertex1->face(i)->vertexesIndexes;
			Triangle3D face1AsTriangle(
				cacheMesh1->vertexes[vertexesIndexFaceObj1[0]],
				cacheMesh1->vertexes[vertexesIndexFaceObj1[1]],
				cacheMesh1->vertexes[vertexesIndexFaceObj1[2]]
			);

			Plane face1AsPlane(face1AsTriangle);

			for (sp_uint j = 0; j < vertex2->edgeLength(); j++)
			{
				edge2.point2 = cacheMesh2->vertexes[vertex2->edgeVertexIndex(j)];

				// the edge and face normal should be perpendicular for edge-face collision
				if (!edge2.isPerpendicular(face1AsPlane.normalVector, 0.1f))
					continue;

				const sp_float distancePoint1 = sp_abs(face1AsPlane.distance(edge2.point1));
				
				if (!NAMESPACE_FOUNDATION::isCloseEnough(distancePoint1, ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
					continue;

				const sp_float distancePoint2 = sp_abs(face1AsPlane.distance(edge2.point2));

				if (!NAMESPACE_FOUNDATION::isCloseEnough(distancePoint1, distancePoint2, ERROR_MARGIN_PHYSIC))
					continue;

				if (!face1AsTriangle.isInside(edge2.point1, ERROR_MARGIN_PHYSIC) 
					&& !face1AsTriangle.isInside(edge2.point2, ERROR_MARGIN_PHYSIC))
					continue;

				faceIndexObj1[0] = vertex1->faceIndex(i);
				vertexIndexObj2[0] = vertex2->edgeVertexIndex(j);
				return true;
			}
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeEdgeCollision(SpCollisionDetails* details) const
	{
		SpWorld* world = SpWorldManagerInstance->current();

		const Vec3* allVertexesObj1 = details->cacheObj1->vertexes;
		const Vec3* allVertexesObj2 = details->cacheObj2->vertexes;

		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->get(details->vertexIndexObj1);
		const sp_uint edgesLengthObj1 = vertexMeshObj1->edgeLength();

		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->get(details->vertexIndexObj2);
		const sp_uint edgesLengthObj2 = vertexMeshObj2->edgeLength();

		Vec3 contact;
		sp_float smallestDistance = SP_FLOAT_MAX;

		for (sp_uint i = 0; i < edgesLengthObj1; i++)
		{
			const SpEdgeMesh* edgeObj1 = vertexMeshObj1->edges(i);

			if (!edgeObj1->isBoundaryEdge())
				continue;

			Line3D line1(
				allVertexesObj1[edgeObj1->vertexIndex1],
				allVertexesObj1[edgeObj1->vertexIndex2]
			);

			for (sp_uint j = 0; j < edgesLengthObj2; j++)
			{
				const SpEdgeMesh* edgeObj2 = vertexMeshObj2->edges(j);

				if (!edgeObj2->isBoundaryEdge())
					continue;

				Line3D line2(
					allVertexesObj2[edgeObj2->vertexIndex1],
					allVertexesObj2[edgeObj2->vertexIndex2]
				);

				Vec3 p1, p2;
				sp_float sqDistance;
				line1.closestPoint(line2, &p1, &p2, &sqDistance);

				// if the distance is greater or the contact is far away, discard
				if (sqDistance >= smallestDistance ||
					!NAMESPACE_FOUNDATION::isCloseEnough(sp_abs(sqDistance), ZERO_FLOAT, 0.4f))
					continue;

				// if the contact is at extreme of edge, it is vertex-edge collison...
				if (NAMESPACE_FOUNDATION::isCloseEnough(sqDistance, ZERO_FLOAT, 0.009f))
				{
					if (isCloseEnough(line2.point1, p2, ERROR_MARGIN_PHYSIC) 
						|| isCloseEnough(line2.point2, p2, ERROR_MARGIN_PHYSIC))
					{
						details->type = SP_COLLISION_TYPE_VERTEX_EDGE;
						details->contactPointsLength = 1u;
						details->contactPoints[0] = p2;
						details->centerContactPoint = p2;

						// get the first face from edge and this will be the collision normal
						SpFaceMesh* face = mesh1->faces->get(edgeObj1->faces[0]);
						SpTransform* transformation = world->transforms(details->objIndex1);
						rotate(transformation->orientation, face->faceNormal, details->collisionNormal);

						return true;
					}

					if (isCloseEnough(line1.point1, p1, ERROR_MARGIN_PHYSIC) 
						|| isCloseEnough(line1.point2, p1, ERROR_MARGIN_PHYSIC))
					{
						details->type = SP_COLLISION_TYPE_VERTEX_EDGE;
						details->contactPointsLength = 1u;
						details->contactPoints[0] = p1;
						details->centerContactPoint = p1;
						
						// get the first face from edge and this will be the collision normal
						SpFaceMesh* face = mesh2->faces->get(edgeObj2->faces[0]);
						SpTransform* transformation = world->transforms(details->objIndex2);
						rotate(transformation->orientation, face->faceNormal, details->collisionNormal);

						return true;
					}
				}

				smallestDistance = sqDistance;
				contact = p1;
				line1.cross(line2, details->collisionNormal);
			}
		}

		if (NAMESPACE_FOUNDATION::isCloseEnough(smallestDistance, ZERO_FLOAT, 0.009f))
		{
			details->type = SP_COLLISION_TYPE_EDGE_EDGE;
			details->contactPointsLength = 1u;
			details->contactPoints[0] = contact;
			details->centerContactPoint = contact;

			normalize(details->collisionNormal);

			return true;
		}

		return false;
	}

	sp_bool SpCollisionDetector::isVertexFaceCollision(SpCollisionDetails* details) const
	{
		SpWorld* world = SpWorldManagerInstance->current();

		const Vec3* allVertexesObj1 = details->cacheObj1->vertexes;
		const Vec3* allVertexesObj2 = details->cacheObj2->vertexes;

		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);
		
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];

		SpFaceMesh** facesObj1 = vertexMeshObj1->mesh()->faces->data();
		SpFaceMesh** facesObj2 = vertexMeshObj2->mesh()->faces->data();

		Line3D line(allVertexesObj2[vertexMeshObj2->index()], Vec3());

		for (sp_uint i = 0; i < vertexMeshObj1->faceIndexLength(); i++)
		{
			sp_uint* vertexesIndexesFaceObj1 = vertexMeshObj1->face(i)->vertexesIndexes;
			Triangle3D triangle(
				allVertexesObj1[vertexesIndexesFaceObj1[0]],
				allVertexesObj1[vertexesIndexesFaceObj1[1]],
				allVertexesObj1[vertexesIndexesFaceObj1[2]]
			);

			for (sp_uint j = 0; j < vertexMeshObj2->edgeLength(); j++)
			{
				line.point2 = allVertexesObj2[vertexMeshObj2->edgeVertexIndex(j)];
			
				if (line.intersection(triangle, &details->centerContactPoint, ERROR_MARGIN_PHYSIC))
				{
					Plane faceContact(triangle);
					
					details->contactPointsLength = ONE_UINT;

					const sp_float distance = faceContact.distance(line.point1);

					if (distance < ZERO_FLOAT + ERROR_MARGIN_PHYSIC) // if point is back-face, ...
						details->contactPoints[0] = line.point1;
					else
						details->contactPoints[0] = line.point2;

					details->type = SP_COLLISION_TYPE_VERTEX_FACE;
					details->collisionNormal = faceContact.normalVector;

					return true;
				}
			}
		}

		line.point1 = allVertexesObj1[vertexMeshObj1->index()];

		for (sp_uint i = 0; i < vertexMeshObj2->faceIndexLength(); i++)
		{
			sp_uint* vertexesIndexesFaceObj2 = vertexMeshObj2->face(i)->vertexesIndexes;
			Triangle3D triangle(
				allVertexesObj2[vertexesIndexesFaceObj2[0]],
				allVertexesObj2[vertexesIndexesFaceObj2[1]],
				allVertexesObj2[vertexesIndexesFaceObj2[2]]
			);

			for (sp_uint j = 0; j < vertexMeshObj1->edgeLength(); j++)
			{
				line.point2 = allVertexesObj1[vertexMeshObj1->edgeVertexIndex(j)];
				
				if (line.intersection(triangle, &details->centerContactPoint, ERROR_MARGIN_PHYSIC))
				{
					Plane faceContact(triangle);

					details->type = SP_COLLISION_TYPE_VERTEX_FACE;
					details->collisionNormal = faceContact.normalVector;
					details->contactPointsLength = ONE_UINT;

					if (faceContact.isBackFace(line.point1))
						details->contactPoints[0] = line.point1;
					else
						details->contactPoints[0] = line.point2;

					return true;
				}
			}
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeFaceCollisionObj1(SpCollisionDetails* details) const
	{
		SpWorld* world = SpWorldManagerInstance->current();

		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpFaceMesh** allFacesObj1 = mesh1->faces->data();
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];

		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];

		sp_uint faceIndexCollisionObj1;
		sp_uint edgeVertexIndexObj2;

		if (isEdgeFaceCollision(vertexMeshObj1, vertexMeshObj2, details->cacheObj1, details->cacheObj1, &faceIndexCollisionObj1, &edgeVertexIndexObj2))
		{
#define MAX_PARALLEL_FACES 10u
#define MAX_CONTACTS 2u
			SpFaceMesh* faceCollisionObj1 = mesh1->faces->data()[faceIndexCollisionObj1];

			Line3D edgeObj2(
				details->cacheObj2->vertexes[vertexMeshObj2->index()],
				details->cacheObj2->vertexes[edgeVertexIndexObj2]
			);
			
			Plane faceAsPlane(
				details->cacheObj1->vertexes[faceCollisionObj1->vertexesIndexes[0]],
				details->cacheObj1->vertexes[faceCollisionObj1->vertexesIndexes[1]],
				details->cacheObj1->vertexes[faceCollisionObj1->vertexesIndexes[2]]
			);
			
			sp_uint parallelFacesIndexes[MAX_PARALLEL_FACES];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			vertexMeshObj1->findParallelFaces(faceAsPlane, details->cacheObj1, parallelFacesIndexes, &parallelFacesIndexesLength, 0.1f);

			sp_assert(parallelFacesIndexesLength < MAX_PARALLEL_FACES, "IndexOutOfRangeException");

			Vec3 contacts[MAX_CONTACTS];
			sp_uint contactsLength = ZERO_UINT;

			// find which faces this edge cross and get the two contact points
			for (sp_uint i = 0; i < parallelFacesIndexesLength; i++)
			{
				SpFaceMesh* face = allFacesObj1[parallelFacesIndexes[i]];
				Triangle3D faceAsTriangle(
					details->cacheObj1->vertexes[face->vertexesIndexes[0]],
					details->cacheObj1->vertexes[face->vertexesIndexes[1]],
					details->cacheObj1->vertexes[face->vertexesIndexes[2]]
				);
				
				Line3D lines[3];
				faceAsTriangle.convert(lines);

				for (sp_uint j = 0; j < 3u; j++)
				{
					SpEdgeMesh* edgeMesh = face->edges(j);

					if (!edgeMesh->isBoundaryEdge())
						continue;

					if (faceAsTriangle.isInside(edgeObj2.point1, ERROR_MARGIN_PHYSIC)
						&& !contains(contacts, contactsLength, edgeObj2.point1, 0.2f))
						contacts[contactsLength++] = edgeObj2.point1;

					if (faceAsTriangle.isInside(edgeObj2.point2, ERROR_MARGIN_PHYSIC)
						&& !contains(contacts, contactsLength, edgeObj2.point2, 0.2f))
						contacts[contactsLength++] = edgeObj2.point2;

					if (contactsLength == MAX_CONTACTS)
						goto break_loops1;

					if (lines[j].intersection(edgeObj2, &contacts[contactsLength], 0.2f)
						&& !contains(contacts, contactsLength, contacts[contactsLength], ERROR_MARGIN_PHYSIC))
						contactsLength++;

					if (contactsLength == MAX_CONTACTS)
						goto break_loops1;
				}
			}
			
		break_loops1:

			if (contactsLength != MAX_CONTACTS)
				return false;

			sp_assert(contactsLength == MAX_CONTACTS, "IndexOutOfRangeException");
			
			details->type = SP_COLLISION_TYPE_EDGE_FACE;
			details->contactPointsLength = 2u;
			details->contactPoints[0] = contacts[0];
			details->contactPoints[1] = contacts[1];
			details->centerContactPoint = (contacts[0] + contacts[1]) * HALF_FLOAT;
			details->collisionNormal = faceAsPlane.normalVector;

			return true;
#undef MAX_PARALLEL_FACES
#undef MAX_CONTACTS
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeFaceCollisionObj2(SpCollisionDetails* details) const
	{
		SpWorld* world = SpWorldManagerInstance->current();

		SpMesh* mesh1 = world->mesh(world->collisionFeatures(details->objIndex1)->meshIndex);
		SpFaceMesh** allFacesObj1 = mesh1->faces->data();
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		Vec3* allVertexesObj1 = details->cacheObj1->vertexes;

		SpMesh* mesh2 = world->mesh(world->collisionFeatures(details->objIndex2)->meshIndex);
		const SpTransform transformObj2 = *world->transforms(details->objIndex2);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];
		Vec3* allVertexesObj2 = details->cacheObj2->vertexes;

		sp_uint faceIndexCollisionObj2;
		sp_uint edgeVertexIndexObj1;

		if (isEdgeFaceCollision(vertexMeshObj2, vertexMeshObj1, details->cacheObj2, details->cacheObj1, &faceIndexCollisionObj2, &edgeVertexIndexObj1))
		{
#define MAX_PARALLEL_FACES 10u
#define MAX_CONTACTS 2u
			SpFaceMesh* faceCollisionObj2 = mesh2->faces->data()[faceIndexCollisionObj2];

			Line3D edge(
				allVertexesObj1[vertexMeshObj1->index()],
				allVertexesObj1[edgeVertexIndexObj1]
			);
			
			Plane faceAsPlane(
				allVertexesObj1[faceCollisionObj2->vertexesIndexes[0]],
				allVertexesObj1[faceCollisionObj2->vertexesIndexes[1]],
				allVertexesObj1[faceCollisionObj2->vertexesIndexes[2]]
			);

			sp_uint parallelFacesIndexes[MAX_PARALLEL_FACES];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			// find all parallel faces
			vertexMeshObj2->findParallelFaces(faceAsPlane, details->cacheObj2, parallelFacesIndexes, &parallelFacesIndexesLength, 0.1f);

			sp_assert(parallelFacesIndexesLength < MAX_PARALLEL_FACES, "ApplicationException");

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

					Line3D line(
						allVertexesObj2[edgeMesh->vertexIndex1],
						allVertexesObj2[edgeMesh->vertexIndex2]
					);
					
					Vec3 contact;
					if (line.intersection(edge, &contact))
					{
						contacts[contactsLength++] = contact;
						
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

			details->type = SP_COLLISION_TYPE_EDGE_FACE;
			details->contactPointsLength = 2u;
			details->contactPoints[0] = contacts[0];
			details->contactPoints[1] = contacts[1];
			details->centerContactPoint = (contacts[0] + contacts[1]) * HALF_FLOAT;
			details->collisionNormal = faceAsPlane.normalVector;

			return true;
#undef MAX_PARALLEL_FACES
#undef MAX_CONTACTS
		}

		return false;
	}

}
