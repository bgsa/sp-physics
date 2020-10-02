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
			obj1Properties->currentState.orientation(obj1Properties->previousState.orientation());
			obj1Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.torque(Vec3(ZERO_FLOAT));

			simulator->translate(details->objIndex2, obj2Properties->previousState.position() - obj2Properties->currentState.position());
			obj2Properties->currentState.position(obj2Properties->previousState.position());
			obj2Properties->currentState.velocity(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.acceleration(Vec3(ZERO_FLOAT));
			obj2Properties->currentState.orientation(obj2Properties->previousState.orientation());
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
			obj2Properties->currentState.orientation(obj2Properties->previousState.orientation());
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
			obj1Properties->currentState.orientation(obj1Properties->previousState.orientation());
			obj1Properties->currentState.angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->currentState.torque(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}
	}

	sp_bool SpCollisionDetector::findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, sp_uint* vertexIndexObj1, Vec3* contactPoint, SpCollisionDetectorCache* cache, SpMeshCache* cacheMesh1, SpMeshCache* cacheMesh2, SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(obj1Index)->meshIndex);
		SpEdgeMesh** edges = mesh1->edges->data();
		const sp_uint edgesLengthObj1 = mesh1->edges->length();

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(obj2Index)->meshIndex);
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

				//allFacesObj2[j]->convert(&face, transformObj2);

				if (!edge.intersection(face, contactPoint, DefaultErrorMargin))
					continue;

				// check which point crossed the face
				Plane3D plane(face);
				cache->distance = plane.distance(edge.point1);

				if (cache->distance < ZERO_FLOAT || isCloseEnough(cache->distance, ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
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
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);

		if (cache->searchOnObj1)
		{
			Line3D line;
			mesh2->edges->get(cache->edgeIndex)->convert(&line, *simulator->transforms(details->objIndex2));

			Triangle3D triangle;
			mesh1->faces->get(cache->faceIndex)->convert(&triangle, *simulator->transforms(details->objIndex1));

			sp_bool hasIntersection = line.intersection(triangle, contactPoint, DefaultErrorMargin);

			if (hasIntersection)
			{
				Plane3D p(triangle);
				cache->distance = p.distance(line.point1);

				if (cache->distance > ZERO_FLOAT)
					cache->distance = p.distance(line.point2);

				return true;
			}

			return false;
		}

		Line3D line;
		mesh1->edges->get(cache->edgeIndex)->convert(&line, *simulator->transforms(details->objIndex1));
		
		Triangle3D triangle;
		mesh2->faces->get(cache->faceIndex)->convert(&triangle, *simulator->transforms(details->objIndex2));
		
		sp_bool hasIntersection = line.intersection(triangle, contactPoint, DefaultErrorMargin);

		if (hasIntersection)
		{
			Plane3D p(triangle);
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

		sp_bool hasCollision = findCollisionEdgeFace(details->objIndex1, details->objIndex2, &details->vertexIndexObj1, contactPoint, cache, cache->cacheMesh1, cache->cacheMesh2, details);
		
		if (hasCollision)
		{
			cache->searchOnObj1 = false;
			return CollisionStatus::INSIDE;
		}

		hasCollision = findCollisionEdgeFace(details->objIndex2, details->objIndex1, &details->vertexIndexObj2, contactPoint, cache, cache->cacheMesh2, cache->cacheMesh1, details);

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

	void SpCollisionDetector::timeOfCollision(SpCollisionDetails* details, SpCollisionDetectorCache* cache)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		sp_assert(details->objIndex1 < simulator->objectsLength(), "InvalidArgumentException");
		sp_assert(details->objIndex2 < simulator->objectsLength(), "InvalidArgumentException");

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);

		CollisionStatus status = CollisionStatus::OUTSIDE;
		const sp_float _epsilon = 0.1f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float diff = TEN_FLOAT;
		sp_float elapsedTime = details->timeStep * HALF_FLOAT;
		sp_float wasInsideAt = ZERO_FLOAT;
		Vec3 contactPoint;
		
		while ((status != CollisionStatus::INSIDE || diff > _epsilon) && diff > 0.01f)
		{
			simulator->backToTime(details->objIndex1);
			simulator->integrator->execute(details->objIndex1, elapsedTime);
			cache->cacheMesh1->update(*mesh1, *simulator->transforms(details->objIndex1));

			simulator->backToTime(details->objIndex2);
			simulator->integrator->execute(details->objIndex2, elapsedTime);
			cache->cacheMesh2->update(*mesh2, *simulator->transforms(details->objIndex2));

			status = collisionStatus(&contactPoint, cache, details);

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

		if (status == CollisionStatus::OUTSIDE)
		{
			if (wasInsideAt == ZERO_FLOAT) 
			{
				details->ignoreCollision = true;
				return;
			}

			if (wasInsideAt != ZERO_FLOAT)
			{
				elapsedTime = wasInsideAt;

				simulator->backToTime(details->objIndex1);
				simulator->integrator->execute(details->objIndex1, elapsedTime);
				cache->cacheMesh1->update(*mesh1, *simulator->transforms(details->objIndex1));

				simulator->backToTime(details->objIndex2);
				simulator->integrator->execute(details->objIndex2, elapsedTime);
				cache->cacheMesh2->update(*mesh2, *simulator->transforms(details->objIndex2));

				collisionStatus(&contactPoint, cache, details);
			}
		}

		if (cache->searchOnObj1)
		{
			details->vertexIndexObj1 = mesh1->findClosest(contactPoint, *simulator->transforms(details->objIndex1))->index();
			details->vertexIndexObj2 = mesh2->vertexesMesh->get(details->vertexIndexObj2)->findClosest(contactPoint, *simulator->transforms(details->objIndex2))->index();
		}
		else
		{
			details->vertexIndexObj2 = mesh2->findClosest(contactPoint, *simulator->transforms(details->objIndex2))->index();
			details->vertexIndexObj1 = mesh1->vertexesMesh->get(details->vertexIndexObj1)->findClosest(contactPoint, *simulator->transforms(details->objIndex1))->index();
		}

		//simulator->physicProperties(details->objIndex1)->integratedTime(elapsedTime);
		//simulator->physicProperties(details->objIndex2)->integratedTime(elapsedTime);
		details->timeOfCollision = elapsedTime;

		sp_assert(details->objIndex1 < simulator->objectsLength(), "InvalidArgumentException");
		sp_assert(details->objIndex2 < simulator->objectsLength(), "InvalidArgumentException");
	}

	void SpCollisionDetector::collisionDetails(SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);

		SpCollisionDetectorCache cache;
		cache.cacheMesh1 = ALLOC_NEW(SpMeshCache)(mesh1->vertexLength());
		cache.cacheMesh2 = ALLOC_NEW(SpMeshCache)(mesh2->vertexLength());

		timeOfCollision(details, &cache);

		if (details->ignoreCollision) // if they are not colliding in geometry
			return;

		fillCollisionDetails(details, &cache);
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

	void SpCollisionDetector::fillCollisionDetails(SpCollisionDetails* details, SpCollisionDetectorCache* cache)
	{
		if (isFaceFaceCollision(details, cache))
			return;

		if (isEdgeFaceCollisionObj1(details, cache))
			return;

		if (isEdgeFaceCollisionObj2(details, cache))
			return;

		if (isEdgeEdgeCollision(details, cache))
			return;

		if (isVertexFaceCollision(details, cache))
			return;
	
		// TODO: REMOVER!!
		details->ignoreCollision = true;
		return;
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		sp_char text[16000];
		sp_uint index = ZERO_UINT;
		Maple::convert(*simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex), *simulator->transforms(details->objIndex1), "mesh1", "red", text, &index);
		Maple::convert(*simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex), *simulator->transforms(details->objIndex2), "mesh2", "blue", text, &index);
		Maple::display("mesh1", "mesh2", text, &index);
	
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

	sp_bool SpCollisionDetector::isFaceFaceCollision(SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
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

		//vertex1->findParallelFaces(vertex2, transform1, transform2, facesIndexesMesh1, &facesIndexesMesh1Length , facesIndexesMesh2, &facesIndexesMesh2Length, ERROR_MARGIN_PHYSIC);
		vertex1->findParallelFaces(vertex2, cache->cacheMesh1, cache->cacheMesh2, facesIndexesMesh1, &facesIndexesMesh1Length , facesIndexesMesh2, &facesIndexesMesh2Length, ERROR_MARGIN_PHYSIC);

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
				cache->cacheMesh1->vertexes[face1->vertexesIndexes[0]],
				cache->cacheMesh1->vertexes[face1->vertexesIndexes[1]],
				cache->cacheMesh1->vertexes[face1->vertexesIndexes[2]]
			);
			
			for (sp_uint j = 0; j < facesIndexesMesh2Length; j++)
			{
				SpFaceMesh* face2 = allFacesObj2[facesIndexesMesh2[j]];
				Triangle3D triangleFace2(
					cache->cacheMesh2->vertexes[face2->vertexesIndexes[0]],
					cache->cacheMesh2->vertexes[face2->vertexesIndexes[1]],
					cache->cacheMesh2->vertexes[face2->vertexesIndexes[2]]
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
			
			Line3D line1(cache->cacheMesh1->vertexes[vertexI->index()], Vec3());
			
			for (sp_uint i2 = 0; i2 < vertexI->edgeLength(); i2++)
			{
				const SpVertexMesh* vertexI2 = mesh1->vertexesMesh->get(vertexI->edgeVertexIndex(i2));
				
				sp_uint eindex1 = mesh1->findEdge(vertexI->index(), vertexI->edgeVertexIndex(i2));
				SpEdgeMesh* e1 = mesh1->edges->get(eindex1);
				if (!e1->isBoundaryEdge())
					continue;
				
				line1.point2 = cache->cacheMesh1->vertexes[vertexI2->index()];

				for (sp_uint j = 0; j < vertexIndexesObj2Length; j++)
				{
					const SpVertexMesh* vertexJ = mesh2->vertexesMesh->get(vertexIndexesObj2[j]);
					
					Line3D line2(cache->cacheMesh2->vertexes[vertexJ->index()], Vec3());

					for (sp_uint j2 = 0; j2 < vertexJ->edgeLength(); j2++)
					{
						const SpVertexMesh* vertexJ2 = mesh2->vertexesMesh->get(vertexJ->edgeVertexIndex(j2));
					
						sp_uint eindex = mesh2->findEdge(vertexJ->index(), vertexJ->edgeVertexIndex(j2));
						SpEdgeMesh* e = mesh2->edges->get(eindex);
						if (!e->isBoundaryEdge())
							continue;
					
						line2.point2 = cache->cacheMesh2->vertexes[vertexJ2->index()];

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

		if (intersectionPointsLength == 2u)
		{
			details->type = SpCollisionType::EdgeFace;

			cross(intersectionPoints[0u], intersectionPoints[1u], &details->collisionNormalObj1);
			normalize(&details->collisionNormalObj1);
			details->collisionNormalObj2 = -details->collisionNormalObj1;

			details->contactPointsLength = 2u;
			details->contactPoints[0u] = intersectionPoints[0u];
			details->contactPoints[1u] = intersectionPoints[1u];
			details->centerContactPoint = (intersectionPoints[0u] + intersectionPoints[1u]) * HALF_FLOAT;
			return true;
		}

		sp_assert(intersectionPointsLength > MIN_INTERSECTION_POINTS, "InvalidOperationException");

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

			Plane3D face1AsPlane(face1AsTriangle);

			for (sp_uint j = 0; j < vertex2->edgeLength(); j++)
			{
				edge2.point2 = cacheMesh2->vertexes[vertex2->edgeVertexIndex(j)];

				// the edge and face normal should be perpendicular for edge-face collision
				if (!edge2.isPerpendicular(face1AsPlane.normalVector, 0.1f))
					continue;

				const sp_float distancePoint1 = fabsf(face1AsPlane.distance(edge2.point1));
				
				if (!isCloseEnough(distancePoint1, ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
					continue;

				const sp_float distancePoint2 = fabsf(face1AsPlane.distance(edge2.point2));

				if (!isCloseEnough(distancePoint1, distancePoint2, ERROR_MARGIN_PHYSIC))
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

	sp_bool SpCollisionDetector::isEdgeEdgeCollision(SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		const Vec3* allVertexesObj1 = cache->cacheMesh1->vertexes;
		const Vec3* allVertexesObj2 = cache->cacheMesh2->vertexes;

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
					!isCloseEnough(std::fabsf(sqDistance), ZERO_FLOAT, 0.4f))
					continue;

				// if the contact is at extreme of edge, it is vertex-edge collison...
				if (isCloseEnough(sqDistance, ZERO_FLOAT, 0.009f))
				{
					if (isCloseEnough(line2.point1, p2, ERROR_MARGIN_PHYSIC) 
						|| isCloseEnough(line2.point2, p2, ERROR_MARGIN_PHYSIC))
					{
						details->type = SpCollisionType::VertexEdge;
						details->contactPointsLength = 1u;
						details->contactPoints[0] = p2;
						details->centerContactPoint = p2;
						cross(line2.point1, line2.point2, &details->collisionNormalObj1);
						normalize(&details->collisionNormalObj1);
						details->collisionNormalObj2 = -details->collisionNormalObj1;
						return true;
					}

					if (isCloseEnough(line1.point1, p1, ERROR_MARGIN_PHYSIC) 
						|| isCloseEnough(line1.point2, p1, ERROR_MARGIN_PHYSIC))
					{
						details->type = SpCollisionType::VertexEdge;
						details->contactPointsLength = 1u;
						details->contactPoints[0] = p1;
						details->centerContactPoint = p1;
						cross(line1.point1, line1.point2, &details->collisionNormalObj1);
						normalize(&details->collisionNormalObj1);
						details->collisionNormalObj2 = -details->collisionNormalObj1;
						return true;
					}
				}

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

	sp_bool SpCollisionDetector::isVertexFaceCollision(SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		const Vec3* allVertexesObj1 = cache->cacheMesh1->vertexes;
		const Vec3* allVertexesObj2 = cache->cacheMesh2->vertexes;

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		
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
					Plane3D faceContact(triangle);
					
					details->contactPointsLength = ONE_UINT;

					const sp_float distance = faceContact.distance(line.point1);

					if (distance < ZERO_FLOAT + ERROR_MARGIN_PHYSIC) // if point is back-face, ...
						details->contactPoints[0] = line.point1;
					else
						details->contactPoints[0] = line.point2;

					details->type = SpCollisionType::VertexFace;
					details->collisionNormalObj1 = faceContact.normalVector;
					details->collisionNormalObj2 = -faceContact.normalVector;

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
					Plane3D faceContact(triangle);
					
					details->contactPointsLength = ONE_UINT;
					if (faceContact.isBackFace(line.point1))
						details->contactPoints[0] = line.point1;
					else
						details->contactPoints[0] = line.point2;

					details->type = SpCollisionType::VertexFace;
					details->collisionNormalObj1 = -faceContact.normalVector;
					details->collisionNormalObj2 = faceContact.normalVector;

					return true;
				}
			}
		}

		return false;
	}

	sp_bool SpCollisionDetector::isEdgeFaceCollisionObj1(SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpFaceMesh** allFacesObj1 = mesh1->faces->data();
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];

		sp_uint faceIndexCollisionObj1;
		sp_uint edgeVertexIndexObj2;

		if (isEdgeFaceCollision(vertexMeshObj1, vertexMeshObj2, cache->cacheMesh1, cache->cacheMesh2, &faceIndexCollisionObj1, &edgeVertexIndexObj2))
		{
#define MAX_PARALLEL_FACES 10u
#define MAX_CONTACTS 2u
			SpFaceMesh* faceCollisionObj1 = mesh1->faces->data()[faceIndexCollisionObj1];

			Line3D edgeObj2(
				cache->cacheMesh2->vertexes[vertexMeshObj2->index()],
				cache->cacheMesh2->vertexes[edgeVertexIndexObj2]
			);
			
			Plane3D faceAsPlane(
				cache->cacheMesh1->vertexes[faceCollisionObj1->vertexesIndexes[0]],
				cache->cacheMesh1->vertexes[faceCollisionObj1->vertexesIndexes[1]],
				cache->cacheMesh1->vertexes[faceCollisionObj1->vertexesIndexes[2]]
			);
			
			sp_uint parallelFacesIndexes[MAX_PARALLEL_FACES];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			vertexMeshObj1->findParallelFaces(faceAsPlane, cache->cacheMesh1, parallelFacesIndexes, &parallelFacesIndexesLength, 0.1f);

			sp_assert(parallelFacesIndexesLength < MAX_PARALLEL_FACES, "IndexOutOfRangeException");

			Vec3 contacts[MAX_CONTACTS];
			sp_uint contactsLength = ZERO_UINT;

			// find which faces this edge cross and get the two contact points
			for (sp_uint i = 0; i < parallelFacesIndexesLength; i++)
			{
				SpFaceMesh* face = allFacesObj1[parallelFacesIndexes[i]];
				Triangle3D faceAsTriangle(
					cache->cacheMesh1->vertexes[face->vertexesIndexes[0]],
					cache->cacheMesh1->vertexes[face->vertexesIndexes[1]],
					cache->cacheMesh1->vertexes[face->vertexesIndexes[2]]
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

	sp_bool SpCollisionDetector::isEdgeFaceCollisionObj2(SpCollisionDetails* details, SpCollisionDetectorCache* cache) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(details->objIndex1)->meshIndex);
		SpFaceMesh** allFacesObj1 = mesh1->faces->data();
		SpVertexMesh* vertexMeshObj1 = mesh1->vertexesMesh->data()[details->vertexIndexObj1];
		Vec3* allVertexesObj1 = cache->cacheMesh1->vertexes;

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(details->objIndex2)->meshIndex);
		const SpTransform transformObj2 = *simulator->transforms(details->objIndex2);
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		SpVertexMesh* vertexMeshObj2 = mesh2->vertexesMesh->data()[details->vertexIndexObj2];
		Vec3* allVertexesObj2 = cache->cacheMesh2->vertexes;

		sp_uint faceIndexCollisionObj2;
		sp_uint edgeVertexIndexObj1;

		if (isEdgeFaceCollision(vertexMeshObj2, vertexMeshObj1, cache->cacheMesh2, cache->cacheMesh1, &faceIndexCollisionObj2, &edgeVertexIndexObj1))
		{
#define MAX_PARALLEL_FACES 10u
#define MAX_CONTACTS 2u
			SpFaceMesh* faceCollisionObj2 = mesh2->faces->data()[faceIndexCollisionObj2];

			Line3D edge(
				allVertexesObj1[vertexMeshObj1->index()],
				allVertexesObj1[edgeVertexIndexObj1]
			);
			
			Plane3D faceAsPlane(
				allVertexesObj1[faceCollisionObj2->vertexesIndexes[0]],
				allVertexesObj1[faceCollisionObj2->vertexesIndexes[1]],
				allVertexesObj1[faceCollisionObj2->vertexesIndexes[2]]
			);

			sp_uint parallelFacesIndexes[MAX_PARALLEL_FACES];
			sp_uint parallelFacesIndexesLength = ZERO_UINT;

			// find all parallel faces
			vertexMeshObj2->findParallelFaces(faceAsPlane, cache->cacheMesh2, parallelFacesIndexes, &parallelFacesIndexesLength, 0.1f);

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
