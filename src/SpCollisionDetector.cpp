#include "SpCollisionDetector.h"

namespace NAMESPACE_PHYSICS
{

	void SpCollisionDetector::filterCollision(sp_uint obj1Index, sp_uint obj2Index, SpCollisionDetails* details) const
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

			simulator->translate(details->objIndex1, obj1Properties->previousPosition() - obj1Properties->position());
			obj1Properties->position(obj1Properties->previousPosition());
			obj1Properties->velocity(Vec3(ZERO_FLOAT));
			obj1Properties->angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->acceleration(Vec3(ZERO_FLOAT));

			simulator->translate(details->objIndex2, obj2Properties->previousPosition() - obj2Properties->position());
			obj2Properties->position(obj2Properties->previousPosition());
			obj2Properties->velocity(Vec3(ZERO_FLOAT));
			obj2Properties->angularVelocity(Vec3(ZERO_FLOAT));
			obj2Properties->acceleration(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (isObj1Static && isObj2Resting)
		{
			simulator->translate(details->objIndex2, obj2Properties->previousPosition() - obj2Properties->position());
			obj2Properties->position(obj2Properties->previousPosition());
			obj2Properties->acceleration(Vec3(ZERO_FLOAT));
			obj2Properties->angularVelocity(Vec3(ZERO_FLOAT));
			obj2Properties->velocity(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (isObj2Static && isObj1Resting)
		{
			simulator->translate(details->objIndex1, obj1Properties->previousPosition() - obj1Properties->position());
			obj1Properties->position(obj1Properties->previousPosition());
			obj1Properties->acceleration(Vec3(ZERO_FLOAT));
			obj1Properties->angularVelocity(Vec3(ZERO_FLOAT));
			obj1Properties->velocity(Vec3(ZERO_FLOAT));

			details->ignoreCollision = true;
			return;
		}

		if (areMovingAway(details->objIndex1, details->objIndex2)) // if the objects are getting distant
			details->ignoreCollision = true;
	}

	void SpCollisionDetector::collisionDetailsWithObj1Static(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details)
	{
		Line3D _edge;
		Triangle3D _face;
		Vec3 contactPoint;
		sp_uint penetratedVertexIndex;

		timeOfCollisionWithObj1Static(details->objIndex1, details->objIndex2, details);

		if (details->ignoreCollision) // if they are not colliding in geometry
			return;

		sp_bool collisionFound = findCollisionEdgeFace(details->objIndex2, details->objIndex1, &_edge, &_face, &contactPoint, &penetratedVertexIndex);

		if (!collisionFound)
		{
			details->ignoreCollision = true;
			return;
		}

		fillCollisionDetails(details->objIndex2, details->objIndex1, _edge, _face, penetratedVertexIndex, details);
	}

	void SpCollisionDetector::timeOfCollision(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(obj1Index)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(obj2Index)->meshIndex);

		CollisionStatus status = CollisionStatus::OUTSIDE;
		const sp_float _epsilon = 1.0f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float diff = TEN_FLOAT;
		sp_float elapsedTime = details->timeStep * HALF_FLOAT;

		while ((status != CollisionStatus::INSIDE || diff > _epsilon) && diff > 0.1f)
		{
			simulator->backToTime(obj1Index);
			simulator->integrate(obj1Index, elapsedTime);

			simulator->backToTime(obj2Index);
			simulator->integrate(obj2Index, elapsedTime);

			status = collisionStatus(obj1Index, obj2Index, details);

			diff = std::fabsf(previousElapsedTime - elapsedTime);
			previousElapsedTime = elapsedTime;

			if (status == CollisionStatus::OUTSIDE)
				elapsedTime += (diff * HALF_FLOAT);
			else
				elapsedTime -= (diff * HALF_FLOAT);
		}

		if (status == CollisionStatus::OUTSIDE)
		{
			details->ignoreCollision = true;
			return;
		}

		details->timeOfCollision = elapsedTime;
	}

	sp_bool SpCollisionDetector::findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, Line3D* edge, Triangle3D* face, Vec3* contactPoint, sp_uint* penetratedVertexIndex)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(obj1Index)->meshIndex);
		Vec3* vertexesObj1 = mesh1->vertexes->data();
		SpPoint2<sp_uint>* edges = mesh1->edgesIndexes->data();
		SpTransform* transformObj1 = simulator->transforms(obj1Index);

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(obj2Index)->meshIndex);
		SpPoint3<sp_uint>* facesObj2 = mesh2->facesIndexes->data();
		Vec3* vertexesObj2 = mesh2->vertexes->data();
		SpTransform* transformObj2 = simulator->transforms(obj2Index);

		// for each edge from mesh 1
		for (sp_uint i = 0; i < mesh1->edgesIndexes->length(); i++)
		{
			transformObj1->transform(vertexesObj1[edges[i].x], &edge->point1);
			transformObj1->transform(vertexesObj1[edges[i].y], &edge->point2);

			// for each face from mesh 2
			for (sp_uint j = 0; j < mesh2->facesIndexes->length(); j++)
			{
				transformObj2->transform(vertexesObj2[facesObj2[j].x], &face->point1);
				transformObj2->transform(vertexesObj2[facesObj2[j].y], &face->point2);
				transformObj2->transform(vertexesObj2[facesObj2[j].z], &face->point3);

				sp_bool hasIntersection;
				edge->intersection(*face, contactPoint, &hasIntersection);

				if (hasIntersection)
				{
					Plane3D faceAsPlane(*face);

					if (faceAsPlane.isBackFace(edge->point1))
						penetratedVertexIndex[0] = edges[i].x;
					else
						penetratedVertexIndex[0] = edges[i].y;

					return true;
				}
			}
		}

		return false;
	}

	CollisionStatus SpCollisionDetector::collisionStatus(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details)
	{
		Line3D edge;
		Triangle3D face;
		Vec3 contactPoint;
		sp_uint penetratedVertexIndex;

		sp_bool hasCollision = findCollisionEdgeFace(obj1Index, obj2Index, &edge, &face, &contactPoint, &penetratedVertexIndex);

		if (hasCollision)
			return CollisionStatus::INSIDE;

		hasCollision = findCollisionEdgeFace(obj2Index, obj1Index, &edge, &face, &contactPoint, &penetratedVertexIndex);

		return hasCollision
			? CollisionStatus::INSIDE
			: CollisionStatus::OUTSIDE;
	}

	void SpCollisionDetector::timeOfCollisionWithObj1Static(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(obj1Index)->meshIndex);
		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(obj2Index)->meshIndex);

		CollisionStatus status = CollisionStatus::OUTSIDE;
		const sp_float _epsilon = 1.0f;
		sp_float previousElapsedTime = details->timeStep;
		sp_float diff = TEN_FLOAT;
		sp_float elapsedTime = details->timeStep * HALF_FLOAT;

		while ((status != CollisionStatus::INSIDE || diff > _epsilon) && diff > 0.1f)
		{
			simulator->backToTime(obj2Index);
			simulator->integrate(obj2Index, elapsedTime);

			status = collisionStatus(obj1Index, obj2Index, details);

			diff = std::fabsf(previousElapsedTime - elapsedTime);
			previousElapsedTime = elapsedTime;

			if (status == CollisionStatus::OUTSIDE)
				elapsedTime += (diff * HALF_FLOAT);
			else
				elapsedTime -= (diff * HALF_FLOAT);
		}

		if (status == CollisionStatus::OUTSIDE)
		{
			details->ignoreCollision = true;
			return;
		}

		details->timeOfCollision = elapsedTime;
	}

	void SpCollisionDetector::collisionDetails(SpCollisionDetails* details)
	{
		sp_uint objIndex1 = details->objIndex1;
		sp_uint objIndex2 = details->objIndex2;

		timeOfCollision(objIndex1, objIndex2, details);

		if (details->ignoreCollision) // if they are not colliding in geometry
			return;

		Line3D _edge;
		Triangle3D _face;
		Vec3 contactPoint;
		sp_uint penetratedVertexIndex;

		sp_bool collisionFound = findCollisionEdgeFace(objIndex1, objIndex2, &_edge, &_face, &contactPoint, &penetratedVertexIndex);

		if (!collisionFound)
		{
			std::swap(objIndex1, objIndex2);
			collisionFound = findCollisionEdgeFace(objIndex1, objIndex2, &_edge, &_face, &contactPoint, &penetratedVertexIndex);
		}

		if (!collisionFound)
		{
			details->ignoreCollision = true;
			return;
		}

		fillCollisionDetails(objIndex1, objIndex2, _edge, _face, penetratedVertexIndex, details);
	}

	void SpCollisionDetector::fillCollisionDetails(const sp_uint obj1Index, const sp_uint obj2Index, const Line3D& edge, const Triangle3D& face, sp_uint penetratedVertexIndex, SpCollisionDetails* details)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpMesh* mesh1 = simulator->mesh(simulator->collisionFeatures(obj1Index)->meshIndex);
		SpTransform* transformObj1 = simulator->transforms(obj1Index);
		Vec3* vertexesObj1 = mesh1->vertexes->data();
		SpVertexEdges* vertexEdgesObj1 = mesh1->vertexEdges()[0];

		SpMesh* mesh2 = simulator->mesh(simulator->collisionFeatures(obj2Index)->meshIndex);
		SpTransform* transformObj2 = simulator->transforms(obj2Index);

		// pega os ponto paralelos normal a face que colidiu (se houver)
		Plane3D faceAsPlane(face);
		SpVertexEdges* parallelPoints[16];
		sp_uint parallelPointsLength = ZERO_UINT;
		SpVertexEdges* vertexEdge = vertexEdgesObj1->find(penetratedVertexIndex);
		vertexEdge->findParallelVertexes(faceAsPlane, *transformObj1, parallelPoints, &parallelPointsLength, penetratedVertexIndex);

		details->extremeVertexObj1 = vertexEdge;

		if (parallelPointsLength == ZERO_UINT) // point-face contact
		{
			details->type = SpCollisionType::PointFace;
			details->collisionNormal = (transformObj1->position - vertexesObj1[penetratedVertexIndex]).normalize();
			details->contactPointsLength = ONE_UINT;
			transformObj1->transform(vertexesObj1[penetratedVertexIndex], &details->contactPoints[0]);
			return;
		}

		if (parallelPointsLength == ONE_UINT) // edge-face or edge-edge contact
		{
			Vec3 edgeEdgePoint, v1, v2;
			transformObj1->transform(vertexesObj1[penetratedVertexIndex], &v1);
			transformObj1->transform(parallelPoints[0]->vertex(), &v2);

			Line3D edgeObj1(v1, v2);
			Line3D edgesObj2[3];
			face.edges(edgesObj2);

			for (sp_uint w = 0; w < 3u; w++) // check edge-edge collision
			{
				edgeObj1.intersection(edgesObj2[w], &edgeEdgePoint, 1.0f);

				if (!edgeEdgePoint.isCloseEnough(ZERO_FLOAT)) // if edge-edge, ...
				{
					SpVertexEdges* edge2Point1 = mesh2->find(edgesObj2[w].point1, transformObj2);
					SpVertexEdges* edge2Point2 = mesh2->find(edgesObj2[w].point2, transformObj2);

					if (edge2Point1->isBoundaryEdge(edge2Point2)) // check this edge is face or boundary face
					{
						edge.cross(edgesObj2[w], &details->collisionNormal); // fill the normal contact

						details->type = SpCollisionType::EdgeEdge;
						details->collisionNormal = details->collisionNormal.normalize();
						details->contactPointsLength = ONE_UINT;
						details->contactPoints[0] = edgeEdgePoint;
						return;
					}
					break;
				}
			}

			details->type = SpCollisionType::EdgeFace;
			details->collisionNormal = faceAsPlane.normalVector;
			details->contactPointsLength = TWO_UINT;
			transformObj1->transform(vertexesObj1[penetratedVertexIndex], &details->contactPoints[0]);
			transformObj1->transform(vertexesObj1[parallelPoints[0]->vertexIndex()], &details->contactPoints[1]);
			return;
		}

		if (parallelPointsLength > TWO_UINT) // face-face contact
		{
			details->type = SpCollisionType::FaceFace;
			details->collisionNormal = faceAsPlane.normalVector;
			details->contactPointsLength = parallelPointsLength;

			for (sp_uint w = 0; w < parallelPointsLength; w++)
				transformObj1->transform(vertexesObj1[parallelPoints[w]->vertexIndex()], &details->contactPoints[w]);

			return;
		}
	}

	sp_bool SpCollisionDetector::areMovingAway(sp_uint objIndex1, sp_uint objIndex2) const
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		const SpPhysicProperties* obj1Properties = simulator->physicProperties(objIndex1);
		const SpPhysicProperties* obj2Properties = simulator->physicProperties(objIndex2);

		Vec3 lineOfAction = obj2Properties->position() - obj1Properties->position();
		const Vec3 velocityToObject2 = obj1Properties->velocity() * lineOfAction;

		lineOfAction = obj1Properties->position() - obj2Properties->position();
		const Vec3 velocityToObject1 = obj2Properties->velocity() * lineOfAction;

		return velocityToObject2 <= ZERO_FLOAT && velocityToObject1 <= ZERO_FLOAT;
	}

}
