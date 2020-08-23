#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
	void SpVertexEdges::findParallelVertexes(const Plane3D& plane, const SpTransform& meshTransform, SpVertexEdges** vertexesOutput,
		sp_uint* vertexOutputLength, sp_uint ignoreVertexIndex,
		sp_float _epsilon) const
	{
		Vec3* vertexes = mesh->vertexes->data();
		Vec3 currentVertex = vertexes[_vertexIndex];
		meshTransform.transform(currentVertex, &currentVertex);

		const sp_float currentDistance = plane.distance(currentVertex);

		for (SpVectorItem<SpVertexEdges*>* item = _edges->begin(); item != nullptr; item = item->next())
		{
			if (ignoreVertexIndex == item->value()->_vertexIndex)
				continue;

			Vec3 newVertex = vertexes[item->value()->_vertexIndex];
			meshTransform.transform(newVertex, &newVertex);

			// check if this vertex is parallel to currentVertex, given a plane
			if (isCloseEnough(plane.distance(newVertex), currentDistance, _epsilon))
			{
				sp_bool vertexFound = false;

				// check if vertex was already added...
				for (sp_uint i = 0; i < *vertexOutputLength; i++)
					if (vertexesOutput[i]->_vertexIndex == item->value()->_vertexIndex)
					{
						vertexFound = true;
						break;
					}

				if (!vertexFound)
				{
					vertexesOutput[*vertexOutputLength] = item->value();
					*vertexOutputLength = vertexOutputLength[0] + 1u;

					item->value()->findParallelVertexes(plane, meshTransform, vertexesOutput, vertexOutputLength, _vertexIndex, _epsilon);
				}
			}
		}
	}

	CollisionStatus SpMesh::collisionStatus(const SpMesh* mesh2,
		const SpTransform* transform1, const SpTransform* transform2,
		SpCollisionDetails* details)
	{
		sp_assert(mesh2 != nullptr, "NullPointerException");
		sp_assert(transform1 != nullptr, "NullPointerException");
		sp_assert(transform1 != nullptr, "NullPointerException");
		sp_assert(details != nullptr, "NullPointerException");

		details->extremeVertexObj1 = this->findExtremeVertex(transform2->position - transform1->position, transform1, details->extremeVertexObj1);
		details->extremeVertexObj2 = mesh2->findExtremeVertex(transform1->position - transform2->position, transform2, details->extremeVertexObj2);

		Triangle3D faces[10];
		sp_uint facesLength;
		details->extremeVertexObj1->findFacesFromIndexes(faces, &facesLength, transform1);

		sp_float distance = SP_FLOAT_MIN;
		const Vec3* vertexesObj2 = mesh2->vertexes->data();

		for (sp_uint i = 0; i < facesLength; i++) // for each face from static obj 1 ...
		{
			Plane3D face(faces[i].point1, faces[i].point2, faces[i].point3);

			for (sp_uint j = 0; j < mesh2->vertexes->length(); j++) // for each vertex of obj2 ...
			{
				Vec3 transformedVertexObj2;
				transform2->transform(vertexesObj2[j], &transformedVertexObj2);

				sp_float distance = face.distance(transformedVertexObj2);

				if (distance < ZERO_FLOAT) // check if point is back-face
					return CollisionStatus::INSIDE;
			}
		}

		return CollisionStatus::OUTSIDE;
	}

}
