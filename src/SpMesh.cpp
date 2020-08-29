#include "SpMesh.h"
#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{
	Vec3 SpVertexEdges::vertex() const
	{
		return mesh->vertexes->data()[_vertexIndex];
	}

	sp_bool SpVertexEdges::isBoundaryEdge(const SpVertexEdges* point2) const
	{
		SpPoint3<sp_uint> tempFaces[2];
		sp_uint tempFacesLength = ZERO_UINT;
		facesWith(point2, tempFaces, &tempFacesLength);

		if (tempFacesLength == ONE_UINT)
			return false;

		Vec3* verts = mesh->vertexes->data();

		Triangle3D face1(
			verts[tempFaces[0].x],
			verts[tempFaces[0].y],
			verts[tempFaces[0].z]
		);

		Triangle3D face2(
			verts[tempFaces[1].x],
			verts[tempFaces[1].y],
			verts[tempFaces[1].z]
		);

		Vec3 normalFace1, normalFace2;
		face1.normal(&normalFace1);
		face2.normal(&normalFace2);

		return normalFace1.abs() != normalFace2.abs();
	}

	SpVertexEdges* SpVertexEdges::find(const Vec3& vertex, const SpTransform* transform, sp_uint* visitedVertexes, sp_uint* visitedVertexesLength)
	{
		Vec3 value;
		if (transform == nullptr)
			value = mesh->vertexes->data()[_vertexIndex];
		else
			transform->transform(mesh->vertexes->data()[_vertexIndex], &value);

		if (value == vertex)
			return this;

		for (SpVectorItem<SpVertexEdges*>* item = _edges->begin(); item != nullptr; item = item->next())
		{
			for (sp_uint i = 0u; i < *visitedVertexesLength; i++)
				if (item->value()->_vertexIndex == visitedVertexes[i])
					continue;

			SpVertexEdges* _elementFound = item->value()->find(vertex, transform, visitedVertexes, visitedVertexesLength);

			if (_elementFound != nullptr)
				return _elementFound;

			visitedVertexes[*visitedVertexesLength] = item->value()->_vertexIndex;
			visitedVertexesLength[0] += ONE_UINT;
		}

		return nullptr;
	}

	void SpVertexEdges::facesWith(const SpVertexEdges* edge, SpPoint3<sp_uint>* output, sp_uint* outputLength) const
	{
		for (SpVectorItem<SpVertexEdges*>* item = edge->_edges->begin(); item != nullptr; item = item->next())
		{
			for (SpVectorItem<SpVertexEdges*>* item2 = item->value()->_edges->begin(); item2 != nullptr; item2 = item2->next())
			{
				if (item2->value()->_vertexIndex == _vertexIndex)
				{
					output[*outputLength] = SpPoint3<sp_uint>(_vertexIndex, edge->_vertexIndex, item->value()->_vertexIndex);
					outputLength[0] += ONE_UINT;
				}
			}
		}
	}

	void SpVertexEdges::facesWith(const Vec3& value, SpPoint3<sp_uint>* output, sp_uint* outputLength) const
	{
		SpVertexEdges* firstEdge = findNeighbor(value);
		facesWith(firstEdge, output, outputLength);
	}

	SpVertexEdges* SpVertexEdges::findNeighbor(const Vec3& value) const
	{
		for (SpVectorItem<SpVertexEdges*>* item = _edges->begin(); item != nullptr; item = item->next())
			if (item->value() != value)
				return item->value();

		return nullptr;
	}

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

}
