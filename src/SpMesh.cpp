#include "SpMesh.h"
#include "SpPhysicSimulator.h"

namespace NAMESPACE_PHYSICS
{

	void SpMesh::vertex(const sp_uint index, Vec3* output) const
	{
		sp_assert(index < vertexesMesh->length(), "IndexOutOfRangeException");
		output[0] = vertexesMesh->data()[index]->_value;
	}

	void SpMesh::vertex(const sp_uint index, const SpTransform& transform, Vec3* output) const
	{
		sp_assert(index < vertexesMesh->length(), "IndexOutOfRangeException");
		transform.transform(vertexesMesh->get(index)->_value, output);
	}

	sp_uint SpMesh::vertexLength() const
	{
		return vertexesMesh->length();
	}

	SpVertexMesh* SpMesh::findClosest(const Vec3& point, const SpTransform& transform, SpVertexMesh* from) const
	{
		if (from == nullptr)
			from = vertexesMesh->data()[0];

		return from->findClosest(point, transform);
	}

	SpVertexMesh* SpMesh::findExtremeVertexDirection(const Vec3& orientation, const SpTransform& transform, SpVertexMesh* from) const
	{
		if (from == nullptr)
			from = vertexesMesh->data()[0];

		return findExtremeVertexDirection(from, orientation, transform);
	}

	SpVertexMesh* SpMesh::findExtremeVertexDirection(const Vec3& orientation, const SpMeshCache* cache, const Vec3& center, SpVertexMesh* startingFrom) const
	{
		if (startingFrom == nullptr)
			startingFrom = vertexesMesh->get(0);

		return findExtremeVertexDirection(startingFrom, orientation, cache, center);
	}

	void SpMesh::findAllClosestDetails(const Vec3& target, const SpMeshCache* cache, const Vec3& center,
		Vec3* closestPoint, 
		sp_uint* closestVertexMesh, sp_uint* closestEdgeMesh, sp_uint* closestFaceMesh,
		SpVertexMesh* startingFrom) const
	{
		Vec3 _direction;
		direction(center, target, &_direction);

		SpVertexMesh* vm = findExtremeVertexDirection(startingFrom, _direction, cache, center);
		closestPoint[0] = cache->vertexes[vm->index()];
		closestVertexMesh[0] = vm->index();
		closestEdgeMesh[0] = ZERO_UINT;
		closestFaceMesh[0] = ZERO_UINT;

		//sp_float distanceToCenter = distance(center, *closestPoint);
		sp_float distanceToCenter = distance(target, *closestPoint);
		
		for (sp_uint i = 0; i < vm->edgeLength(); i++)
		{
			SpEdgeMesh* edgeMesh = vm->edges(i);
			Line3D line(cache->vertexes[edgeMesh->vertexIndex1], cache->vertexes[edgeMesh->vertexIndex2]);
			
			Vec3 temp;
			line.closestPointOnTheLine(target, &temp);

			//const sp_float d = distance(center, temp);
			const sp_float newDistance = distance(target, temp);

			if (newDistance < distanceToCenter)
			{
				distanceToCenter = newDistance;
				closestPoint[0] = temp;
				closestEdgeMesh[0] = edgeMesh->index();
			}
		}

		SpEdgeMesh* edge = edges->get(closestEdgeMesh[0]);
		for (sp_uint i = 0; i < edge->faces.length(); i++)
		{
			sp_uint faceIndex = edge->faces.get(i);
			sp_uint* faceVertexIndexes = faces->get(faceIndex)->vertexesIndexes;

			Triangle3D face(
				cache->vertexes[faceVertexIndexes[0]],
				cache->vertexes[faceVertexIndexes[1]],
				cache->vertexes[faceVertexIndexes[2]]);

			Vec3 _closest;
			sp_float d = face.distance(target, &_closest);

			if (d < distanceToCenter)
			{
				distanceToCenter = d;
				closestPoint[0] = _closest;
				closestFaceMesh[0] = faceIndex;
			}
		}
	}

	sp_uint SpMesh::findEdge(const sp_uint vertexIndex1, const sp_uint vertexIndex2) const
	{
		SpEdgeMesh** _edges = edges->data();

		for (sp_uint i = 0; i < edges->length(); i++)
			if ((_edges[i]->vertexIndex1 == vertexIndex1 && _edges[i]->vertexIndex2 == vertexIndex2)
				|| (_edges[i]->vertexIndex1 == vertexIndex2 && _edges[i]->vertexIndex2 == vertexIndex1))
				return i;

		return SP_UINT_MAX;
	}

	sp_bool hasTempEdge(sp_uint** mapVertexEdge, sp_uint index, sp_uint value)
	{
		for (sp_uint i = 0; i < mapVertexEdge[index][0] + 1; i++)
			if (mapVertexEdge[index][i + 1] == value)
				return true;

		return false;
	}
	void addTempEdge(sp_uint** mapVertexEdge, sp_uint index, sp_uint value)
	{
		mapVertexEdge[index][mapVertexEdge[index][0] + 1] = value;
		mapVertexEdge[index][0] ++;
	}

	void SpMesh::init()
	{
		if (faces == nullptr || faces->length() == ZERO_UINT)
			return;

		sp_uint** mapVertexEdge = ALLOC_ARRAY(sp_uint*, vertexLength());
		for (sp_uint i = 0; i < vertexLength(); i++)
		{
			mapVertexEdge[i] = ALLOC_ARRAY(sp_uint, 20);
			mapVertexEdge[i][0] = ZERO_UINT;
		}

		edges = sp_mem_new(SpArray<SpEdgeMesh*>)(3u * 6u * faces->length() - 2u);
		sp_uint edgIndex = ZERO_UINT;
		for (sp_uint i = 0; i < faces->length(); i++)
		{
			SpFaceMesh* face = faces->data()[i];
			sp_uint* vertexesIndexes = face->vertexesIndexes;

			sp_uint vertexIndex1 = vertexesIndexes[0];
			sp_uint vertexIndex2 = vertexesIndexes[1];
			sp_uint vertexIndex3 = vertexesIndexes[2];

			// map vertex-edges
			if (!hasTempEdge(mapVertexEdge, vertexIndex1, vertexIndex2))
				addTempEdge(mapVertexEdge, vertexIndex1, vertexIndex2);

			if (!hasTempEdge(mapVertexEdge, vertexIndex2, vertexIndex1))
				addTempEdge(mapVertexEdge, vertexIndex2, vertexIndex1);

			if (!hasTempEdge(mapVertexEdge, vertexIndex2, vertexIndex3))
				addTempEdge(mapVertexEdge, vertexIndex2, vertexIndex3);

			if (!hasTempEdge(mapVertexEdge, vertexIndex3, vertexIndex2))
				addTempEdge(mapVertexEdge, vertexIndex3, vertexIndex2);

			if (!hasTempEdge(mapVertexEdge, vertexIndex3, vertexIndex1))
				addTempEdge(mapVertexEdge, vertexIndex3, vertexIndex1);

			if (!hasTempEdge(mapVertexEdge, vertexIndex1, vertexIndex3))
				addTempEdge(mapVertexEdge, vertexIndex1, vertexIndex3);

			// add thi face to vertex
			vertexesMesh->data()[vertexIndex1]->facesIndexes->add(i);
			vertexesMesh->data()[vertexIndex2]->facesIndexes->add(i);
			vertexesMesh->data()[vertexIndex3]->facesIndexes->add(i);

			sp_uint existEdge = findEdge(vertexIndex1, vertexIndex2);
			if (existEdge == SP_UINT_MAX)
			{
				existEdge = findEdge(vertexIndex2, vertexIndex1);
				if (existEdge == SP_UINT_MAX)
				{
					existEdge = edgIndex++;

					SpEdgeMesh* _edge = sp_mem_new(SpEdgeMesh(this, existEdge, vertexIndex1, vertexIndex2, *face));
					edges->add(_edge);
					face->edgesIndexes[0] = existEdge;
				}
				else
				{
					edges->data()[existEdge]->addFace(*face);
					face->edgesIndexes[0] = existEdge;
				}
			}
			else
			{
				edges->data()[existEdge]->addFace(*face);
				face->edgesIndexes[0] = existEdge;
			}

			existEdge = findEdge(vertexIndex2, vertexIndex3);
			if (existEdge == SP_UINT_MAX)
			{
				existEdge = findEdge(vertexIndex3, vertexIndex2);
				if (existEdge == SP_UINT_MAX)
				{
					existEdge = edgIndex++;
					SpEdgeMesh* _edge = sp_mem_new(SpEdgeMesh(this, existEdge, vertexIndex2, vertexIndex3, *face));
					edges->add(_edge);
					face->edgesIndexes[1] = existEdge;
				}
				else
				{
					edges->data()[existEdge]->addFace(*face);
					face->edgesIndexes[1] = existEdge;
				}
			}
			else
			{
				edges->data()[existEdge]->addFace(*face);
				face->edgesIndexes[1] = existEdge;
			}

			existEdge = findEdge(vertexIndex3, vertexIndex1);
			if (existEdge == SP_UINT_MAX)
			{
				existEdge = findEdge(vertexIndex1, vertexIndex3);
				if (existEdge == SP_UINT_MAX)
				{
					existEdge = edgIndex++;
					SpEdgeMesh* _edge = sp_mem_new(SpEdgeMesh(this, existEdge, vertexIndex3, vertexIndex1, *face));
					edges->add(_edge);
					face->edgesIndexes[2] = existEdge;
				}
				else
				{
					edges->data()[existEdge]->addFace(*face);
					face->edgesIndexes[2] = existEdge;
				}
			}
			else
			{
				edges->data()[existEdge]->addFace(*face);
				face->edgesIndexes[2] = existEdge;
			}
		}

		// fill edges with fixed length
		for (sp_uint i = 0; i < vertexLength(); i++)
		{
			SpVertexMesh* vm = vertexesMesh->data()[i];
			vm->_edgeVertexIndex = sp_mem_new(SpArray<sp_uint>)(mapVertexEdge[i][0]);
			vm->_edges = sp_mem_new(SpArray<sp_uint>)(mapVertexEdge[i][0]);

			for (sp_uint j = 0; j < mapVertexEdge[i][0]; j++)
			{
				sp_uint edgIndex = findEdge(vm->_index, mapVertexEdge[i][j + 1]);

				vm->_edgeVertexIndex->add(mapVertexEdge[i][j + 1]);
				vm->_edges->add(edgIndex);
			}
		}

		for (sp_uint i = 0; i < faces->length(); i++)
			faces->get(i)->fillAttributes();

		for (sp_uint i = 0; i < edges->length(); i++)
		{
			SpEdgeMesh* _edge = edges->get(i);
			_edge->fillAttributes();
		}
	}
	
	SpVertexMesh* SpMesh::findExtremeVertexDirection(SpVertexMesh* from, const Vec3& orientation, const SpTransform& transform) const
	{
		const Plane3D plane(transform.position, orientation);

		Vec3 newVertexPosition;
		vertex(from->index(), transform, &newVertexPosition);

		const sp_float distance = plane.distance(newVertexPosition);

		for (sp_uint i = 0; i < from->edgeLength(); i++)
		{
			const sp_uint newIndex = from->edgeVertexIndex(i);

			Vec3 vertexPosition2;
			vertex(newIndex, transform, &vertexPosition2);

			const sp_float newDistance = plane.distance(vertexPosition2);

			if (newDistance > distance)
				return findExtremeVertexDirection(vertexesMesh->get(newIndex), orientation, transform);
		}

		return from;
	}

	SpVertexMesh* SpMesh::findExtremeVertexDirection(SpVertexMesh* from, const Vec3& orientation, const SpMeshCache* cache, const Vec3& center) const
	{
		const Plane3D plane(center, orientation);

		const sp_float distance = plane.distance(cache->vertexes[from->index()]);

		for (sp_uint i = 0; i < from->edgeLength(); i++)
		{
			const sp_uint newIndex = from->edgeVertexIndex(i);
			const sp_float newDistance = plane.distance(cache->vertexes[newIndex]);

			if (newDistance > distance)
				return findExtremeVertexDirection(vertexesMesh->get(newIndex), orientation, cache, center);
		}

		return from;
	}

	sp_bool SpMesh::containsEdge(sp_uint* edge, sp_uint* vertexesIndexes, sp_uint* length) const
	{
		for (sp_uint i = 0; i < *length; i++)
			if (
				(vertexesIndexes[multiplyBy2(i)] == edge[0] && vertexesIndexes[multiplyBy2(i) + 1u] == edge[1]) ||
				(vertexesIndexes[multiplyBy2(i)] == edge[1] && vertexesIndexes[multiplyBy2(i) + 1u] == edge[0]))
				return true;

		return false;
	}

	sp_bool SpMesh::isInside(const SpMesh* mesh2, const SpTransform& transformObj1, const SpTransform& transformObj2) const
	{
		SpFaceMesh** allFacesObj2 = mesh2->faces->data();
		const sp_uint facesLengthObj2 = mesh2->faces->length();

		for (sp_uint i = 0; i < vertexLength(); i++)
		{
			Vec3 vertex;
			this->vertex(i, transformObj1, &vertex);

			for (sp_uint j = 0; j < facesLengthObj2; j++)
			{
				Plane3D face;
				allFacesObj2[j]->convert(&face, transformObj2);

				if ( ! face.isBackFace(vertex) )
					return false;
			}
		}

		return true;
	}


	void SpMesh::convert(Vec3* vertexes, const SpTransform& transform) const
	{
		for (sp_uint i = 0; i < vertexLength(); i++)
			vertex(i, transform, &vertexes[i]);
	}

}
