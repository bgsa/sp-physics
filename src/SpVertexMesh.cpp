#include "SpVertexMesh.h"
#include "SpMeshCache.h"

namespace NAMESPACE_PHYSICS
{

	SpEdgeMesh* SpVertexMesh::edges(const sp_uint localEdgeIndex) const
	{
		return _mesh->edges->get(_edges->get(localEdgeIndex));
	}

	SpEdgeMesh* SpVertexMesh::findEdges(const sp_uint vertexIndex) const
	{
		const sp_uint edgeIndex = _mesh->findEdge(_index, _edgeVertexIndex->get(vertexIndex));
		return _mesh->edges->get(edgeIndex);
	}

	void SpVertexMesh::findFacesFromIndexes(Triangle3D* outputFaces, sp_uint* facesLength, const SpTransform* transform) const
	{
		facesLength[0] = ZERO_UINT;

		SpFaceMesh** allFaces = _mesh->faces->data();
		
		for (sp_uint i = 0; i < _mesh->faces->length(); i++)
		{
			sp_uint* facesVertexIndexes = allFaces[i]->vertexesIndexes;

			if (facesVertexIndexes[0] == _index || 
				facesVertexIndexes[1] == _index || 
				facesVertexIndexes[2] == _index)
			{
				_mesh->vertex(facesVertexIndexes[0], *transform, outputFaces[*facesLength].point1);
				_mesh->vertex(facesVertexIndexes[1], *transform, outputFaces[*facesLength].point2);
				_mesh->vertex(facesVertexIndexes[2], *transform, outputFaces[*facesLength].point3);
				facesLength[0] ++;
			}
		}
	}

	SpVertexMesh* SpVertexMesh::findClosest(const Vec3& point, const SpTransform& transform)
	{
		SpVertexMesh** allVertexesMesh = _mesh->vertexesMesh->data();

		Vec3 currentVertex;
		_mesh->vertex(_index, transform, currentVertex);

		const sp_float distance = currentVertex.squaredDistance(point);

		for (sp_uint i = 0; i < edgeLength(); i++)
		{
			const sp_uint index = edgeVertexIndex(i);

			Vec3 newVertex;
			_mesh->vertex(index, transform, newVertex);

			if (newVertex.squaredDistance(point) < distance)
				return allVertexesMesh[index]->findClosest(point, transform);
		}

		return this;
	}

	Vec3 SpVertexMesh::value() const
	{
		return _value;
	}

	SpFaceMesh* SpVertexMesh::face(const sp_uint index) const
	{
		return _mesh->faces->get(facesIndexes->get(index));
	}

	void SpVertexMesh::findParallelVertexes(const Plane& plane, const SpTransform& meshTransform, 
		Vec3* vertexesOutput, sp_uint* vertexOutputLength, sp_float _epsilon) const
	{
		const sp_uint* vertexFaces = facesIndexes->data();
		SpFaceMesh** faces = _mesh->faces->data();

		Vec3 currentVertex;
		_mesh->vertex(_index, meshTransform, currentVertex);

		const sp_float distance = plane.distance(currentVertex);

		for (sp_uint i = 0; i < edgeLength(); i++)
		{
			const sp_uint vertexIndex = edgeVertexIndex(i);
			_mesh->vertex(vertexIndex, meshTransform, currentVertex);

			const sp_float newDistance = plane.distance(currentVertex);

			if (NAMESPACE_FOUNDATION::isCloseEnough(distance, newDistance, _epsilon))
			{
				sp_uint vertexFound = false;
				// if vertex is already added on list, ignore
				for (sp_uint i = 0; i < *vertexOutputLength; i++)
					if (vertexesOutput[i] == currentVertex)
					{
						vertexFound = true;
						break;
					}

				if (!vertexFound)
				{
					vertexesOutput[*vertexOutputLength] = currentVertex;
					vertexOutputLength[0] += ONE_UINT;

					// search new parallel vertexes on this vertex added
					_mesh->vertexesMesh->data()[vertexIndex]
						->findParallelVertexes(plane, meshTransform, vertexesOutput, vertexOutputLength, _epsilon);
				}
			}
		}
	}

	void SpVertexMesh::findParallelVertexes(const Plane& plane, const SpMeshCache* cache,
		Vec3* vertexesOutput, sp_uint* vertexOutputLength, sp_float _epsilon) const
	{
		const sp_uint* vertexFaces = facesIndexes->data();
		SpFaceMesh** faces = _mesh->faces->data();

		Vec3 currentVertex = cache->vertexes[_index];

		const sp_float distance = plane.distance(currentVertex);

		for (sp_uint i = 0; i < edgeLength(); i++)
		{
			const sp_uint vertexIndex = edgeVertexIndex(i);
			currentVertex = cache->vertexes[vertexIndex];
			
			const sp_float newDistance = plane.distance(currentVertex);

			if (NAMESPACE_FOUNDATION::isCloseEnough(distance, newDistance, _epsilon))
			{
				sp_uint vertexFound = false;
				// if vertex is already added on list, ignore
				for (sp_uint i = 0; i < *vertexOutputLength; i++)
					if (vertexesOutput[i] == currentVertex)
					{
						vertexFound = true;
						break;
					}

				if (!vertexFound)
				{
					vertexesOutput[*vertexOutputLength] = currentVertex;
					vertexOutputLength[0] += ONE_UINT;

					// search new parallel vertexes on this vertex added
					_mesh->vertexesMesh->data()[vertexIndex]
						->findParallelVertexes(plane, cache, vertexesOutput, vertexOutputLength, _epsilon);
				}
			}
		}
	}

	void SpVertexMesh::findParallelFaces(const Plane& plane, const SpTransform& meshTransform,
		sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length, const sp_float _epsilon) const
	{
		SpVertexMesh** allVertexesMesh = _mesh->vertexesMesh->data();
		SpFaceMesh** allFaces = _mesh->faces->data();
		sp_uint* vertexFaces = facesIndexes->data();

		for (sp_uint i = 0; i < facesIndexes->length(); i++)
		{
			Plane planeAsFace;
			const sp_uint vertexFaceIndex = vertexFaces[i];
			allFaces[vertexFaceIndex]->convert(&planeAsFace, meshTransform);
			
			if (planeAsFace.isParallel(plane, _epsilon)
				&& !NAMESPACE_FOUNDATION::contains(facesIndexesMesh1, facesIndexesMesh1Length[0], vertexFaceIndex))
			{
				facesIndexesMesh1[facesIndexesMesh1Length[0]] = vertexFaceIndex;
				facesIndexesMesh1Length[0] ++;

				sp_uint vertexIndex = allFaces[vertexFaceIndex]->vertexesIndexes[0];
				if (vertexIndex != _index)
					allVertexesMesh[vertexIndex]->findParallelFaces(plane, meshTransform, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

				vertexIndex = allFaces[vertexFaceIndex]->vertexesIndexes[1];
				if (vertexIndex != _index)
					allVertexesMesh[vertexIndex]->findParallelFaces(plane, meshTransform, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

				vertexIndex = allFaces[vertexFaceIndex]->vertexesIndexes[2];
				if (vertexIndex != _index)
					allVertexesMesh[vertexIndex]->findParallelFaces(plane, meshTransform, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);
			}
		}
	}

	void SpVertexMesh::findParallelFaces(const Plane& plane, const SpMeshCache* cache,
		sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length, const sp_float _epsilon) const
	{
		SpVertexMesh** allVertexesMesh = _mesh->vertexesMesh->data();
		SpFaceMesh** allFaces = _mesh->faces->data();
		sp_uint* vertexFaces = facesIndexes->data();

		for (sp_uint i = 0; i < facesIndexes->length(); i++)
		{
			const sp_uint vertexFaceIndex = vertexFaces[i];
			sp_uint* vertexesFacesIndexes1 = allFaces[vertexFaceIndex]->vertexesIndexes;
			Plane planeAsFace(
				cache->vertexes[vertexesFacesIndexes1[0]],
				cache->vertexes[vertexesFacesIndexes1[1]],
				cache->vertexes[vertexesFacesIndexes1[2]]
			);
			
			if (planeAsFace.isParallel(plane, _epsilon)
				&& !NAMESPACE_FOUNDATION::contains(facesIndexesMesh1, facesIndexesMesh1Length[0], vertexFaceIndex))
			{
				facesIndexesMesh1[facesIndexesMesh1Length[0]] = vertexFaceIndex;
				facesIndexesMesh1Length[0] ++;

				sp_uint vertexIndex = allFaces[vertexFaceIndex]->vertexesIndexes[0];
				if (vertexIndex != _index)
					allVertexesMesh[vertexIndex]->findParallelFaces(plane, cache, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

				vertexIndex = allFaces[vertexFaceIndex]->vertexesIndexes[1];
				if (vertexIndex != _index)
					allVertexesMesh[vertexIndex]->findParallelFaces(plane, cache, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

				vertexIndex = allFaces[vertexFaceIndex]->vertexesIndexes[2];
				if (vertexIndex != _index)
					allVertexesMesh[vertexIndex]->findParallelFaces(plane, cache, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);
			}
		}
	}

	void SpVertexMesh::findParallelFaces(const SpVertexMesh* vertex2,
		const SpMeshCache* cache1, const SpMeshCache* cache2,
		sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length, 
		sp_uint* facesIndexesMesh2, sp_uint* facesIndexesMesh2Length,
		const sp_float _epsilon) const
	{
		SpFaceMesh** allFacesMesh1 = _mesh->faces->data();
		SpVertexMesh** allVertexesMesh1 = _mesh->vertexesMesh->data();
		sp_uint* facesVertex1 = facesIndexes->data();

		SpFaceMesh** allFacesMesh2 = vertex2->_mesh->faces->data();
		SpVertexMesh** allVertexesMesh2 = vertex2->_mesh->vertexesMesh->data();
		sp_uint* facesVertex2 = vertex2->facesIndexes->data();

		for (sp_uint i = 0; i < facesIndexes->length(); i++)
		{
			sp_uint* vertexesIndexesFace1 = allFacesMesh1[facesVertex1[i]]->vertexesIndexes;
			Plane plane1(
				cache1->vertexes[vertexesIndexesFace1[0]],
				cache1->vertexes[vertexesIndexesFace1[1]],
				cache1->vertexes[vertexesIndexesFace1[2]]
			);
			
			for (sp_uint j = 0; j < vertex2->facesIndexes->length(); j++)
			{
				sp_uint* vertexesIndexesFace2 = allFacesMesh2[facesVertex2[j]]->vertexesIndexes;
				Plane plane2(
					cache2->vertexes[vertexesIndexesFace2[0]],
					cache2->vertexes[vertexesIndexesFace2[1]],
					cache2->vertexes[vertexesIndexesFace2[2]]
				);

				// faces should be parallel and opposite normals (front-face agains front-face)
				if (plane1.isParallel(plane2, _epsilon) 
					&& (plane1.normalVector - plane2.normalVector) != ZERO_FLOAT
					&& NAMESPACE_FOUNDATION::isCloseEnough(plane1.distance(plane2), ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
				{
					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh1, facesIndexesMesh1Length[0], facesVertex1[i]))
					{
						facesIndexesMesh1[facesIndexesMesh1Length[0]] = facesVertex1[i];
						facesIndexesMesh1Length[0] ++;

						// check the faces of the left 2 vertex of this face  
						sp_uint vertexIndex = allFacesMesh1[facesVertex1[i]]->vertexesIndexes[0];
						if (vertexIndex != _index)
							allVertexesMesh1[vertexIndex]->findParallelFaces(plane1, cache1, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

						vertexIndex = allFacesMesh1[facesVertex1[i]]->vertexesIndexes[1];
						if (vertexIndex != _index)
							allVertexesMesh1[vertexIndex]->findParallelFaces(plane1, cache1, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

						vertexIndex = allFacesMesh1[facesVertex1[i]]->vertexesIndexes[2];
						if (vertexIndex != _index)
							allVertexesMesh1[vertexIndex]->findParallelFaces(plane1, cache1, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);
					}

					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, facesIndexesMesh2Length[0], facesVertex2[j]))
					{
						facesIndexesMesh2[facesIndexesMesh2Length[0]] = facesVertex2[j];
						facesIndexesMesh2Length[0] ++;

						// check the faces of the left 2 vertex of this face
						sp_uint vertexIndex = allFacesMesh2[facesVertex2[j]]->vertexesIndexes[0];
						if (vertexIndex != _index)
							allVertexesMesh2[vertexIndex]->findParallelFaces(plane2, cache2, facesIndexesMesh2, facesIndexesMesh2Length, _epsilon);

						vertexIndex = allFacesMesh2[facesVertex2[j]]->vertexesIndexes[1];
						if (vertexIndex != _index)
							allVertexesMesh2[vertexIndex]->findParallelFaces(plane2, cache2, facesIndexesMesh2, facesIndexesMesh2Length, _epsilon);

						vertexIndex = allFacesMesh2[facesVertex2[j]]->vertexesIndexes[2];
						if (vertexIndex != _index)
							allVertexesMesh2[vertexIndex]->findParallelFaces(plane2, cache2, facesIndexesMesh2, facesIndexesMesh2Length, _epsilon);
					}

					return;
				}
			}
		}
	}

	void SpVertexMesh::findParallelFaces(const SpVertexMesh* vertex2,
		const SpTransform& meshTransform1, const SpTransform& meshTransform2,
		sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length,
		sp_uint* facesIndexesMesh2, sp_uint* facesIndexesMesh2Length,
		const sp_float _epsilon) const
	{
		SpFaceMesh** allFacesMesh1 = _mesh->faces->data();
		SpVertexMesh** allVertexesMesh1 = _mesh->vertexesMesh->data();
		sp_uint* facesVertex1 = facesIndexes->data();

		SpFaceMesh** allFacesMesh2 = vertex2->_mesh->faces->data();
		SpVertexMesh** allVertexesMesh2 = vertex2->_mesh->vertexesMesh->data();
		sp_uint* facesVertex2 = vertex2->facesIndexes->data();

		for (sp_uint i = 0; i < facesIndexes->length(); i++)
		{
			Plane plane1;
			allFacesMesh1[facesVertex1[i]]->convert(&plane1, meshTransform1);

			for (sp_uint j = 0; j < vertex2->facesIndexes->length(); j++)
			{
				Plane plane2;
				allFacesMesh2[facesVertex2[j]]->convert(&plane2, meshTransform2);

				// faces should be parallel and opposite normals (front-face agains front-face)
				if (plane1.isParallel(plane2, _epsilon)
					&& (plane1.normalVector - plane2.normalVector) != ZERO_FLOAT
					&& NAMESPACE_FOUNDATION::isCloseEnough(plane1.distance(plane2), ZERO_FLOAT, ERROR_MARGIN_PHYSIC))
				{
					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh1, facesIndexesMesh1Length[0], facesVertex1[i]))
					{
						facesIndexesMesh1[facesIndexesMesh1Length[0]] = facesVertex1[i];
						facesIndexesMesh1Length[0] ++;

						// check the faces of the left 2 vertex of this face  
						sp_uint vertexIndex = allFacesMesh1[facesVertex1[i]]->vertexesIndexes[0];
						if (vertexIndex != _index)
							allVertexesMesh1[vertexIndex]->findParallelFaces(plane1, meshTransform1, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

						vertexIndex = allFacesMesh1[facesVertex1[i]]->vertexesIndexes[1];
						if (vertexIndex != _index)
							allVertexesMesh1[vertexIndex]->findParallelFaces(plane1, meshTransform1, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);

						vertexIndex = allFacesMesh1[facesVertex1[i]]->vertexesIndexes[2];
						if (vertexIndex != _index)
							allVertexesMesh1[vertexIndex]->findParallelFaces(plane1, meshTransform1, facesIndexesMesh1, facesIndexesMesh1Length, _epsilon);
					}

					if (!NAMESPACE_FOUNDATION::contains(facesIndexesMesh2, facesIndexesMesh2Length[0], facesVertex2[j]))
					{
						facesIndexesMesh2[facesIndexesMesh2Length[0]] = facesVertex2[j];
						facesIndexesMesh2Length[0] ++;

						// check the faces of the left 2 vertex of this face
						sp_uint vertexIndex = allFacesMesh2[facesVertex2[j]]->vertexesIndexes[0];
						if (vertexIndex != _index)
							allVertexesMesh2[vertexIndex]->findParallelFaces(plane2, meshTransform2, facesIndexesMesh2, facesIndexesMesh2Length, _epsilon);

						vertexIndex = allFacesMesh2[facesVertex2[j]]->vertexesIndexes[1];
						if (vertexIndex != _index)
							allVertexesMesh2[vertexIndex]->findParallelFaces(plane2, meshTransform2, facesIndexesMesh2, facesIndexesMesh2Length, _epsilon);

						vertexIndex = allFacesMesh2[facesVertex2[j]]->vertexesIndexes[2];
						if (vertexIndex != _index)
							allVertexesMesh2[vertexIndex]->findParallelFaces(plane2, meshTransform2, facesIndexesMesh2, facesIndexesMesh2Length, _epsilon);
					}

					return;
				}
			}
		}
	}

}