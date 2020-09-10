#include "SpFaceMesh.h"

namespace NAMESPACE_PHYSICS
{

	SpFaceMesh::SpFaceMesh(SpMesh* mesh, sp_uint index)
	{
		this->mesh = mesh;
		this->_index = index;
	}

	SpFaceMesh::SpFaceMesh(SpMesh* mesh, sp_uint index, sp_uint vertexesIndexes1, sp_uint vertexesIndexes2, sp_uint vertexesIndexes3)
	{
		this->mesh = mesh;
		this->_index = index;

		this->vertexesIndexes[0] = vertexesIndexes1;
		this->vertexesIndexes[1] = vertexesIndexes2;
		this->vertexesIndexes[2] = vertexesIndexes3;
	}

	void SpFaceMesh::fillAttributes()
	{
		SpVertexMesh** vertexes = mesh->vertexesMesh->data();

		normal(
			vertexes[vertexesIndexes[0]]->value(),
			vertexes[vertexesIndexes[1]]->value(),
			vertexes[vertexesIndexes[2]]->value(),
			&faceNormal);
	}

	SpEdgeMesh* SpFaceMesh::edges(const sp_uint index) const
	{
		sp_assert(index < 3u, "IndexOutOfRangeException");

		return mesh->edges->data()[edgesIndexes[index]];
	}

	void SpFaceMesh::convert(Vec3* vertexes, const SpTransform& transform) const
	{
		mesh->vertex(vertexesIndexes[0], transform, &vertexes[0]);
		mesh->vertex(vertexesIndexes[1], transform, &vertexes[1]);
		mesh->vertex(vertexesIndexes[2], transform, &vertexes[2]);
	}

	void SpFaceMesh::convert(Plane3D* plane, const SpTransform& transform) const
	{
		mesh->vertex(vertexesIndexes[0], transform, &plane->point);
		rotate(transform.orientation, faceNormal, &plane->normalVector);
		plane->distanceFromOrigin = plane->normalVector.dot(plane->point);
	}

	void SpFaceMesh::convert(Triangle3D* triangle, const SpTransform& transform) const
	{
		mesh->vertex(vertexesIndexes[0], transform, &triangle->point1);
		mesh->vertex(vertexesIndexes[1], transform, &triangle->point2);
		mesh->vertex(vertexesIndexes[2], transform, &triangle->point3);
	}

	void SpFaceMesh::convert(Line3D lines[3], const SpTransform& transform) const
	{
		mesh->vertex(vertexesIndexes[0], transform, &lines[0].point1);
		mesh->vertex(vertexesIndexes[1], transform, &lines[0].point2);
		mesh->vertex(vertexesIndexes[2], transform, &lines[1].point2);

		lines[1].point1 = lines[0].point2;
		lines[2].point1 = lines[1].point2;
		lines[2].point2 = lines[0].point1;
	}

	sp_bool SpFaceMesh::isBackFace(const Vec3& point, const SpTransform& transformFace) const
	{
		Plane3D plane;
		convert(&plane, transformFace);

		return plane.isBackFace(point);
	}

	sp_bool SpFaceMesh::intersection(const Line3D& edge, Vec3* contactPoint, const SpTransform& faceTransform) const
	{
		Triangle3D tringle;
		convert(&tringle, faceTransform);
		
		return edge.intersection(tringle, contactPoint);
	}

	void SpFaceMesh::intersection(const SpFaceMesh* face, const SpTransform& meshTransform, const SpTransform& faceTransform, Ray* ray) const
	{
		Plane3D plane1;
		convert(&plane1, meshTransform);

		Plane3D plane2;
		face->convert(&plane2, faceTransform);

		plane1.intersection(plane2, ray);
	}

	sp_bool SpFaceMesh::intersection(const SpFaceMesh* face, const SpTransform& meshTransform, sp_uint* edgeIndex) const
	{
		for (sp_uint i = 0; i < 3u; i++)
		{
			if (face->vertexesIndexes[0] == vertexesIndexes[i])
			{
				edgeIndex[i];
				return true;
			}
			if (face->vertexesIndexes[1] == vertexesIndexes[i])
			{
				edgeIndex[i];
				return true;
			}
			if (face->vertexesIndexes[2] == vertexesIndexes[i])
			{
				edgeIndex[i];
				return true;
			}
		}

		return false;
	}

	sp_bool SpFaceMesh::isInside(const Vec3& vertex, const SpTransform& transformFace, const sp_float _epsilon ) const
	{
		Triangle3D triangle;
		this->convert(&triangle, transformFace);

		return triangle.isInside(vertex, _epsilon);
	}

}