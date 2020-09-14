#include "SpEdgeMesh.h"

namespace NAMESPACE_PHYSICS
{

	void SpEdgeMesh::fillAttributes()
	{
		SpFaceMesh** allFaces = mesh->faces->data();

		// fill isBoundaryEdge attribute
		if (faces.length() == ONE_UINT)
			_isBoundaryEdge = true;
		else
		{
			const Vec3 firstNormalFace = allFaces[faces[0]]->faceNormal;
			const Vec3 secondNormalFace = allFaces[faces[1]]->faceNormal;

			_isBoundaryEdge = !firstNormalFace.isCloseEnough(secondNormalFace);
		}
	}

	SpEdgeMesh::SpEdgeMesh(SpMesh* mesh, sp_uint index, sp_uint vertexIndex1, sp_uint vertexIndex2, const SpFaceMesh& face)
	{
		this->mesh = mesh;
		this->_index = index;
		this->vertexIndex1 = vertexIndex1;
		this->vertexIndex2 = vertexIndex2;
		this->addFace(face);
	}

	void SpEdgeMesh::convert(Line3D* line, const SpTransform& transform) const
	{
		mesh->vertex(vertexIndex1, transform, &line->point1);
		mesh->vertex(vertexIndex2, transform, &line->point2);
	}

	void SpEdgeMesh::convert(Ray* ray, const SpTransform& transform) const
	{
		mesh->vertex(vertexIndex1, transform, &ray->point);
		mesh->vertex(vertexIndex2, transform, &ray->direction);

		diff(ray->direction, ray->point, &ray->direction);
		
		normalize(&ray->direction);
	}

	void SpEdgeMesh::addFace(const SpFaceMesh& face)
	{
		if (faces.length() > ZERO_UINT && faces[0] == face.index())
			return;

		if (faces.length() > ONE_UINT && faces[1] == face.index())
			return;
	
		sp_assert(faces.length() < TWO_UINT, "IndexOutOfRangeException");

		faces.add(face.index());
	}

	sp_bool SpEdgeMesh::intersection(const SpFaceMesh& face, Vec3* contactPoint, const SpTransform& transormEdge, const SpTransform& transormFace) const
	{
		Line3D line;
		convert(&line, transormEdge);

		Triangle3D plane;
		face.convert(&plane, transormFace);

		return line.intersection(plane, contactPoint);
	}

	sp_bool SpEdgeMesh::intersection(const SpEdgeMesh* edge2, Vec3* contactPoint, const SpTransform& transormEdge1, const SpTransform& transormEdge2, const sp_float _epsilon) const
	{
		Line3D line1;
		convert(&line1, transormEdge1);

		Line3D line2;
		edge2->convert(&line2, transormEdge2);

		return line1.intersection(line2, contactPoint, _epsilon);
	}

}