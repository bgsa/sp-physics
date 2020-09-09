#ifndef SP_FACE_MESH_HEADER
#define SP_FACE_MESH_HEADER

#include "SpectrumPhysics.h"
#include "SpMesh.h"
#include "Plane3D.h"

namespace NAMESPACE_PHYSICS
{
	class SpFaceMesh
	{
	private:
		SpMesh* mesh;
		sp_uint _index;

	public:
		sp_uint vertexesIndexes[3];
		sp_uint edgesIndexes[3];

		API_INTERFACE inline sp_uint index() const
		{
			return _index;
		}

		API_INTERFACE SpFaceMesh() { }

		API_INTERFACE SpFaceMesh(SpMesh* mesh, sp_uint index);

		API_INTERFACE SpFaceMesh(SpMesh* mesh, sp_uint index, sp_uint edgesIndexes1, sp_uint edgesIndexes2, sp_uint edgesIndexes3);

		API_INTERFACE void convert(Plane3D* plane, const SpTransform& transorm) const;
		API_INTERFACE void convert(Triangle3D* triangle, const SpTransform& transorm) const;
		API_INTERFACE void convert(Line3D lines[3], const SpTransform& transorm) const;
		API_INTERFACE void convert(Vec3* vertexes, const SpTransform& transorm) const;

		API_INTERFACE void normalVector(Vec3* normalVector) const;
		API_INTERFACE void normalVector(Vec3* normalVector, const SpTransform& transform) const;

		API_INTERFACE SpEdgeMesh* edges(const sp_uint index) const;

		API_INTERFACE sp_bool isBackFace(const Vec3& point, const SpTransform& transformFace) const;
	
		API_INTERFACE inline sp_bool isFrontFace(const Vec3& point, const SpTransform& transformFace) const
		{
			return !isBackFace(point, transformFace);
		}

		API_INTERFACE sp_bool intersection(const Line3D& edge, Vec3* contactPoint, const SpTransform& faceTransform) const;

		API_INTERFACE void intersection(const SpFaceMesh* face, const SpTransform& meshTransform, const SpTransform& faceTransform, Ray* ray) const;

		API_INTERFACE sp_bool intersection(const SpFaceMesh* face, const SpTransform& meshTransform, sp_uint* edgeIndex) const;

		API_INTERFACE sp_bool isInside(const Vec3& vertex, const SpTransform& transformFace, const sp_float _epsilon = DefaultErrorMargin) const;

	};

	API_INTERFACE inline sp_bool contains(sp_uint length, SpFaceMesh** faces, SpFaceMesh* face)
	{
		for (sp_uint i = 0; i < length; i++)
			if (faces[i]->index() == face->index())
				return true;

		return false;
	}

}

#endif // SP_FACE_MESH_HEADER