#ifndef SP_EDGE_MESH_HEADER
#define SP_EDGE_MESH_HEADER

#include "SpectrumPhysics.h"
#include "SpMesh.h"
#include "SpFaceMesh.h"
#include "SpArray.h"

namespace NAMESPACE_PHYSICS
{
	class SpEdgeMesh
	{
		friend class SpMesh;
	private:
		SpMesh* mesh;
		sp_uint _index;
		sp_bool _isBoundaryEdge;

		void fillAttributes();
		
	public:
		sp_uint vertexIndex1;
		sp_uint vertexIndex2;
		SpArray<sp_uint> faces = SpArray<sp_uint>(2u);

		API_INTERFACE SpEdgeMesh(SpMesh* mesh, sp_uint index)
		{
			this->mesh = mesh;
			this->_index = index;
		}

		API_INTERFACE SpEdgeMesh(SpMesh* mesh, sp_uint index, sp_uint vertexIndex1, sp_uint vertexIndex2)
		{
			this->mesh = mesh;
			this->_index = index;
			this->vertexIndex1 = vertexIndex1;
			this->vertexIndex2 = vertexIndex2;
		}
		
		API_INTERFACE SpEdgeMesh(SpMesh* mesh, sp_uint index, sp_uint vertexIndex1, sp_uint vertexIndex2, const SpFaceMesh& face);

		API_INTERFACE inline sp_uint index() const
		{
			return _index;
		}

		API_INTERFACE sp_bool operator==(const SpEdgeMesh& edge) const
		{
			return (_index == edge._index)
				|| (vertexIndex1 == edge.vertexIndex1 && vertexIndex2 == edge.vertexIndex2)
				|| (vertexIndex1 == edge.vertexIndex2 && vertexIndex1 == edge.vertexIndex1);
		}

		API_INTERFACE void convert(Line3D* line, const SpTransform& transform) const;
		
		API_INTERFACE void convert(Ray* ray, const SpTransform& transform) const;

		API_INTERFACE inline sp_bool isBoundaryEdge() const
		{
			return _isBoundaryEdge;
		}

		API_INTERFACE void addFace(const SpFaceMesh& face);

		API_INTERFACE sp_bool intersection(const SpFaceMesh& face, Vec3* contactPoint, const SpTransform& transormEdge, const SpTransform& transormFace) const;

		API_INTERFACE sp_bool intersection(const SpEdgeMesh* face, Vec3* contactPoint, const SpTransform& transormEdge1, const SpTransform& transormEdge2, const sp_float _epsilon = DefaultErrorMargin) const;

	};

}

#endif // SP_EDGE_MESH_HEADER