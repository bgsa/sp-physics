#ifndef SP_MESH_HEADER
#define SP_MESH_HEADER

#include "SpectrumPhysics.h"
#include "Plane3D.h"
#include "Triangle3D.h"
#include "SpArray.h"
#include "SpTransform.h"
#include "SpVertexMesh.h"
#include "SpFaceMesh.h"
#include "SpEdgeMesh.h"

namespace NAMESPACE_PHYSICS
{
	class SpMesh
	{
	private:

		SpVertexMesh* findExtremeVertex(SpVertexMesh* from, const Vec3& orientation, const SpTransform& transform) const;

		sp_bool containsEdge(sp_uint* edge, sp_uint* vertexesIndexes, sp_uint* length) const;

	public:
		SpArray<SpVertexMesh*>* vertexesMesh;
		SpArray<SpFaceMesh*>* faces;
		SpArray<SpEdgeMesh*>* edges;
		
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE SpMesh()
		{
			vertexesMesh = nullptr;
			faces = nullptr;
			edges = nullptr;
		}

		/// <summary>
		/// Initialize vertexes, edges and faces
		/// </summary>
		/// <returns></returns>
		API_INTERFACE void init();

		API_INTERFACE void vertex(const sp_uint index, Vec3* output) const;

		API_INTERFACE void vertex(const sp_uint index, const SpTransform& transform, Vec3* output) const;

		API_INTERFACE inline sp_uint vertexLength() const;
	
		API_INTERFACE SpVertexMesh* findExtremeVertex(const Vec3& orientation, const SpTransform& transform, SpVertexMesh* from = nullptr) const;

		/// <summary>
		/// Find the closest point given an arbitrary point
		/// </summary>
		/// <param name="point">Arbitrary point</param>
		/// <param name="transform">Transformation</param>
		/// <param name="from">(optional) Searching from this point</param>
		/// <returns>Closest vertex</returns>
		API_INTERFACE SpVertexMesh* findClosest(const Vec3& point, const SpTransform& transform, SpVertexMesh* from = nullptr) const;

		API_INTERFACE sp_bool isInside(const SpMesh* mesh2, const SpTransform& transformObj1, const SpTransform& transformObj2) const;

		API_INTERFACE sp_uint findEdge(const sp_uint vertexIndex1, const sp_uint vertexIndex2) const;
	
		API_INTERFACE void convert(Vec3* vertexes, const SpTransform& transform) const;

	};

}

#endif // SP_MESH_HEADER