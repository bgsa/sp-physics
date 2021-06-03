#ifndef SP_MESH_HEADER
#define SP_MESH_HEADER

#include "SpectrumPhysics.h"
#include "Plane.h"
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

		SpVertexMesh* support(SpVertexMesh* from, const Vec3& orientation, const SpTransform& transform) const;
		SpVertexMesh* support(SpVertexMesh* from, const Vec3& orientation, const Vec3* vertexes) const;

		void supportAll(SpVertexMesh* from, const Vec3& orientation, const Vec3* vertexes, sp_uint& outputLength, SpVertexMesh** output, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;

		SpVertexMesh* findExtremeVertexPoint(SpVertexMesh* from, const Vec3& target, const SpMeshCache* cache, const Vec3& center) const;

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

		API_INTERFACE sp_uint vertexLength() const;
	
		API_INTERFACE SpVertexMesh* support(const Vec3& direction, const SpTransform& transform, SpVertexMesh* from) const;
		API_INTERFACE SpVertexMesh* support(const Vec3& direction, const Vec3* vertexes, SpVertexMesh* startingFrom) const;

		API_INTERFACE void supportAll(const Vec3& direction, const Vec3* vertexes, SpVertexMesh* startingFrom, sp_uint& outputLength, SpVertexMesh** output, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;
		API_INTERFACE void supportAll(const Vec3& direction, const Vec3* vertexes, sp_uint& outputLength, SpVertexMesh** output, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;

		API_INTERFACE SpVertexMesh* support(const Vec3& direction, const Vec3* vertexes) const
		{
			return support(vertexesMesh->get(0), direction, vertexes);
		}

		API_INTERFACE SpVertexMesh* findExtremeVertexPoint(const Vec3& target, const SpMeshCache* cache, const Vec3& center, SpVertexMesh* startingFrom) const;

		API_INTERFACE void findAllClosestDetails(const Vec3& target, const SpMeshCache* cache, const Vec3& center,
			Vec3* closestPoint,
			sp_uint* closestVertexMesh, sp_uint* closestEdgeMesh, sp_uint* closestFaceMesh,
			SpVertexMesh* startingFrom) const;

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
	
		API_INTERFACE sp_uint findVertex(const Vec3 value) const;

		API_INTERFACE void convert(Vec3* vertexes, const SpTransform& transform) const;

		API_INTERFACE void surfaceSquareSubdivision();

		/// <summary>
		/// Get the size in bytes of the mesh
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_size size() const;

		/// <summary>
		/// Get the index memory of the Vertexes, Faces and Edges
		/// </summary>
		/// <param name="indexes">Mesh's Indexes</param>
		/// <returns>indexes parameter</returns>
		API_INTERFACE inline void indexes(sp_size indexes[3], const sp_size shift = ZERO_SIZE) const
		{
			const sp_size initialMemoryIndex = (sp_size)this;
			sp_size vertexMemoryIndex1 = (sp_size)vertexesMesh->data()[0];
			sp_size faceMemoryIndex1 = (sp_size)faces->data()[0];
			sp_size edgeMemoryIndex1 = (sp_size)edges->data()[0];

			indexes[0] = divideBy4(vertexMemoryIndex1 - initialMemoryIndex) + 1 + shift;
			indexes[1] = divideBy4(faceMemoryIndex1 - initialMemoryIndex) + shift;
			indexes[2] = divideBy4(edgeMemoryIndex1 - initialMemoryIndex) + shift;
		}

		/// <summary>
		/// Get the strides of Vertexes, Faces and Edges
		/// </summary>
		/// <param name="strides">Mesh's Strides</param>
		/// <returns>strides parameter</returns>
		API_INTERFACE inline void strides(sp_uint strides[3]) const
		{
			const sp_size initialMemoryIndex = (sp_size)this;
			sp_size vertexMemoryIndex1 = (sp_size)vertexesMesh->data()[0];
			sp_size vertexMemoryIndex2 = (sp_size)vertexesMesh->data()[1];
			sp_size faceMemoryIndex1 = (sp_size)faces->data()[0];
			sp_size faceMemoryIndex2 = (sp_size)faces->data()[1];
			sp_size edgeMemoryIndex1 = (sp_size)edges->data()[0];
			sp_size edgeMemoryIndex2 = (sp_size)edges->data()[1];

			strides[0] = (sp_uint)divideBy4(vertexMemoryIndex2 - vertexMemoryIndex1); // vertex stride for mesh 1
			strides[1] = (sp_uint)divideBy4(faceMemoryIndex2 - faceMemoryIndex1); // faces stride for mesh 1
			strides[2] = (sp_uint)divideBy4(edgeMemoryIndex2 - edgeMemoryIndex1); // edge stride for mesh 1
		}

	};

}

#endif // SP_MESH_HEADER