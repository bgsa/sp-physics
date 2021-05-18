#ifndef SP_VERTEX_MESH_HEADER
#define SP_VERTEX_MESH_HEADER

#include "SpectrumPhysics.h"
#include "Plane.h"
#include "SpPoint3.h"
#include "Triangle3D.h"
#include "SpArray.h"
#include "SpTransform.h"
#include "SpMesh.h"
#include "SpMeshCache.h"
#include "SpVector.h"

namespace NAMESPACE_PHYSICS
{
	class SpVertexMesh
	{
		friend class SpMesh;
	private:
		sp_uint _index;
		Vec3 _value;
		SpMesh* _mesh;
		SpArray<sp_uint>* _edges;
		SpArray<sp_uint>* _edgeVertexIndex;
		SpArray<sp_uint>* facesIndexes;

	public:

		API_INTERFACE Vec3 value() const;

		API_INTERFACE inline SpMesh* mesh() const
		{
			return _mesh;
		}

		API_INTERFACE inline sp_uint edgeLength() const
		{
			return _edges->length();
		}

		/// <summary>
		/// Get the Edge Object, given a local edge index
		/// </summary>
		/// <param name="localEdgeIndex">Local Edge Index</param>
		/// <returns>Edge Object</returns>
		API_INTERFACE SpEdgeMesh* edges(const sp_uint localEdgeIndex) const;

		/// <summary>
		/// Get the global edge index, given a vertex index
		/// </summary>
		/// <param name="vertexIndex">Local Vertex Index</param>
		/// <returns>Global Vertex Index</returns>
		API_INTERFACE inline sp_uint edgeVertexIndex(const sp_uint vertexIndex) const
		{
			sp_assert(vertexIndex < _edgeVertexIndex->length(), "InvalidArgumentException");
			return _edgeVertexIndex->get(vertexIndex);
		}
		
		/// <summary>
		/// Get the Edge Object, given a vertex index
		/// </summary>
		/// <param name="vertexIndex">Local Vertex Index</param>
		/// <returns>Edge Object</returns>
		API_INTERFACE SpEdgeMesh* findEdges(const sp_uint vertexIndex) const;

		API_INTERFACE inline sp_uint faceIndexLength() const
		{
			return facesIndexes->length();
		}
		API_INTERFACE inline sp_uint faceIndex(const sp_uint index) const
		{
			return facesIndexes->get(index);
		}
		API_INTERFACE SpFaceMesh* face(const sp_uint index) const;

		API_INTERFACE inline sp_bool operator==(const Vec3& _value) const
		{
			return value() == _value;
		}

		API_INTERFACE inline sp_bool operator==(const SpVertexMesh* value) const
		{
			return _index == value->_index;
		}

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="mesh">Where this vertex belong</param>
		/// <param name="vertexIndex">Invex of this vertex</param>
		/// <returns>SpVertexMesh</returns>
		API_INTERFACE SpVertexMesh(SpMesh* mesh, const sp_uint vertexIndex)
		{
			_index = vertexIndex;
			_mesh = mesh;
			facesIndexes = sp_mem_new(SpArray<sp_uint>)(10u);
		}

		/// <summary>
		/// Constructor
		/// </summary>
		/// <param name="mesh">Where this vertex belong</param>
		/// <param name="vertexIndex">Invex of this vertex</param>
		/// <returns>SpVertexMesh</returns>
		API_INTERFACE SpVertexMesh(SpMesh* mesh, const sp_uint vertexIndex, const Vec3& value)
		{
			_index = vertexIndex;
			_mesh = mesh;
			facesIndexes = sp_mem_new(SpArray<sp_uint>)(10u);
			this->_value = value;
		}

		/// <summary>
		/// The index of this vertex on mesh
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint index() const
		{
			return _index;
		}

		/// <summary>
		/// Add a double edge from this vertex and the vertex parameter
		/// </summary>
		/// <param name="vertexEdge">Other vertex to build edges</param>
		/// <returns>void</returns>
		API_INTERFACE inline void addEdge(SpVertexMesh* vertexEdge)
		{
			if (vertexEdge->_index == _index)
				return;

			for (sp_uint i = 0 ; i < _edgeVertexIndex->length(); i++)
				if (_edgeVertexIndex->data()[i] == vertexEdge->_index)
					return;

			vertexEdge->addEdge(this);
		}

		/// <summary>
		/// Find parallel vertex from this vertex and a orientation
		/// </summary>
		/// <param name="plane">Orientation</param>
		/// <param name="meshTransform">Transformation</param>
		/// <param name="vertexesOutput">Vertexes</param>
		/// <param name="vertexOutputLength">Vertexes length</param>
		/// <param name="_epsilon">Error margin</param>
		API_INTERFACE void findParallelVertexes(const Plane& plane, const SpTransform& meshTransform,
			Vec3* vertexesOutput, sp_uint* vertexOutputLength, sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Find parallel vertex from this vertex and a orientation
		/// </summary>
		/// <param name="plane">Orientation</param>
		/// <param name="cache">Cache vertexes mesh</param>
		/// <param name="vertexesOutput">Vertexes</param>
		/// <param name="vertexOutputLength">Vertexes length</param>
		/// <param name="_epsilon">Error margin</param>
		API_INTERFACE void findParallelVertexes(const Plane& plane, const SpMeshCache* cache,
			Vec3* vertexesOutput, sp_uint* vertexOutputLength, sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Find parallel faces starting from this vertex of mesh 1 and vertex2 of mesh 2.
		/// </summary>
		/// <param name="vertex2">Second mesh starting from this vertex parameter</param>
		/// <param name="meshTransform1">Transformation of mesh 1</param>
		/// <param name="meshTransform2">Transformation of mesh 2</param>
		/// <param name="facesIndexesMesh1">Output faces indexes of mesh 1</param>
		/// <param name="facesIndexesMesh2">Output faces indexes of mesh 2</param>
		/// <param name="_epsilon">Error margin</param>
		/// <returns>void</returns>
		API_INTERFACE void findParallelFaces(const SpVertexMesh* vertex2, 
			const SpTransform& meshTransform1, const SpTransform& meshTransform2, 
			sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length, 
			sp_uint* facesIndexesMesh2, sp_uint* facesIndexesMesh2Length,
			const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Find parallel faces starting from this vertex of mesh 1 and vertex2 of mesh 2.
		/// This version uses cached transformed vertexes.
		/// </summary>
		/// <param name="vertex2">Second mesh starting from this vertex parameter</param>
		/// <param name="cache1">Cache vertexes mesh 1</param>
		/// <param name="cache2">Cache vertexes mesh 2</param>
		/// <param name="facesIndexesMesh1">Output faces indexes of mesh 1</param>
		/// <param name="facesIndexesMesh2">Output faces indexes of mesh 2</param>
		/// <param name="_epsilon">Error margin</param>
		/// <returns>void</returns>
		API_INTERFACE void findParallelFaces(const SpVertexMesh* vertex2,
			const SpMeshCache* cache1, const SpMeshCache* cache2,
			sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length,
			sp_uint* facesIndexesMesh2, sp_uint* facesIndexesMesh2Length,
			const sp_float _epsilon = DefaultErrorMargin) const;
		
		
		/// <summary>
		/// Find the faces of this vertex using all elements from faces indexes of the mesh
		/// </summary>
		/// <param name="outputFaces">Faces</param>
		/// <param name="facesLength">Length</param>
		/// <param name="transform">Vertexes transformation</param>
		/// <returns>void</returns>
		API_INTERFACE void findFacesFromIndexes(Triangle3D* outputFaces, sp_uint* facesLength, const SpTransform* transform) const;

		/// <summary>
		/// Find the closest vertex given an arbitrary point
		/// </summary>
		/// <param name="point">Arbitrary point</param>
		/// <param name="transform">Transformation</param>
		/// <returns>Vertex found or null if not found</returns>
		API_INTERFACE SpVertexMesh* findClosest(const Vec3& point, const SpTransform& transform);

		/// <summary>
		/// Find parallel faces with the plane
		/// </summary>
		/// <param name="plane">Plane</param>
		/// <param name="meshTransform">Mesh transformation</param>
		/// <param name="facesIndexesMesh1">List of parallel faces indexes</param>
		/// <param name="facesIndexesMesh1Length">List Length of parallel faces indexes</param>
		/// <param name="_epsilon">Error margin</param>
		/// <returns>void</returns>
		API_INTERFACE void findParallelFaces(const Plane& plane, const SpTransform& meshTransform,
			sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length, const sp_float _epsilon) const;

		/// <summary>
		/// Find parallel faces with the plane
		/// </summary>
		/// <param name="plane">Plane</param>
		/// <param name="cache">Cache vertex mesh</param>
		/// <param name="facesIndexesMesh1">List of parallel faces indexes</param>
		/// <param name="facesIndexesMesh1Length">List Length of parallel faces indexes</param>
		/// <param name="_epsilon">Error margin</param>
		/// <returns>void</returns>
		API_INTERFACE void findParallelFaces(const Plane& plane, const SpMeshCache* cache,
			sp_uint* facesIndexesMesh1, sp_uint* facesIndexesMesh1Length, const sp_float _epsilon) const;

	};

}

#endif // SP_VERTEX_MESH_HEADER