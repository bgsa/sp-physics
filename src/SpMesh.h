#ifndef SP_MESH_HEADER
#define SP_MESH_HEADER

#include "SpectrumPhysics.h"
#include "SpVector.h"
#include "Quat.h"
#include "Plane3D.h"
#include "SpPoint3.h"
#include "SpPoint2.h"
#include "Triangle3D.h"
#include "SpArray.h"
#include "SpCollisionDetails.h"
#include "SpPhysicProperties.h"

namespace NAMESPACE_PHYSICS
{
	class SpVertexEdges
	{
		friend class SpMesh;
	private:
		sp_uint _vertexIndex;
		SpVector<SpVertexEdges*>* _edges;
		SpMesh* mesh;

		inline SpVertexEdges* find(const sp_uint vertexIndex, SpVector<sp_uint>& visitedVertexes)
		{
			// if looked up vertex is this, return this
			if (_vertexIndex == vertexIndex)
				return this;

			// check if this vertex was already visited
			for (SpVectorItem<sp_uint>* item = visitedVertexes.begin(); item != nullptr; item = item->next())
				if (item->value() == _vertexIndex)
					return nullptr;

			visitedVertexes.add(_vertexIndex);

			// search for vertex index on edges
			for (SpVectorItem<SpVertexEdges*>* item = _edges->begin(); item != nullptr; item = item->next())
			{
				SpVertexEdges* result = item->value()->find(vertexIndex, visitedVertexes);
				if (result != nullptr)
					return result;
			}

			return nullptr;
		}

	public:
		
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="mesh">Where this vertex belong</param>
		/// <param name="vertexIndex">Invex of this vertex</param>
		/// <returns>SpVertexEdge</returns>
		API_INTERFACE SpVertexEdges(SpMesh* mesh, const sp_uint vertexIndex)
		{
			_vertexIndex = vertexIndex;
			_edges = sp_mem_new(SpVector<SpVertexEdges*>)();
			this->mesh = mesh;
		}

		/// <summary>
		/// The index of this vertex on mesh
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint vertexIndex() const
		{
			return _vertexIndex;
		}

		/// <summary>
		/// Find a vertex by index, starting from this vertex
		/// </summary>
		/// <param name="vertexIndex">Index to look up</param>
		/// <returns>Vertex found</returns>
		API_INTERFACE inline SpVertexEdges* find(const sp_uint vertexIndex)
		{
			SpVector<sp_uint> visitedVertexes;
			return find(vertexIndex, visitedVertexes);
		}

		/// <summary>
		/// Add a double edge from this vertex and the vertex parameter
		/// </summary>
		/// <param name="vertexEdge">Other vertex to build edges</param>
		/// <returns>void</returns>
		API_INTERFACE inline void addEdge(SpVertexEdges* vertexEdge) 
		{
			if (vertexEdge->_vertexIndex == _vertexIndex)
				return;

			for (SpVectorItem<SpVertexEdges*>* item = _edges->begin(); item != nullptr; item = item->next())
				if (item->value()->_vertexIndex == vertexEdge->_vertexIndex)
					return;

			_edges->add(vertexEdge);
			vertexEdge->addEdge(this);
		}

		/// <summary>
		/// Find parallel vertex from this vertex and a orientation
		/// </summary>
		/// <param name="orientation">Build a support plane from this vertex and orientation</param>
		/// <param name="vertexes">Output vertexes</param>
		/// <param name="vertexLength">Output vertexes length</param>
		/// <param name="ignoreVertexIndex">Ignore this index in order to avoid cyclic graph</param>
		/// <param name="_epsilon">Error margin of the distance to accept</param>
		/// <returns></returns>
		API_INTERFACE void findParallelVertexes(const Plane3D& plane, const SpTransform& meshTransform, SpVertexEdges** vertexesOutput, 
			sp_uint* vertexOutputLength, sp_uint ignoreVertexIndex = SP_UINT_MAX, 
			sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Find the faces of this vertex using his edges and check the orientation of the others vertexes
		/// </summary>
		/// <param name="outputFaces">Faces</param>
		/// <param name="facesLength">Length</param>
		/// <param name="transform">Vertexes transformation</param>
		/// <returns>void</returns>
		API_INTERFACE void findFacesFromEdges(Triangle3D* outputFaces, sp_uint* facesLength, const SpTransform* transform) const;
		
		/// <summary>
		/// Find the faces of this vertex using all elements from faces indexes of the mesh
		/// </summary>
		/// <param name="outputFaces">Faces</param>
		/// <param name="facesLength">Length</param>
		/// <param name="transform">Vertexes transformation</param>
		/// <returns>void</returns>
		API_INTERFACE void findFacesFromIndexes(Triangle3D* outputFaces, sp_uint* facesLength, const SpTransform* transform) const;

	};

	class SpMesh
	{
	private:
		SpVertexEdges** _vertexEdges;

		void allEdges(SpVertexEdges* startingFrom, sp_uint* vertexesIndexes, sp_uint* length) const
		{
			sp_uint edgeIndex[2];
			edgeIndex[0] = startingFrom->_vertexIndex;

			for (SpVectorItem<SpVertexEdges*>* item = startingFrom->_edges->begin(); item != nullptr; item = item->next())
			{
				edgeIndex[1] = item->value()->_vertexIndex;

				if (!containsEdge(edgeIndex, vertexesIndexes, length))
				{
					vertexesIndexes[multiplyBy2(*length)] = edgeIndex[0];
					vertexesIndexes[multiplyBy2(*length) + 1] = edgeIndex[1];
					length[0] ++;

					allEdges(item->value(), vertexesIndexes, length);
				}
			}
		}

		SpVertexEdges* findExtremeVertex(SpVertexEdges* from, const Vec3& orientation, const SpTransform* transform) const
		{
			const Plane3D halfSpace = Plane3D(transform->position, orientation);

			Vec3 newVertexPosition;
			transform->transform(vertexes->data()[from->_vertexIndex], &newVertexPosition);

			const sp_float distance = halfSpace.distance(newVertexPosition);

			SpVectorItem<SpVertexEdges*>* item = from->_edges->begin();

			while (item != nullptr)
			{
				Vec3 vertexPosition2;
				transform->transform(vertexes->data()[item->value()->_vertexIndex], &vertexPosition2);

				const sp_float newDistance = halfSpace.distance(vertexPosition2);

				if (newDistance > distance)
					return findExtremeVertex(item->value(), orientation, transform);

				item = item->next();
			}

			return from;
		}

		sp_bool containsEdge(sp_uint* edge, sp_uint* vertexesIndexes, sp_uint* length) const
		{
			for (sp_uint i = 0; i < *length; i++)
				if (
					(vertexesIndexes[multiplyBy2(i)] == edge[0] && vertexesIndexes[multiplyBy2(i) + 1u] == edge[1]) ||
					(vertexesIndexes[multiplyBy2(i)] == edge[1] && vertexesIndexes[multiplyBy2(i) + 1u] == edge[0]))
					return true;

			return false;
		}

		inline void allEdges(sp_uint* vertexesIndexes, sp_uint* length) const
		{
			*length = ZERO_UINT;
			allEdges(_vertexEdges[0], vertexesIndexes, length);
		}

	public:
		SpArray<Vec3>* vertexes;
		SpArray<SpPoint3<sp_uint>>* facesIndexes;
		SpArray<SpPoint2<sp_uint>>* edgesIndexes;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE SpMesh()
		{
			_vertexEdges = nullptr;
			vertexes = nullptr;
			facesIndexes = nullptr;
			edgesIndexes = nullptr;
		}

		/// <summary>
		/// Initialize vertexes, edges and faces
		/// </summary>
		/// <returns></returns>
		API_INTERFACE void init()
		{
			if (facesIndexes == nullptr || facesIndexes->length() == ZERO_UINT)
				return;

			_vertexEdges = sp_mem_new_array(SpVertexEdges*, vertexes->length());
			for (sp_uint i = 0; i < vertexes->length(); i++)
				_vertexEdges[i] = sp_mem_new(SpVertexEdges)(this, i);

			for (sp_uint i = 0; i < facesIndexes->length(); i++)
			{
				const SpPoint3<sp_uint> indexes = facesIndexes->data()[i];

				_vertexEdges[indexes.x]->addEdge(_vertexEdges[indexes.y]);
				_vertexEdges[indexes.y]->addEdge(_vertexEdges[indexes.z]);
				_vertexEdges[indexes.z]->addEdge(_vertexEdges[indexes.x]);
			}

			sp_uint* tempEdges = ALLOC_ARRAY(sp_uint, vertexes->length() * 2u * 10u);
			sp_uint edgesLength;
			allEdges(tempEdges, &edgesLength);
			ALLOC_RELEASE(tempEdges);

			edgesIndexes = sp_mem_new(SpArray<SpPoint2<sp_uint>>)(edgesLength, edgesLength);
			allEdges((sp_uint*)edgesIndexes->data(), &edgesLength);
		}

		API_INTERFACE CollisionStatus collisionStatus(const SpMesh* mesh2,
			const SpTransform* transform1, const SpTransform* transform2,
			SpCollisionDetails* details);
		
		API_INTERFACE inline SpVertexEdges** vertexEdges() const
		{
			return _vertexEdges;
		}

		API_INTERFACE inline SpVertexEdges* findExtremeVertex(const Vec3& orientation, const SpTransform* transform, SpVertexEdges* from = nullptr) const
		{
			if (from == nullptr)
				from = _vertexEdges[0];

			return findExtremeVertex(from, orientation, transform);
		}

	};

}

#endif // SP_MESH_HEADER