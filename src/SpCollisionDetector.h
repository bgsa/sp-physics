#ifndef SP_COLLISION_DETECTOR_HEADER
#define SP_COLLISION_DETECTOR_HEADER

#include "SpectrumPhysics.h"
#include "SpCollisionDetails.h"
#include "SpCollisionFeatures.h"
#include "SpMeshCache.h"
#include "SpCollisionDetectorCache.h"
#include "SpWorldManager.h"

namespace NAMESPACE_PHYSICS
{

	class SpCollisionDetector
	{
	private:
		
		void fillCollisionDetails(SpCollisionDetails* details);
		
		void timeOfCollision(SpCollisionDetails* details);

		sp_bool findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, 
			sp_uint* vertexIndexObj1, Vec3* contactPoint, SpCollisionDetectorCache* cache,
			SpMeshCache* cacheMesh1, SpMeshCache* cacheMesh2, SpCollisionDetails* details);

		sp_bool isFaceFaceCollision(SpCollisionDetails* details) const;

		sp_bool hasPlaneCollision(sp_uint objIndex1, sp_uint objIndex2, SpCollisionDetails* details, SpCollisionDetectorCache* cache) const;

		/// <summary>
		/// Check all faces from first vertex against edges from second vertex
		/// </summary>
		/// <param name="vertex1">First vertex</param>
		/// <param name="vertex2">Second vertx</param>
		/// <param name="transform1">Transformation mesh 1</param>
		/// <param name="transform2">Transformation mesh 2</param>
		/// <param name="faceIndexObj1">Face Index Obj 1</param>
		/// <param name="vertexIndexObj2">Edge Index Obj 2</param>
		/// <returns>True if it is Edge-Face collision or else False</returns>
		sp_bool isEdgeFaceCollision(SpVertexMesh* vertex1, SpVertexMesh* vertex2, 
			const SpMeshCache* cacheMesh1, const SpMeshCache* cacheMesh2,
			sp_uint* faceIndexObj1, sp_uint* vertexIndexObj2) const;

		sp_bool isEdgeEdgeCollision(SpCollisionDetails* details) const;

		sp_bool isEdgeFaceCollisionObj1(SpCollisionDetails* details) const;
		sp_bool isEdgeFaceCollisionObj2(SpCollisionDetails* detail) const;

		sp_bool fillCollisionDetailsEdgeEdge(const Line3D& edge, SpVertexMesh* vertex, const SpTransform& vertexTransform, sp_uint* edgeIndexOutput, const sp_float _epsilon = DefaultErrorMargin);

		sp_bool collisionStatusCache(SpCollisionDetectorCache* cache, Vec3* contactPoint, SpCollisionDetails* details);

		sp_bool isVertexFaceCollision(SpCollisionDetails* details) const;

	public:

		API_INTERFACE void collisionDetails(SpCollisionDetails* details);

		API_INTERFACE void filterCollision(SpCollisionDetails* details) const;

		API_INTERFACE CollisionStatus collisionStatus(Vec3* contactPoint, SpCollisionDetectorCache* cache, SpCollisionDetails* details);

		API_INTERFACE sp_bool areMovingAway(sp_uint objIndex1, sp_uint objIndex2) const;
		
		API_INTERFACE sp_bool hasCollision(sp_uint objIndex1, sp_uint objIndex2, SpCollisionDetails* details, SpCollisionDetectorCache* cache) const;

		API_INTERFACE sp_bool hasCollisionCache(const SpMesh* mesh1, const SpMesh* mesh2, SpCollisionDetails* details, SpCollisionDetectorCache* cache) const;

	};

}

#endif // SP_COLLISION_DETECTOR_HEADER