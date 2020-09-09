#ifndef SP_COLLISION_DETECTOR_HEADER
#define SP_COLLISION_DETECTOR_HEADER

#include "SpectrumPhysics.h"
#include "SpCollisionDetails.h"
#include "SpThreadPool.h"
#include "SpPhysicSimulator.h"
#include "SpCollisionFeatures.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionDetector
	{
	private:
		
		void fillCollisionDetails(SpCollisionDetails* details);
		
		void timeOfCollision(SpCollisionDetails* details);

		sp_bool findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, 
			sp_uint* vertexIndexObj1, sp_uint* vertexIndexObj2, 
			SpCollisionDetails* details);

		sp_bool isFaceFaceCollision(SpCollisionDetails* details) const;

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
			const SpTransform& transform1, const SpTransform& transform2, 
			sp_uint* faceIndexObj1, sp_uint* vertexIndexObj2) const;

		sp_bool isEdgeEdgeCollision(SpCollisionDetails* details) const;

		sp_bool isEdgeFaceCollisionObj1(SpCollisionDetails* details) const;
		sp_bool isEdgeFaceCollisionObj2(SpCollisionDetails* details) const;

		sp_bool fillCollisionDetailsEdgeEdge(const Line3D& edge, SpVertexMesh* vertex, const SpTransform& vertexTransform, sp_uint* edgeIndexOutput, const sp_float _epsilon = DefaultErrorMargin);

	public:

		API_INTERFACE void collisionDetails(SpCollisionDetails* details);

		API_INTERFACE void filterCollision(SpCollisionDetails* details) const;

		API_INTERFACE CollisionStatus collisionStatus(SpCollisionDetails* details);

		API_INTERFACE sp_bool areMovingAway(sp_uint objIndex1, sp_uint objIndex2) const;

		API_INTERFACE sp_bool isVertexFaceCollision(SpCollisionDetails* details) const;

	};

}

#endif // SP_COLLISION_DETECTOR_HEADER