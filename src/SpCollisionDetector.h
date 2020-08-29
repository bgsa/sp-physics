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
		
		void fillCollisionDetails(const sp_uint obj1Index, const sp_uint obj2Index, const Line3D& edge, const Triangle3D& face, sp_uint penetratedVertexIndex, SpCollisionDetails* details);
		
		void timeOfCollisionWithObj1Static(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details);

		void timeOfCollision(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details);

		sp_bool findCollisionEdgeFace(sp_uint obj1Index, sp_uint obj2Index, Line3D* line, Triangle3D* face, Vec3* contactPoint, sp_uint* penetratedVertexIndex);

	public:

		API_INTERFACE void filterCollision(sp_uint obj1Index, sp_uint obj2Index, SpCollisionDetails* details) const;

		API_INTERFACE void collisionDetails(SpCollisionDetails* details);

		API_INTERFACE void collisionDetailsWithObj1Static(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details);

		API_INTERFACE CollisionStatus collisionStatus(const sp_uint obj1Index, const sp_uint obj2Index, SpCollisionDetails* details);

		API_INTERFACE sp_bool areMovingAway(sp_uint objIndex1, sp_uint objIndex2) const;

	};

}

#endif // SP_COLLISION_DETECTOR_HEADER