#ifndef SP_COLLISION_DETAILS_HEADER
#define SP_COLLISION_DETAILS_HEADER

#include "SpectrumPhysics.h"
#include "Plane3D.h"
#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
	enum class SpCollisionType
	{
		None = 0,
		PointFace = 1,
		EdgeFace  = 2,
		FaceFace  = 3,
		EdgeEdge  = 4
	};

	class SpCollisionDetails
	{
	public:
		sp_uint objIndex1;
		sp_uint objIndex2;
		sp_uint objectIndexPlane1;
		sp_uint objectIndexPlane2;
		sp_float timeOfCollision;
		sp_float timeStep;
		Vec3 contactPoints[8];
		sp_uint contactPointsLength;
		Vec3 collisionNormal;
		sp_float depth;
		sp_bool ignoreCollision;
		
		SpCollisionType type;

		SpVertexEdges* extremeVertexObj1;
		SpVertexEdges* extremeVertexObj2;

		SpCollisionDetails()
		{
			ignoreCollision = false;
			extremeVertexObj1 = nullptr;
			extremeVertexObj2 = nullptr;
			type = SpCollisionType::None;
			contactPointsLength = ZERO_UINT;
		}

	};

}

#endif // SP_COLLISION_DETAILS_HEADER