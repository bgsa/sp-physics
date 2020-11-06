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
		VertexFace = 1,
		EdgeFace  = 2,
		FaceFace  = 3,
		EdgeEdge  = 4,
		VertexEdge = 5
	};

	class SpCollisionDetails
	{
	public:
		sp_uint objIndex1;
		sp_uint objIndex2;
		sp_float timeStep;
		sp_bool ignoreCollision;

		SpCollisionType type;
		sp_float timeOfCollision;
		Vec3 centerContactPoint;

		sp_uint vertexIndexObj1;
		sp_uint vertexIndexObj2;
		Vec3 collisionNormal;

		Vec3 contactPoints[8];
		sp_uint contactPointsLength;

		SpMeshCache* cacheObj1;
		SpMeshCache* cacheObj2;

		SpCollisionDetails()
		{
			cacheObj1 = nullptr;
			cacheObj2 = nullptr;

			ignoreCollision = false;
			type = SpCollisionType::None;
			contactPointsLength = ZERO_UINT;
		}

	};

}

#endif // SP_COLLISION_DETAILS_HEADER