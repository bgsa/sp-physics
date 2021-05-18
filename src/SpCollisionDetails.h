#ifndef SP_COLLISION_DETAILS_HEADER
#define SP_COLLISION_DETAILS_HEADER

#include "SpectrumPhysics.h"
#include "Plane.h"
#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
#define SP_COLLISION_TYPE_NONE        0
#define SP_COLLISION_TYPE_VERTEX_FACE 1
#define SP_COLLISION_TYPE_EDGE_FACE   2
#define SP_COLLISION_TYPE_FACE_FACE   3
#define SP_COLLISION_TYPE_EDGE_EDGE   4
#define SP_COLLISION_TYPE_VERTEX_EDGE 5

	class SpCollisionDetails
	{
	public:
		sp_uint objIndex1;
		sp_uint objIndex2;
		sp_bool ignoreCollision;
		sp_int  type;
		sp_float timeStep;
		sp_float timeOfCollision;

		Vec3 collisionNormal;
		sp_float depth;
		sp_uint vertexObjectIndex;
		sp_uint edgeObjectIndex;
		sp_uint faceObjectIndex;
		sp_uint vertexIndex;
		sp_uint edgeIndex;
		sp_uint faceIndex;

		Vec3 centerContactPoint;
		Vec3 contactPoints[8];
		sp_uint contactPointsLength;

		sp_uint vertexIndexObj1;
		sp_uint vertexIndexObj2;

		SpMeshCache* cacheObj1;
		SpMeshCache* cacheObj2;

		SpCollisionDetails()
		{
			cacheObj1 = nullptr;
			cacheObj2 = nullptr;

			ignoreCollision = false;
			type = SP_COLLISION_TYPE_NONE;
			contactPointsLength = ZERO_UINT;
			
			vertexIndexObj1 = SP_UINT_MAX;
			vertexIndexObj2 = SP_UINT_MAX;

			depth = SP_FLOAT_MAX;
			edgeIndex = SP_UINT_MAX;
		}

	};

}

#endif // SP_COLLISION_DETAILS_HEADER