#ifndef SP_COLISION_RESPONSE_SHAPE_MATCHING_HEADER
#define SP_COLISION_RESPONSE_SHAPE_MATCHING_HEADER

#include "SpectrumPhysics.h"
#include "SpCollisionDetectorCache.h"
#include "SpCollisionDetector.h"
#include "SpMesh.h"
#include "SpGJK.h"
#include "SweepAndPrune.h"

namespace NAMESPACE_PHYSICS
{
	class SpRigidBodyShapeMatch
	{
	public:
		sp_uint objectIndex;
		sp_bool isDirty;
		Vec3 initialPosition;
		sp_uint particlesLength;
		Vec3* particles;
		Vec3* initialParticles;

		API_INTERFACE SpRigidBodyShapeMatch()
		{
			particlesLength = ZERO_UINT;
			particles = nullptr;
			isDirty = false;
		}

		API_INTERFACE inline void centerOfMass(Vec3& output) const
		{
			output = Vec3Zeros;

			for (sp_uint i = 0; i < particlesLength; i++)
				output += particles[i];

			output /= (sp_float)particlesLength;
		}
	};

	class SpCollisionResponseShapeMatching :
		public SpCollisionResponse
	{
	private:

		void updateParticlesShape(SpRigidBodyShapeMatch* shape, const Plane& plane);

		void updateParticles(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, const SpCollisionDetails& collisionManifold);

		sp_bool hasCollision(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2, SpCollisionDetails* collisionManifold);

		sp_bool hasPlaneCollision(SpRigidBodyShapeMatch* shape, SpCollisionDetails* collisionManifold);

		sp_bool shapeMatch(SpRigidBodyShapeMatch* shape);

	public:

		API_INTERFACE void initShape(const sp_uint objIndex, SpRigidBodyShapeMatch* shape);

		API_INTERFACE void run(const SweepAndPruneResult& sapResult) override;

		API_INTERFACE void solve(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2);
		API_INTERFACE void firstSolve(SpRigidBodyShapeMatch* shape1, SpRigidBodyShapeMatch* shape2);

		API_INTERFACE void updateFromShape(SpRigidBodyShapeMatch* shape);

	};

}

#endif // SP_COLISION_RESPONSE_SHAPE_MATCHING_HEADER