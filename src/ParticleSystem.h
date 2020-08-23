#ifndef PARTICLE_SYSTEM_HEADER
#define PARTICLE_SYSTEM_HEADER

#include "SpectrumPhysics.h"
#include "Particle.h"
#include <vector>

namespace NAMESPACE_PHYSICS
{

	class ParticleSystem
	{
	private:
		std::vector<Vec3> constantForces;

	public:
		Particle* particles;
		size_t particlesCount;
		Mat3 orientation;
		Vec3 angularVelocity;

		API_INTERFACE ParticleSystem(sp_size particlesCount);
		API_INTERFACE ParticleSystem(Particle* particles, sp_size particlesCount);

		API_INTERFACE void addForce(const Vec3& force);

		API_INTERFACE size_t addConstantForce(const Vec3& constantForce);

		//API_INTERFACE void update(sp_float elapsedTime);
	};

}

#endif // ! PARTICLE_SYSTEM