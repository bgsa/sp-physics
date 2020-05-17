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

		API_INTERFACE ParticleSystem(size_t particlesCount);
		API_INTERFACE ParticleSystem(Particle* particles, size_t particlesCount);

		API_INTERFACE void addForce(const Vec3& force);

		API_INTERFACE size_t addConstantForce(const Vec3& constantForce);

		API_INTERFACE void update(long long elapsedTime);
	};

}

#endif // ! PARTICLE_SYSTEM