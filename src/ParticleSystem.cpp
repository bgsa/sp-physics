#include "ParticleSystem.h"

namespace NAMESPACE_PHYSICS
{
	ParticleSystem::ParticleSystem(sp_size particlesCount)
	{
		sp_assert(particlesCount > 0, "InvalidArgumentException");

		this->particles = ALLOC_NEW_ARRAY(Particle, particlesCount);
		this->particlesCount = particlesCount;
		this->orientation = Mat3::identity();
		this->angularVelocity = Vec3Zeros;
	}

	ParticleSystem::ParticleSystem(Particle* particles, sp_size particlesCount)
	{
		sp_assert(particlesCount > 0, "InvalidArgumentException");

		this->particles = particles;
		this->particlesCount = particlesCount;
	}

	void ParticleSystem::addForce(const Vec3& force)
	{
		for (size_t i = 0; i < particlesCount; i++)
			this->particles[i].addForce(force);
	}

	sp_size ParticleSystem::addConstantForce(const Vec3& constantForce)
	{
		constantForces.push_back(constantForce);

		return constantForces.size() - 1;
	}

	/*
	void ParticleSystem::update(sp_float elapsedTime)
	{
		for (sp_size i = 0; i < particlesCount; i++)
		{
			for (std::vector<Vec3>::iterator contantForce = constantForces.begin(); contantForce != constantForces.end(); ++contantForce)
				this->particles[i].addForce(*contantForce);

			this->particles[i].update(elapsedTime);
		}
	}
	*/
}