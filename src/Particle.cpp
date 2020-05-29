#include "Particle.h"

namespace NAMESPACE_PHYSICS
{
	Particle::Particle(const Vec3& position, const Vec3& velocity, const float inverseMass, const float velocityDamping, float coeficientOfRestitution)
	{
		this->position = position;
		this->previousPosition = position;
		this->velocity = velocity;
		this->previousVelocity = velocity;
		this->inverseMass = inverseMass;
		this->velocityDamping = velocityDamping;
		this->coeficientOfRestitution = coeficientOfRestitution;

		acceleration = Vec3(0.0f);
		force = Vec3(0.0f);
		lifeTime = FLT_MAX;
	}

	void Particle::addForce(const Vec3& force) 
	{
		this->force.add(force);
	}

	void Particle::update(sp_float elapsedTime)
	{
		sp_assert(elapsedTime > 0, "InvalidArgumentException");

		Vec3 newAcceleration = force * inverseMass;

		Vec3 newVelocity = (velocity * velocityDamping) + ((newAcceleration - acceleration) / elapsedTime);

		Vec3 newPosition = position + (newVelocity / elapsedTime);

		acceleration = newAcceleration;

		previousVelocity = velocity;
		velocity = newVelocity;

		previousPosition = position;
		position = newPosition;

		force = 0.0f;
	}

	Vec3 Particle::direction() 
	{
		return (position - previousPosition).normalize();
	}

	Vec3 Particle::relativeVelocity(const Particle& particle)
	{
		return this->velocity - particle.velocity;
	}

	float Particle::closingVelocity(const Particle& particle)
	{
		Vec3 direction = this->position - particle.position;

		return direction.dot(direction.normalize());
	}
}