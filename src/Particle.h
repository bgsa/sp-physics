#ifndef PARTICLE_HEADER
#define PARTICLE_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class Particle
	{
	private:
		Vec3 _position;
		Vec3 _velocity;
		Vec3 _acceleration;
		Vec3 _force;
		sp_float _inverseMass;
		sp_float _damping;
		sp_float _lifeTime;

	public:

		/// <summary>
		/// Create a particle with optional arguments
		/// </summary>
		API_INTERFACE Particle(const Vec3& position = Vec3Zeros)
		{
			this->_position = position;
			this->_velocity = Vec3Zeros;
			this->_acceleration = Vec3Zeros;
			this->_force = Vec3Zeros;
			mass(8.0f);
			_damping = 0.9f;
			_lifeTime = SP_FLOAT_MAX;
		}

		/// <summary>
		/// The the inverse mass of the particle
		/// </summary>
		/// <returns>1.0/mass</returns>
		API_INTERFACE inline sp_float inverseMass() const
		{
			return _inverseMass;
		}

		/// <summary>
		/// Set the mass of particle
		/// </summary>
		/// <param name="mass">New Mass</param>
		/// <returns>void</returns>
		API_INTERFACE inline void mass(const sp_float mass)
		{
				this->_inverseMass = mass != ZERO_FLOAT 
					? ONE_FLOAT / mass
					: ZERO_FLOAT;
		}

		/// <summary>
		/// Get the lifetime of the particle
		/// </summary>
		/// <returns>Lifetime</returns>
		API_INTERFACE inline sp_float lifeTime() const
		{
			return _lifeTime;
		}

		/// <summary>
		/// Set the life time of particle
		/// </summary>
		/// <param name="newLifeTime">New Life Time</param>
		/// <returns>void</returns>
		API_INTERFACE inline void lifeTime(const sp_float newLifeTime)
		{
			this->_lifeTime = newLifeTime;
		}

		/// <summary>
		/// Add a force/impulse to a particle
		/// Constant force are not covered in this force. Use ConstantForce on ParticleSystem
		/// </summary>
		API_INTERFACE void addForce(const Vec3& force)
		{
			this->_force += force;
		}

		/// <summary>
		/// Add an impulse directely on velocity of the particle
		/// </summary>
		API_INTERFACE void addImpulse(const Vec3& impulse)
		{
			this->_velocity += impulse;
		}

		/// <summary>
		/// Get the position of the particle
		/// </summary>
		/// <returns>Position</returns>
		API_INTERFACE inline Vec3 position() const
		{
			return _position;
		}

		/// <summary>
		/// Set the position of particle
		/// </summary>
		/// <param name="newPosition">New Position</param>
		/// <returns>void</returns>
		API_INTERFACE inline void position(const Vec3 newPosition)
		{
			this->_position = newPosition;
		}

		/// <summary>
		/// Get the velocity of the particle
		/// </summary>
		/// <returns>Velocity</returns>
		API_INTERFACE inline Vec3 velocity() const
		{
			return _velocity;
		}

		/// <summary>
		/// Set the velocity of particle
		/// </summary>
		/// <param name="newVelocity">New Velocity</param>
		/// <returns>void</returns>
		API_INTERFACE inline void velocity(const Vec3 newVelocity)
		{
			this->_velocity = newVelocity;
		}

		/// <summary>
		/// Get the acceleration of the particle
		/// </summary>
		/// <returns>Acceleration</returns>
		API_INTERFACE inline Vec3 acceleration() const
		{
			return _acceleration;
		}

		/// <summary>
		/// Get the damping of the particle
		/// </summary>
		/// <returns>Damping</returns>
		API_INTERFACE inline sp_float damping() const
		{
			return _damping;
		}

		/// <summary>
		/// Set the damping of particle
		/// </summary>
		/// <param name="newDamping">New Damping</param>
		/// <returns>void</returns>
		API_INTERFACE inline void damping(const sp_float newDamping)
		{
			this->_damping = newDamping;
		}

		/// <summary>
		/// Integrate the particle using Euler method
		/// </summary>
		API_INTERFACE inline void update(const sp_float elapsedTime)
		{
			_acceleration = _force * _inverseMass;
			_velocity = _velocity * _damping + _acceleration * elapsedTime;
			_position = _position + _velocity * elapsedTime;
		}

	};
}

#endif // PARTICLE_HEADER