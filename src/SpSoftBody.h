#ifndef SP_SOFT_BODY_HEADER
#define SP_SOFT_BODY_HEADER

#include "SpectrumPhysics.h"
#include "Particle.h"
#include "SpSpring.h"

namespace NAMESPACE_PHYSICS
{
	class SpSoftBody
		: public SpBody
	{
	private:
		Vec3 _position;
		sp_uint _particlesLength;
		Particle* _particles;
		sp_uint _springsLength;
		SpSpring* _springs;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpSoftBody()
		{
			_particlesLength = ZERO_UINT;
			_springsLength = ZERO_UINT;
		}

		/// <summary>
		/// Init the soft body
		/// </summary>
		/// <param name="particlesLength"></param>
		/// <param name="springsLength"></param>
		/// <returns></returns>
		API_INTERFACE inline void init(const sp_uint particlesLength, const sp_uint springsLength)
		{
			_particlesLength = particlesLength;
			_particles = sp_mem_new_array(Particle, particlesLength);

			_springsLength = springsLength;
			_springs = sp_mem_new_array(SpSpring, springsLength);
		}

		/// <summary>
		/// Translate all particles given a translation vector
		/// </summary>
		API_INTERFACE inline void translate(const Vec3& translation)
		{
			for (sp_uint i = 0; i < _particlesLength; i++)
				_particles[i].position(_particles[i].position() + translation);
		}

		/// <summary>
		/// Get the position of the object
		/// </summary>
		API_INTERFACE inline Vec3 position() const
		{
			return _position;
		}

		/// <summary>
		/// Set the current position
		/// </summary>
		API_INTERFACE inline void position(const Vec3& newPosition)
		{
			_position = newPosition;
		}

		/// <summary>
		/// Set the mass for all particles of soft body
		/// </summary>
		/// <param name="newMass">Mass</param>
		/// <returns>void</returns>
		API_INTERFACE inline void mass(const sp_float newMass)
		{
			for (sp_uint i = 0; i < _particlesLength; i++)
				_particles->mass(newMass);
		}

		/// <summary>
		/// Get the particles of the soft body
		/// </summary>
		/// <returns>Particles</returns>
		API_INTERFACE inline Particle* particles() const
		{
			return _particles;
		}

		/// <summary>
		/// Get the particles length of the soft body
		/// </summary>
		/// <returns>Length</returns>
		API_INTERFACE inline sp_uint particlesLength() const
		{
			return _particlesLength;
		}

		/// <summary>
		/// Get the springs of the soft body
		/// </summary>
		/// <returns>Springs</returns>
		API_INTERFACE inline SpSpring* springs() const
		{
			return _springs;
		}

		/// <summary>
		/// Get the springs length of the soft body
		/// </summary>
		/// <returns>Length</returns>
		API_INTERFACE inline sp_uint springsLength() const
		{
			return _springsLength;
		}

		/// <summary>
		/// Get the size in bytes of the soft body
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_size size()
		{
			return sizeof(SpSoftBody)
				+ (sizeof(Particle) * _particlesLength)
				+ (sizeof(SpSpring) * _springsLength);
		}

	};

}

#endif // SP_SOFT_BODY_HEADER