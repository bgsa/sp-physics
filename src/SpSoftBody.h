#ifndef SP_SOFT_BODY_HEADER
#define SP_SOFT_BODY_HEADER

#include "SpectrumPhysics.h"
#include "Particle.h"
#include "SpSpring.h"

namespace NAMESPACE_PHYSICS
{
	class SpSoftBody
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
		API_INTERFACE inline SpSoftBody(const sp_uint particlesLength, const sp_uint springsLength)
		{
			_particlesLength = particlesLength;
			_particles = sp_mem_new_array(Particle, particlesLength);

			_springsLength = springsLength;
			_springs = sp_mem_new_array(SpSpring, springsLength);
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

	};

}

#endif // SP_SOFT_BODY_HEADER