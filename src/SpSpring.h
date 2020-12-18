#ifndef SP_SPRING_HEADER
#define SP_SPRING_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpSpring
	{
	private:
		
	public:
		Particle* particle1;
		Particle* particle2;
		sp_float stiffness;
		sp_float restingLength;
		sp_float friction;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE inline SpSpring()
		{
		}

		/// <summary>
		/// Constructor with particles
		/// </summary>
		/// <param name="particle1">Particle 1</param>
		/// <param name="particle2">Particle 2</param>
		/// <param name="stifiness">Stifiness</param>
		API_INTERFACE inline SpSpring(Particle* particle1, Particle* particle2, const sp_float stifiness = 0.8f)
		{
			this->particle1 = particle1;
			this->particle2 = particle2;
			this->stiffness = stifiness;
			restingLength = ONE_FLOAT;
			friction = 0.95f;
		}

		/// <summary>
		/// Apply the forces in the particles using the spring
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void update()
		{
			const Vec3 relativePosition = particle2->position() - particle1->position();
			const Vec3 relativeVelelocity = particle2->velocity() - particle2->velocity();

			const sp_float x = length(relativePosition) - restingLength;
			const sp_float v = length(relativeVelelocity);

			const sp_float F = (-stiffness * x) + (-friction * v);
			
			Vec3 relativePositionNormalized;
			normalize(relativePosition, &relativePositionNormalized);

			const Vec3 impulse = relativePositionNormalized * F;
			
			particle1->addImpulse(impulse * particle1->inverseMass());
			particle2->addImpulse(-impulse * particle2->inverseMass());
		}

	};

}

#endif // SP_SPRING_HEADER