#ifndef SP_PHYSIC_INTEGRATOR_HEADER
#define SP_PHYSIC_INTEGRATOR_HEADER

#include "SpectrumPhysics.h"
#include "SpLogger.h"

namespace NAMESPACE_PHYSICS
{

	class SpPhysicIntegrator
	{
	public:

		/// <summary>
		/// Execute the integrator
		/// </summary>
		/// <param name="globalIndex">Object global index</param>
		/// <returns>void</returns>
		API_INTERFACE virtual void execute(const sp_uint globalIndex, const sp_float elapsedTime) = 0;

	};

	class SpPhysicIntegratorVelocityVerlet
		: public SpPhysicIntegrator
	{
	public:
		
		/// <summary>
		/// Execute the integrator
		/// </summary>
		/// <param name="globalIndex">Object global index</param>
		/// <returns>void</returns>
		API_INTERFACE void execute(const sp_uint globalIndex, const sp_float elapsedTime) override;

	};

	class SpPhysicIntegratorEuler
		: public SpPhysicIntegrator
	{
	public:

		/// <summary>
		/// Execute the integrator
		/// </summary>
		/// <param name="globalIndex">Object global index</param>
		/// <returns>void</returns>
		API_INTERFACE void execute(const sp_uint globalIndex, const sp_float elapsedTime) override;

	};

}

#endif // SP_PHYSIC_INTEGRATOR_HEADER