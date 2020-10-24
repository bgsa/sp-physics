#ifndef GAME_SETTINGS_HEADER
#define GAME_SETTINGS_HEADER

#define GLEW_STATIC

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSettings
	{
	private:
		sp_float _physicVelocity;
		sp_float _restingVelocityEpsilon;
		sp_bool _simulationEnabled;
		Vec3 _gravityForce;

		SpPhysicSettings()
		{
			//_physicVelocity = 0.009f;
			//_physicVelocity = 0.01f;
			_physicVelocity = 0.005f;
			_gravityForce = Vec3(0.0f, -9.8f, 0.0f);
			_restingVelocityEpsilon = 0.09f;
			enableSimulation();
		}
		
	public:

		API_INTERFACE inline static SpPhysicSettings* instance()
		{
			static SpPhysicSettings* _settings = sp_mem_new(SpPhysicSettings)();
			return _settings;
		}

		API_INTERFACE inline sp_float physicVelocity()
		{
			return _physicVelocity;
		}

		API_INTERFACE inline void physicVelocity(sp_float newVelocity)
		{
			_physicVelocity = newVelocity;
		}

		API_INTERFACE inline Vec3 gravityForce()
		{
			return _gravityForce;
		}

		API_INTERFACE inline void gravityForce(const Vec3& newGravityForce)
		{
			_gravityForce = newGravityForce;
		}

		API_INTERFACE inline sp_float restingVelocityEpsilon()
		{
			return _restingVelocityEpsilon;
		}

		API_INTERFACE inline sp_bool isSimulationEnabled()
		{
			return _simulationEnabled;
		}

		API_INTERFACE inline void enableSimulation()
		{
			_simulationEnabled = true;
		}

		API_INTERFACE inline void disableSimulation()
		{
			_simulationEnabled = false;
		}

	};
}

#endif // GAME_SETTINGS_HEADER