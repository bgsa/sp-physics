#ifndef GAME_SETTINGS_HEADER
#define GAME_SETTINGS_HEADER

#define GLEW_STATIC

#include "SpectrumPhysics.h"
#include "BoundingVolume.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSettings
	{
	private:
		sp_float _physicVelocity;
		sp_float _restingVelocityEpsilon;
		sp_size _frameId;
		Vec3 _gravityForce;
		sp_bool _simulationEnabled;
		sp_bool _profilingEnabled;
		sp_uint _pcaExecutionPerFrame;
		BoundingVolumeType _boundingVolumeType;

		SpPhysicSettings()
		{
			_frameId = ZERO_SIZE;
			_physicVelocity = 0.003f;
			_gravityForce = Vec3(0.0f, -9.8f, 0.0f);
			_restingVelocityEpsilon = 0.09f;
			_pcaExecutionPerFrame = 30u;
			_profilingEnabled = false;
			_boundingVolumeType = BoundingVolumeType::DOP18;
			enableSimulation();
		}
		
	public:

		API_INTERFACE inline static SpPhysicSettings* instance()
		{
			static SpPhysicSettings* _settings = sp_mem_new(SpPhysicSettings)();
			return _settings;
		}

		/// <summary>
		/// Get how many frames PCA should wait to execute
		/// </summary>
		/// <returns>Frame Length</returns>
		API_INTERFACE inline sp_uint pcaExecutionPerFrame() const
		{
			return _pcaExecutionPerFrame;
		}
		

		API_INTERFACE inline sp_size frameId() const
		{
			return _frameId;
		}

		API_INTERFACE inline BoundingVolumeType boundingVolumeType() const
		{
			return _boundingVolumeType;
		}
		API_INTERFACE inline void boundingVolumeType(BoundingVolumeType type)
		{
			_boundingVolumeType = type;
		}

		API_INTERFACE inline sp_float physicVelocity() const
		{
			return _physicVelocity;
		}

		API_INTERFACE inline void physicVelocity(sp_float newVelocity)
		{
			_physicVelocity = newVelocity;
		}

		API_INTERFACE inline Vec3 gravityForce() const
		{
			return _gravityForce;
		}

		API_INTERFACE inline void gravityForce(const Vec3& newGravityForce)
		{
			_gravityForce = newGravityForce;
		}

		API_INTERFACE inline sp_float restingVelocityEpsilon() const
		{
			return _restingVelocityEpsilon;
		}

		API_INTERFACE inline sp_bool isSimulationEnabled() const
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

		API_INTERFACE inline sp_bool isProfilingEnabled()
		{
			return _profilingEnabled;
		}

		API_INTERFACE inline void enableProfiling()
		{
			_profilingEnabled = true;
		}

		API_INTERFACE inline void disableProfiling()
		{
			_profilingEnabled = false;
		}

		API_INTERFACE inline void nextFrame()
		{
			_frameId++;
		}
	};
}

#endif // GAME_SETTINGS_HEADER