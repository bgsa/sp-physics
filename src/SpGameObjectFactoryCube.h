#ifndef SP_GAME_OBJECT_FACTORY_CUBE_HEADER
#define SP_GAME_OBJECT_FACTORY_CUBE_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObjectFactory.h"
#include "SpGpuBuffer.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObjectFactoryCube
		: public SpGameObjectFactory
	{
	private:
		SpGpuBuffer* buffers[2];
		sp_uint cubeMeshIndex;

		void initMesh(SpScene* scene);

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGameObjectFactoryCube()
			: SpGameObjectFactory()
		{
			cubeMeshIndex = SP_UINT_MAX;
			_buffersLength = 2;
		}

		/// <summary>
		/// Initialize this game object factory for Cube
		/// </summary>
		/// <param name="scene"></param>
		/// <returns></returns>
		API_INTERFACE void init(SpScene* scene) override;

		/// <summary>
		/// Create a new in Cube in scene
		/// </summary>
		/// <param name="scene"></param>
		/// <returns></returns>
		API_INTERFACE sp_uint create(SpScene* scene) override;

		/// <summary>
		/// Get all buffer used for Cube objects
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE SpGpuBuffer* buffer(const sp_uint index) const override
		{
			return buffers[index];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE void dispose() override;

	};
}

#endif // SP_GAME_OBJECT_FACTORY_CUBE_HEADER