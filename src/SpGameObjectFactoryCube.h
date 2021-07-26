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
		SpGpuBuffer* gpuVertexBuffer;
		SpGpuBuffer* gpuIndexBuffer;
		sp_uint cubeMeshIndex;

		void initMesh(SpScene* scene);

	public:

		API_INTERFACE inline SpGameObjectFactoryCube()
		{
			cubeMeshIndex = SP_UINT_MAX;
		}

		API_INTERFACE void init(SpScene* scene) override;

		API_INTERFACE sp_uint create(SpScene* scene) override;

		API_INTERFACE void dispose() override;

	};
}

#endif // SP_GAME_OBJECT_FACTORY_CUBE_HEADER