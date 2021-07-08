#ifndef SP_GAME_OBJECT_FACTORY_PLANE_HEADER
#define SP_GAME_OBJECT_FACTORY_PLANE_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObjectFactory.h"
#include "SpGpuBuffer.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObjectFactoryPlane
		: public SpGameObjectFactory
	{
	private:
		SpGpuBuffer* gpuVertexBuffer;
		SpGpuBuffer* gpuIndexBuffer;

	public:

		API_INTERFACE void init(SpScene* scene) override;

		API_INTERFACE sp_uint create(SpScene* scene) override;

		API_INTERFACE void dispose() override;

	};
}

#endif // SP_GAME_OBJECT_FACTORY_PLANE_HEADER