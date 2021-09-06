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
		SpGpuBuffer* buffers[2];
		sp_uint planeMeshIndex;

		void initMesh(SpScene* scene);

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGameObjectFactoryPlane()
			: SpGameObjectFactory()
		{
			planeMeshIndex = SP_UINT_MAX;
			_buffersLength = 2;
		}

		/// <summary>
		/// Initialize the Game Object Factory for Plane
		/// </summary>
		API_INTERFACE void init(SpScene* scene) override;

		/// <summary>
		/// Create a new Plane game object
		/// </summary>
		API_INTERFACE sp_uint create(SpScene* scene) override;

		API_INTERFACE SpGpuBuffer* buffer(const sp_uint index) const override
		{
			return buffers[index];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		API_INTERFACE void dispose() override;

	};
}

#endif // SP_GAME_OBJECT_FACTORY_PLANE_HEADER