#ifndef SP_GAME_OBJECT_FACTORY_HEADER
#define SP_GAME_OBJECT_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "SpGpuBuffer.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObjectFactory
	{
	protected:
		sp_uint _buffersLength;

	public:

		API_INTERFACE SpGameObjectFactory()
		{
			_buffersLength = 0;
		}

		API_INTERFACE virtual void init(SpScene* scene) = 0;

		API_INTERFACE virtual sp_uint create(SpScene* scene) = 0;

		API_INTERFACE sp_uint buffersLength() const
		{
			return _buffersLength;
		}

		API_INTERFACE virtual SpGpuBuffer* buffer(const sp_uint index) const = 0;

		API_INTERFACE virtual void dispose() = 0;

	};
}

#endif // SP_GAME_OBJECT_FACTORY_HEADER