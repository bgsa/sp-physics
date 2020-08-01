#ifndef SP_GPU_TEXTURE_HEADER
#define SP_GPU_TEXTURE_HEADER

#include "SpectrumPhysics.h"
#include "SpSize.h"

namespace NAMESPACE_PHYSICS
{
	class SpGpuTextureBuffer
		: public Object
	{
	protected:
		SpSize<sp_int> _size;


		sp_uint _textureId = ZERO_UINT;
		sp_uint _bufferId = ZERO_UINT;

	public:

		API_INTERFACE inline sp_uint textureId() const
		{
			return _textureId;
		}

		API_INTERFACE inline sp_uint bufferId() const
		{
			return _bufferId;
		}

		API_INTERFACE inline SpSize<sp_int> size() const
		{
			return _size;
		}

		API_INTERFACE inline void resize(const SpSize<sp_int>& newSize)
		{
			_size = newSize;
		}

		API_INTERFACE virtual SpGpuTextureBuffer* use() = 0;

#ifndef GL_DYNAMIC_DRAW
	#define GL_DYNAMIC_DRAW 0x88E8
#endif
		API_INTERFACE virtual void updateData(const sp_size size, const void* buffer, sp_uint usage = GL_DYNAMIC_DRAW) = 0;

		API_INTERFACE virtual void disable() = 0;

	};
}

#endif // !SP_GPU_TEXTURE_HEADER