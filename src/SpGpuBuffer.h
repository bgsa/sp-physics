#ifndef SP_GPU_BUFFER_HEADER
#define SP_GPU_BUFFER_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpGpuBuffer
	{
	protected:
		sp_uint _id;
		sp_int _type;

	public:

		API_INTERFACE virtual SpGpuBuffer* use() = 0;

		API_INTERFACE inline sp_uint id() const
		{
			return _id;
		}

		API_INTERFACE inline sp_int type() const
		{
			return _type;
		}

		API_INTERFACE inline SpGpuBuffer* type(sp_int bufferType)
		{
			_type = bufferType;
			return this;
		}

		API_INTERFACE virtual SpGpuBuffer* updateData(const sp_size size, const void* data, const sp_int usageType) = 0;

		API_INTERFACE virtual void disable() = 0;

		API_INTERFACE virtual void dispose() = 0;

	};

}

#endif // SP_GPU_BUFFER_HEADER