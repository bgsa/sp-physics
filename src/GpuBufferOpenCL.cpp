#include "GpuBufferOpenCL.h"

namespace NAMESPACE_PHYSICS
{

	GpuBufferOpenCL* GpuBufferOpenCL::init(const sp_size size, cl_mem_flags flags)
	{
		_size = size;
		_buffer = _gpu->createBuffer(_size, flags);

		return this;
	}

	GpuBufferOpenCL* GpuBufferOpenCL::init(const sp_size size, void* value, cl_mem_flags flags)
	{
		_size = size;
		_buffer = _gpu->createBuffer(value, _size, flags, true);

		return this;
	}

	GpuBufferOpenCL* GpuBufferOpenCL::update(void* value, const sp_uint eventLength, cl_event* events)
	{
		_gpu->commandManager->updateBuffer(_buffer, _size, value, eventLength, events);
		return this;
	}

	void GpuBufferOpenCL::dispose()
	{
		if (_buffer != nullptr)
		{
			_gpu->releaseBuffer(_buffer);
			_buffer = nullptr;
		}
	}

}