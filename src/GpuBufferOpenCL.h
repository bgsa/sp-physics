#if OPENCL_ENABLED

#ifndef GPU_BUFFER_OPENCL_HEADER
#define GPU_BUFFER_OPENCL_HEADER

#include "SpectrumPhysics.h"
#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{
	class GpuBufferOpenCL
	{
	private:
		sp_size _size;
		cl_mem _buffer;
		GpuDevice* _gpu;

	public:

		API_INTERFACE GpuBufferOpenCL(GpuDevice* gpu)
		{
			_gpu = gpu;
			_buffer = nullptr;
		}

		API_INTERFACE inline sp_size size()
		{
			return _size;
		}

		API_INTERFACE inline cl_mem buffer()
		{
			return _buffer;
		}

		API_INTERFACE GpuBufferOpenCL* init(const sp_size size, cl_mem_flags flags = CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		API_INTERFACE GpuBufferOpenCL* init(const sp_size size, void* value, cl_mem_flags flags = CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		
		API_INTERFACE GpuBufferOpenCL* update(void* value, const sp_uint eventLength, cl_event* events, cl_event* evt);

		API_INTERFACE void dispose();

		~GpuBufferOpenCL()
		{
			dispose();
		}

	};
}

#endif // GPU_BUFFER_OPENCL_HEADER

#endif // OPENCL_ENABLED