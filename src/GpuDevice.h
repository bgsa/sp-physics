#if OPENCL_ENABLED

#ifndef GPU_DEVICE_HEADER
#define GPU_DEVICE_HEADER

#include "SpectrumPhysics.h"
#include <algorithm>
#include <CL/cl.h>

#include "GpuCommandManager.h"

namespace NAMESPACE_PHYSICS
{
	class GpuDevice
	{
	private:
		cl_device_id id;
		cl_context deviceContext;

	public:
		char* name;
		char* version;
		char* driverVersion;
		cl_device_type type;
		cl_uint computeUnits;
		std::vector<std::string> extensions;
		sp_uint maxParameterSize = 256;

		/// <summary>
		/// Maximum number of work - items in a work - group executing a kernel using the data parallel execution model
		/// </summary>
		sp_uint maxWorkGroupSize = 1;

		
		/// <summary>
		/// Maximum dimensions that specify the global and local work - item IDs used by the data parallel execution model
		/// </summary>
		cl_uint maxWorkItemDimension = 3;
		
		/// <summary>
		/// Maximum number of work - items that can be specified in each dimension of the work - group to clEnqueueNDRangeKernel
		/// </summary>
		sp_uint maxWorkItemSizes[3];

		cl_ulong globalMemorySize;
		cl_ulong globalMemoryCacheSize;
		cl_ulong localMemorySize;
		cl_ulong localMemoryLength;
		cl_ulong maxMemoryAllocSize;
		cl_ulong constantsBufferSize;
		cl_uint maxConstantArgument;
		cl_uint memoryAlign;
		cl_uint clockFrequency;
		std::string profile;

		GpuCommandManager* commandManager = nullptr;
		
		API_INTERFACE GpuDevice(cl_device_id id);

		API_INTERFACE bool isGPU();
		API_INTERFACE bool isCPU();

		API_INTERFACE cl_mem createBuffer(size_t sizeOfValue, cl_mem_flags memoryFlags);
		API_INTERFACE cl_mem createBuffer(void * value, size_t sizeOfValue, cl_mem_flags memoryFlags, bool writeValueOnDevice = true);
		API_INTERFACE cl_mem createSubBuffer(cl_mem buffer, cl_buffer_region* region, cl_mem_flags memoryFlags);

		API_INTERFACE void releaseBuffer(cl_mem memoryBuffer);
		API_INTERFACE void releaseBuffer(size_t length, cl_mem memoryBuffers ...);
		
		API_INTERFACE sp_uint getThreadLength(sp_uint inputLength);
		API_INTERFACE sp_uint getGroupLength(sp_uint threadLength, sp_uint inputLength);
		API_INTERFACE sp_uint getDefaultGroupLength();

		~GpuDevice();
	};

}

#endif

#endif // !GPU_DEVICE_HEADER