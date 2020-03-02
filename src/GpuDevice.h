#if OPENCL_ENABLED

#ifndef GPU_DEVICE_HEADER
#define GPU_DEVICE_HEADER

#include "OpenML.h"
#include <algorithm>
#include <CL/cl.h>

#include "GpuCommandManager.h"

namespace OpenML
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
		//bool isAvailable = false;
		std::vector<std::string> extensions;
		sp_uint maxParameterSize = 256;
		sp_uint maxWorkGroupSize = 1;
		cl_uint maxWorkItemDimension = 3;
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
		
		GpuDevice(cl_device_id id);

		bool isGPU();
		bool isCPU();

		cl_mem createBuffer(size_t sizeOfValue, cl_mem_flags memoryFlags);
		cl_mem createBuffer(void * value, size_t sizeOfValue, cl_mem_flags memoryFlags, bool writeValueOnDevice = true);
		void releaseBuffer(cl_mem memoryBuffer);
		void releaseBuffer(size_t length, cl_mem memoryBuffers ...);

		sp_uint getLocalWorkSize(sp_uint elementsLength);

		sp_uint getThreadLength(sp_uint elementsLength);

		~GpuDevice();
	};

}

#endif

#endif // !GPU_DEVICE_HEADER