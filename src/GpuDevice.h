#if OPENCL_ENABLED

#ifndef GPU_DEVICE_HEADER
#define GPU_DEVICE_HEADER

#include "SpectrumPhysics.h"
#include <algorithm>
#define CL_TARGET_OPENCL_VERSION 120
#include <CL/opencl.h>

#include "GpuCommandManager.h"
#include "SpGpuBuffer.h"
#include "SpGpuTextureBuffer.h"

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
		cl_uint memoryBaseAddressAlign;
		cl_uint memoryAlignmentRequirement;
		cl_uint clockFrequency;
		std::string profile;

		GpuCommandManager* commandManager = nullptr;
		
		API_INTERFACE GpuDevice(cl_device_id id, cl_platform_id platformId);

		API_INTERFACE inline sp_bool isGPU()
		{
			return (type & CL_DEVICE_TYPE_GPU) || (type & CL_DEVICE_TYPE_ALL);
		}

		API_INTERFACE inline sp_bool isCPU()
		{
			return (type & CL_DEVICE_TYPE_CPU) || (type & CL_DEVICE_TYPE_ALL);
		}

		API_INTERFACE inline cl_mem createBuffer(sp_size sizeOfValue, cl_mem_flags memoryFlags)
		{
			cl_int errorCode;
			cl_mem memoryBuffer = clCreateBuffer(deviceContext, memoryFlags, sizeOfValue, NULL, &errorCode);
			HANDLE_OPENCL_ERROR(errorCode);

			return memoryBuffer;
		}

		API_INTERFACE inline cl_mem createBuffer(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, sp_bool writeValueOnDevice = true)
		{		
			cl_int errorCode;
			cl_mem memoryBuffer = clCreateBuffer(deviceContext, memoryFlags, sizeOfValue, value, &errorCode);
			HANDLE_OPENCL_ERROR(errorCode);

			if (writeValueOnDevice)
				HANDLE_OPENCL_ERROR(clEnqueueWriteBuffer(commandManager->commandQueue, memoryBuffer, CL_FALSE, 0, sizeOfValue, value, 0, NULL, NULL));

			return memoryBuffer;
		}

		API_INTERFACE inline cl_mem createSubBuffer(cl_mem buffer, cl_buffer_region* region, cl_mem_flags memoryFlags)
		{
			cl_int errorCode;
			cl_mem subBuffer = clCreateSubBuffer(buffer, memoryFlags, CL_BUFFER_CREATE_TYPE_REGION, region, &errorCode);
			HANDLE_OPENCL_ERROR(errorCode);

			return subBuffer;
		}

		API_INTERFACE inline cl_mem createBufferFromOpenGL(SpGpuBuffer* buffer, cl_mem_flags memoryFlags = CL_MEM_READ_WRITE)
		{
			cl_int errorCode;
			cl_mem openCLBuffer = clCreateFromGLBuffer(deviceContext, memoryFlags, buffer->id(), &errorCode);			
			HANDLE_OPENCL_ERROR(errorCode);
		
			return openCLBuffer;
		}

		API_INTERFACE inline cl_mem createBufferFromOpenGL(SpGpuTextureBuffer* buffer, cl_mem_flags memoryFlags = CL_MEM_READ_WRITE)
		{
#ifndef GL_TEXTURE_BUFFER
	#define GL_TEXTURE_BUFFER 0x8C2A
#endif
			cl_int errorCode;
			cl_mem openCLBuffer = clCreateFromGLBuffer(deviceContext, memoryFlags, buffer->bufferId(), &errorCode);
			HANDLE_OPENCL_ERROR(errorCode);

			return openCLBuffer;
		}

		API_INTERFACE inline void releaseBuffer(cl_mem memoryBuffer)
		{
			HANDLE_OPENCL_ERROR(clReleaseMemObject(memoryBuffer));
		}

		API_INTERFACE inline void releaseBuffer(size_t length, cl_mem memoryBuffers ...)
		{
			va_list parameters;
			va_start(parameters, memoryBuffers);

			for (sp_uint i = 0; i < length - 1; i++)
				releaseBuffer(va_arg(parameters, cl_mem));

			va_end(parameters);
		}
		
		API_INTERFACE sp_uint getThreadLength(sp_uint inputLength);
		API_INTERFACE sp_uint getGroupLength(sp_uint threadLength, sp_uint inputLength);
		API_INTERFACE inline sp_uint getDefaultGroupLength()
		{
			return nextPowOf2(multiplyBy2(computeUnits));
		}

		API_INTERFACE inline void waitEvents(sp_size eventsLength, cl_event* events)
		{
			clWaitForEvents(eventsLength, events);
		}

		~GpuDevice();
	};

}

#endif

#endif // !GPU_DEVICE_HEADER