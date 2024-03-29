#ifdef OPENCL_ENABLED

#ifndef GPU_DEVICE_HEADER
#define GPU_DEVICE_HEADER

#include "SpectrumPhysics.h"
#include <algorithm>
#include "SpOpenCL.h"

#include "GpuCommandManager.h"
#include "SpGpuBuffer.h"
#include "SpGpuTextureBuffer.h"
#include "SpGpuPlatform.h"

#ifdef LINUX
	#include <GL/glx.h>
#endif

namespace NAMESPACE_PHYSICS
{
	class GpuDevice
	{
	private:
		cl_device_id id;
		cl_context deviceContext;

	public:
		sp_char* name;
		sp_char* version;
		sp_char* driverVersion;
		sp_char* profile;
		sp_size extensionsLength;
		sp_char** extensions;

		cl_device_type type;
		sp_uint computeUnits;
		sp_size maxParameterSize = 256;

		/// <summary>
		/// Maximum number of work - items in a work - group executing a kernel using the data parallel execution model
		/// </summary>
		sp_size maxWorkGroupSize = 1;

		/// <summary>
		/// Maximum dimensions that specify the global and local work - item IDs used by the data parallel execution model
		/// </summary>
		cl_uint maxWorkItemDimension = 3;
		
		/// <summary>
		/// Maximum number of work - items that can be specified in each dimension of the work - group to clEnqueueNDRangeKernel
		/// </summary>
		sp_size maxWorkItemSizes[3];

		sp_size globalMemorySize;
		sp_size globalMemoryCacheSize;
		sp_size localMemorySize;
		sp_size localMemoryLength;
		sp_size maxMemoryAllocSize;
		sp_size constantsBufferSize;
		cl_uint maxConstantArgument;
		cl_uint memoryBaseAddressAlign;
		cl_uint memoryAlignmentRequirement;
		cl_uint clockFrequency;
		
		GpuCommandManager* commandManager = nullptr;
		
		API_INTERFACE inline GpuDevice()
		{
			name = nullptr;
			version = nullptr;
			driverVersion = nullptr;
			profile = nullptr;
			extensionsLength = ZERO_SIZE;
			extensions = nullptr;
		}

		API_INTERFACE void init(cl_device_id id, const SpGpuPlatform& platform);

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

		API_INTERFACE cl_mem createBuffer(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, sp_bool writeValueOnDevice = true);

		API_INTERFACE cl_mem createSubBuffer(cl_mem buffer, cl_buffer_region* region, cl_mem_flags memoryFlags);

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

		API_INTERFACE inline void releaseEvent(cl_event evt)
		{
			HANDLE_OPENCL_ERROR(clReleaseEvent(evt));
		}

		API_INTERFACE inline void releaseEvents(const sp_uint eventsLength, cl_event* evts)
		{
			for (sp_uint i = 0; i < eventsLength; i++)
				releaseEvent(evts[i]);
		}

		API_INTERFACE inline void releaseBuffer(size_t length, cl_mem memoryBuffers ...)
		{
			va_list parameters;
			va_start(parameters, memoryBuffers);

			for (sp_uint i = 0; i < length - 1; i++)
				releaseBuffer(va_arg(parameters, cl_mem));

			va_end(parameters);
		}
		
		API_INTERFACE sp_size getThreadLength(sp_size inputLength);
		API_INTERFACE sp_size getGroupLength(sp_size threadLength, sp_size inputLength);
		API_INTERFACE inline sp_size getDefaultGroupLength()
		{
			return nextPowOf2(multiplyBy2(computeUnits));
		}

		API_INTERFACE inline void waitEvents(sp_uint eventsLength, cl_event* events)
		{
			clWaitForEvents(eventsLength, events);
		}

		~GpuDevice();
	};

}

#endif

#endif // !GPU_DEVICE_HEADER