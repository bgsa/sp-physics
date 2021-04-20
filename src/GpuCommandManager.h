#ifdef OPENCL_ENABLED

#ifndef GPU_COMMAND_MANAGER
#define GPU_COMMAND_MANAGER

#include "SpectrumPhysics.h"
#include "GpuCommand.h"
#include "SpDirectory.h"

namespace NAMESPACE_PHYSICS
{
	class GpuCommandManager
	{
		friend class GpuDevice;

	private:
		cl_device_id deviceId;
		cl_context deviceContext;
		cl_command_queue commandQueue;

		GpuCommandManager(cl_context deviceContext, cl_device_id deviceId, cl_queue_properties queueProperties = ZERO_SIZE);

	public:

		API_INTERFACE GpuCommand* createCommand();

		API_INTERFACE void buildProgram(const sp_char* source, sp_size sourceSize, const sp_char* buildOptions, cl_program* program);

		API_INTERFACE inline void acquireGLObjects(cl_mem buffer, sp_uint eventLength, cl_event* events, cl_event* evt)
		{	
			clEnqueueAcquireGLObjects(commandQueue, ONE_UINT, &buffer, eventLength, events, evt);
		}

		API_INTERFACE inline void releaseGLObjects(cl_mem buffer, sp_uint eventLength, cl_event* events, cl_event* evt)
		{
			clEnqueueReleaseGLObjects(commandQueue, ONE_UINT, &buffer, eventLength, events, evt);
		}

		API_INTERFACE cl_event readBuffer(cl_mem gpuBuffer, sp_size bufferSize, void* cpuBuffer, sp_uint eventsLength = ZERO_UINT, cl_event* eventsToWait = nullptr)
		{
			cl_event evt;
			HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, gpuBuffer, true, 0, bufferSize, cpuBuffer, eventsLength, eventsToWait, &evt));
			return evt;
		}
		
		API_INTERFACE cl_event readBufferAsync(cl_mem gpuBuffer, sp_size bufferSize, void* cpuBuffer, sp_uint eventsLength = ZERO_UINT, cl_event* eventsToWait = nullptr)
		{
			cl_event evt;
			HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, gpuBuffer, false, 0, bufferSize, cpuBuffer, eventsLength, eventsToWait, &evt));
			return evt;
		}

		API_INTERFACE void updateBuffer(cl_mem gpuBuffer, sp_size gpuSizeBuffer, const void* value, sp_uint eventsLength, cl_event* eventsToWait, cl_event* evt);

		API_INTERFACE cl_event copyBuffer(cl_mem source, cl_mem destiny, const sp_size sizeToCopy
			, const sp_size sourceOffset = 0, const sp_size destinyOffset = 0
			, sp_uint eventsLength = 0, cl_event* eventsToWait = nullptr);

		API_INTERFACE void flush();

		API_INTERFACE ~GpuCommandManager();
	};

}

#endif // GPU_COMMAND_MANAGER

#endif // OPENCL_ENABLED
