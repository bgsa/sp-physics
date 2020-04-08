#ifdef OPENCL_ENABLED

#ifndef GPU_COMMAND_MANAGER
#define GPU_COMMAND_MANAGER

#include "GpuCommand.h"

namespace NAMESPACE_PHYSICS
{
	class GpuCommandManager
	{
		friend class GpuDevice;

	private:
		cl_device_id deviceId;
		cl_context deviceContext;
		cl_command_queue commandQueue;

		GpuCommandManager(cl_context deviceContext, cl_device_id deviceId, cl_command_queue_properties queueProperties = NULL);

	public:
		std::vector<cl_program> cachedPrograms;

		API_INTERFACE GpuCommand* createCommand();

		API_INTERFACE sp_uint cacheProgram(const char* source, size_t sourceSize, const char* buildOptions);

		API_INTERFACE void executeReadBuffer(cl_mem gpuBuffer, size_t bufferSize, void* cpuBuffer, bool waitToFinish);

		API_INTERFACE void flush();

		API_INTERFACE ~GpuCommandManager();
	};

}

#endif // GPU_COMMAND_MANAGER

#endif // OPENCL_ENABLED