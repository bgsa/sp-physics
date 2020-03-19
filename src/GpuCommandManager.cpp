#ifdef OPENCL_ENABLED

#include "GpuCommandManager.h"

namespace NAMESPACE_PHYSICS
{
	GpuCommandManager::GpuCommandManager(cl_context deviceContext, cl_device_id deviceId, cl_command_queue_properties queueProperties)
	{
		this->deviceContext = deviceContext;
		this->deviceId = deviceId;

		cl_int errorCode;

	#ifdef DEBUG
			queueProperties |= CL_QUEUE_PROFILING_ENABLE;
	#endif

		commandQueue = clCreateCommandQueue(deviceContext, deviceId, queueProperties, &errorCode);

		HANDLE_OPENCL_ERROR(errorCode);
	}

	GpuCommand* GpuCommandManager::createCommand()
	{
		return ALLOC_NEW(GpuCommand)(deviceId, deviceContext, commandQueue);
	}

	sp_uint GpuCommandManager::cacheProgram(const char* source, size_t sourceSize, const char* buildOptions)
	{
		cl_int errorCode;
		cl_program program = clCreateProgramWithSource(deviceContext, 1, &source, &sourceSize, &errorCode);
		HANDLE_OPENCL_ERROR(errorCode);

		cachedPrograms.emplace_back(program);

		sp_uint computeUnits;
		clGetDeviceInfo(deviceId, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(computeUnits), &computeUnits, NULL);

		std::ostringstream options;
		options << " -I . "
			<< " -Werror "
			<< " -cl-denorms-are-zero "
			<< " -cl-mad-enable "
			<< " -cl-finite-math-only "
			<< " -DCOMPUTE_UNITS=" << computeUnits;
		// -cl-no-signed-zero   -cl-fast-relaxed-math    not working

		if (buildOptions != NULL)
			options	<< " " << buildOptions;
		

		HANDLE_OPENCL_BUILD_ERROR(clBuildProgram(program, 1, &deviceId, options.str().c_str(), NULL, NULL), program, deviceId);

		return (sp_uint) cachedPrograms.size() - 1;
	}

	void GpuCommandManager::executeReadBuffer(cl_mem gpuBuffer, size_t bufferSize, void* cpuBuffer, bool waitToFinish)
	{
		HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, gpuBuffer, waitToFinish, 0, bufferSize, cpuBuffer, 0, NULL, NULL));
	}

	void GpuCommandManager::flush()
	{
		HANDLE_OPENCL_ERROR(clFlush(commandQueue));
		HANDLE_OPENCL_ERROR(clFinish(commandQueue));
	}

	GpuCommandManager::~GpuCommandManager()
	{
		HANDLE_OPENCL_ERROR(clFlush(commandQueue));
		HANDLE_OPENCL_ERROR(clFinish(commandQueue));

		for (sp_uint i = 0 ; i < cachedPrograms.size() ; i++ )
			HANDLE_OPENCL_ERROR(clReleaseProgram(cachedPrograms[i]));

		HANDLE_OPENCL_ERROR(clReleaseCommandQueue(commandQueue));
	}
}

#endif