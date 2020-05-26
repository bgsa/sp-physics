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
	#if defined(CL_VERSION_1_0) || defined(CL_VERSION_1_1) || defined(CL_VERSION_1_2)
			commandQueue = clCreateCommandQueue(deviceContext, deviceId, queueProperties, &errorCode);

	#else
			commandQueue = clCreateCommandQueueWithProperties(deviceContext, deviceId, queueProperties, &errorCode);
	#endif

		HANDLE_OPENCL_ERROR(errorCode);
	}

	GpuCommand* GpuCommandManager::createCommand()
	{
		return sp_mem_new(GpuCommand)(deviceId, deviceContext, commandQueue);
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
		
		errorCode = clBuildProgram(program, 1, &deviceId, options.str().c_str(), NULL, NULL);
		HANDLE_OPENCL_BUILD_ERROR(errorCode, program, deviceId);

		return (sp_uint) cachedPrograms.size() - 1;
	}

	void GpuCommandManager::executeReadBuffer(cl_mem gpuBuffer, size_t bufferSize, void* cpuBuffer, bool waitToFinish)
	{
		HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, gpuBuffer, waitToFinish, 0, bufferSize, cpuBuffer, 0, NULL, NULL));
	}

	cl_event GpuCommandManager::updateBuffer(cl_mem gpuBuffer, sp_size gpuSizeBuffer, const void* value, sp_uint eventsLength, cl_event* eventsToWait)
	{
		cl_event evt;

		HANDLE_OPENCL_ERROR(clEnqueueWriteBuffer(commandQueue,
			gpuBuffer,
			CL_TRUE,
			0,
			gpuSizeBuffer,
			value,
			eventsLength, eventsToWait,
			&evt));

		return evt;
	}

	cl_event GpuCommandManager::copyBuffer(cl_mem source, cl_mem destiny, const sp_size sizeToCopy, const sp_size sourceOffset, const sp_size destinyOffset, sp_uint eventsLength, cl_event* eventsToWait)
	{
		cl_event evt;

		HANDLE_OPENCL_ERROR(clEnqueueCopyBuffer(commandQueue, 
			source, destiny,
			sourceOffset, destinyOffset,
			sizeToCopy,
			eventsLength, eventsToWait,
			&evt));

		return evt;
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
