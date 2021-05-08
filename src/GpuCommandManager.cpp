#ifdef OPENCL_ENABLED

#include "GpuCommandManager.h"

namespace NAMESPACE_PHYSICS
{
	GpuCommandManager::GpuCommandManager(cl_context deviceContext, cl_device_id deviceId, cl_queue_properties queueProperties)
	{
		this->deviceContext = deviceContext;
		this->deviceId = deviceId;

		cl_int errorCode;

		if (SpPhysicSettings::instance()->isProfilingEnabled())
			queueProperties |= CL_QUEUE_PROFILING_ENABLE;

	#if (CL_TARGET_OPENCL_VERSION > 120)
			commandQueue = clCreateCommandQueueWithProperties(deviceContext, deviceId, &queueProperties, &errorCode);
	#else
			#pragma warning(suppress : 4996)
			commandQueue = clCreateCommandQueue(deviceContext, deviceId, queueProperties, &errorCode);
	#endif

		HANDLE_OPENCL_ERROR(errorCode);
	}

	GpuCommand* GpuCommandManager::createCommand()
	{
		return sp_mem_new(GpuCommand)(deviceId, deviceContext, commandQueue);
	}

	void GpuCommandManager::buildProgram(const sp_char* source, sp_size sourceSize, const sp_char* buildOptions, cl_program* program)
	{
		sp_assert(program != nullptr, "InvalidArgumentException");

		cl_int errorCode;
		program[0] = clCreateProgramWithSource(deviceContext, 1, &source, &sourceSize, &errorCode);
		HANDLE_OPENCL_ERROR(errorCode);

		sp_uint computeUnits;
		clGetDeviceInfo(deviceId, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(computeUnits), &computeUnits, NULL);

		SpDirectory* sourceDirectory = SpDirectory::currentDirectory()->add(SP_DIRECTORY_OPENCL_SOURCE);

		std::ostringstream options;
		options << " -I . "
			<< " -I " << sourceDirectory->name()->data()
			<< " -Werror "
			<< " -cl-denorms-are-zero "
			<< " -cl-mad-enable "
			<< " -cl-finite-math-only "
			<< " -DCOMPUTE_UNITS=" << computeUnits;
		// -cl-no-signed-zero   -cl-fast-relaxed-math    not working

		sp_mem_delete(sourceDirectory, SpDirectory);

		if (buildOptions != NULL)
			options	<< " " << buildOptions;
		
		errorCode = clBuildProgram(program[0], 1, &deviceId, options.str().c_str(), NULL, NULL);
		HANDLE_OPENCL_BUILD_ERROR(errorCode, program[0], deviceId);
	}

	void GpuCommandManager::updateBuffer(cl_mem gpuBuffer, sp_size gpuSizeBuffer, const void* value, sp_uint eventsLength, cl_event* eventsToWait, cl_event* evt)
	{
		HANDLE_OPENCL_ERROR(clEnqueueWriteBuffer(commandQueue,
			gpuBuffer,
			CL_TRUE,
			0,
			gpuSizeBuffer,
			value,
			eventsLength, eventsToWait,
			evt));
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

		HANDLE_OPENCL_ERROR(clReleaseCommandQueue(commandQueue));
	}
}

#endif
