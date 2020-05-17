#if OPENCL_ENABLED

#include "GpuCommand.h"

namespace NAMESPACE_PHYSICS
{
	GpuCommand::GpuCommand(cl_device_id deviceId, cl_context deviceContext, cl_command_queue commandQueue)
	{
		this->deviceId = deviceId;
		this->deviceContext= deviceContext;
		this->commandQueue = commandQueue;
	}

	cl_mem GpuCommand::getInputParameter(sp_uint index)
	{
		return inputParameters[index];
	}

	GpuCommand* GpuCommand::setInputParameter(cl_mem buffer, sp_size sizeOfValue)
	{	
		inputParameters.emplace_back(buffer);
		inputParametersSize.emplace_back(sizeOfValue);
		inputParametersKeep.emplace_back(true);

		return this;
	}

	GpuCommand* GpuCommand::setInputParameter(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, bool keepBuffer)
	{
		/* if (value == NULL)
			memoryFlags |= CL_MEM_ALLOC_HOST_PTR;
		else
			memoryFlags |= CL_MEM_USE_HOST_PTR; */

		cl_int errorCode;
		cl_mem memoryBuffer = clCreateBuffer(deviceContext, memoryFlags, sizeOfValue, value, &errorCode);
		HANDLE_OPENCL_ERROR(errorCode);

		HANDLE_OPENCL_ERROR(clEnqueueWriteBuffer(commandQueue, memoryBuffer, CL_FALSE, 0, sizeOfValue, value, 0, NULL, NULL));

		inputParameters.emplace_back(memoryBuffer);
		inputParametersSize.emplace_back(sizeOfValue);
		inputParametersKeep.emplace_back(keepBuffer);

		return this;
	}

	GpuCommand* GpuCommand::setInputParameter(void* value, sp_size sizeOfValue)
	{
		return setInputParameter(value, sizeOfValue, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
	}

	cl_mem GpuCommand::getOutputParameter()
	{
		return outputParameter;
	}

	GpuCommand* GpuCommand::setOutputParameter(sp_size sizeOfValue)
	{
		cl_int errorCode;
		outputParameter = clCreateBuffer(deviceContext, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR, sizeOfValue, NULL, &errorCode);
		outputSize = sizeOfValue;

		HANDLE_OPENCL_ERROR(errorCode);

		return this;
	}

	GpuCommand* GpuCommand::updateInputParameterValue(sp_uint index, const void* value)
	{
		HANDLE_OPENCL_ERROR(clEnqueueWriteBuffer(commandQueue, 
			inputParameters[index], 
			CL_TRUE, 
			0, 
			inputParametersSize[index],
			value, 
			0, NULL, NULL));

		return this;
	}

	GpuCommand* GpuCommand::updateInputParameterValueAsync(sp_uint index, const void* value)
	{
		HANDLE_OPENCL_ERROR(
			clEnqueueWriteBuffer(
				commandQueue,
				inputParameters[index],
				CL_FALSE,
				0,
				inputParametersSize[index],
				value,
				0, NULL,
				&lastEvent
			)
		);

		return this;
	}

	GpuCommand* GpuCommand::updateInputParameter(sp_uint index, cl_mem memoryBuffer)
	{
		inputParameters[index] = memoryBuffer;

		HANDLE_OPENCL_ERROR(clSetKernelArg(kernel, (cl_uint) index, sizeof(cl_mem), &inputParameters[index]));

		return this;
	}

	GpuCommand* GpuCommand::swapInputParameter(sp_uint index1, sp_uint index2)
	{
		cl_mem temp1 = inputParameters[index1];
		inputParameters[index1] = inputParameters[index2];
		inputParameters[index2] = temp1;

		sp_size temp2 = inputParametersSize[index1];
		inputParametersSize[index1] = inputParametersSize[index2];
		inputParametersSize[index2] = temp2;

		sp_bool temp3 = inputParametersKeep[index1];
		inputParametersKeep[index1] = inputParametersKeep[index2];
		inputParametersKeep[index2] = temp3;

		HANDLE_OPENCL_ERROR(clSetKernelArg(kernel, (cl_uint)index1, sizeof(cl_mem), &inputParameters[index1]));
		HANDLE_OPENCL_ERROR(clSetKernelArg(kernel, (cl_uint)index2, sizeof(cl_mem), &inputParameters[index2]));

		return this;
	}

	GpuCommand* GpuCommand::copyParameters(sp_uint targetParameterIndex, cl_mem source)
	{
		HANDLE_OPENCL_ERROR(clEnqueueCopyBuffer(
			commandQueue, 
			source,
			inputParameters[targetParameterIndex], 
			0, 
			0, 
			inputParametersSize[targetParameterIndex],
			0,
			NULL, 
			NULL));

		return this;
	}

	GpuCommand* GpuCommand::buildFromProgram(cl_program program, const sp_char* kernelName)
	{
		cl_int errorCode;

		kernel = clCreateKernel(program, kernelName, &errorCode);
		HANDLE_OPENCL_ERROR(errorCode);

		for (cl_uint i = 0; i < inputParameters.size(); i++)
			HANDLE_OPENCL_ERROR(clSetKernelArg(kernel, i, sizeof(cl_mem), &inputParameters[i]));

		if (outputParameter != NULL)
			HANDLE_OPENCL_ERROR(clSetKernelArg(kernel, (cl_uint) inputParameters.size(), sizeof(cl_mem), &outputParameter));

		HANDLE_OPENCL_ERROR(clGetKernelWorkGroupInfo(kernel, deviceId, CL_KERNEL_WORK_GROUP_SIZE, SIZEOF_SIZE, &workGroupSize, NULL));
		HANDLE_OPENCL_ERROR(clGetKernelWorkGroupInfo(kernel, deviceId, CL_KERNEL_COMPILE_WORK_GROUP_SIZE, 3 * SIZEOF_SIZE, &compileWorkGroupSize, NULL));
		HANDLE_OPENCL_ERROR(clGetKernelWorkGroupInfo(kernel, deviceId, CL_KERNEL_LOCAL_MEM_SIZE, SIZEOF_LONG, &localMemorySizeRequired, NULL));

		return this;
	}

	GpuCommand* GpuCommand::build(const sp_char* source, sp_size sourceSize, const sp_char* kernelName, const sp_char* buildOptions)
	{
		cl_int errorCode;
		program = clCreateProgramWithSource(deviceContext, 1, &source, &sourceSize, &errorCode);

		HANDLE_OPENCL_ERROR(errorCode);

		HANDLE_OPENCL_BUILD_ERROR(clBuildProgram(program, 1, &deviceId, buildOptions, NULL, NULL), program, deviceId);

		return buildFromProgram(program, kernelName);
	}

	sp_double GpuCommand::getTimeOfExecution()
	{
		sp_assert(lastEvent != NULL, "NullPointerException");

		cl_ulong timeStart, timeEnd;

		clWaitForEvents(1, &lastEvent);
		clGetEventProfilingInfo(lastEvent, CL_PROFILING_COMMAND_START, sizeof(timeStart), &timeStart, NULL);
		clGetEventProfilingInfo(lastEvent, CL_PROFILING_COMMAND_END, sizeof(timeEnd), &timeEnd, NULL);

		return (timeEnd - timeStart) / 1000000.0;
	}

	void GpuCommand::waitToFinish()
	{
		clFinish(commandQueue);
	}

	GpuCommand* GpuCommand::execute(sp_uint workDimnmsion, const sp_size globalWorkSize[3], const sp_size localWorkSize[3], const sp_size* threadOffset, cl_event* eventstoWait, const sp_uint eventLength)
	{
		sp_assert(globalWorkSize[0] % localWorkSize[0] == 0, "InvalidArgumentException");
		
		HANDLE_OPENCL_ERROR(clEnqueueNDRangeKernel(commandQueue, kernel, workDimnmsion, threadOffset, globalWorkSize, localWorkSize, eventLength, eventstoWait, &lastEvent));

		return this;
	}

	void GpuCommand::fetch(void* buffer)
	{
		HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, outputParameter, CL_TRUE, 0, outputSize, buffer, 0, NULL, NULL));
	}

	template <typename T>
	T* GpuCommand::fetchInOutParameter(sp_uint index)
	{
		sp_size size = inputParametersSize[index];

		void* result = ALLOC_SIZE(size);

		HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, inputParameters[index], CL_TRUE, 0, size, result, 0, NULL, NULL));

		return (T*)result;
	}
	template void*        GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_bool*     GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_char*     GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_short*    GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_int*      GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_size*     GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_longlong* GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_float*    GpuCommand::fetchInOutParameter(sp_uint index);
	template sp_double*   GpuCommand::fetchInOutParameter(sp_uint index);

	void GpuCommand::fetchInOutParameter(void* buffer, sp_uint index)
	{
		sp_size size = inputParametersSize[index];

		HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, inputParameters[index], CL_TRUE, 0, size, buffer, 0, NULL, NULL));
	}

	GpuCommand::~GpuCommand()
	{
		if (kernel != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseKernel(kernel));
			kernel = nullptr;
		}
			
		if (program != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseProgram(program));
			program = nullptr;
		}
		
		for (sp_uint i = 0; i < inputParameters.size(); i++)
			if (!inputParametersKeep[i]) 
				HANDLE_OPENCL_ERROR(clReleaseMemObject(inputParameters[i]));

		if (outputParameter != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseMemObject(outputParameter));
			outputParameter = nullptr;
		}
	}
}

#endif