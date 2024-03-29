#ifdef OPENCL_ENABLED

#ifndef GPU_COMMAND_HEADER
#define GPU_COMMAND_HEADER

#include "SpectrumPhysics.h"
#include "GpuLog.hpp"
#include "GpuBufferOpenCL.h"

namespace NAMESPACE_PHYSICS
{
	class GpuCommand
	{
		friend class GpuCommandManager;

	private:
		cl_device_id deviceId;
		cl_context deviceContext;

		cl_program program = NULL;
		cl_command_queue commandQueue = NULL;
		cl_kernel kernel = NULL;

		std::vector<cl_mem> inputParameters;
		std::vector<sp_size> inputParametersSize;
		std::vector<sp_bool> inputParametersKeep;
		cl_mem outputParameter = NULL;
		sp_size outputSize = 0;
				
		GpuCommand(cl_device_id deviceId, cl_context deviceContext, cl_command_queue commandQueue);

	public:
		sp_size workGroupSize;
		sp_size compileWorkGroupSize[3];
		sp_ulong localMemorySizeRequired;

		sp_double timeToExecuteInMiliseconds = 0.0;

		API_INTERFACE cl_mem getInputParameter(sp_uint index);

		API_INTERFACE GpuCommand* setInputParameter(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, bool keepBuffer = false);
		API_INTERFACE GpuCommand* setInputParameter(void* value, sp_size sizeOfValue);
		API_INTERFACE GpuCommand* setInputParameter(cl_mem buffer, sp_size sizeOfValue);
		API_INTERFACE GpuCommand* setInputParameter(GpuBufferOpenCL* buffer);

		API_INTERFACE GpuCommand* updateInputParameterValue(sp_uint index, const void* value, const sp_uint eventsLength, cl_event* events, cl_event* currentEvent);
		API_INTERFACE GpuCommand* updateInputParameterValueAsync(sp_uint index, const void* value, cl_event* currentEvent);
		API_INTERFACE GpuCommand* updateInputParameter(sp_uint index, cl_mem memoryBuffer);

		API_INTERFACE cl_mem getOutputParameter();
		API_INTERFACE GpuCommand* setOutputParameter(sp_size sizeOfValue);

		API_INTERFACE GpuCommand* swapInputParameter(sp_uint index1, sp_uint index2);

		API_INTERFACE GpuCommand* copyParameters(sp_uint targetParameterIndex, cl_mem destination);
		
		API_INTERFACE GpuCommand* buildFromProgram(cl_program program, const sp_char* kernelName);
		API_INTERFACE GpuCommand* build(const sp_char* source, sp_size sourceSize, const sp_char* kernelName, const sp_char* buildOptions = NULL);

		API_INTERFACE GpuCommand* execute(sp_uint workDimnmsion, const sp_size globalWorkSize[3], const sp_size localWorkSize[3], const sp_size* threadOffset, const sp_uint eventLength, cl_event* events, cl_event* evt);

		API_INTERFACE void waitToFinish();

		API_INTERFACE sp_double getTimeOfExecution(cl_event evt);

		API_INTERFACE inline void fetch(void* buffer)
		{
			HANDLE_OPENCL_RUNTIME_ERROR(clEnqueueReadBuffer(commandQueue, outputParameter, CL_TRUE, 0, outputSize, buffer, 0, NULL, NULL));
		}

		template <typename T>
		API_INTERFACE inline void fetchInOutParameter(sp_uint index, T* result, const sp_uint eventsLength, cl_event* events, cl_event* currentEvent)
		{
			sp_size size = inputParametersSize[index];
			HANDLE_OPENCL_ERROR(clEnqueueReadBuffer(commandQueue, inputParameters[index], CL_TRUE, 0, size, result, eventsLength, events, currentEvent));
		}

		API_INTERFACE void fetchInOutParameter(void* buffer, sp_uint index);

		API_INTERFACE ~GpuCommand();
	};
}

#endif // !GPU_COMMAND_HEADER

#endif // !OPENCL_ENABLED