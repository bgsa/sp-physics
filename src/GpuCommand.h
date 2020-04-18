#if OPENCL_ENABLED

#ifndef GPU_COMMAND_HEADER
#define GPU_COMMAND_HEADER

#include "SpectrumPhysics.h"
#include "GpuLog.hpp"
#include "CL/cl.h"

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

		cl_event lastEvent = NULL;

		sp_size workGroupSize;
		sp_size compileWorkGroupSize[3];
		sp_ulong localMemorySizeRequired;

		sp_double timeToExecuteInMiliseconds = 0.0;

		API_INTERFACE cl_mem getInputParameter(sp_uint index);

		API_INTERFACE GpuCommand* setInputParameter(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, bool keepBuffer = false);
		API_INTERFACE GpuCommand* setInputParameter(void* value, sp_size sizeOfValue);
		API_INTERFACE GpuCommand* setInputParameter(cl_mem buffer, sp_size sizeOfValue);

		API_INTERFACE GpuCommand* updateInputParameterValue(sp_uint index, const void* value);
		API_INTERFACE GpuCommand* updateInputParameterValueAsync(sp_uint index, const void* value);
		API_INTERFACE GpuCommand* updateInputParameter(sp_uint index, cl_mem memoryBuffer);

		API_INTERFACE cl_mem getOutputParameter();
		API_INTERFACE GpuCommand* setOutputParameter(sp_size sizeOfValue);

		API_INTERFACE GpuCommand* swapInputParameter(sp_uint index1, sp_uint index2);

		API_INTERFACE GpuCommand* copyParameters(sp_uint targetParameterIndex, cl_mem destination);
		
		API_INTERFACE GpuCommand* buildFromProgram(cl_program program, const sp_char* kernelName);
		API_INTERFACE GpuCommand* build(const sp_char* source, sp_size sourceSize, const sp_char* kernelName, const sp_char* buildOptions = NULL);

		API_INTERFACE GpuCommand* execute(sp_uint workDimnmsion, const sp_size globalWorkSize[3], const sp_size localWorkSize[3], const sp_size* threadOffset = 0, cl_event* events = NULL, const sp_uint eventLength = 0);

		API_INTERFACE void waitToFinish();

		API_INTERFACE sp_double getTimeOfExecution();

		API_INTERFACE void fetch(void* buffer);

		template <typename T>
		API_INTERFACE T* fetchInOutParameter(sp_uint index);

		API_INTERFACE void fetchInOutParameter(void* buffer, sp_uint index);

		API_INTERFACE ~GpuCommand();
	};
}

#endif // !GPU_COMMAND_HEADER

#endif // !OPENCL_ENABLED