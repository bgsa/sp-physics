#if OPENCL_ENABLED

#pragma once

#include "OpenML.h"
#include "GpuLog.hpp"
#include "CL/cl.h"

namespace OpenML
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

		double timeToExecuteInMiliseconds = 0.0;

		cl_mem getInputParameter(sp_uint index);

		GpuCommand* setInputParameter(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, bool keepBuffer = false);
		GpuCommand* setInputParameter(void* value, sp_size sizeOfValue);
		GpuCommand* setInputParameter(cl_mem buffer, sp_size sizeOfValue);

		GpuCommand* updateInputParameterValue(sp_uint index, const void* value);
		GpuCommand* updateInputParameter(sp_uint index, cl_mem memoryBuffer);

		cl_mem getOutputParameter();
		GpuCommand* setOutputParameter(sp_size sizeOfValue);

		GpuCommand* swapInputParameter(sp_uint index1, sp_uint index2);

		GpuCommand* copyParameters(sp_uint targetParameterIndex, cl_mem destination);
		
		GpuCommand* buildFromProgram(cl_program program, const sp_char* kernelName);
		GpuCommand* build(const sp_char* source, sp_size sourceSize, const sp_char* kernelName, const sp_char* buildOptions = NULL);

		GpuCommand* execute(sp_size workDimnmsion, const sp_size* globalWorkSize, const sp_size* localWorkSize, const sp_size* globalOffset = NULL);

		void fetch(void* buffer);

		template <typename T> 
		T* fetch();

		void fetchInOutParameter(void* buffer, sp_uint index);

		template <typename T>
		T* fetchInOutParameter(sp_uint index);

		~GpuCommand();
	};

}

#endif