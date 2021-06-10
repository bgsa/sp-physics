#ifdef OPENCL_ENABLED

#ifndef GPU_LOG_HEADER
#define GPU_LOG_HEADER

#include "SpectrumPhysics.h"
#include "SpOpenCL.h"
#include "SpLogger.h"

#if DEBUG
	#define HANDLE_OPENCL_ERROR(errorCode) GpuLog::handleCompileError(errorCode);
#else
	#define HANDLE_OPENCL_ERROR
#endif

#if DEBUG
#define HANDLE_OPENCL_RUNTIME_ERROR(errorCode) GpuLog::handleRuntimeError(errorCode);
#else
#define HANDLE_OPENCL_RUNTIME_ERROR
#endif

#if DEBUG
	#define HANDLE_OPENCL_BUILD_ERROR(errorCode, program, deviceId) GpuLog::handleBuildError(errorCode, program, deviceId);
#else
	#define HANDLE_OPENCL_BUILD_ERROR
#endif

namespace NAMESPACE_PHYSICS
{
	class GpuLog
	{
	public:

		///<summary>
		///Handle Build errors
		///</summary>
		API_INTERFACE static void handleBuildError(sp_int errorCode, cl_program program, cl_device_id deviceId)
		{
			if (errorCode == CL_BUILD_PROGRAM_FAILURE)
			{
				sp_size logSize;

				errorCode = clGetProgramBuildInfo(program, deviceId, CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);

				sp_char* errorMessage = (char*)ALLOC_SIZE(logSize + 1);

				clGetProgramBuildInfo(program, deviceId, CL_PROGRAM_BUILD_LOG, logSize, errorMessage, NULL);

				errorMessage[logSize] = END_OF_STRING;

				sp_log_error1s(errorMessage);
				sp_assert(false, "OpenCLBuildException");
				ALLOC_RELEASE(errorMessage);
			}
		}

		/// <summary>
		/// Get the runtime error message, given the opencl code
		/// </summary>
		/// <param name="errorCode"></param>
		/// <param name="message"></param>
		/// <returns></returns>
		API_INTERFACE static void runtimeErrorMessage(sp_int errorCode, sp_char* message)
		{
			switch (errorCode)
			{
			case CL_SUCCESS:
				std::strcpy(message, "Success");
				return;

			case CL_DEVICE_NOT_FOUND:
				std::strcpy(message, "Device not found");
				return;

			case CL_DEVICE_NOT_AVAILABLE:
				std::strcpy(message, "Device not available");
				return;

			case CL_COMPILER_NOT_AVAILABLE:
				std::strcpy(message, "Compiler not available");
				return;

			case CL_MEM_OBJECT_ALLOCATION_FAILURE:
				std::strcpy(message, "Memory allocation failure");
				return;

			case CL_OUT_OF_RESOURCES:
				std::strcpy(message, "Out of resources");
				return;

			case CL_OUT_OF_HOST_MEMORY:
				std::strcpy(message, "Out of host memory");
				return;

			case CL_PROFILING_INFO_NOT_AVAILABLE:
				std::strcpy(message, "Profiling Info not available");
				return;

			case CL_MEM_COPY_OVERLAP:
				std::strcpy(message, "Memory copy overlapy (eg: source and destination buffer are the same)");
				return;

			case CL_IMAGE_FORMAT_MISMATCH:
				std::strcpy(message, "Image format mismatch (eg: the source and destination image do not use the same format)");
				return;

			case CL_IMAGE_FORMAT_NOT_SUPPORTED:
				std::strcpy(message, "Image format not supported");
				return;

			case CL_BUILD_PROGRAM_FAILURE:
				std::strcpy(message, "Build program failure");
				return;

			case CL_MAP_FAILURE:
				std::strcpy(message, "Map failure: failure to map the requested region into the host address space");
				return;

			case CL_MISALIGNED_SUB_BUFFER_OFFSET:
				std::strcpy(message, "Sub-buffer object is specified as the value for an argument that is a buffer object and the offset specified when the sub-buffer object is created is not aligned to the device memory allignment");
				return;

			case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST:
				std::strcpy(message, "The execution status of any of the events in event_list is a negative integer value");
				return;

			case CL_COMPILE_PROGRAM_FAILURE:
				std::strcpy(message, "There is a failure to compile the program source");
				return;

			case CL_LINKER_NOT_AVAILABLE:
				std::strcpy(message, "Linker is not available (eg: CL_DEVICE_LINKER_AVAILABLE specified in the table of allowed values for param_name for clGetDeviceInfo is set to CL_FALSE)");
				return;

			case CL_LINK_PROGRAM_FAILURE:
				std::strcpy(message, "There is a failure to link the compiled binaries and/or libraries");
				return;

			case CL_DEVICE_PARTITION_FAILED:
				std::strcpy(message, "The partition name is supported by the implementation but in_device could not be further partitioned");
				return;

			case CL_KERNEL_ARG_INFO_NOT_AVAILABLE:
				std::strcpy(message, "The argument information is not available for kernel");
				return;

			case CL_INVALID_EVENT_WAIT_LIST:
				std::strcpy(message, "Invalid Event Wait List");
				return;

			default:
				std::strcpy(message, "Unknonwn OpenCL runtime error");
				return;
			}
		}

		///<summary>
		///Handle Runtime errors
		///</summary>
		API_INTERFACE static void handleRuntimeError(sp_int errorCode)
		{
			sp_char message[1024];
			runtimeErrorMessage(errorCode, message);
			sp_assert(errorCode == CL_SUCCESS, message);
		}

		///<summary>
		///Handle Compile errors
		///</summary>
		API_INTERFACE static void handleCompileError(sp_int errorCode)
		{
			sp_char message[1024];
			compileErrorMessage(errorCode, message);
			sp_assert(errorCode == CL_SUCCESS, message);
		}

		///<summary>
		///	Get the compile error message, given the opencl code
		///</summary>
		API_INTERFACE static void compileErrorMessage(sp_int errorCode, sp_char* message)
		{
			switch (errorCode)
			{
			case CL_SUCCESS:
				std::strcpy(message, "Success");
				return;

			case CL_INVALID_VALUE:
				std::strcpy(message, "Invalid value");
				return;

			case CL_INVALID_DEVICE_TYPE:
				std::strcpy(message, "Invalid device type");
				return;

			case CL_INVALID_PLATFORM:
				std::strcpy(message, "Invalid plataform");
				return;

			case CL_INVALID_DEVICE:
				std::strcpy(message, "Invalid device");
				return;

			case CL_INVALID_CONTEXT:
				std::strcpy(message, "Invalid context");
				return;

			case CL_INVALID_QUEUE_PROPERTIES:
				std::strcpy(message, "Invalid queue properties");
				return;

			case CL_INVALID_COMMAND_QUEUE:
				std::strcpy(message, "Invalid command queue");
				return;

			case CL_INVALID_HOST_PTR:
				std::strcpy(message, "Invalid host pointer. This flag is valid only if host_ptr is not NULL");
				return;

			case CL_INVALID_MEM_OBJECT:
				std::strcpy(message, "Invalid memory object");
				return;

			case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR:
				std::strcpy(message, "Invalid image format descriptor");
				return;

			case CL_INVALID_IMAGE_SIZE:
				std::strcpy(message, "Invalid image size");
				return;

			case CL_INVALID_SAMPLER:
				std::strcpy(message, "Invalid sampler");
				return;

			case CL_INVALID_BINARY:
				std::strcpy(message, "Invalid binary");
				return;

			case CL_INVALID_BUILD_OPTIONS:
				std::strcpy(message, "Invalid build options");
				return;

			case CL_INVALID_PROGRAM:
				std::strcpy(message, "Invalid program");
				return;

			case CL_INVALID_PROGRAM_EXECUTABLE:
				std::strcpy(message, "Invalid program executable");
				return;

			case CL_INVALID_KERNEL_NAME:
				std::strcpy(message, "Invalid kernel name");
				return;

			case CL_INVALID_KERNEL_DEFINITION:
				std::strcpy(message, "Invalid kernel definition");
				return;

			case CL_INVALID_KERNEL:
				std::strcpy(message, "Invalid kernel");
				return;

			case CL_INVALID_ARG_INDEX:
				std::strcpy(message, "Invalid argument index");
				return;

			case CL_INVALID_ARG_VALUE:
				std::strcpy(message, "Invalid argument value");
				return;

			case CL_INVALID_ARG_SIZE:
				std::strcpy(message, "Invalid argument size");
				return;

			case CL_INVALID_KERNEL_ARGS:
				std::strcpy(message, "Invalid kernel arguments");
				return;

			case CL_INVALID_WORK_DIMENSION:
				std::strcpy(message, "Invalid work dimension");
				return;

			case CL_INVALID_WORK_GROUP_SIZE:
				std::strcpy(message, "Invalid work group size");
				return;

			case CL_INVALID_WORK_ITEM_SIZE:
				std::strcpy(message, "Invalid work item size");
				return;

			case CL_INVALID_GLOBAL_OFFSET:
				std::strcpy(message, "Invalid global offset");
				return;

			case CL_INVALID_EVENT_WAIT_LIST:
				std::strcpy(message, "Invalid event wait list");
				return;

			case CL_INVALID_EVENT:
				std::strcpy(message, "Invalid event");
				return;

			case CL_INVALID_OPERATION:
				std::strcpy(message, "Invalid operation");
				return;

			case CL_INVALID_GL_OBJECT:
				std::strcpy(message, "Invalid GL object");
				return;

			case CL_INVALID_BUFFER_SIZE:
				std::strcpy(message, "Invalid buffer size");
				return;

			case CL_INVALID_MIP_LEVEL:
				std::strcpy(message, "MipLevel is greater than zero and the OpenGL implementation does not support creating from non-zero mipmap levels");
				return;

			case CL_INVALID_GLOBAL_WORK_SIZE:
				std::strcpy(message, "Invalid global work size");
				return;

			case CL_INVALID_PROPERTY:
				std::strcpy(message, "Invalid property");
				return;

			case CL_INVALID_IMAGE_DESCRIPTOR:
				std::strcpy(message, "Invalid image descriptor");
				return;

			case CL_INVALID_COMPILER_OPTIONS:
				std::strcpy(message, "Invalid compiler options");
				return;

			case CL_INVALID_LINKER_OPTIONS:
				std::strcpy(message, "Invalid linker options");
				return;

			case CL_INVALID_DEVICE_PARTITION_COUNT:
				std::strcpy(message, "Invalid device partition count");
				return;

			case -13: // OPENCL_2.0 ->  CL_MISALIGNED_SUB _BUFFER_OFFSET
				std::strcpy(message, "Misaligned subbuffer offset");
				return;

				/*
			case CL_INVALID_PIPE_SIZE:
				return "Invalid pipe size";

			case CL_INVALID_DEVICE_QUEUE:
				return "Invalid device queue";
				*/

				// extensions error
			case -1000: 
				std::strcpy(message, "CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR");
				return;

			case -1001: 
				std::strcpy(message, "CL_PLATFORM_NOT_FOUND_KHR");
				return;

			case -1002: 
				std::strcpy(message, "CL_INVALID_D3D10_DEVICE_KHR");
				return;

			case -1003:
				std::strcpy(message, "CL_INVALID_D3D10_RESOURCE_KHR");
				return;

			case -1004: 
				std::strcpy(message, "CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR");
				return;

			case -1005: 
				std::strcpy(message, "CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR");
				return;

			default:
				std::strcpy(message, "Unknown OpenCL compile error");
				return;
			}
		}
	};
}

#endif // GPU_LOG_HEADER

#endif // OPENCL_ENALBED
