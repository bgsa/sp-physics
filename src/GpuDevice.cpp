#if OPENCL_ENABLED

#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{
	GpuDevice::GpuDevice(cl_device_id id, cl_platform_id platformId)
	{
		this->id = id;

		cl_int errorCode;

	#ifdef WINDOWS
		HGLRC glCtx = wglGetCurrentContext();
		if (glCtx != nullptr)
		{
			cl_context_properties contextProperties[] = { CL_CONTEXT_PLATFORM,(cl_context_properties)platformId,
												CL_WGL_HDC_KHR,(intptr_t)wglGetCurrentDC(),
												CL_GL_CONTEXT_KHR,(intptr_t)glCtx,0 };

			this->deviceContext = clCreateContext(contextProperties, 1, &id, NULL, NULL, &errorCode);
		}
		else
			this->deviceContext = clCreateContext(NULL, 1, &id, NULL, NULL, &errorCode);
	#else
		if (glCtx != nullptr)
		{
			GLXContext glCtx = glXGetCurrentContext();
			cl_context_properties contextProperties[] = { CL_CONTEXT_PLATFORM,(cl_context_properties)platform,
												CL_GLX_DISPLAY_KHR,(intptr_t)glXGetCurrentDisplay(),
												CL_GL_CONTEXT_KHR,(intptr_t)glCtx,0 };
			this->deviceContext = clCreateContext(NULL, 1, &id, NULL, NULL, &errorCode);
		}
		else
			this->deviceContext = clCreateContext(contextProperties, 1, &id, NULL, NULL, &errorCode);
	#endif
		HANDLE_OPENCL_ERROR(errorCode);
		
		this->commandManager = sp_mem_new(GpuCommandManager)(deviceContext, id);
			
		sp_size valueSize;
		clGetDeviceInfo(id, CL_DEVICE_NAME, 0, NULL, &valueSize);
		name = (sp_char*)ALLOC_SIZE(valueSize);
		clGetDeviceInfo(id, CL_DEVICE_NAME, valueSize, name, NULL);

		clGetDeviceInfo(id, CL_DEVICE_VERSION, 0, NULL, &valueSize);
		version = (sp_char*)ALLOC_SIZE(valueSize);
		clGetDeviceInfo(id, CL_DEVICE_VERSION, valueSize, version, NULL);

		clGetDeviceInfo(id, CL_DRIVER_VERSION, 0, NULL, &valueSize);
		driverVersion = (sp_char*)ALLOC_SIZE(valueSize);
		clGetDeviceInfo(id, CL_DRIVER_VERSION, valueSize, driverVersion, NULL);
		
		//clGetDeviceInfo(id, CL_DEVICE_AVAILABLE, sizeof(isAvailable), &isAvailable, NULL);	
		clGetDeviceInfo(id, CL_DEVICE_TYPE, sizeof(type), &type, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_COMPUTE_UNITS, sizeof(computeUnits), &computeUnits, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_PARAMETER_SIZE, sizeof(maxParameterSize), &maxParameterSize, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_WORK_GROUP_SIZE, sizeof(maxWorkGroupSize), &maxWorkGroupSize, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS, sizeof(maxWorkItemDimension), &maxWorkItemDimension, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_WORK_ITEM_SIZES, sizeof(size_t) * 3, maxWorkItemSizes, NULL);
		clGetDeviceInfo(id, CL_DEVICE_GLOBAL_MEM_SIZE, sizeof(cl_ulong), &globalMemorySize, NULL);
		clGetDeviceInfo(id, CL_DEVICE_GLOBAL_MEM_CACHE_SIZE, sizeof(cl_ulong), &globalMemoryCacheSize, NULL);	
		clGetDeviceInfo(id, CL_DEVICE_LOCAL_MEM_SIZE, sizeof(cl_ulong), &localMemorySize, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_CONSTANT_BUFFER_SIZE, sizeof(cl_ulong), &constantsBufferSize, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_CONSTANT_ARGS, sizeof(cl_ulong), &maxConstantArgument, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MEM_BASE_ADDR_ALIGN, sizeof(cl_uint), &memoryBaseAddressAlign, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_CLOCK_FREQUENCY, sizeof(cl_uint), &clockFrequency, NULL);
		clGetDeviceInfo(id, CL_DEVICE_MAX_MEM_ALLOC_SIZE, sizeof(cl_ulong), &maxMemoryAllocSize, NULL);

		memoryAlignmentRequirement = lcm(1, memoryBaseAddressAlign);

		localMemoryLength = divideBy4(localMemorySize);
		
		clGetDeviceInfo(id, CL_DEVICE_PROFILE, 0, NULL, &valueSize);
		sp_char* profileAsArray = (sp_char*)ALLOC_SIZE(valueSize);
		clGetDeviceInfo(id, CL_DEVICE_PROFILE, valueSize, profileAsArray, NULL);
		profile = std::string(profileAsArray);

		clGetDeviceInfo(id, CL_DEVICE_EXTENSIONS, 0, NULL, &valueSize);
		sp_char* extensionsAsArray = (sp_char*)ALLOC_SIZE(valueSize);
		clGetDeviceInfo(id, CL_DEVICE_EXTENSIONS, valueSize, extensionsAsArray, NULL);

		std::string extensionName;
		for (sp_uint i = 0; i < valueSize; i++)
		{
			if (extensionsAsArray[i] == ' ')
			{
				extensions.emplace_back(extensionName);
				extensionName.clear();
			}
			else
				extensionName += extensionsAsArray[i];
		}

		ALLOC_RELEASE(profileAsArray);
	}

	sp_uint GpuDevice::getThreadLength(sp_uint inputLength)
	{
		if (isOdd(inputLength))
			inputLength--;

		sp_uint defaultGroupLength = getDefaultGroupLength();

		if (inputLength < defaultGroupLength)
			return inputLength;

		inputLength = divideBy2(inputLength);

		sp_uint maxLength = 2u;
		for (; maxLength < inputLength; maxLength *= 2)
		{
			sp_uint divisor = nextDivisorOf(maxLength - 1, defaultGroupLength);
			if (divisor > maxWorkGroupSize)
				break;
		}
		while (inputLength >= maxLength)
			inputLength = divideBy2(inputLength);

		return inputLength;
	}

	sp_uint GpuDevice::getGroupLength(sp_uint threadLength, sp_uint inputLength)
	{
		sp_uint groupLength = getDefaultGroupLength();

		//while (groupLength > threadLength)
		//	groupLength = divideBy2(groupLength);
		if (groupLength > threadLength)
			return ONE_UINT;

		groupLength = nextDivisorOf(threadLength, groupLength);

		if (threadLength == ONE_UINT)
		{
			threadLength = (sp_uint)std::ceil(nextPowOf2(inputLength) / TWO_DOUBLE);
			groupLength = (sp_uint)std::ceil(threadLength / TWO_DOUBLE);
		}

		return groupLength;
	}

	cl_mem GpuDevice::createBuffer(void* value, sp_size sizeOfValue, cl_mem_flags memoryFlags, sp_bool writeValueOnDevice)
	{
		cl_int errorCode;
		cl_mem memoryBuffer = clCreateBuffer(deviceContext, memoryFlags, sizeOfValue, value, &errorCode);
		HANDLE_OPENCL_ERROR(errorCode);

		if (writeValueOnDevice)
			HANDLE_OPENCL_ERROR(clEnqueueWriteBuffer(commandManager->commandQueue, memoryBuffer, CL_FALSE, 0, sizeOfValue, value, 0, NULL, NULL));

		return memoryBuffer;
	}
	
	cl_mem GpuDevice::createSubBuffer(cl_mem buffer, cl_buffer_region* region, cl_mem_flags memoryFlags)
	{
		cl_int errorCode;
		cl_mem subBuffer = clCreateSubBuffer(buffer, memoryFlags, CL_BUFFER_CREATE_TYPE_REGION, region, &errorCode);
		HANDLE_OPENCL_ERROR(errorCode);

		return subBuffer;
	}

	GpuDevice::~GpuDevice()
	{
		if (commandManager != nullptr)
		{
			sp_mem_delete(commandManager, GpuCommandManager);
			commandManager = nullptr;
		}

		clReleaseContext(deviceContext);

		ALLOC_RELEASE(driverVersion);
		ALLOC_RELEASE(version);
		ALLOC_RELEASE(name);
	}
}

#endif