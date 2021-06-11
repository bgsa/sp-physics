#if OPENCL_ENABLED

#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{
	void GpuDevice::init(cl_device_id id, const SpGpuPlatform& platform)
	{
		this->id = id;

		cl_int errorCode;

		deviceContext = nullptr;

	#ifdef WINDOWS
		HGLRC glCtx = wglGetCurrentContext();
		if (glCtx != nullptr)
		{
			cl_context_properties contextProperties[] = { CL_CONTEXT_PLATFORM,(cl_context_properties)platform.id,
												CL_WGL_HDC_KHR,(intptr_t)wglGetCurrentDC(),
												CL_GL_CONTEXT_KHR,(intptr_t)glCtx,0 };

			deviceContext = clCreateContext(contextProperties, 1, &id, NULL, NULL, &errorCode);
		}
	#else
		GLXContext glCtx = glXGetCurrentContext();
		if (glCtx != nullptr)
		{			
			cl_context_properties contextProperties[] = { CL_CONTEXT_PLATFORM,(cl_context_properties)platformId,
												CL_GLX_DISPLAY_KHR,(intptr_t)glXGetCurrentDisplay(),
												CL_GL_CONTEXT_KHR,(intptr_t)glCtx,0 };
			deviceContext = clCreateContext(contextProperties, 1, &id, NULL, NULL, &errorCode);
		}
	#endif
		else
			deviceContext = clCreateContext(NULL, 1, &id, NULL, NULL, &errorCode);

		HANDLE_OPENCL_ERROR(errorCode);
		
		this->commandManager = sp_mem_new(GpuCommandManager)(deviceContext, id);
			
		sp_size valueSize;
		clGetDeviceInfo(id, CL_DEVICE_NAME, 0, NULL, &valueSize);
		name = sp_mem_new_array(sp_char, valueSize);
		clGetDeviceInfo(id, CL_DEVICE_NAME, valueSize, name, NULL);

		clGetDeviceInfo(id, CL_DEVICE_VERSION, 0, NULL, &valueSize);
		version = sp_mem_new_array(sp_char, valueSize);
		clGetDeviceInfo(id, CL_DEVICE_VERSION, valueSize, version, NULL);

		clGetDeviceInfo(id, CL_DRIVER_VERSION, 0, NULL, &valueSize);
		driverVersion = sp_mem_new_array(sp_char, valueSize);
		clGetDeviceInfo(id, CL_DRIVER_VERSION, valueSize, driverVersion, NULL);

		clGetDeviceInfo(id, CL_DEVICE_PROFILE, 0, NULL, &valueSize);
		profile = sp_mem_new_array(sp_char, valueSize);
		clGetDeviceInfo(id, CL_DEVICE_PROFILE, valueSize, profile, NULL);
		
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

		clGetDeviceInfo(id, CL_DEVICE_EXTENSIONS, 0, NULL, &valueSize);
		sp_char* extensionsAsArray = ALLOC_ARRAY(sp_char, valueSize);
		clGetDeviceInfo(id, CL_DEVICE_EXTENSIONS, valueSize, extensionsAsArray, NULL);

		sp_size indexes[1000];
		strCountChar(extensionsAsArray, ' ', extensionsLength, indexes);

		if (extensionsLength > ZERO_SIZE)
		{
			extensions = sp_mem_new_array(sp_char*, extensionsLength);

			extensions[0] = sp_mem_new_array(sp_char, indexes[0] + 1);
			std::memcpy(extensions[0], extensionsAsArray, indexes[0]);
			extensions[0][indexes[0]] = END_OF_STRING;

			for (sp_size j = 1; j < extensionsLength; j++)
			{
				sp_size size = indexes[j] - indexes[j - 1];

				extensions[j] = sp_mem_new_array(sp_char, size + 1);

 				std::memcpy(extensions[j], &extensionsAsArray[indexes[j - 1]], size);
				extensions[j][size] = END_OF_STRING;
			}
		}

		ALLOC_RELEASE(extensionsAsArray);
	}

	sp_size GpuDevice::getThreadLength(sp_size inputLength)
	{
		if (isOdd(inputLength))
			inputLength--;

		sp_size defaultGroupLength = getDefaultGroupLength();

		if (inputLength < defaultGroupLength)
			return inputLength;

		inputLength = divideBy2(inputLength);

		sp_size maxLength = 2;
		for (; maxLength < inputLength; maxLength *= 2)
		{
			sp_size divisor = nextDivisorOf(maxLength - 1, defaultGroupLength);
			if (divisor > maxWorkGroupSize)
				break;
		}
		while (inputLength >= maxLength)
			inputLength = divideBy2(inputLength);

		return inputLength;
	}

	sp_size GpuDevice::getGroupLength(sp_size threadLength, sp_size inputLength)
	{
		sp_size groupLength = getDefaultGroupLength();

		//while (groupLength > threadLength)
		//	groupLength = divideBy2(groupLength);
		if (groupLength > threadLength)
			return ONE_SIZE;

		groupLength = nextDivisorOf(threadLength, groupLength);

		if (threadLength == ONE_SIZE)
		{
			threadLength = (sp_size)std::ceil(nextPowOf2(inputLength) / TWO_DOUBLE);
			groupLength = (sp_size)std::ceil(threadLength / TWO_DOUBLE);
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
		if (name != nullptr)
		{
			sp_mem_release(name);
			name = nullptr;
		}

		if (version != nullptr)
		{
			sp_mem_release(version);
			version = nullptr;
		}

		if (driverVersion != nullptr)
		{
			sp_mem_release(driverVersion);
			driverVersion = nullptr;
		}

		if (profile != nullptr)
		{
			sp_mem_release(profile);
			profile = nullptr;
		}

		if (extensions != nullptr)
		{
			for (sp_size i = 0; i < extensionsLength; i++)
			{
				sp_mem_release(extensions[i]);
				extensions[i] = nullptr;
			}
			extensions = nullptr;
			extensionsLength = ZERO_SIZE;
		}

		if (commandManager != nullptr)
		{
			sp_mem_delete(commandManager, GpuCommandManager);
			commandManager = nullptr;
		}

		if (deviceContext != nullptr)
		{
			clReleaseContext(deviceContext);
			deviceContext = nullptr;
		}
	}
}

#endif