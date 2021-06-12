#include "SpGpuPlatformManager.h"
#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{
	SpGpuPlatformManager* SpGpuPlatformManagerInstance = nullptr;

	void SpGpuPlatformManager::loadDevices(SpGpuPlatform& platform)
	{
		clGetDeviceIDs((cl_platform_id)platform.id, CL_DEVICE_TYPE_ALL, 0, NULL, &platform.gpuDevicesLength);

		if (platform.gpuDevicesLength > 0)
		{
			platform.gpuDevices = sp_mem_new_array(GpuDevice*, platform.gpuDevicesLength);

			cl_device_id* devicesAsArray = ALLOC_ARRAY(cl_device_id, platform.gpuDevicesLength);
			clGetDeviceIDs((cl_platform_id)platform.id, CL_DEVICE_TYPE_ALL, platform.gpuDevicesLength, devicesAsArray, NULL);

			for (sp_uint i = 0; i < platform.gpuDevicesLength; i++)
			{
				platform.gpuDevices[i] = sp_mem_new(GpuDevice)();
				platform.gpuDevices[i]->init(devicesAsArray[i], platform);
			}

			ALLOC_RELEASE(devicesAsArray);
		}
	}

	void SpGpuPlatformManager::init()
	{
		if (SpGpuPlatformManagerInstance == nullptr)
		{
			SpGpuPlatformManagerInstance = sp_mem_new(SpGpuPlatformManager)();

			cl_uint platformCount;
			HANDLE_OPENCL_ERROR(clGetPlatformIDs(ZERO_UINT, NULL, &platformCount));

			cl_platform_id* platformsTemp = ALLOC_ARRAY(cl_platform_id, platformCount);

			HANDLE_OPENCL_ERROR(clGetPlatformIDs(platformCount, platformsTemp, NULL));

			SpGpuPlatformManagerInstance->platformsLength = platformCount;
			SpGpuPlatformManagerInstance->platforms = sp_mem_new_array(SpGpuPlatform, platformCount)();

			for (sp_uint i = 0; i < platformCount; i++)
			{
				cl_platform_id id = platformsTemp[i];

				SpGpuPlatformManagerInstance->platforms[i].id = id;
				
				sp_size parameterSize;
				clGetPlatformInfo(id, CL_PLATFORM_NAME, NULL, NULL, &parameterSize);
				SpGpuPlatformManagerInstance->platforms[i].name = sp_mem_new_array(sp_char, parameterSize);
				clGetPlatformInfo(id, CL_PLATFORM_NAME, parameterSize, SpGpuPlatformManagerInstance->platforms[i].name, NULL);
				
				clGetPlatformInfo(id, CL_PLATFORM_VERSION, NULL, NULL, &parameterSize);
				SpGpuPlatformManagerInstance->platforms[i].version = sp_mem_new_array(sp_char, parameterSize);
				clGetPlatformInfo(id, CL_PLATFORM_VERSION, parameterSize, SpGpuPlatformManagerInstance->platforms[i].version, NULL);

				clGetPlatformInfo(id, CL_PLATFORM_VENDOR, NULL, NULL, &parameterSize);
				SpGpuPlatformManagerInstance->platforms[i].vendor = sp_mem_new_array(sp_char, parameterSize);
				clGetPlatformInfo(id, CL_PLATFORM_VENDOR, parameterSize, SpGpuPlatformManagerInstance->platforms[i].vendor, NULL);

				clGetPlatformInfo(id, CL_PLATFORM_PROFILE, NULL, NULL, &parameterSize);
				SpGpuPlatformManagerInstance->platforms[i].profile = sp_mem_new_array(sp_char, parameterSize);
				clGetPlatformInfo(id, CL_PLATFORM_PROFILE, parameterSize, SpGpuPlatformManagerInstance->platforms[i].profile, NULL);

				clGetPlatformInfo(id, CL_PLATFORM_EXTENSIONS, NULL, NULL, &parameterSize);
				sp_char* extensions = ALLOC_NEW_ARRAY(sp_char, parameterSize);
				clGetPlatformInfo(id, CL_PLATFORM_EXTENSIONS, parameterSize, extensions, NULL);

				sp_size indexes[1000];
				strCountChar(extensions, ' ', parameterSize, indexes);

				if (parameterSize > ZERO_SIZE)
				{
					SpGpuPlatformManagerInstance->platforms[i].extensionsLength = (sp_uint)parameterSize;
					SpGpuPlatformManagerInstance->platforms[i].extensions = sp_mem_new_array(sp_char*, parameterSize + 1);

					SpGpuPlatformManagerInstance->platforms[i].extensions[0] = sp_mem_new_array(sp_char, indexes[0] + 1);
					std::memcpy(SpGpuPlatformManagerInstance->platforms[i].extensions[0], extensions, indexes[0]);
					SpGpuPlatformManagerInstance->platforms[i].extensions[0][indexes[0]] = END_OF_STRING;

					for (sp_size j = 1; j < parameterSize; j++)
					{
						sp_size size = indexes[j] - indexes[j - 1];

						SpGpuPlatformManagerInstance->platforms[i].extensions[j] = sp_mem_new_array(sp_char, size + 1);

						std::memcpy(SpGpuPlatformManagerInstance->platforms[i].extensions[j], &extensions[indexes[j - 1]], size);
						SpGpuPlatformManagerInstance->platforms[i].extensions[j][size] = END_OF_STRING;
					}
				}

				loadDevices(SpGpuPlatformManagerInstance->platforms[i]);
			}

			ALLOC_RELEASE(platformsTemp);
		}
	}

	void SpGpuPlatformManager::release()
	{
		if (SpGpuPlatformManagerInstance != nullptr)
		{
			sp_mem_delete(SpGpuPlatformManagerInstance, SpGpuPlatformManager);
			SpGpuPlatformManagerInstance = nullptr;
		}
	}
	
}