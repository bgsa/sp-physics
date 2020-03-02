#ifdef OPENCL_ENABLED

#include "GpuContext.h"

static GpuContext* gpu = nullptr;
static std::vector<cl_platform_id> platforms;

GpuContext::GpuContext(cl_platform_id platformId)
{
	this->platformId = platformId;

	cl_uint devicesCount;
	clGetDeviceIDs(platformId, CL_DEVICE_TYPE_ALL, 0, NULL, &devicesCount);

	cl_device_id* devicesAsArray = new cl_device_id[devicesCount];
	clGetDeviceIDs(platformId, CL_DEVICE_TYPE_ALL, devicesCount, devicesAsArray, NULL);		

	for (size_t i = 0; i < devicesCount; i++) 
	{
		GpuDevice* device = new GpuDevice(devicesAsArray[i]);
		devices.emplace_back(device);

		if (device->type & CL_DEVICE_TYPE_DEFAULT)
			defaultDevice = device;
	}

	if (defaultDevice == nullptr && devicesCount > 0)
		defaultDevice = devices[0];
}

GpuContext* GpuContext::init(cl_platform_id platformId)
{
	if (gpu == nullptr)
		gpu = new GpuContext(platformId);

	return gpu;
}

GpuContext* GpuContext::init()
{
	return init(getDefaultPlatforms());
}

std::vector<cl_platform_id> GpuContext::getPlatforms()
{
	if (platforms.size() > 0)
		return platforms;
	
	cl_uint platformCount;
	HANDLE_OPENCL_ERROR(clGetPlatformIDs(NULL, NULL, &platformCount));

	cl_platform_id* platformsAsArray = new cl_platform_id[platformCount];
	platforms.resize(platformCount);

	HANDLE_OPENCL_ERROR(clGetPlatformIDs(platformCount, platformsAsArray, NULL));

	platforms.assign(platformsAsArray, platformsAsArray + platformCount);

	delete[] platformsAsArray;
	return platforms;
}

cl_platform_id GpuContext::getDefaultPlatforms()
{
	return getPlatforms()[0];
}

GpuContext::~GpuContext()
{
	for (size_t i = 0; i < devices.size(); i++)
		delete devices[i];
}

#endif