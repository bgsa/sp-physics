#ifdef OPENCL_ENABLED

#include "GpuContext.h"

namespace NAMESPACE_PHYSICS
{
	static SpArray<cl_platform_id>* _platforms;
	static GpuContext* _gpuContext;

	GpuContext::GpuContext(cl_platform_id platformId)
	{
		this->platformId = platformId;

		cl_uint devicesCount;
		clGetDeviceIDs(platformId, CL_DEVICE_TYPE_ALL, 0, NULL, &devicesCount);

		_devices = sp_mem_new(SpArray<GpuDevice*>)(devicesCount);

		cl_device_id* devicesAsArray = ALLOC_ARRAY(cl_device_id, devicesCount);
		clGetDeviceIDs(platformId, CL_DEVICE_TYPE_ALL, devicesCount, devicesAsArray, NULL);		

		for (sp_uint i = 0; i < devicesCount; i++) 
		{
			GpuDevice* device = sp_mem_new(GpuDevice)(devicesAsArray[i], platformId);

			if (device->type & CL_DEVICE_TYPE_DEFAULT)
				_defaultDevice = device;

			_devices->add(device);
		}

		if (_defaultDevice == nullptr && devicesCount > 0)
			_defaultDevice = _devices->data()[0];

		ALLOC_RELEASE(devicesAsArray);
	}

	GpuContext* GpuContext::init(cl_platform_id platformId)
	{
		_gpuContext = sp_mem_new(GpuContext)(platformId);
		return _gpuContext;
	}

	GpuContext* GpuContext::instance()
	{
		return _gpuContext;
	}

	GpuContext* GpuContext::init()
	{
		return init(defaultPlatforms());
	}

	SpArray<GpuDevice*>* GpuContext::devices() const
	{
		return _devices;
	}

	GpuDevice* GpuContext::defaultDevice() const
	{
		return _defaultDevice;
	}

	SpArray<cl_platform_id>* GpuContext::platforms()
	{
		if (_platforms != nullptr)
			return _platforms;
		
		cl_uint platformCount;
		HANDLE_OPENCL_ERROR(clGetPlatformIDs(ZERO_UINT, NULL, &platformCount));

		_platforms = sp_mem_new(SpArray< cl_platform_id>)(platformCount, platformCount);

		HANDLE_OPENCL_ERROR(clGetPlatformIDs(platformCount, _platforms->data(), NULL));
		
		return _platforms;
	}

	cl_platform_id GpuContext::defaultPlatforms()
	{
		return platforms()->get(1);
	}

	GpuContext::~GpuContext()
	{
		for (sp_uint i = ZERO_UINT; i < _devices->length(); i++)
		{
			sp_mem_delete(_devices->data()[i], GpuDevice);
		}
	}
}

#endif
