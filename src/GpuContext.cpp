#ifdef OPENCL_ENABLED

#include "GpuContext.h"

namespace NAMESPACE_PHYSICS
{
	GpuContext* GpuContextInstance = nullptr;

	GpuContext::GpuContext(SpGpuPlatform* platform)
	{
		this->platform = platform;

		cl_uint devicesCount;
		clGetDeviceIDs((cl_platform_id)platform->id, CL_DEVICE_TYPE_ALL, 0, NULL, &devicesCount);

		_devices = sp_mem_new(SpArray<GpuDevice*>)(devicesCount);

		cl_device_id* devicesAsArray = ALLOC_ARRAY(cl_device_id, devicesCount);
		clGetDeviceIDs((cl_platform_id)platform->id, CL_DEVICE_TYPE_ALL, devicesCount, devicesAsArray, NULL);

		for (sp_uint i = 0; i < devicesCount; i++) 
		{
			GpuDevice* device = sp_mem_new(GpuDevice)(devicesAsArray[i], (cl_platform_id)platform->id);

			if (device->type & CL_DEVICE_TYPE_DEFAULT)
				_defaultDevice = device;

			_devices->add(device);
		}

		if (_defaultDevice == nullptr && devicesCount > 0)
			_defaultDevice = _devices->data()[0];

		ALLOC_RELEASE(devicesAsArray);
	}

	void GpuContext::init(SpGpuPlatform* platform)
	{
		if (GpuContextInstance == nullptr)
		{
			GpuContextInstance = sp_mem_new(GpuContext)(platform);
		}
	}

	void GpuContext::init()
	{
		SpGpuPlatformManager::init();
		init(SpGpuPlatformManagerInstance->defaultPlatform());
	}

	SpArray<GpuDevice*>* GpuContext::devices() const
	{
		return _devices;
	}

	GpuDevice* GpuContext::defaultDevice() const
	{
		return _defaultDevice;
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
