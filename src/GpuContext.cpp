#ifdef OPENCL_ENABLED

#include "GpuContext.h"

namespace NAMESPACE_PHYSICS
{
	GpuContext* GpuContextInstance = nullptr;

	GpuContext::GpuContext(SpGpuPlatform* platform)
	{
		this->platform = platform;

		for (sp_uint i = 0; i < platform->gpuDevicesLength; i++) 
		{
			if (platform->gpuDevices[i]->type & CL_DEVICE_TYPE_DEFAULT)
				_defaultDevice = platform->gpuDevices[i];
		}

		if (platform->gpuDevicesLength > 0)
			_defaultDevice = platform->gpuDevices[0];
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

	void GpuContext::release()
	{
		if (GpuContextInstance != nullptr)
		{
			sp_mem_delete(GpuContextInstance, GpuContext);
			GpuContextInstance = nullptr;
		}
	}

	GpuDevice* GpuContext::defaultDevice() const
	{
		return _defaultDevice;
	}

}

#endif
