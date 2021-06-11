#include "SpGpuPlatform.h"
#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{

	void SpGpuPlatform::dispose()
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

		if (vendor != nullptr)
		{
			sp_mem_release(vendor);
			vendor = nullptr;
		}

		if (profile != nullptr)
		{
			sp_mem_release(profile);
			profile = nullptr;
		}

		if (extensions != nullptr)
		{
			for (sp_uint i = 0; i < extensionsLength; i++)
			{
				sp_mem_release(extensions[i]);
				extensions[i] = nullptr;
			}
			sp_mem_release(extensions);
			extensions = nullptr;
		}

		if (gpuDevicesLength > ZERO_UINT)
		{
			for (sp_uint i = 0; i < gpuDevicesLength; i++)
			{
				sp_mem_delete(gpuDevices[i], GpuDevice);
				gpuDevices[i] = nullptr;
			}
			gpuDevicesLength = ZERO_UINT;
		}

	}

}