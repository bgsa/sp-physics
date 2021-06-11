#ifdef OPENCL_ENABLED

#ifndef GPU_CONTEXT_HEADER
#define GPU_CONTEXT_HEADER

#include "SpectrumPhysics.h"
#include "GpuDevice.h"
#include "SpArray.h"
#include "SpGpuPlatformManager.h"
#include "SpGpuPlatform.h"

namespace NAMESPACE_PHYSICS
{
	class GpuContext
	{
	private:
		SpGpuPlatform* platform;
		GpuDevice* _defaultDevice;

		GpuContext(SpGpuPlatform* platform);

	public:
		
		API_INTERFACE static void init();
		API_INTERFACE static void init(SpGpuPlatform* platform);
		API_INTERFACE static void release();

		API_INTERFACE GpuDevice* defaultDevice() const;

	};

	extern GpuContext* GpuContextInstance;

}

#endif // GPU_CONTEXT_HEADER

#endif // OPENCL_ENABLED