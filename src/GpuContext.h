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
		
		GpuContext(SpGpuPlatform* platform);

		SpArray<GpuDevice*>* _devices;
		GpuDevice* _defaultDevice = nullptr;

	public:
		
		API_INTERFACE static void init();
		API_INTERFACE static void init(SpGpuPlatform* platform);

		API_INTERFACE SpArray<GpuDevice*>* devices() const;
		API_INTERFACE GpuDevice* defaultDevice() const;
		
		~GpuContext();
	};

	extern GpuContext* GpuContextInstance;

}

#endif // GPU_CONTEXT_HEADER

#endif // OPENCL_ENABLED