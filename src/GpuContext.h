#ifdef OPENCL_ENABLED

#ifndef GPU_CONTEXT_HEADER
#define GPU_CONTEXT_HEADER

#ifdef APPLE
	#include <OpenCL/opencl.h>
#else
	#include <CL/cl.h>
#endif

#include "SpectrumPhysics.h"
#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{
	class GpuContext
	{
	private:
		cl_platform_id platformId;
		
		GpuContext(cl_platform_id platformId);

	public:
		std::vector<GpuDevice*> devices;
		GpuDevice* defaultDevice = nullptr;
		
		API_INTERFACE static GpuContext* init();
		API_INTERFACE static GpuContext* init(cl_platform_id platformId);

		API_INTERFACE static std::vector<cl_platform_id> getPlatforms();

		API_INTERFACE static cl_platform_id getDefaultPlatforms();
		
		~GpuContext();
	};

}

#endif // GPU_CONTEXT_HEADER

#endif // OPENCL_ENABLED