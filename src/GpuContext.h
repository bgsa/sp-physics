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
#include "SpArray.h"

namespace NAMESPACE_PHYSICS
{
	class GpuContext
	{
	private:
		cl_platform_id platformId;
		
		GpuContext(cl_platform_id platformId);

		SpArray<GpuDevice*>* _devices;
		GpuDevice* _defaultDevice = nullptr;

	public:
		
		API_INTERFACE static GpuContext* init();
		API_INTERFACE static GpuContext* init(cl_platform_id platformId);

		API_INTERFACE static GpuContext* instance();

		API_INTERFACE SpArray<GpuDevice*>* devices() const;
		API_INTERFACE GpuDevice* defaultDevice() const;

		API_INTERFACE static SpArray<cl_platform_id>* platforms();

		API_INTERFACE static cl_platform_id defaultPlatforms();
		
		~GpuContext();
	};

}

#endif // GPU_CONTEXT_HEADER

#endif // OPENCL_ENABLED