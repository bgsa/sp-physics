#ifdef OPENCL_ENABLED

#ifndef GPU_COMPOSED_COMMAND
#define GPU_COMPOSED_COMMAND

#include "SpectrumPhysics.h"
#include "GpuDevice.h"

namespace NAMESPACE_PHYSICS
{
	class GpuComposedCommand : 
		public Object
	{
	protected:
		GpuDevice* gpu = nullptr;

	public:

		/// <summary>
		/// Init the command
		/// </summary>
		API_INTERFACE virtual GpuComposedCommand* init(GpuDevice* gpu, const sp_char* buildOptions);

		/// <summary>
		/// Execute the command
		/// </summary>
		API_INTERFACE virtual cl_mem execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt) = 0;

	};

}

#endif // GPU_COMPOSED_COMMAND

#endif // OPENCL_ENABLED