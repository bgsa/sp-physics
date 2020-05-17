#ifdef OPENCL_ENABLED

#ifndef GPU_COMPOSED_COMMAND
#define GPU_COMPOSED_COMMAND

#include "SpectrumPhysics.h"
#include "GpuDevice.h"
#include "GpuCommand.h"

namespace NAMESPACE_PHYSICS
{
	class GpuComposedCommand
	{

	private:
		
	protected:
		GpuDevice* gpu = NULL;

	public:

		cl_event lastEvent = NULL;

		/// <summary>
		/// Init the command
		/// </summary>
		API_INTERFACE virtual GpuComposedCommand* init(GpuDevice* gpu, const sp_char* buildOptions);

		/// <summary>
		/// Execute the command
		/// </summary>
		API_INTERFACE virtual cl_mem execute() = 0;

	};

}

#endif // GPU_COMPOSED_COMMAND

#endif // OPENCL_ENABLED