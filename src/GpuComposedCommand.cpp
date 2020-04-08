#ifdef OPENCL_ENABLED

#include "GpuComposedCommand.h"

namespace NAMESPACE_PHYSICS
{

	GpuComposedCommand* GpuComposedCommand::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;

		return this;
	}

}

#endif // OPENCL_ENABLED