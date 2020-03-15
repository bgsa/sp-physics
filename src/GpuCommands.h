#ifdef OPENCL_ENABLED

#ifndef GPU_COMMANDS_HEADER
#define GPU_COMMANDS_HEADER

#include "OpenML.h"
#include <algorithm>
#include "GpuContext.h"
#include "GpuCommand.h"
#include "IFileManager.h"
#include "Factory.h"
#include "GpuContext.h"

namespace NAMESPACE_PHYSICS
{

	class GpuCommands
	{
	public:

		/// <summary>
		/// Init algorithms for GPU
		/// </summary>
		API_INTERFACE static void init(GpuDevice* gpu, const sp_char* buildOptions);

		/// <summary>
		/// Create Indexes buffer ( 0, 1, 2, ... , length ) on GPU
		/// </summary>
		API_INTERFACE static cl_mem creteIndexes(GpuDevice * gpu, sp_size length);

	};
}


#endif // ! GPU_COMMANDS_HEADER

#endif // ! OPENCL_ENABLED