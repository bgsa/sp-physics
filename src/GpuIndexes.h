#ifdef OPENCL_ENABLED

#ifndef GPU_INDEXES_HEADER
#define GPU_INDEXES_HEADER

#include "OpenML.h"
#include <algorithm>
#include "GpuDevice.h"
#include "GpuCommand.h"
#include "IFileManager.h"
#include "Factory.h"

namespace NAMESPACE_PHYSICS
{

	class GpuIndexes
	{
	private:
		GpuDevice* gpu;
		cl_program program;

		GpuCommand* commandInitIndexes = NULL;
		GpuCommand* commandInitIndexes_Odd = NULL;

		cl_mem output = NULL;
		sp_bool isInputOdd;

		sp_uint createIndexesProgramIndex = UINT_MAX;

		sp_size createIndexesGlobalWorkSize[3] = { 0, 0, 0 };
		sp_size createIndexesLocalWorkSize[3] = { 0, 0, 0 };

	public:

		/// <summary>
		/// Init algorithms for GPU
		/// </summary>
		API_INTERFACE void init(GpuDevice* gpu, const sp_char* buildOptions);

		/// <summary>
		/// Set parameters for create indexes command
		/// </summary>
		API_INTERFACE void setParametersCreateIndexes(sp_uint length);
		
		/// <summary>
		/// Create Indexes buffer ( 0, 1, 2, ... , length ) on GPU
		/// </summary>
		API_INTERFACE cl_mem execute();

		API_INTERFACE ~GpuIndexes();

	};
}


#endif // GPU_INDEXES_HEADER

#endif // OPENCL_ENABLED