#ifdef OPENCL_ENABLED

#ifndef GPU_INSERTION_SORTING_HEADER
#define GPU_INSERTION_SORTING_HEADER

#include "GpuCommands.h"
#include <algorithm>

namespace NAMESPACE_PHYSICS
{

	class GpuInsertionSorting
	{
	private:
		GpuDevice* gpu = NULL;
		GpuCommand* sortingCommand = NULL;
		cl_uint insertionSortProgramIndex;

		cl_mem inputGpu = NULL;
		cl_mem indexesGpu = NULL;
		cl_mem inputLengthGpu = NULL;
		cl_mem outputGpu = NULL;

		sp_size globalWorkSize[3];
		sp_size localWorkSize[3];

	public:

		API_INTERFACE GpuInsertionSorting* init(GpuDevice* gpu, const sp_char* buildOptions);

		API_INTERFACE GpuInsertionSorting* setParameters(sp_float* input, sp_uint indexesLength, sp_uint striderCpu, sp_uint offsetCpu);

		API_INTERFACE cl_mem execute();

		API_INTERFACE ~GpuInsertionSorting();

	};

}

#endif // GPU_INSERTION_SORTING_HEADER

#endif // OPENCL_ENABLED