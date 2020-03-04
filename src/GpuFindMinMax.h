#ifdef OPENCL_ENABLED

#ifndef GPU_FIND_MIN_MAX_HEADER
#define GPU_FIND_MIN_MAX_HEADER

#include "GpuCommands.h"


namespace OpenML
{

	class GpuFindMinMax
	{
	private:
		sp_uint findMinMaxProgramIndex;
		GpuDevice* gpu = NULL;
		sp_size globalWorkSize[3];
		sp_size localWorkSize[3];
		sp_uint threadsCount;
		sp_uint groupStride;

		GpuCommand* commandFindMinMaxByThread;
		GpuCommand* commandFindMinMaxParallelReduce;

		cl_mem inputGpu;
		cl_mem indexesGpu;
		cl_mem inputLengthGpu;
		cl_mem offsetGpu;
		cl_mem prefixScanOffsetGpu;

	public:

		cl_mem output;

		API_INTERFACE GpuFindMinMax* init(GpuDevice* gpu, const sp_char* buildOptions);

		API_INTERFACE GpuFindMinMax* setParameters(sp_float* input, sp_uint indexesLength, sp_uint stride, sp_uint offset = 0);

		API_INTERFACE cl_mem execute();

		API_INTERFACE ~GpuFindMinMax();
	};

}


#endif // !GPU_FIND_MIN_MAX_HEADER

#endif // OPENCL_ENABLED