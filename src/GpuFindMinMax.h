#ifdef OPENCL_ENABLED

#ifndef GPU_FIND_MIN_MAX_HEADER
#define GPU_FIND_MIN_MAX_HEADER

#include "GpuDevice.h"
#include "GpuIndexes.h"
#include "GpuCommand.h"

namespace NAMESPACE_PHYSICS
{

	class GpuFindMinMax
	{
	private:
		sp_uint findMinMaxProgramIndex;
		GpuDevice* gpu = NULL;
		sp_size globalWorkSize[3] = {0, 0, 0};
		sp_size localWorkSize[3] = { 0, 0, 0 };
		sp_uint threadLength;
		sp_uint indexLength;

		GpuIndexes* commandIndexes;
		GpuCommand* commandFindMinMaxByThread;
		GpuCommand* commandFindMinMaxParallelReduce;
		GpuCommand* commandFindMinMaxParallelReduce_Odd = NULL;
		GpuCommand* commandFindMinMaxParallelReduce_OneThread = NULL;

		cl_mem inputGpu;
		cl_mem indexesGpu;
		cl_mem inputLengthGpu;
		cl_mem offsetGpu;

	public:

		cl_mem output;

		API_INTERFACE GpuFindMinMax* init(GpuDevice* gpu, const sp_char* buildOptions);

		API_INTERFACE GpuFindMinMax* setParameters(sp_float* input, sp_uint indexesLength, sp_uint stride, sp_uint offset = 0, cl_mem indexes = NULL);

		API_INTERFACE cl_mem execute();
		
		API_INTERFACE ~GpuFindMinMax();
	};

}


#endif // !GPU_FIND_MIN_MAX_HEADER

#endif // OPENCL_ENABLED