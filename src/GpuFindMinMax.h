#ifdef OPENCL_ENABLED

#ifndef GPU_FIND_MIN_MAX_HEADER
#define GPU_FIND_MIN_MAX_HEADER

#include "SpectrumPhysics.h"
#include "GpuDevice.h"
#include "GpuIndexes.h"
#include "GpuCommand.h"
#include "FileSystem.h"

namespace NAMESPACE_PHYSICS
{

	class GpuFindMinMax
	{
	private:
		sp_uint findMinMaxProgramIndex;
		GpuDevice* gpu = NULL;
		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };
		sp_uint indexLength;

		sp_size globalWorkSizeOdd[3] = { 1, 0, 0 };
		sp_size localWorkSizeOdd[3] = { 1, 0, 0 };

		sp_bool isIndexLengthOdd = false;

		GpuIndexes* commandIndexes;
		GpuCommand* commandFindMinMaxByThread;
		GpuCommand* commandFindMinMaxParallelReduce;
		GpuCommand* commandFindMinMaxParallelReduce_Odd = NULL;
		GpuCommand* commandFindMinMaxParallelReduce_OneThread = NULL;
		GpuCommand* commandUpdateOffset = NULL;

		cl_mem inputGpu;
		cl_mem indexesGpu;
		cl_mem inputLengthGpu;

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