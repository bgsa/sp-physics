#if OPENCL_ENABLED

#ifndef GPU_RADIX_DORTING
#define GPU_RADIX_DORTING

#include "GpuFindMinMax.h"

namespace NAMESPACE_PHYSICS
{

	class GpuRadixSorting
	{
	private:
		sp_uint radixSortProgramIndex;
		GpuDevice* gpu = NULL;
		sp_float minMaxValues[2];
		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };
		sp_uint maxDigits = MAX_DIGITS_MANTISSA - 1;
		sp_uint threadsLength;
		sp_uint offsetPrefixScanCpu = 10u;

		GpuCommand* commandCount;
		GpuCommand* commandPrefixScan;
		GpuCommand* commandPrefixScanSwaped;
		//GpuCommand* commandUpdatePrefixScan;
		GpuCommand* commandReorder;
		GpuFindMinMax* findMinMax;
		GpuIndexes* commandCreateIndexes;

		cl_program program;

		cl_mem offsetTable1;
		cl_mem offsetTable2;
		cl_mem offsetTableResult;
		cl_mem offsetPrefixScanGpu;
		cl_mem digitIndexGpu;
		cl_mem useExpoentGpu;

		cl_mem outputIndexes;

		const sp_size globalWorkSizeOneThread[3] = { 1, 0, 0 };
		const sp_size localWorkSizeOneThread[3] = { 1, 0, 0 };

	public:
		cl_mem inputGpu;
		cl_mem indexesGpu;
		cl_mem indexesLengthGpu;
		cl_mem offsetGpu;

		API_INTERFACE GpuRadixSorting* init(GpuDevice* gpu, const sp_char* buildOptions);

		API_INTERFACE GpuRadixSorting* setParameters(sp_float* input, sp_uint indexesLengthCpu, sp_uint striderCpu, sp_uint offsetCpu);

		API_INTERFACE cl_mem execute();

		API_INTERFACE ~GpuRadixSorting();

	};
}

#endif // GPU_RADIX_DORTING

#endif // OPENCL_ENABLED

