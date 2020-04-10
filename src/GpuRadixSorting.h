#if OPENCL_ENABLED

#ifndef GPU_RADIX_DORTING
#define GPU_RADIX_DORTING

#include "GpuFindMinMax.h"
#include "GpuComposedCommand.h"

namespace NAMESPACE_PHYSICS
{

	class GpuRadixSorting
		: public GpuComposedCommand
	{
	private:
		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };
		sp_uint maxDigits = MAX_DIGITS_MANTISSA - 1;
		sp_uint threadsLength;
		sp_uint defaultLocalWorkSize;

		GpuIndexes* commandCreateIndexes;
		GpuCommand* commandCount;
		GpuCommand* commandCountSwapped;
		GpuCommand* commandPrefixScanUp;
		GpuCommand* commandPrefixScanDown;
		GpuCommand* commandReorder;
		GpuCommand* commandReorderSwapped;

		cl_program program;

		cl_mem offsetTable;
		cl_mem indexesGpu;
		cl_mem outputIndexes;

	public:
		cl_mem inputGpu;
		cl_mem indexesLengthGpu;
		cl_mem offsetGpu;
		cl_mem output = NULL;

		API_INTERFACE GpuRadixSorting* init(GpuDevice* gpu, const sp_char* buildOptions) override;

		API_INTERFACE GpuRadixSorting* setParameters(sp_float* input, sp_uint indexesLengthCpu, sp_uint striderCpu, sp_uint offsetCpu);

		API_INTERFACE cl_mem execute() override;

		API_INTERFACE ~GpuRadixSorting();

	};
}

#endif // GPU_RADIX_DORTING

#endif // OPENCL_ENABLED

