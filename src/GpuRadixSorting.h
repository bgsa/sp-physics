#if OPENCL_ENABLED

#ifndef GPU_RADIX_DORTING
#define GPU_RADIX_DORTING

#include "SpectrumPhysics.h"
#include "GpuIndexes.h"
#include "GpuReverse.h"
#include "GpuComposedCommand.h"
#include <FileSystem.h>

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
		sp_uint maxIteration;

		cl_program program;

		GpuIndexes* commandCreateIndexes;
		GpuCommand* commandCount;
		GpuCommand* commandCountSwapped;
		GpuCommand* commandPrefixScan;
		GpuCommand* commandPrefixScanSwaped;
		GpuCommand* commandReorder;
		GpuCommand* commandReorderSwapped;

		GpuCommand* commandCountNegative;
		GpuCommand* commandPrefixScanNegative;
		GpuCommand* commandReorderNegative;
		GpuReverse* commandReverse;

		cl_mem offsetTable1;
		cl_mem offsetTable1Negatives;
		cl_mem offsetTable2;
		cl_mem offsetTable2Negatives;
		cl_mem offsetPrefixScanGpu;

		cl_mem negativeCounterLocation1;
		cl_mem negativeCounterLocation2;

		cl_mem inputIndexesGpu;
		cl_mem outputIndexesGpu;

	public:
		cl_mem inputGpu;
		cl_mem indexesLengthGpu;
		cl_mem output = NULL;

		API_INTERFACE GpuRadixSorting* init(GpuDevice* gpu, const sp_char* buildOptions) override;

		API_INTERFACE GpuRadixSorting* setParameters(sp_float* input, sp_uint indexesLengthCpu, sp_uint striderCpu, sp_uint offsetCpu);

		API_INTERFACE cl_mem execute() override;

		API_INTERFACE cl_mem executeNegatives(sp_bool indexChanged, sp_bool offsetChanged);

		API_INTERFACE void dispose() override;

		API_INTERFACE const sp_char* toString() override
		{
			return "GPU Radix Sorting";
		}

		API_INTERFACE ~GpuRadixSorting() { dispose(); }

	};
}

#endif // GPU_RADIX_DORTING

#endif // OPENCL_ENABLED

