#if OPENCL_ENABLED

#ifndef GPU_REVERSE
#define GPU_REVERSE

#include "GpuDevice.h"
#include "GpuComposedCommand.h"
#include "IFileManager.h"
#include "Factory.h"

namespace NAMESPACE_PHYSICS
{

	class GpuReverse
		: public GpuComposedCommand
	{
	private:
		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };

		GpuCommand* commandReverse = NULL;
		cl_program program = NULL;
		cl_mem inputGpu = NULL;
		cl_mem outputGpu = NULL;
		cl_mem inputLengthGpu = NULL;

		sp_uint inputLength;
		sp_bool indexesSwapped = false;

		sp_bool releaseBuffers = false;

	public:

		API_INTERFACE GpuReverse* init(GpuDevice* gpu, const sp_char* buildOptions) override;

		API_INTERFACE GpuReverse* setParameters(sp_uint* input, sp_uint inputLength);
		API_INTERFACE GpuReverse* setParameters(cl_mem inputGpu, cl_mem outputGpu, cl_mem inputLengthGpu);

		API_INTERFACE GpuReverse* updateInputLength(sp_uint inputLength);
		API_INTERFACE GpuReverse* updateInputLength(cl_mem inputLengthGpu);

		API_INTERFACE void swapIndexes();

		API_INTERFACE cl_mem execute() override;

		API_INTERFACE ~GpuReverse();

	};
}

#endif // GPU_REVERSE

#endif // OPENCL_ENABLED

