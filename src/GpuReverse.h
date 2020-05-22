#if OPENCL_ENABLED

#ifndef GPU_REVERSE_HEADER
#define GPU_REVERSE_HEADER

#include "SpectrumPhysics.h"
#include "GpuDevice.h"
#include "GpuComposedCommand.h"
#include "FileSystem.h"

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

		API_INTERFACE const sp_char* toString() override
		{
			return "GPU Reverse Command";
		}

		API_INTERFACE void dispose() override;

		API_INTERFACE ~GpuReverse() { dispose(); }

	};
}

#endif // GPU_REVERSE_HEADER

#endif // OPENCL_ENABLED