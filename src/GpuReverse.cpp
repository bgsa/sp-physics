#ifdef OPENCL_ENABLED

#include "GpuReverse.h"

namespace NAMESPACE_PHYSICS
{
	
	GpuReverse* GpuReverse::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (program != NULL)
			return this;

		this->gpu = gpu;

		SP_FILE file;
		file.open("Reverse.cl", std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);

		sp_uint programIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions);

		ALLOC_RELEASE(source);

		program = gpu->commandManager->cachedPrograms[programIndex];

		return this;
	}

	GpuReverse* GpuReverse::setParameters(sp_uint* input, sp_uint inputLength)
	{
		const sp_uint inputSize = inputLength * SIZEOF_UINT;
		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);
		outputGpu = gpu->createBuffer(inputSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		inputLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_ALLOC_HOST_PTR);

		updateInputLength(inputLength);

		releaseBuffers = true;

		return this->setParameters(inputGpu, outputGpu, inputLengthGpu);
	}

	GpuReverse* GpuReverse::updateInputLength(sp_uint inputLength)
	{
		this->inputLength = inputLength;

		globalWorkSize[0] = inputLength;
		localWorkSize[0] = gpu->getDefaultGroupLength();

		if (globalWorkSize[0] < localWorkSize[0])
			localWorkSize[0] = ONE_UINT;

		return this;
	}

	GpuReverse* GpuReverse::setParameters(cl_mem inputGpu, cl_mem outputGpu, cl_mem inputLengthGpu)
	{
		this->inputGpu = inputGpu;
		this->outputGpu = outputGpu;
		this->inputLengthGpu = inputLengthGpu;

		const sp_uint inputSize = inputLength * SIZEOF_UINT;

		commandReverse = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(outputGpu, inputSize)
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
			->buildFromProgram(program, "reverseOrCopy");

		return this;
	}

	GpuReverse* GpuReverse::updateInputLength(cl_mem inputLengthGpu)
	{
		this->inputLengthGpu = inputLengthGpu;

		commandReverse
			->updateInputParameter(TWO_UINT, inputLengthGpu);

		return this;
	}

	void GpuReverse::swapIndexes()
	{
		commandReverse
			->swapInputParameter(ZERO_UINT, ONE_UINT);

		indexesSwapped = !indexesSwapped;
	}

	cl_mem GpuReverse::execute()
	{
		commandReverse->execute(ONE_UINT, globalWorkSize, localWorkSize);

		if (indexesSwapped)
			return inputGpu;
		else
			return outputGpu;
	}

	GpuReverse::~GpuReverse()
	{
		if (releaseBuffers)
		{
			gpu->releaseBuffer(inputGpu);
			inputGpu = NULL;

			gpu->releaseBuffer(inputLengthGpu);
			inputLengthGpu = NULL;
		}

		if (commandReverse != NULL)
		{
			commandReverse->~GpuCommand();
			commandReverse = NULL;
		}
	}

}

#endif // OPENCL_ENABLED