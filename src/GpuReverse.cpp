#ifdef OPENCL_ENABLED

#include "GpuReverse.h"

namespace NAMESPACE_PHYSICS
{
	
	GpuReverse* GpuReverse::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (program != NULL)
			return this;

		this->gpu = gpu;

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("Reverse.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		sp_mem_delete(filename, SpDirectory);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		gpu->commandManager->buildProgram(source, SIZEOF_CHAR * fileSize, buildOptions, &program);

		ALLOC_RELEASE(source);
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

	void GpuReverse::reset()
	{
		commandReverse
			->updateInputParameter(ZERO_UINT, inputGpu)
			->updateInputParameter(ONE_UINT, outputGpu);
	}

	void GpuReverse::updateInput(cl_mem newInput, cl_mem newOutput)
	{
		this->inputGpu = newInput;
		this->outputGpu = newOutput;

		commandReverse
			->updateInputParameter(ZERO_UINT, newInput)
			->updateInputParameter(ONE_UINT, newOutput);
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

	cl_mem GpuReverse::execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvent)
	{
		commandReverse->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, previousEventsLength, previousEvents, currentEvent);

		if (indexesSwapped)
			return inputGpu;
		else
			return outputGpu;
	}

	void GpuReverse::dispose()
	{
		if (releaseBuffers)
		{
			gpu->releaseBuffer(inputGpu);
			inputGpu = nullptr;

			gpu->releaseBuffer(inputLengthGpu);
			inputLengthGpu = nullptr;
		}

		if (commandReverse != nullptr)
		{
			sp_mem_delete(commandReverse, GpuCommand);
			commandReverse = nullptr;
		}

		if (program != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseProgram(program));
			program = nullptr;
		}
	}

}

#endif // OPENCL_ENABLED