#ifdef OPENCL_ENABLED

#include "GpuFindMinMax.h"

namespace NAMESPACE_PHYSICS
{
	GpuFindMinMax* GpuFindMinMax::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;

		commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("FindMinMax.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());
		
		gpu->commandManager->buildProgram(source->data(), sizeof(sp_char) * source->length(), buildOptions, &findMinMaxProgram);
		
		sp_mem_delete(source, SpString);
		sp_mem_delete(filename, SpDirectory);
		return this;
	}

	GpuFindMinMax* GpuFindMinMax::setParameters(sp_float* input, sp_uint indexLength, sp_uint stride, sp_uint offset, cl_mem indexes)
	{
		if (indexes == NULL)
		{
			commandIndexes->setParametersCreateIndexes(indexLength);
			indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, NULL);
		}

		const sp_uint realIndexLength = indexLength;
		if (isOdd(indexLength)) 
		{
			isIndexLengthOdd = true;
			indexLength--;
		}
		this->indexLength = indexLength;

		sp_uint threadLength = divideBy2(indexLength);

		globalWorkSize[0] = threadLength;
		localWorkSize[0] = gpu->getGroupLength(threadLength, realIndexLength);

		const sp_uint inputSize = realIndexLength * stride * sizeof(sp_float);
		const sp_uint outputSize = multiplyBy2(threadLength) * sizeof(sp_float);

		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
		inputLengthGpu = gpu->createBuffer(&indexLength, sizeof(sp_uint), CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		output = gpu->createBuffer(outputSize, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);

		commandFindMinMaxByThread = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * realIndexLength)
			->setInputParameter(inputLengthGpu, sizeof(sp_uint))
			->setInputParameter(output, outputSize)
			->buildFromProgram(findMinMaxProgram, "findMinMaxByThread");

		commandFindMinMaxParallelReduce = gpu->commandManager->createCommand()
			->setInputParameter(output, outputSize)
			->buildFromProgram(findMinMaxProgram, "findMinMaxParallelReduce");

		commandFindMinMaxParallelReduce_OneThread = gpu->commandManager->createCommand()
			->setInputParameter(output, outputSize)
			->buildFromProgram(findMinMaxProgram, "findMinMaxParallelReduce_OneThread");

		if (isIndexLengthOdd)
		{
			commandFindMinMaxParallelReduce_Odd = gpu->commandManager->createCommand()
				->setInputParameter(inputGpu, inputSize)
				->setInputParameter(indexesGpu, sizeof(sp_uint) * realIndexLength)
				->setInputParameter(output, outputSize)
				->setInputParameter(inputLengthGpu, sizeof(sp_uint))
				->buildFromProgram(findMinMaxProgram, "findMinMaxParallelReduce_Odd");
		}

		return this;
	}

	cl_mem GpuFindMinMax::execute()
	{
		cl_event previousEvent = nullptr, currentEvent = nullptr;

		commandFindMinMaxByThread
			->execute(1, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &previousEvent);

		globalWorkSize[0] = (sp_size) std::ceil(globalWorkSize[0] / TWO_DOUBLE);

		if (isIndexLengthOdd) 
		{
			commandFindMinMaxParallelReduce_Odd
				->execute(ONE_UINT, globalWorkSizeOdd, localWorkSizeOdd, 0, ONE_UINT, &previousEvent, &currentEvent);

			gpu->releaseEvent(previousEvent);
			std::swap(currentEvent, previousEvent);
		}

		if (localWorkSize[0] > 1)
		{
			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, ONE_UINT , &previousEvent, &currentEvent);

			gpu->releaseEvent(previousEvent);
			std::swap(currentEvent, previousEvent);

			globalWorkSize[0] = sp_max((sp_size)std::ceil(globalWorkSize[0] / TWO_DOUBLE), ONE_SIZE);
			localWorkSize[0]  = sp_max((sp_size)std::ceil(localWorkSize[0] / TWO_DOUBLE), ONE_SIZE);

			while (globalWorkSize[0] != localWorkSize[0])
			{
				commandFindMinMaxParallelReduce
					->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, ONE_UINT, &previousEvent, &currentEvent);

				gpu->releaseEvent(previousEvent);
				std::swap(currentEvent, previousEvent);

				globalWorkSize[0] = divideBy2(globalWorkSize[0]);
			}
		}

		localWorkSize[0] = 1;

		//while (globalWorkSize[0] != 1 && globalWorkSize[0] == localWorkSize[0])
		while (globalWorkSize[0] != 1)
		{
			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, ONE_UINT, &previousEvent, &currentEvent);

			gpu->releaseEvent(previousEvent);
			std::swap(currentEvent, previousEvent);

			globalWorkSize[0] = (sp_uint) std::ceil( globalWorkSize[0] / TWO_DOUBLE );
			//localWorkSize[0] = globalWorkSize[0];
		}

		if (indexLength > TWO_UINT)
		{
			commandFindMinMaxParallelReduce_OneThread
				->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, ONE_UINT, &previousEvent, &currentEvent);

			gpu->releaseEvent(previousEvent);
		}

		gpu->releaseEvent(currentEvent);

		return output;
	}

	GpuFindMinMax::~GpuFindMinMax()
	{
		gpu->releaseBuffer(3, inputGpu, indexesGpu, inputLengthGpu);

		commandIndexes->~GpuIndexes();
		commandFindMinMaxByThread->~GpuCommand();
		commandFindMinMaxParallelReduce->~GpuCommand();
		commandFindMinMaxParallelReduce_OneThread->~GpuCommand();

		if (commandFindMinMaxParallelReduce_Odd != nullptr)
			commandFindMinMaxParallelReduce_Odd->~GpuCommand();

		if (findMinMaxProgram != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseProgram(findMinMaxProgram));
			findMinMaxProgram = nullptr;
		}
	}
}

#endif // OPENCL_ENABLED