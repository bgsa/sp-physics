#ifdef OPENCL_ENABLED

#include "GpuFindMinMax.h"

namespace NAMESPACE_PHYSICS
{
	GpuFindMinMax* GpuFindMinMax::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;
		this->lastEvent = nullptr;

		commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("FindMinMax.cl");

		SP_FILE file;
		SpString* source = file.readTextFile(filename->name()->data());
		
		const sp_uint findMinMaxProgramIndex = this->gpu->commandManager->cacheProgram(source->data(), SIZEOF_CHAR * source->length(), buildOptions);
		findMinMaxProgram = gpu->commandManager->cachedPrograms[findMinMaxProgramIndex];

		sp_mem_delete(source, SpString);
		sp_mem_delete(filename, SpDirectory);
		return this;
	}

	GpuFindMinMax* GpuFindMinMax::setParameters(sp_float* input, sp_uint indexLength, sp_uint stride, sp_uint offset, cl_mem indexes)
	{
		if (indexes == NULL)
		{
			commandIndexes->setParametersCreateIndexes(indexLength);
			indexesGpu = commandIndexes->execute();
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

		const sp_uint inputSize = realIndexLength * stride * SIZEOF_FLOAT;
		const sp_uint outputSize = multiplyBy2(threadLength) * SIZEOF_FLOAT;

		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
		inputLengthGpu = gpu->createBuffer(&indexLength, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		output = gpu->createBuffer(outputSize, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);

		commandFindMinMaxByThread = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * realIndexLength)
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
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
				->setInputParameter(indexesGpu, SIZEOF_UINT * realIndexLength)
				->setInputParameter(output, outputSize)
				->setInputParameter(inputLengthGpu, SIZEOF_UINT)
				->buildFromProgram(findMinMaxProgram, "findMinMaxParallelReduce_Odd");
		}

		return this;
	}

	cl_mem GpuFindMinMax::execute()
	{
		cl_event previousEvent = nullptr;
		lastEvent = nullptr;

		commandFindMinMaxByThread
			->execute(1, globalWorkSize, localWorkSize, ZERO_UINT, NULL, ZERO_UINT);
		previousEvent = commandFindMinMaxByThread->lastEvent;

		globalWorkSize[0] = (sp_size) std::ceil(globalWorkSize[0] / TWO_DOUBLE);

		if (isIndexLengthOdd) 
		{
			commandFindMinMaxParallelReduce_Odd
				->execute(ONE_UINT, globalWorkSizeOdd, localWorkSizeOdd, 0, &previousEvent, ONE_UINT);
			lastEvent = commandFindMinMaxParallelReduce_Odd->lastEvent;
			std::swap(lastEvent, previousEvent);
		}

		if (localWorkSize[0] > 1)
		{
			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
			lastEvent = commandFindMinMaxParallelReduce->lastEvent;
			std::swap(lastEvent, previousEvent);

			globalWorkSize[0] = std::max((sp_uint)std::ceil(globalWorkSize[0] / TWO_DOUBLE), ONE_UINT);
			localWorkSize[0]  = std::max((sp_uint)std::ceil(localWorkSize[0] / TWO_DOUBLE), ONE_UINT);

			while (globalWorkSize[0] != localWorkSize[0])
			{
				commandFindMinMaxParallelReduce
					->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
				lastEvent = commandFindMinMaxParallelReduce->lastEvent;
				std::swap(lastEvent, previousEvent);

				globalWorkSize[0] = divideBy2(globalWorkSize[0]);
			}
		}

		localWorkSize[0] = 1;

		//while (globalWorkSize[0] != 1 && globalWorkSize[0] == localWorkSize[0])
		while (globalWorkSize[0] != 1)
		{
			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
			lastEvent = commandFindMinMaxParallelReduce->lastEvent;
			std::swap(lastEvent, previousEvent);

			globalWorkSize[0] = (sp_uint) std::ceil( globalWorkSize[0] / TWO_DOUBLE );
			//localWorkSize[0] = globalWorkSize[0];
		}

		if (indexLength > TWO_UINT)
		{
			commandFindMinMaxParallelReduce_OneThread
				->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
			lastEvent = commandFindMinMaxParallelReduce_OneThread->lastEvent;
		}

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
	}
}

#endif // OPENCL_ENABLED