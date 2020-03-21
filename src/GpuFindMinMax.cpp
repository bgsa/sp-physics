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

		IFileManager* fileManager = Factory::getFileManagerInstance();

		std::string sourceRadixSort = fileManager->readTextFile("FindMinMax.cl");
		findMinMaxProgramIndex = this->gpu->commandManager->cacheProgram(sourceRadixSort.c_str(), SIZEOF_CHAR * sourceRadixSort.length(), buildOptions);

		delete fileManager;
		return this;
	}

	GpuFindMinMax* GpuFindMinMax::setParameters(sp_float* input, sp_uint indexLength, sp_uint stride, sp_uint offset, cl_mem indexes)
	{
		this->indexLength = indexLength;

		if (indexes == NULL)
		{
			commandIndexes->setParametersCreateIndexes(this->indexLength);
			indexesGpu = commandIndexes->execute();
		}
		
		sp_uint groupLength = nextPowOf2(multiplyBy4(gpu->computeUnits));
		sp_uint threadLength = std::max(  (sp_uint)(this->indexLength / groupLength) , ONE_UINT);

		if (threadLength == ONE_UINT)
		{
			threadLength = nextPowOf2(indexLength);
			groupLength = divideBy2(threadLength);
		}
		else 
		{
			if (threadLength * groupLength != indexLength)
			{
				threadLength = divideBy2(nextPowOf2(indexLength));
				groupLength = divideBy2(groupLength);
			}
		}

		globalWorkSize[0] = threadLength;
		localWorkSize[0] = groupLength;
	
		inputGpu = gpu->createBuffer(input, indexLength * stride * SIZEOF_FLOAT, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
		inputLengthGpu = gpu->createBuffer(&this->indexLength, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		output = gpu->createBuffer( multiplyBy2(threadLength) * SIZEOF_FLOAT, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);

		commandFindMinMaxByThread = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, this->indexLength * stride * SIZEOF_FLOAT)
			->setInputParameter(indexesGpu, SIZEOF_UINT * this->indexLength)
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetGpu, SIZEOF_UINT)
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxByThread");

		commandFindMinMaxParallelReduce = gpu->commandManager->createCommand()
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxParallelReduce");

		commandFindMinMaxParallelReduce_OneThread = gpu->commandManager->createCommand()
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxParallelReduce_OneThread");

		if (groupLength == 1 && isOdd(this->indexLength))
		{
			commandFindMinMaxParallelReduce_Odd = gpu->commandManager->createCommand()
				->setInputParameter(inputGpu, this->indexLength * stride * SIZEOF_FLOAT)
				->setInputParameter(indexesGpu, SIZEOF_UINT * this->indexLength)
				->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
				->setInputParameter(offsetGpu, SIZEOF_UINT)
				->setInputParameter(inputLengthGpu, SIZEOF_UINT)
				->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxParallelReduce_Odd");
		}

		return this;
	}

	cl_mem GpuFindMinMax::execute()
	{
		cl_event previousEvent = NULL;
		cl_event lastEvent = NULL;
		sp_bool eventSwapped = false;

		commandFindMinMaxByThread
			->execute(1, globalWorkSize, localWorkSize, 0, NULL, 0u, &previousEvent);
		
		if (localWorkSize[0] > 1)
		{
			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);
			std::swap(lastEvent, previousEvent);
			eventSwapped = true;

			globalWorkSize[0] = divideBy2(globalWorkSize[0]);
			localWorkSize[0] = std::max(divideBy2(localWorkSize[0]), ONE_UINT);

			while (globalWorkSize[0] != localWorkSize[0])
			{
				commandFindMinMaxParallelReduce
					->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);
				std::swap(lastEvent, previousEvent);

				globalWorkSize[0] = divideBy2(globalWorkSize[0]);
			}
		}

		while (globalWorkSize[0] != 1 && globalWorkSize[0] == localWorkSize[0])
		{
			if (eventSwapped)
			{
				std::swap(lastEvent, previousEvent);
				eventSwapped = true;
			}

			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);

			globalWorkSize[0] = divideBy2(globalWorkSize[0]);
			localWorkSize[0] = divideBy2(localWorkSize[0]);
		}

		if (eventSwapped)
		{
			std::swap(lastEvent, previousEvent);
			eventSwapped = true;
		}

		commandFindMinMaxParallelReduce_OneThread
			->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);

		if (isOdd(this->indexLength))
		{
			if (eventSwapped)
				std::swap(lastEvent, previousEvent);

			commandFindMinMaxParallelReduce_Odd
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);
		}

		return output;
	}

	GpuFindMinMax::~GpuFindMinMax()
	{
		gpu->releaseBuffer(4, inputGpu, indexesGpu, inputLengthGpu, offsetGpu);

		commandIndexes->~GpuIndexes();
		commandFindMinMaxByThread->~GpuCommand();
		commandFindMinMaxParallelReduce->~GpuCommand();
		commandFindMinMaxParallelReduce_OneThread->~GpuCommand();

		if (commandFindMinMaxParallelReduce_Odd != NULL)
			commandFindMinMaxParallelReduce_Odd->~GpuCommand();
	}
}

#endif // OPENCL_ENABLED