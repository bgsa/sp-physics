#ifdef OPENCL_ENABLED

#include "GpuFindMinMax.h"

namespace NAMESPACE_PHYSICS
{
	GpuFindMinMax* GpuFindMinMax::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;

		GpuCommands::init(gpu, buildOptions);

		IFileManager* fileManager = Factory::getFileManagerInstance();

		std::string sourceRadixSort = fileManager->readTextFile("FindMinMax.cl");
		findMinMaxProgramIndex = this->gpu->commandManager->cacheProgram(sourceRadixSort.c_str(), SIZEOF_CHAR * sourceRadixSort.length(), buildOptions);

		delete fileManager;
		return this;
	}

	GpuFindMinMax* GpuFindMinMax::setParameters(sp_float* input, sp_uint inputLength, sp_uint stride, sp_uint offset)
	{
		this->inputLength = inputLength;

		indexesGpu = GpuCommands::creteIndexes(gpu, inputLength);
		
		sp_uint groupLength = groupStride = nextPowOf2(multiplyBy4(gpu->computeUnits));
		sp_uint threadLength = nextPowOf2((sp_uint)(inputLength / groupLength));

		if (threadLength == 1u)
		{
			threadLength = divideBy2(inputLength);
			groupLength = groupStride = 1;
		}

		globalWorkSize[0] = threadLength;
		globalWorkSize[1] = 0;
		globalWorkSize[2] = 0;
		localWorkSize[0] = groupStride;
		localWorkSize[1] = 0;
		localWorkSize[2] = 0;
	
		inputGpu = gpu->createBuffer(input, inputLength * stride * SIZEOF_FLOAT, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
		inputLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		groupStrideGpu = gpu->createBuffer(&groupStride, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		output = gpu->createBuffer( multiplyBy2(threadLength) * SIZEOF_FLOAT, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);

		commandFindMinMaxByThread = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputLength * stride * SIZEOF_FLOAT)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetGpu, SIZEOF_UINT)
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxByThread");

		commandFindMinMaxParallelReduce = gpu->commandManager->createCommand()
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxParallelReduce");

		if (groupLength == 1 && isOdd(inputLength))
		{
			commandFindMinMaxParallelReduce_Odd = gpu->commandManager->createCommand()
				->setInputParameter(inputGpu, inputLength * stride * SIZEOF_FLOAT)
				->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
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

		commandFindMinMaxByThread
			->execute(1, globalWorkSize, localWorkSize, 0, NULL, 0U, &previousEvent);

		sp_float* result = ALLOC_ARRAY(sp_float, 4);
		gpu->commandManager->executeReadBuffer(output, 4 * SIZEOF_FLOAT, result, true);
		
		commandFindMinMaxParallelReduce
			->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);

		gpu->commandManager->executeReadBuffer(output, 4 * SIZEOF_FLOAT, result, true);

		if (groupStride > 1)
		{
			for (sp_uint i = divideBy2(groupStride); i != 1; i = divideBy2(i))
			{
				globalWorkSize[0] = divideBy2(globalWorkSize[0]);
				localWorkSize[0] = i;

				std::swap(lastEvent, previousEvent);

				commandFindMinMaxParallelReduce
					->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);
			}

			std::swap(lastEvent, previousEvent);

			globalWorkSize[0] = globalWorkSize[0] >> 1;
			localWorkSize[0] = localWorkSize[0] >> 1;

			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);
		}

		for (sp_uint i = globalWorkSize[0]; i != 1; i = divideBy2(i))
		{
			globalWorkSize[0] = divideBy2(globalWorkSize[0]);

			std::swap(lastEvent, previousEvent);

			commandFindMinMaxParallelReduce
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);

			gpu->commandManager->executeReadBuffer(output, 4 * SIZEOF_FLOAT, result, true);
		}

		if (isOdd(inputLength))
		{
			std::swap(lastEvent, previousEvent);

			commandFindMinMaxParallelReduce_Odd
				->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT, &lastEvent);

			gpu->commandManager->executeReadBuffer(output, 4 * SIZEOF_FLOAT, result, true);
		}

		return output;
	}

	GpuFindMinMax::~GpuFindMinMax()
	{
		gpu->releaseBuffer(4, inputGpu, indexesGpu, inputLengthGpu, offsetGpu);

		commandFindMinMaxByThread->~GpuCommand();
		commandFindMinMaxParallelReduce->~GpuCommand();
	}
}

#endif // OPENCL_ENABLED