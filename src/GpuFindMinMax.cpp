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
		indexesGpu = GpuCommands::creteIndexes(gpu, inputLength);

		sp_uint groupLength = groupStride = multiplyBy2(gpu->computeUnits);
		const sp_uint threadLength = nextPowOf2((sp_uint)(inputLength / groupLength));
		const sp_uint elementsPerThread = (sp_uint)(threadLength / gpu->computeUnits);

		globalWorkSize[0] = threadLength;
		globalWorkSize[1] = 0;
		globalWorkSize[2] = 0;
		localWorkSize[0] = gpu->computeUnits;
		localWorkSize[1] = 0;
		localWorkSize[2] = 0;
	
		inputGpu = gpu->createBuffer(input, inputLength * stride * SIZEOF_FLOAT, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
		inputLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		prefixScanOffsetGpu = gpu->createBuffer(&groupStride, SIZEOF_UINT, CL_MEM_COPY_HOST_PTR | CL_MEM_READ_ONLY);
		output = gpu->createBuffer( multiplyBy2(threadLength) * SIZEOF_FLOAT, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);

		commandFindMinMaxByThread = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputLength * stride * SIZEOF_FLOAT)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetGpu, SIZEOF_UINT)
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxByThread");

		commandFindMinMaxParallelReduce = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, SIZEOF_FLOAT * inputLength * stride)
			->setInputParameter(inputLengthGpu, SIZEOF_UINT)
			->setInputParameter(prefixScanOffsetGpu, SIZEOF_UINT)
			->setInputParameter(output, multiplyBy2(threadLength) * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxParallelReduce");

		return this;
	}

	cl_mem GpuFindMinMax::execute()
	{
		commandFindMinMaxByThread
			->execute(1, globalWorkSize, localWorkSize);

	//gpu->commandManager->executeReadBuffer(output, threadLength * 2 * SIZEOF_FLOAT, result, true);

		const size_t threadLength = gpu->getThreadLength(131072);
		float* result = ALLOC_ARRAY(float, multiplyBy2(threadLength));
		gpu->commandManager->executeReadBuffer(output, threadLength * 2 * SIZEOF_FLOAT, result, true);

		commandFindMinMaxParallelReduce
			->execute(1, globalWorkSize, localWorkSize);

		//gpu->commandManager->executeReadBuffer(output, threadLength * 2 * SIZEOF_FLOAT, result, true);

		for (sp_uint i = divideBy2(groupStride); i > 0; i = divideBy2(i) )
		{
			commandFindMinMaxParallelReduce
				->updateInputParameterValue(2, &i)
				->execute(1, globalWorkSize, localWorkSize);

			gpu->commandManager->executeReadBuffer(output, threadLength * 2 * SIZEOF_FLOAT, result, true);
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