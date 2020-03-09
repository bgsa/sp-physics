#ifdef OPENCL_ENABLED

#include "GpuCommands.h"

namespace OpenML
{

	static size_t findMinMaxProgramIndex = UINT_MAX;
	static size_t basicProgramIndex = UINT_MAX;

	void GpuCommands::init(GpuDevice* gpu, const char* buildOptions)
	{
		if (findMinMaxProgramIndex != UINT_MAX)
			return;

		IFileManager* fileManager = Factory::getFileManagerInstance();

		std::string sourceFindMinMax = fileManager->readTextFile("FindMinMax.cl");
		findMinMaxProgramIndex = gpu->commandManager->cacheProgram(sourceFindMinMax.c_str(), SIZEOF_CHAR * sourceFindMinMax.length(), buildOptions);

		std::string sourceBasic = fileManager->readTextFile("BasicCommands.cl");
		basicProgramIndex = gpu->commandManager->cacheProgram(sourceBasic.c_str(), SIZEOF_CHAR * sourceBasic.length(), buildOptions);

		delete fileManager;
	}

	cl_mem GpuCommands::creteIndexes(GpuDevice* gpu, sp_size length)
	{
		const sp_size globalWorkSize[3] = { nextPowOf2(std::min((sp_size)gpu->maxWorkGroupSize, length)), 0 , 0 };
		const sp_size localWorkSize[3] = { std::max((sp_uint) (nextPowOf2(length) / gpu->maxWorkGroupSize), 1U), 0, 0 };
		const sp_size groupCount = globalWorkSize[0] / localWorkSize[0];

		GpuCommand* commandInitIndexes = gpu->commandManager->createCommand();

		sp_size indexesSize = length * SIZEOF_UINT;
		cl_mem indexesBuffer = gpu->createBuffer(indexesSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		commandInitIndexes
			->setInputParameter(&length, SIZEOF_UINT)
			->setInputParameter(indexesBuffer, indexesSize)
			->buildFromProgram(gpu->commandManager->cachedPrograms[basicProgramIndex], "initIndexes")
			->execute(1, globalWorkSize, localWorkSize);

		commandInitIndexes->~GpuCommand();

		return indexesBuffer;
	}

	cl_mem GpuCommands::findMaxGPUBuffer(GpuDevice* gpu, float* input, size_t n, size_t strider, size_t offset)
	{
		const sp_size globalWorkSize[3] = { gpu->maxWorkGroupSize, 0 , 0 };
		const sp_size localWorkSize[3] = { nextPowOf2(n) / gpu->maxWorkGroupSize, 0, 0 };
		const sp_int groupCount = std::max((sp_int)(gpu->maxWorkGroupSize / localWorkSize[0]), 1);

		cl_mem outputBuffer = gpu->createBuffer(SIZEOF_FLOAT * groupCount, CL_MEM_READ_WRITE);

		gpu->commandManager->createCommand()
			->setInputParameter(input, SIZEOF_FLOAT * n, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY, false)
			->setInputParameter(&n, SIZEOF_UINT)
			->setInputParameter(outputBuffer, SIZEOF_FLOAT * groupCount)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMax")
			->execute(1, globalWorkSize, localWorkSize)
			->~GpuCommand();

		return  outputBuffer;
	}

	float* GpuCommands::findMinMaxGPU(GpuDevice* gpu, float* input, sp_size n, sp_size strider, sp_size offset)
	{
		sp_float* output = ALLOC_ARRAY(sp_float, 2);
		const sp_size globalWorkSize[3] = { nextPowOf2(std::min((sp_size)gpu->maxWorkGroupSize, n)), 0 , 0 };
		const sp_size localWorkSize[3] = { std::max(nextPowOf2(n) / gpu->maxWorkGroupSize, sp_size(1)), 0, 0 };
		const sp_uint groupCount = std::max((sp_uint)(globalWorkSize[0] / localWorkSize[0]), ONE_UINT);

		cl_mem outputBuffer = gpu->createBuffer(SIZEOF_FLOAT * groupCount * 2, CL_MEM_ALLOC_HOST_PTR | CL_MEM_READ_WRITE);

		GpuCommand* commandFindMinMax = gpu->commandManager->createCommand();

		float* outputGPU = commandFindMinMax
			->setInputParameter(input, SIZEOF_FLOAT * n * strider, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY, false)
			->setInputParameter(&n, SIZEOF_UINT * strider)
			->setInputParameter(outputBuffer, SIZEOF_FLOAT * groupCount * 2)
			->setInputParameter(&strider, SIZEOF_UINT)
			->setInputParameter(&offset, SIZEOF_UINT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMax")
			->execute(1, globalWorkSize, localWorkSize)
			->fetchInOutParameter<float>(2);

		output[0] = SP_FLOAT_MAX;
		output[1] = SP_FLOAT_MIN;
		for (sp_uint i = 0; i < groupCount - 1; i++)   // check the remaining itens provided from GPU
		{
			if (output[0] > outputGPU[i])
				output[0] = outputGPU[i];

			if (output[1] < outputGPU[groupCount + i])
				output[1] = outputGPU[groupCount + i];
		}

		commandFindMinMax->~GpuCommand();
		gpu->releaseBuffer(outputBuffer);
		return output;
	}

	void GpuCommands::findMinMaxIndexesGPU(GpuDevice* gpu, cl_mem elements, cl_mem indexes, cl_mem indexesLength, cl_mem offset, size_t indexesLengthCpu, size_t striderCpu, cl_mem output)
	{
		const sp_size globalWorkSize[3] = { nextPowOf2(std::min( (sp_size)gpu->maxWorkGroupSize, indexesLengthCpu)), 0 , 0 };
		const sp_size localWorkSize[3] = { gpu->getLocalWorkSize(indexesLengthCpu) , 0, 0 };
		const sp_size threadLength = gpu->getThreadLength(indexesLengthCpu);

		//const size_t globalWorkSize[3] = { 1024, 0 , 0 };
		//const size_t localWorkSize[3] = { 64, 0, 0 };
		//const size_t threadLength = 1024;
		
		GpuCommand* commandFindMinMaxByWorkGroup = gpu->commandManager->createCommand();

		commandFindMinMaxByWorkGroup
			->setInputParameter(elements, SIZEOF_FLOAT * indexesLengthCpu * striderCpu)
			->setInputParameter(indexes, SIZEOF_UINT * indexesLengthCpu)
			->setInputParameter(indexesLength, SIZEOF_UINT)
			->setInputParameter(offset, SIZEOF_UINT)
			->setInputParameter(output, threadLength * 2 * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxByWorkGroup")
			->execute(1, globalWorkSize, localWorkSize);

		GpuCommand* commandFindMinMaxParallelReduce = gpu->commandManager->createCommand();

		size_t i = 1;

		commandFindMinMaxParallelReduce
			->setInputParameter(elements, SIZEOF_FLOAT * indexesLengthCpu * striderCpu)
			->setInputParameter(indexes, SIZEOF_UINT * indexesLengthCpu)
			->setInputParameter(indexesLength, SIZEOF_UINT)
			->setInputParameter(offset, SIZEOF_UINT)
			->setInputParameter(&i, SIZEOF_UINT, CL_MEM_READ_ONLY, false)
			->setInputParameter(output, threadLength * 2 * SIZEOF_FLOAT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[findMinMaxProgramIndex], "findMinMaxParallelReduce")
			->execute(1, globalWorkSize, localWorkSize);

		for (i = 2; i < 8+1; i*=2)
		{
			commandFindMinMaxParallelReduce
				->updateInputParameterValue(4, &i)
				->execute(1, globalWorkSize, localWorkSize);
		}

		commandFindMinMaxByWorkGroup->~GpuCommand();
		commandFindMinMaxParallelReduce->~GpuCommand();
	}

	float GpuCommands::findMaxGPU(GpuDevice* gpu, float* input, size_t n, size_t strider, size_t offset)
	{
		sp_float* output = ALLOC_ARRAY(sp_float, 1);
		cl_mem outputBuffer = GpuCommands::findMaxGPUBuffer(gpu, input, n, strider, offset);

		gpu->commandManager->executeReadBuffer(outputBuffer, SIZEOF_FLOAT, output, true);

		gpu->releaseBuffer(outputBuffer);

		return output[0];
	}

}

#endif // ! OPENCL_ENABLED