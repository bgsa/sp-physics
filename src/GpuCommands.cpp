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
		GpuCommand* commandInitIndexes = gpu->commandManager->createCommand();

		sp_size indexesSize = length * SIZEOF_SIZE;
		cl_mem indexesBuffer = gpu->createBuffer(indexesSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		commandInitIndexes
			->setInputParameter(&length, SIZEOF_SIZE)
			->setInputParameter(indexesBuffer, indexesSize)
			->buildFromProgram(gpu->commandManager->cachedPrograms[basicProgramIndex], "initIndexes");

		const sp_size globalWorkSize[3] = { nextPowOf2(std::min(commandInitIndexes->workGroupSize, length)), 0 , 0 };
		const sp_size localWorkSize[3] = { std::max((sp_uint)(nextPowOf2(length) / commandInitIndexes->workGroupSize), 1U), 0, 0 };
		const sp_size groupCount = globalWorkSize[0] / localWorkSize[0];

		commandInitIndexes->execute(1, globalWorkSize, localWorkSize);

		commandInitIndexes->~GpuCommand();

		return indexesBuffer;
	}

}

#endif // ! OPENCL_ENABLED