#ifdef OPENCL_ENABLED

#include "GpuCommands.h"

namespace NAMESPACE_PHYSICS
{
	static sp_uint basicProgramIndex = UINT_MAX;

	void GpuCommands::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (basicProgramIndex != UINT_MAX)
			return;

		IFileManager* fileManager = Factory::getFileManagerInstance();

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

		/*
		const sp_size globalWorkSize[3] = { nextPowOf2(std::min(commandInitIndexes->workGroupSize, length)), 0 , 0 };
		const sp_size localWorkSize[3] = { std::max((sp_uint)(nextPowOf2(length) / commandInitIndexes->workGroupSize), 1U), 0, 0 };
		const sp_size groupCount = globalWorkSize[0] / localWorkSize[0];
		*/

		sp_uint groupLength = multiplyBy2(gpu->computeUnits);
		const sp_uint threadLength = nextPowOf2((sp_uint)(length / groupLength));
		const sp_uint elementsPerThread = (sp_uint)(threadLength / groupLength);

		const sp_size globalWorkSize[3] = { threadLength, 0 , 0 };
		const sp_size localWorkSize[3] = { gpu->computeUnits, 0 , 0 };

		commandInitIndexes->execute(1, globalWorkSize, localWorkSize);

		commandInitIndexes->~GpuCommand();

		return indexesBuffer;
	}

}

#endif // ! OPENCL_ENABLED