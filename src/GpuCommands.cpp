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

	cl_mem GpuCommands::creteIndexes(GpuDevice* gpu, sp_uint length)
	{
		GpuCommand* commandInitIndexes = gpu->commandManager->createCommand();

		cl_mem output = gpu->createBuffer(length * SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		commandInitIndexes
			->setInputParameter(&length, SIZEOF_UINT)
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[basicProgramIndex], "initIndexes");

		sp_uint groupLength = multiplyBy4(nextPowOf2(gpu->computeUnits));
		sp_uint threadLength = nextPowOf2((sp_uint)(length / groupLength));

		if (threadLength == 1u)
		{
			threadLength = length;
			groupLength = 1;
		}

		const sp_size globalWorkSize[3] = { threadLength, 0 , 0 };
		const sp_size localWorkSize[3] = { groupLength, 0 , 0 };

		commandInitIndexes->execute(1, globalWorkSize, localWorkSize);

		commandInitIndexes->~GpuCommand();

		return output;
	}

}

#endif // ! OPENCL_ENABLED