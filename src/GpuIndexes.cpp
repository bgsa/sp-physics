#ifdef OPENCL_ENABLED

#include "GpuIndexes.h"

namespace NAMESPACE_PHYSICS
{
	
	void GpuIndexes::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (createIndexesProgramIndex != UINT_MAX)
			return;

		this->gpu = gpu;

		IFileManager* fileManager = Factory::getFileManagerInstance();

		std::string sourceBasic = fileManager->readTextFile("Indexes.cl");
		createIndexesProgramIndex = gpu->commandManager->cacheProgram(sourceBasic.c_str(), SIZEOF_CHAR * sourceBasic.length(), buildOptions);
		program = gpu->commandManager->cachedPrograms[createIndexesProgramIndex];

		delete fileManager;
	}

	void GpuIndexes::setParametersCreateIndexes(sp_uint length)
	{
		isInputOdd = isOdd(length);

		output = gpu->createBuffer(length * SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		
		createIndexesGlobalWorkSize[0] = gpu->getThreadLength(length);
		createIndexesLocalWorkSize[0] = gpu->getGroupLength(createIndexesGlobalWorkSize[0], length);

		commandInitIndexes = gpu->commandManager->createCommand()
			->setInputParameter(&length, SIZEOF_UINT, CL_MEM_READ_ONLY)
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(program, "create");

		commandInitIndexes_Odd = gpu->commandManager->createCommand()
			->setInputParameter(commandInitIndexes->getInputParameter(0), SIZEOF_UINT)
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(program, "create_Odd");
	}

	cl_mem GpuIndexes::execute()
	{
		commandInitIndexes->execute(1, createIndexesGlobalWorkSize, createIndexesLocalWorkSize);		

		if (isInputOdd)
		{
			const sp_size globalWorkSizeOdd[3] = { 1, 0, 0 };
			const sp_size localWorkSizeOdd[3] = { 1, 0, 0 };

			commandInitIndexes_Odd->execute(1, globalWorkSizeOdd, localWorkSizeOdd);
		}

		return output;
	}

	GpuIndexes::~GpuIndexes()
	{
		if (commandInitIndexes != NULL)
			commandInitIndexes->~GpuCommand();

		if (commandInitIndexes_Odd != NULL)
			commandInitIndexes_Odd->~GpuCommand();
	}

}

#endif // OPENCL_ENABLED