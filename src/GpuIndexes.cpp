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

		delete fileManager;
	}

	void GpuIndexes::setParametersCreateIndexes(sp_uint length)
	{
		commandInitIndexes = gpu->commandManager->createCommand();
		output = gpu->createBuffer(length * SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		sp_size* gridConfig = gpu->getGridConfigForOneDimension(length);

		createIndexesGlobalWorkSize[0] = gridConfig[0];
		createIndexesLocalWorkSize[0] = gridConfig[1];

		ALLOC_RELEASE(gridConfig);

		commandInitIndexes
			->setInputParameter(&length, SIZEOF_UINT, CL_MEM_READ_ONLY)
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(gpu->commandManager->cachedPrograms[createIndexesProgramIndex], "create");
	}

	cl_mem GpuIndexes::execute()
	{
		commandInitIndexes->execute(1, createIndexesGlobalWorkSize, createIndexesLocalWorkSize);		
		return output;
	}

	GpuIndexes::~GpuIndexes()
	{
		if (commandInitIndexes != NULL)
			commandInitIndexes->~GpuCommand();
	}

}

#endif // OPENCL_ENABLED