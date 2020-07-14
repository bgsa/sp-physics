#ifdef OPENCL_ENABLED

#include "GpuIndexes.h"

namespace NAMESPACE_PHYSICS
{
	
	GpuIndexes* GpuIndexes::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (program != nullptr)
			return this;

		this->gpu = gpu;

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("Indexes.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		sp_mem_delete(filename, SpDirectory);
		const sp_size fileSize = file.length();
		sp_char* sourceBasic = ALLOC_ARRAY(sp_char, fileSize);
		file.read(sourceBasic, fileSize);
		file.close();

		const sp_uint createIndexesProgramIndex = gpu->commandManager->cacheProgram(sourceBasic, SIZEOF_CHAR * fileSize, buildOptions);

		ALLOC_RELEASE(sourceBasic);

		program = gpu->commandManager->cachedPrograms[createIndexesProgramIndex];

		return this;
	}

	void GpuIndexes::setParametersCreateIndexes(sp_uint length)
	{
		output = gpu->createBuffer(length * SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		//output = gpu->createBuffer(length * SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		createIndexesGlobalWorkSize[0] = length;
		createIndexesLocalWorkSize[0] = gpu->getGroupLength(length, length);

		commandInitIndexes = gpu->commandManager->createCommand()
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(program, "create");
	}

	cl_mem GpuIndexes::execute(sp_uint previousEventsLength, cl_event* previousEvents)
	{
		commandInitIndexes->execute(1, createIndexesGlobalWorkSize, createIndexesLocalWorkSize, 0, previousEvents, previousEventsLength);
		lastEvent = commandInitIndexes->lastEvent;

		return output;
	}

	void GpuIndexes::dispose()
	{
		if (commandInitIndexes != nullptr)
		{
			sp_mem_delete(commandInitIndexes, GpuCommand);
			commandInitIndexes = nullptr;
		}
	}

	GpuIndexes::~GpuIndexes() { dispose();	}

}

#endif // OPENCL_ENABLED