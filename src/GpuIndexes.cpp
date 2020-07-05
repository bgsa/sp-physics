#ifdef OPENCL_ENABLED

#include "GpuIndexes.h"

namespace NAMESPACE_PHYSICS
{
	
	GpuIndexes* GpuIndexes::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (createIndexesProgramIndex != UINT_MAX)
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

		createIndexesProgramIndex = gpu->commandManager->cacheProgram(sourceBasic, SIZEOF_CHAR * fileSize, buildOptions);

		ALLOC_RELEASE(sourceBasic);

		program = gpu->commandManager->cachedPrograms[createIndexesProgramIndex];
		return this;
	}

	void GpuIndexes::setParametersCreateIndexes(sp_uint length)
	{
		isInputOdd = isOdd(length);

		output = gpu->createBuffer(length * SIZEOF_UINT, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		
		createIndexesGlobalWorkSize[0] = gpu->getThreadLength(length);
		createIndexesLocalWorkSize[0] = gpu->getGroupLength(createIndexesGlobalWorkSize[0], length);

		commandInitIndexes = gpu->commandManager->createCommand()
			->setInputParameter(&length, SIZEOF_UINT, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR)
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(program, "create");

		commandInitIndexes_Odd = gpu->commandManager->createCommand()
			->setInputParameter(commandInitIndexes->getInputParameter(0), SIZEOF_UINT)
			->setInputParameter(output, length * SIZEOF_UINT)
			->buildFromProgram(program, "create_Odd");
	}

	cl_mem GpuIndexes::execute(sp_uint previousEventsLength, cl_event* previousEvents)
	{
		commandInitIndexes->execute(1, createIndexesGlobalWorkSize, createIndexesLocalWorkSize, 0, previousEvents, previousEventsLength);
		lastEvent = commandInitIndexes->lastEvent;

		if (isInputOdd)
		{
			const sp_size globalWorkSizeOdd[3] = { 1, 0, 0 };
			const sp_size localWorkSizeOdd[3] = { 1, 0, 0 };

			commandInitIndexes_Odd->execute(1, globalWorkSizeOdd, localWorkSizeOdd, 0, &lastEvent, ONE_UINT);
			lastEvent = commandInitIndexes_Odd->lastEvent;
		}

		return output;
	}

	void GpuIndexes::dispose()
	{
		if (commandInitIndexes != nullptr)
		{
			sp_mem_delete(commandInitIndexes, GpuCommand);
			commandInitIndexes = nullptr;
		}

		if (commandInitIndexes_Odd != nullptr)
		{
			sp_mem_delete(commandInitIndexes_Odd, GpuCommand);
			commandInitIndexes_Odd = nullptr;
		}
	}


	GpuIndexes::~GpuIndexes() { dispose();	}

}

#endif // OPENCL_ENABLED