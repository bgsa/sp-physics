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

		gpu->commandManager->buildProgram(sourceBasic, sizeof(sp_char) * fileSize, buildOptions, &program);

		ALLOC_RELEASE(sourceBasic);
		return this;
	}

	void GpuIndexes::setParametersCreateIndexes(sp_uint length)
	{
		output = gpu->createBuffer(length * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		//output = gpu->createBuffer(length * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		createIndexesGlobalWorkSize[0] = length;
		createIndexesLocalWorkSize[0] = gpu->getGroupLength(length, length);

		commandInitIndexes = gpu->commandManager->createCommand()
			->setInputParameter(output, length * sizeof(sp_uint))
			->buildFromProgram(program, "create");
	}

	cl_mem GpuIndexes::execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* currentEvents)
	{
		commandInitIndexes->execute(1, createIndexesGlobalWorkSize, createIndexesLocalWorkSize, 0, previousEventsLength, previousEvents, currentEvents);
		return output;
	}

	void GpuIndexes::dispose()
	{
		if (commandInitIndexes != nullptr)
		{
			sp_mem_delete(commandInitIndexes, GpuCommand);
			commandInitIndexes = nullptr;
		}

		if (program != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseProgram(program));
			program = nullptr;
		}
	}

	GpuIndexes::~GpuIndexes() { dispose();	}

}

#endif // OPENCL_ENABLED