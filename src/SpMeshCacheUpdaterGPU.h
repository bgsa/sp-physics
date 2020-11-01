#if OPENCL_ENABLED

#ifndef SP_MESH_CACHE_UPDATER_GPU_HEADER
#define SP_MESH_CACHE_UPDATER_GPU_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "SpTransform.h"
#include "FileSystem.h"

namespace NAMESPACE_PHYSICS
{
	class SpMeshCacheUpdaterGPU
	{
	private:
		GpuDevice* gpu = nullptr;
		GpuCommand* command = nullptr;
		cl_program program = nullptr;
		
		sp_size globalWorkSize[3] = { 0,0,0 };
		sp_size localWorkSize[3] = { 0,0,0 };

	public:
		cl_event lastEvent = nullptr;

		API_INTERFACE void init(GpuDevice* gpu, const char* buildOptions = NULL)
		{
			this->gpu = gpu;

			SpDirectory* filename = SpDirectory::currentDirectory()
				->add(SP_DIRECTORY_OPENCL_SOURCE)
				->add("SpMeshCache.cl");

			SP_FILE file;
			file.open(filename->name()->data(), std::ios::in);
			sp_mem_delete(filename, SpDirectory);
			const sp_size fileSize = file.length();
			sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
			file.read(source, fileSize);
			file.close();

			const sp_uint commandIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions);
			program = gpu->commandManager->cachedPrograms[commandIndex];

			ALLOC_RELEASE(source);
		}

		API_INTERFACE void setParameters(GpuBufferOpenCL* indexesLengthGPU, GpuBufferOpenCL* objectMapperGPU, GpuBufferOpenCL* meshesGPU, GpuBufferOpenCL* meshesIndexesGPU, GpuBufferOpenCL* meshesIndexesLengthGPU, cl_mem transformationsGPU, GpuBufferOpenCL* meshCacheIndexesGPU, GpuBufferOpenCL* meshCacheGPU, sp_uint inputLength)
		{
			globalWorkSize[0] = inputLength;
			localWorkSize[0] = gpu->getGroupLength(inputLength, inputLength);

			command = gpu->commandManager
				->createCommand()
				->setInputParameter(indexesLengthGPU)
				->setInputParameter(objectMapperGPU)
				->setInputParameter(meshesGPU)
				->setInputParameter(meshesIndexesGPU)
				->setInputParameter(meshesIndexesLengthGPU)
				->setInputParameter(transformationsGPU, sizeof(SpTransform) * inputLength)
				->setInputParameter(meshCacheIndexesGPU)
				->setInputParameter(meshCacheGPU)
				->buildFromProgram(program, "update");
		}
	
		API_INTERFACE inline void execute(sp_uint previousEventsLength = ZERO_UINT, cl_event* previousEvents = nullptr)
		{
			const sp_uint zeroValue = ZERO_UINT;

			command
				->execute(1, globalWorkSize, localWorkSize, 0, previousEvents, previousEventsLength);

			lastEvent = command->lastEvent;
		}

		/// <summary>
		/// Release allocated resources from this object
		/// </summary>
		API_INTERFACE inline void dispose()
		{
			if (command != nullptr)
			{
				sp_mem_delete(command, GpuCommand);
				command = nullptr;
			}
		}

		~SpMeshCacheUpdaterGPU() { dispose(); }

	};

}

#endif // SP_MESH_CACHE_UPDATER_GPU_HEADER

#endif // OPENCL_ENABLED