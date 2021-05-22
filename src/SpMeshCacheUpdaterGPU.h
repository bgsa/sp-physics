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
		GpuDevice* gpu;
		GpuCommand* command;
		cl_program program;
		
		sp_size globalWorkSize[3] = { 0,0,0 };
		sp_size localWorkSize[3] = { 0,0,0 };

	public:

		API_INTERFACE SpMeshCacheUpdaterGPU()
		{
			gpu = nullptr;
			command = nullptr;
			program = nullptr;
		}

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

			gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions, &program);
			ALLOC_RELEASE(source);
		}

		API_INTERFACE void setParameters(GpuBufferOpenCL* indexesLengthGPU, GpuBufferOpenCL* meshesGPU, GpuBufferOpenCL* meshesIndexesGPU, GpuBufferOpenCL* meshesStridesGPU, GpuBufferOpenCL* meshesIndexesLengthGPU, cl_mem transformationsGPU, GpuBufferOpenCL* meshCacheIndexesGPU, GpuBufferOpenCL* meshCacheGPU, sp_uint inputLength)
		{
			globalWorkSize[0] = inputLength;
			localWorkSize[0] = gpu->getGroupLength(inputLength, inputLength);

			command = gpu->commandManager
				->createCommand()
				->setInputParameter(indexesLengthGPU)
				->setInputParameter(meshesGPU)
				->setInputParameter(meshesIndexesGPU)
				->setInputParameter(meshesStridesGPU)
				->setInputParameter(meshesIndexesLengthGPU)
				->setInputParameter(transformationsGPU, sizeof(SpTransform) * inputLength)
				->setInputParameter(meshCacheIndexesGPU)
				->setInputParameter(meshCacheGPU)
				->buildFromProgram(program, "update");
		}
	
		API_INTERFACE inline void execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt)
		{
			command->execute(1, globalWorkSize, localWorkSize, 0, previousEventsLength, previousEvents, evt);
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

			if (program != nullptr)
			{
				HANDLE_OPENCL_ERROR(clReleaseProgram(program));
				program = nullptr;
			}
		}

		~SpMeshCacheUpdaterGPU() { dispose(); }

	};

}

#endif // SP_MESH_CACHE_UPDATER_GPU_HEADER

#endif // OPENCL_ENABLED