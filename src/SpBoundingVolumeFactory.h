#ifndef SP_BOUNDING_VOLUME_FACTORY_HEADER
#define SP_BOUNDING_VOLUME_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "FileSystem.h"
#include "DOP18.h"

#ifdef OPENCL_ENABLED
	#include "GpuCommand.h"
#endif

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Base Factory for Bounding Volumes
	/// </summary>
	class SpBoundingVolumeFactory
	{
	protected:
		GpuDevice* gpu;
		GpuCommand* command;
		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };
		cl_program program;
		cl_mem _boundingVolumesGPU;

		void initProgram(GpuDevice* gpu)
		{
			SpDirectory* filename = SpDirectory::currentDirectory()
				->add(SP_DIRECTORY_OPENCL_SOURCE)
				->add("BoundingVolumeFactory.cl");

			SP_FILE file;
			file.open(filename->name()->data(), std::ios::in);
			sp_mem_delete(filename, SpDirectory);
			sp_size fileSize = file.length();
			sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
			file.read(source, fileSize);
			file.close();

			const sp_char* buildOptions = nullptr;

			sp_uint programIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions);
			ALLOC_RELEASE(source);
			program = gpu->commandManager->cachedPrograms[programIndex];
		}

	public:

		API_INTERFACE inline SpBoundingVolumeFactory()
		{
			gpu = nullptr;
			command = nullptr;
			program = nullptr;
			_boundingVolumesGPU = nullptr;
		}

		API_INTERFACE inline cl_mem boundingVolumeGPU() const
		{
			return _boundingVolumesGPU;
		}

		API_INTERFACE inline void initOutput(GpuDevice* gpu, cl_mem boundingVolumeGPU)
		{
			this->gpu = gpu;
			_boundingVolumesGPU = boundingVolumeGPU;
		}

		API_INTERFACE inline void initOutput(GpuDevice* gpu, const sp_uint objectsLength)
		{
			this->gpu = gpu;
			_boundingVolumesGPU = gpu->createBuffer(sizeof(DOP18) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		}
	
		API_INTERFACE virtual void execute() const
		{
			command->execute(1u, globalWorkSize, localWorkSize);
		}

		API_INTERFACE inline sp_double timeOfLastExecution() const
		{
			return command->getTimeOfExecution();
		}

		API_INTERFACE virtual void dispose()
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

			if (_boundingVolumesGPU != nullptr)
			{
				gpu->releaseBuffer(_boundingVolumesGPU);
				_boundingVolumesGPU = nullptr;
			}
		}

	};

}

#endif // DOP18_HEADER