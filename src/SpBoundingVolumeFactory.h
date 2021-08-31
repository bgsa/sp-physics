#ifndef SP_BOUNDING_VOLUME_FACTORY_HEADER
#define SP_BOUNDING_VOLUME_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "FileSystem.h"
#include "BoundingVolume.h"

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
		sp_bool _releaseBoundingVolumeGPU;

		void initProgram(GpuDevice* gpu)
		{
			sp_char filename[512];
			currentDirectory(filename, 512);
			directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
			directoryAddPath(filename, std::strlen(filename), "BoundingVolumeFactory.cl", 24);
		
			SP_FILE file;
			file.open(filename, std::ios::in);
			sp_size fileSize = file.length();
			sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
			file.read(source, fileSize);
			file.close();

			const sp_char* buildOptions = nullptr;

			gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions, &program);
			ALLOC_RELEASE(source);
		}

	public:

		API_INTERFACE static SpBoundingVolumeFactory* create(const BoundingVolumeType type);

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

		API_INTERFACE virtual BoundingVolumeType boundingVolumeType() const = 0;

		API_INTERFACE inline void initOutput(GpuDevice* gpu, cl_mem boundingVolumeGPU)
		{
			this->gpu = gpu;
			_boundingVolumesGPU = boundingVolumeGPU;
			_releaseBoundingVolumeGPU = false;
		}

		API_INTERFACE virtual void initOutput(GpuDevice* gpu, const sp_uint objectsLength) = 0;
	
		API_INTERFACE virtual void build(SpMesh* mesh, SpMeshCache* cache, const SpTransform& transform, void* boundingVolumes) = 0;

		API_INTERFACE virtual void execute(const sp_uint eventsLength, cl_event* eventsToWait, cl_event* evt) const
		{
			command->execute(1u, globalWorkSize, localWorkSize, NULL, eventsLength, eventsToWait, evt);
		}

		API_INTERFACE inline sp_double timeOfLastExecution(cl_event evt) const
		{
			return command->getTimeOfExecution(evt);
		}

		API_INTERFACE virtual void dispose()
		{
			if (command != nullptr)
			{
				sp_mem_delete(command, GpuCommand);
				command = nullptr;
			}

			if (_boundingVolumesGPU != nullptr && _releaseBoundingVolumeGPU)
			{
				gpu->releaseBuffer(_boundingVolumesGPU);
				_boundingVolumesGPU = nullptr;
			}

			if (program != nullptr)
			{
				HANDLE_OPENCL_ERROR(clReleaseProgram(program));
				program = nullptr;
			}

		}

#ifdef OPENCL_ENABLED

		API_INTERFACE virtual void init(GpuDevice* gpu, GpuBufferOpenCL* inputLengthGPU, sp_uint inputLength, GpuBufferOpenCL* meshCacheGPU, GpuBufferOpenCL* meshCacheIndexes, GpuBufferOpenCL* meshCacheVertexesLength, cl_mem transformationsGPU) = 0;

#endif

	};

}

#endif // DOP18_HEADER