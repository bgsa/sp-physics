#if OPENCL_ENABLED

#ifndef SP_COLLISION_RESPONSE_GPU_HEADER
#define SP_COLLISION_RESPONSE_GPU_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "SpRigidBody3D.h"
#include "FileSystem.h"

namespace NAMESPACE_PHYSICS
{
	class SpCollisionResponseGPU
		: public Object
	{
#define SP_MAX_COLLISION_PER_OBJECT 8
	private:
		GpuDevice* gpu;
		GpuCommand* command;
		cl_program program;
		
		sp_size globalWorkSize[3] = { 0,0,0 };
		sp_size localWorkSize[3] = { 0,0,0 };

	public:

		API_INTERFACE SpCollisionResponseGPU()
			: Object()
		{
			gpu = nullptr;
			command = nullptr;
			program = nullptr;
		}

		///<summary>
		/// Init Collision Response Algorithm on GPU
		///</summary>
		API_INTERFACE void init(GpuDevice* gpu, const char* buildOptions = NULL)
		{
			if (program != nullptr)
				return;

			this->gpu = gpu;

			sp_char filename[512];
			currentDirectory(filename, 512);
			directoryAddPath(filename, std::strlen(filename), SP_DIRECTORY_OPENCL_SOURCE, SP_DIRECTORY_OPENCL_SOURCE_LENGTH);
			directoryAddPath(filename, std::strlen(filename), "CollisionResponse.cl", 20);

			SP_FILE file;
			file.open(filename, std::ios::in);
			const sp_size fileSize = file.length();
			sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
			file.read(source, fileSize);
			file.close();

			gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, buildOptions, &program);
			
			ALLOC_RELEASE(source);
		}

		///<summary>
		/// Set the parameters for running SweepAndPrune Command
		///</summary>
		API_INTERFACE void setParameters(cl_mem indexesGPU, cl_mem indexesLengthGPU, sp_uint indexesLength, cl_mem rigidBody3DGPU, cl_mem outputIndexesGPU, cl_mem outputIndexLengthGPU, sp_size outputIndexSize)
		{
			sp_size threadLength = indexesLength * SP_MAX_COLLISION_PER_OBJECT;
			globalWorkSize[0] = threadLength;
			localWorkSize[0] = (sp_size) gpu->getGroupLength(threadLength, indexesLength);

			command = gpu->commandManager->createCommand()
				->setInputParameter(indexesGPU, outputIndexSize)
				->setInputParameter(indexesLengthGPU, sizeof(sp_uint))
				->setInputParameter(rigidBody3DGPU, indexesLength * sizeof(SpRigidBody3D))
				->setInputParameter(outputIndexLengthGPU, sizeof(sp_uint))
				->setInputParameter(outputIndexesGPU, outputIndexSize)
				->buildFromProgram(program, "handleCollision");
		}
		
		///<summary>
		/// Find the collisions using COllision Response method in GPU
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE void execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt)
		{
			const sp_uint zeroValue = ZERO_UINT;

			command
				->updateInputParameterValue(3u, &zeroValue, previousEventsLength, previousEvents, NULL)
				->execute(1, globalWorkSize, localWorkSize, 0, previousEventsLength, previousEvents, evt);
		}

		///<summary>
		/// Get the length of collisions pairs
		///</summary>
		API_INTERFACE inline void fetchCollisionLength(sp_uint* length, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt)
		{
			command->fetchInOutParameter<sp_uint>(3u, length, previousEventsLength, previousEvents, evt);
			length[0] = divideBy2(length[0]);
		}

		///<summary>
		/// Get the collisions pairs
		///</summary>
		API_INTERFACE inline void fetchCollisions(sp_uint* indexes, const sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt)
		{
			command->fetchInOutParameter<sp_uint>(4u, indexes, previousEventsLength, previousEvents, evt);
		}

		///<summary>
		///</summary>
		API_INTERFACE inline void updateParameters(cl_mem indexes, cl_mem length)
		{
			command
				->updateInputParameter(0u, indexes)
				->updateInputParameter(1u, length);
		}

		/// <summary>
		/// Release allocated resources from this object
		/// </summary>
		API_INTERFACE void dispose() override
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

		/// <summary>
		/// Description of the object
		/// </summary>
		API_INTERFACE const sp_char* toString() override
		{
			return "Collision Response GPU";
		}

		~SpCollisionResponseGPU() { dispose(); }

#undef SP_MAX_COLLISION_PER_OBJECT
	};

}

#endif // SP_COLLISION_RESPONSE_GPU_HEADER

#endif // OPENCL_ENABLED