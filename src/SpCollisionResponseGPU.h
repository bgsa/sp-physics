#if OPENCL_ENABLED

#ifndef SP_COLLISION_RESPONSE_GPU_HEADER
#define SP_COLLISION_RESPONSE_GPU_HEADER

#include "SpectrumPhysics.h"
#include "DOP18.h"
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
		GpuDevice* gpu = nullptr;
		GpuCommand* command = nullptr;
		cl_program program = nullptr;
		
		sp_size globalWorkSize[3] = { 0,0,0 };
		sp_size localWorkSize[3] = { 0,0,0 };

	public:
		cl_event lastEvent = nullptr;

		///<summary>
		/// Init Collision Response Algorithm on GPU
		///</summary>
		API_INTERFACE void init(GpuDevice* gpu, const char* buildOptions = NULL)
		{
			if (program != nullptr)
				return;

			this->gpu = gpu;

			SpDirectory* filename = SpDirectory::currentDirectory()
				->add(SP_DIRECTORY_OPENCL_SOURCE)
				->add("CollisionResponse.cl");

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

		///<summary>
		/// Set the parameters for running SweepAndPrune Command
		///</summary>
		API_INTERFACE void setParameters(cl_mem indexesGPU, cl_mem indexesLengthGPU, sp_uint indexesLength, cl_mem physicPropertiesGPU, cl_mem outputIndexesGPU, cl_mem outputIndexLengthGPU, sp_size outputIndexSize)
		{
			sp_size threadLength = indexesLength * SP_MAX_COLLISION_PER_OBJECT;
			globalWorkSize[0] = threadLength;
			localWorkSize[0] = gpu->getGroupLength(threadLength, indexesLength);

			command = gpu->commandManager->createCommand()
				->setInputParameter(indexesGPU, outputIndexSize)
				->setInputParameter(indexesLengthGPU, SIZEOF_UINT)
				->setInputParameter(physicPropertiesGPU, indexesLength * sizeof(SpRigidBody3D))
				->setInputParameter(outputIndexLengthGPU, SIZEOF_UINT)
				->setInputParameter(outputIndexesGPU, outputIndexSize)
				->buildFromProgram(program, "handleCollision");
		}
		
		///<summary>
		/// Find the collisions using COllision Response method in GPU
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE void execute(sp_uint previousEventsLength = ZERO_UINT, cl_event* previousEvents = nullptr)
		{
			const sp_uint zeroValue = ZERO_UINT;

			command
				->updateInputParameterValue(3u, &zeroValue)
				->execute(1, globalWorkSize, localWorkSize, 0, previousEvents, previousEventsLength);

			lastEvent = command->lastEvent;
		}

		///<summary>
		/// Get the length of collisions pairs
		///</summary>
		API_INTERFACE inline void fetchCollisionLength(sp_uint* length)
		{
			command->fetchInOutParameter<sp_uint>(3u, length);
			length[0] = divideBy2(length[0]);
		}

		///<summary>
		/// Get the collisions pairs
		///</summary>
		API_INTERFACE inline void fetchCollisions(sp_uint* indexes)
		{
			command->fetchInOutParameter<sp_uint>(4u, indexes);
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