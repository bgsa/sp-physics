#ifndef SWEEP_AND_PRUNE_HEADER
#define SWEEP_AND_PRUNE_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "DOP18.h"
#include "AlgorithmSorting.h"

#if OPENCL_ENABLED
	#include "GpuComposedCommand.h"
	#include "GpuContext.h"
	#include "GpuRadixSorting.h"
	#include <FileSystem.h>
#endif

namespace NAMESPACE_PHYSICS
{
#define SP_SAP_MAX_COLLISION_PER_OBJECT 8

	class SweepAndPruneResult
	{
	public:
		sp_uint* indexes;
		sp_uint length;

		SweepAndPruneResult()
		{
			this->indexes = nullptr;
			this->length = ZERO_UINT;
		}

		SweepAndPruneResult(sp_uint* indexes, sp_uint count)
		{
			this->indexes = indexes;
			this->length = count;
		}

		~SweepAndPruneResult()
		{
			if (indexes != nullptr)
			{
				sp_mem_release(indexes);
				indexes = nullptr;
			}
		}
	};
	
	class SweepAndPrune
		: public GpuComposedCommand
	{
	private:

#if OPENCL_ENABLED
		GpuDevice* gpu = nullptr;
		cl_program sapProgram = nullptr;

		GpuRadixSorting* radixSorting = nullptr;
		GpuCommand* commandSaPCollisions = nullptr;

		cl_mem indexesLengthGPU = nullptr;
		cl_mem indexesGPU = nullptr;

		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };

		void initIndexes(sp_uint inputLength);
#endif // OPENCL_ENABLED

	public:

		///<summary>
		/// Find the collisions using Sweep and Prune method
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE static SweepAndPruneResult findCollisions(AABB* aabbs, sp_uint length);

		///<summary>
		/// Find the collisions using Sweep and Prune method
		/// Returns the pair indexes and length in the last parameter
		///</summary>
		API_INTERFACE static void findCollisions(DOP18* kdops, sp_uint* indexes, sp_uint length, SweepAndPruneResult* resultCpu);

#ifdef OPENCL_ENABLED

		///<summary>
		/// Init Sweep And Prune Algorithm on GPU
		///</summary>
		API_INTERFACE SweepAndPrune* init(GpuDevice* gpu, const char* buildOptions = NULL) override;

		///<summary>
		/// Set the parameters for running SweepAndPrune Command
		///</summary>
		API_INTERFACE void setParameters(cl_mem input, sp_uint inputLength, sp_uint strider, sp_uint offset, sp_size axisLength, cl_mem physicProperties, const sp_uint physicPropertySize, cl_mem outputIndexLength, cl_mem outputIndex, const sp_char* commandName);

		///<summary>
		/// Find the collisions using Sweep and Prune method in GPU
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE cl_mem execute(sp_uint previousEventsLength = ZERO_UINT, cl_event* previousEvents = nullptr) override;

		///<summary>
		/// Get the length of collisions pairs detected
		///</summary>
		API_INTERFACE sp_uint fetchCollisionLength();

		/// <summary>
		/// Get the colision index pairs
		/// </summary>
		API_INTERFACE void fetchCollisionIndexes(sp_uint* output) const;

		///<summary>
		/// Update physic data on GPU
		///</summary>
		API_INTERFACE inline void updatePhysicProperties(void* physicProperties)
		{
			commandSaPCollisions->updateInputParameterValue(1u, physicProperties);
			lastEvent = commandSaPCollisions->lastEvent;
		}

		///<summary>
		/// Update bouding volume parameter
		///</summary>
		API_INTERFACE inline void updateBoundingVolumes(void* boudingVolumes)
		{
			commandSaPCollisions->updateInputParameterValue(0u, boudingVolumes);
			lastEvent = commandSaPCollisions->lastEvent;
		}

#endif // OPENCL_ENABLED

		/// <summary>
		/// Release allocated resources from this object
		/// </summary>
		API_INTERFACE void dispose() override
		{
#if OPENCL_ENABLED
			if (radixSorting != nullptr)
			{
				sp_mem_delete(radixSorting, GpuRadixSorting);
				radixSorting = nullptr;
			}

			if (commandSaPCollisions != nullptr)
			{
				sp_mem_delete(commandSaPCollisions, GpuCommand);
				commandSaPCollisions = nullptr;
			}

			if (indexesGPU != nullptr)
			{
				gpu->releaseBuffer(indexesGPU);
				indexesGPU = nullptr;
			}
			
			if (indexesLengthGPU != nullptr)
			{
				gpu->releaseBuffer(indexesLengthGPU);
				indexesLengthGPU = nullptr;
			}
#endif // OPENCL_ENABLED
		}

		/// <summary>
		/// Description of the object
		/// </summary>
		API_INTERFACE const sp_char* toString() override
		{
			return "Sweep And Prune";
		}

		~SweepAndPrune() { dispose(); }

	};

}

#endif // SWEEP_AND_PRUNE_HEADER