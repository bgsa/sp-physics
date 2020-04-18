#ifndef SWEEP_AND_PRUNE_HEADER
#define SWEEP_AND_PRUNE_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "AlgorithmSorting.h"

#if OPENCL_ENABLED
	#include "GpuComposedCommand.h"
	#include "GpuContext.h"
	#include "Factory.h"
	#include "IFileManager.h"
	#include "GpuRadixSorting.h"
#endif

namespace NAMESPACE_PHYSICS
{

	class SweepAndPruneResultCpu
	{
	public:
		sp_uint* indexes;
		sp_uint count;

		SweepAndPruneResultCpu(sp_uint* indexes, sp_uint count)
		{
			this->indexes = indexes;
			this->count = count;
		}

		~SweepAndPruneResultCpu()
		{
			ALLOC_RELEASE(indexes);
		}
	};
	
#ifdef OPENCL_ENABLED
	class SweepAndPruneResultGpu
	{
	public:
		cl_mem indexes;
		sp_uint count;

		SweepAndPruneResultGpu(cl_mem indexes, sp_uint count)
		{
			this->indexes = indexes;
			this->count = count;
		}
	};
#endif // OPENCL_ENABLED

	class SweepAndPrune
		: public GpuComposedCommand
	{
	private:

#if OPENCL_ENABLED
		GpuDevice* gpu = NULL;
		GpuRadixSorting* radixSorting = NULL;
		GpuCommand* commandSaP = NULL;
		cl_mem output = NULL;

		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };
#endif

	public:

		///<summary>
		/// Find the collisions using Sweep and Prune method
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE static SweepAndPruneResultCpu findCollisions(AABB* aabbs, sp_uint count);

#ifdef OPENCL_ENABLED

		///<summary>
		/// Init Sweep And Prune Algorithm on GPU
		///</summary>
		API_INTERFACE SweepAndPrune* init(GpuDevice* gpu, const char* buildOptions = NULL) override;

		///<summary>
		/// Set the parameters for running SweepAndPrune Command
		///</summary>
		API_INTERFACE void setParameters(sp_float* input, sp_uint inputLength, sp_uint strider, sp_uint offset, sp_uint minPointIndex, sp_uint maxPointIndex);

		///<summary>
		/// Find the collisions using Sweep and Prune method in GPU
		/// Returns the pair indexes
		///</summary>
		API_INTERFACE cl_mem execute() override;

		///<summary>
		/// Get the length of collisions pairs detected
		///</summary>
		API_INTERFACE sp_uint fetchCollisionLength();

#endif

		API_INTERFACE ~SweepAndPrune();

	};

}

#endif // SWEEP_AND_PRUNE_HEADER