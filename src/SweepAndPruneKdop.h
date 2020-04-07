#ifndef SWEEP_AND_PRUNE_KDOP_HEADER
#define SWEEP_AND_PRUNE_KDOP_HEADER

#include "SweepAndPrune.h"
#include "DOP18.h"

namespace NAMESPACE_PHYSICS
{
	class SweepAndPruneKdop
		: public SweepAndPrune
	{
	private:

#if OPENCL_ENABLED
		GpuRadixSorting* radixSorting;
#endif

	public:

		/// <summary>
		/// Find the collisions using Sweep and Prune method
		/// Returns the pair indexes
		/// </summary>
		API_INTERFACE static SweepAndPruneResultCpu findCollisions(DOP18* kdops, size_t count);

#ifdef OPENCL_ENABLED

		/// <summary>
		/// Init Sweep And Prune Algorithm on GPU
		/// </summary>
		API_INTERFACE void init(GpuDevice* gpu, const char* buildOptions);

		/// <summary>
		/// Find the collisions using Sweep and Prune method On GPU
		/// Returns the pair indexes
		/// </summary>
		API_INTERFACE SweepAndPruneResultGpu findCollisions(GpuDevice* gpu, DOP18* aabbs, size_t count);

#endif // OPENCL_ENABLED

	};
}
#endif // SWEEP_AND_PRUNE_KDOP_HEADER