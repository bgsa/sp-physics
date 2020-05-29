#ifndef SP_PHYSIC_SIMULATOR_HEADER
#define SP_PHYSIC_SIMULATOR_HEADER

#include "SpectrumPhysics.h"
#include "GpuContext.h"
#include "DOP18.h"
#include "SweepAndPrune.h"
#include "SpEventDispatcher.h"
#include "CL/cl.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSimulator
	{
	private:
		GpuDevice* gpu;
		SweepAndPrune* sap;

		cl_mem boundingVolumeBuffer = nullptr;
		DOP18* boundingVolumes;
		sp_uint boundingVolumesLengthAllocated;
		sp_uint boundingVolumesLength;

		SpPhysicSimulator() { }

	public:

		API_INTERFACE static SpPhysicSimulator* instance();

		API_INTERFACE static void init(sp_uint objectsLength);

		API_INTERFACE BoundingVolume* alloc(sp_uint length);

		API_INTERFACE void run();

		API_INTERFACE void dispose();

		~SpPhysicSimulator();

	};
}

#endif // SP_PHYSIC_SIMULATOR_HEADER