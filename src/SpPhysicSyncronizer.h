#ifndef SP_PHYSIC_SYNCRONIZER_HEADER
#define SP_PHYSIC_SYNCRONIZER_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpPhysicSyncronizer
	{
	public:
		
		API_INTERFACE virtual void sync(const sp_uint objectIndex, const Vec3& position, const Quat& orientation) = 0;

	};

}

#endif // SP_PHYSIC_SYNCRONIZER_HEADER