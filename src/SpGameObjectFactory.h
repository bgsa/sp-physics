#ifndef SP_GAME_OBJECT_FACTORY_HEADER
#define SP_GAME_OBJECT_FACTORY_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObjectFactory
	{
	public:

		API_INTERFACE virtual void init(SpScene* scene) = 0;

		API_INTERFACE virtual sp_uint create(SpScene* scene) = 0;

		API_INTERFACE virtual void dispose() = 0;

	};
}

#endif // SP_GAME_OBJECT_FACTORY_HEADER