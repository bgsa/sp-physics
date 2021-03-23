#ifndef SP_WORLD_MANAGER_HEADER
#define SP_WORLD_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpWorld.h"

namespace NAMESPACE_PHYSICS
{
	class SpWorldManager
	{
	private:
		SpWorld* _current;

	public:

		API_INTERFACE inline SpWorld* current() const
		{
			return _current;
		}

		API_INTERFACE static void init();
		
	};

	extern SpWorldManager* SpWorldManagerInstance;

}

#endif // SP_WORLD_MANAGER_HEADER