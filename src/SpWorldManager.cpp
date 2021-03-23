#include "SpWorldManager.h"

namespace NAMESPACE_PHYSICS
{

	SpWorldManager* SpWorldManagerInstance = nullptr;

	void SpWorldManager::init()
	{
		SpWorldManagerInstance = sp_mem_new(SpWorldManager)();
		SpWorldManagerInstance->_current = sp_mem_new(SpWorld)();

		//const sp_uint maxObjects = 2u;
		//const sp_uint maxObjects = 3u;
		//const sp_uint maxObjects = 4u;
		//const sp_uint maxObjects = 16u;
		//const sp_uint maxObjects = 32u;
		//const sp_uint maxObjects = 64u;
		//const sp_uint maxObjects = 128u;
		//const sp_uint maxObjects = 256u;
		const sp_uint maxObjects = 512u;

		SpWorldManagerInstance->_current->init(maxObjects);
	}

}