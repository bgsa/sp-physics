#include "SpWorldManager.h"

namespace NAMESPACE_PHYSICS
{

	SpWorldManager* SpWorldManagerInstance = nullptr;

	void SpWorldManager::init()
	{
		SpWorldManagerInstance = sp_mem_new(SpWorldManager)();
		
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

	SpWorldManager::SpWorldManager()
	{
		_worlds = sp_mem_new(SpVector<SpWorld*>)();

		_current = sp_mem_new(SpWorld)();
		strcpy(_current->name, "World 1");

		_worlds->add(_current);
	}

	sp_bool SpWorldManager::isInitialized()
	{
		return SpWorldManagerInstance != nullptr;
	}

	void SpWorldManager::dispose()
	{
		if (SpWorldManagerInstance == nullptr)
			return;

		if (SpWorldManagerInstance->_worlds != nullptr)
		{
			sp_mem_delete(SpWorldManagerInstance->_worlds, SpVector<SpWorld*>);
			SpWorldManagerInstance->_worlds = nullptr;
		}
		SpWorldManagerInstance->_current = nullptr;

		sp_mem_delete(SpWorldManagerInstance, SpWorldManager);
		SpWorldManagerInstance = nullptr;
	}

}