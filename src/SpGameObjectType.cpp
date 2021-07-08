#include "SpGameObjectType.h"

namespace NAMESPACE_PHYSICS
{

	SpGameObjectType::SpGameObjectType(const sp_uint type, SpObjectManager* manager, SpGameObjectFactory* factory)
	{
		_type = type;
		_manager = manager;
		_factory = factory;
		_name = nullptr;
	}

	SpGameObjectType::SpGameObjectType(const sp_uint type, const sp_char* name, SpObjectManager* manager, SpGameObjectFactory* factory)
	{
		_type = type;
		_manager = manager;
		_factory = factory;
		this->name(name, std::strlen(name));
	}

	void SpGameObjectType::dispose()
	{
		if (_name != nullptr)
		{
			sp_mem_release(_name);
			_name = nullptr;
		}

		if (_factory != nullptr)
		{
			_factory->dispose();
			sp_mem_release(_factory);
			_factory = nullptr;
		}
	}

}