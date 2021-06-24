#ifndef SP_GAME_OBJECT_TYPE_HEADER
#define SP_GAME_OBJECT_TYPE_HEADER

#include "SpMap.h"
#include "SpObjectManager.h"

#define SP_GAME_OBJECT_TYPE_NONE   0
#define SP_GAME_OBJECT_TYPE_CAMERA 1
#define SP_GAME_OBJECT_TYPE_END    2

namespace NAMESPACE_PHYSICS
{
	class SpGameObjectType
	{
	private:		
		sp_uint _type;
		SpObjectManager* _manager;
		sp_char* _name;

	public:

		API_INTERFACE inline SpGameObjectType(const sp_uint type, SpObjectManager* manager)
		{
			_type = type;
			_manager = manager;
			_name = nullptr;
		}

		API_INTERFACE inline SpGameObjectType(const sp_uint type, const sp_char* name, SpObjectManager* manager)
		{
			_type = type;
			_manager = manager;
			this->name(name, std::strlen(name));
		}

		API_INTERFACE inline sp_uint type()
		{
			return _type;
		}

		API_INTERFACE inline SpObjectManager* manager()
		{
			return _manager;
		}

		API_INTERFACE inline sp_char* name()
		{
			return _name;
		}

		API_INTERFACE inline void name(const sp_char* newName, const sp_size newNameLength)
		{
			_name = sp_mem_new_array(sp_char, newNameLength + 1);
			std::memcpy(_name, newName, newNameLength);
			_name[newNameLength + 1] = END_OF_STRING;
		}

		API_INTERFACE inline void dispose()
		{
			if (_name != nullptr)
			{
				sp_mem_release(_name);
				_name = nullptr;
			}
		}

		~SpGameObjectType()
		{
			dispose();
		}

	};

}

#endif // SP_GAME_OBJECT_TYPE_HEADER