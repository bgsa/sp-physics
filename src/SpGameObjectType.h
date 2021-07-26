#ifndef SP_GAME_OBJECT_TYPE_HEADER
#define SP_GAME_OBJECT_TYPE_HEADER

#include "SpMap.h"
#include "SpObjectManager.h"
#include "SpGameObjectFactory.h"

#define SP_GAME_OBJECT_TYPE_NONE   0
#define SP_GAME_OBJECT_TYPE_PLANE  1
#define SP_GAME_OBJECT_TYPE_CUBE   2
#define SP_GAME_OBJECT_TYPE_SPHERE 3
#define SP_GAME_OBJECT_TYPE_CUSTOM 4
#define SP_GAME_OBJECT_TYPE_END    5

namespace NAMESPACE_PHYSICS
{
	class SpGameObjectType
	{
	private:		
		sp_uint _type;
		SpObjectManager* _manager;
		SpGameObjectFactory* _factory;
		sp_char* _name;

	public:

		API_INTERFACE SpGameObjectType(const sp_uint type, SpObjectManager* manager, SpGameObjectFactory* factory);

		API_INTERFACE SpGameObjectType(const sp_uint type, const sp_char* name, SpObjectManager* manager, SpGameObjectFactory* factory);

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

		API_INTERFACE inline SpGameObjectFactory* factory() const
		{
			return _factory;
		}

		API_INTERFACE inline void name(const sp_char* newName, const sp_size newNameLength)
		{
			_name = sp_mem_new_array(sp_char, newNameLength + 1);
			std::memcpy(_name, newName, newNameLength);
			_name[newNameLength + 1] = END_OF_STRING;
		}

		API_INTERFACE void dispose();

		~SpGameObjectType()
		{
			dispose();
		}

	};

}

#endif // SP_GAME_OBJECT_TYPE_HEADER