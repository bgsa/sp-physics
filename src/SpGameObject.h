#ifndef SP_GAME_OBJECT_HEADER
#define SP_GAME_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObjectType.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObject
	{
		friend class SpGameObjectManager;

	private:
		sp_uint _type;
		sp_size _index;
		SpStringId _name;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject()
		{
			_type = SP_GAME_OBJECT_TYPE_NONE;
			_index = ZERO_SIZE;
		}
		
		/// <summary>
		/// Get the type of game object
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint type() const
		{
			return _type;
		}

		/// <summary>
		/// Set the type of game object
		/// </summary>
		/// <param name="newType"></param>
		/// <returns></returns>
		API_INTERFACE inline void type(const sp_uint newType)
		{
			sp_assert(newType >= SP_GAME_OBJECT_TYPE_NONE && newType <= SP_GAME_OBJECT_TYPE_END, "InvalidArgumentException");
			_type = newType;
		}

		/// <summary>
		/// Get the index of game object in the list manager
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint index() const
		{
			return _index;
		}

		/// <summary>
		/// Get the name of game object
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_char* name() const
		{
			return _name.name;
		}

	};
}

#endif // SP_GAME_OBJECT_HEADER