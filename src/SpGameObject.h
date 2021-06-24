#ifndef SP_GAME_OBJECT_HEADER
#define SP_GAME_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObjectType.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObject
	{
	private:
		sp_uint _type;
		sp_size _indexListManager;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject()
		{
			_type = SP_GAME_OBJECT_TYPE_NONE;
			_indexListManager = ZERO_SIZE;
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

	};
}

#endif // SP_GAME_OBJECT_HEADER