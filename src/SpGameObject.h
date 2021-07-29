#ifndef SP_GAME_OBJECT_HEADER
#define SP_GAME_OBJECT_HEADER

#include "SpectrumPhysics.h"
#include "SpStringId.h"

namespace NAMESPACE_PHYSICS
{
	class SpGameObject
	{
		friend class SpGameObjectManager;

	private:
		sp_uint _type;
		sp_size _index;
		sp_size _renderableObjectIndex;
		sp_size _managerIndex;
		SpStringId _name;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject()
		{
			_type = SP_UINT_MAX;
			_index = 0;
			_renderableObjectIndex = SP_UINT_MAX;
			_managerIndex = 0;
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
			//sp_assert(newType >= SP_GAME_OBJECT_TYPE_NONE && newType <= SP_GAME_OBJECT_TYPE_END, "InvalidArgumentException");
			_type = newType;
		}

		/// <summary>
		/// Get the index of game object in the list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint index() const
		{
			return _index;
		}

		/// <summary>
		/// Check this game object is renderable
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint isRenderableObject() const
		{
			return _renderableObjectIndex != SP_UINT_MAX;
		}

		/// <summary>
		/// Get the renderable object index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint renderableObjectIndex() const
		{
			return _renderableObjectIndex;
		}

		/// <summary>
		/// Set the renderable object to this game object
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void renderableObjectIndex(const sp_uint index)
		{
			sp_assert(index != SP_UINT_MAX, "InvalidArgumentException");
			_renderableObjectIndex = index;
		}


		/// <summary>
		/// Get the index of game object in the list Manager
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint managerIndex() const
		{
			return _managerIndex;
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