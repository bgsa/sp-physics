#ifndef SP_WORLD_MANAGER_HEADER
#define SP_WORLD_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpWorld.h"
#include "SpVector.h"

namespace NAMESPACE_PHYSICS
{
	class SpWorldManager
	{
	private:
		SpWorld* _current;
		SpVector<SpWorld*>* _worlds;

	public:

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpWorldManager();

		/// <summary>
		/// Get the current world
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpWorld* current() const
		{
			return _current;
		}

		/// <summary>
		/// Get the worlds
		/// </summary>
		/// <returns>World List</returns>
		API_INTERFACE inline SpVector<SpWorld*>* worlds() const
		{
			return _worlds;
		}

		/// <summary>
		/// Initialize the world manager
		/// </summary>
		/// <returns></returns>
		API_INTERFACE static void init();

		/// <summary>
		/// Check the world manager is initialized
		/// </summary>
		/// <returns></returns>
		API_INTERFACE static sp_bool isInitialized();

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE static void dispose();
		
	};

	extern SpWorldManager* SpWorldManagerInstance;

}

#endif // SP_WORLD_MANAGER_HEADER