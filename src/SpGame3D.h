#ifndef SP_GAME_3D_HEADER
#define SP_GAME_3D_HEADER

#include "SpGame.h"
#include "SpWorldManager.h"

namespace NAMESPACE_PHYSICS
{

	class SpGame3D
		: public SpGame
	{
	public:

		/// <summary>
		/// Default Constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpGame3D()
			: SpGame()
		{
			if (!SpWorldManager::isInitialized())
				SpWorldManager::init();
		}

		API_INTERFACE inline sp_int gameType() const override
		{
			return 3;
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose() override
		{
			if (SpWorldManager::isInitialized())
				SpWorldManager::dispose();
		}

	};

}

#endif // SP_GAME_3D_HEADER