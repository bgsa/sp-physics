#ifndef SP_GAME_2D_HEADER
#define SP_GAME_2D_HEADER

#include "SpGame.h"

namespace NAMESPACE_PHYSICS
{

	class SpGame2D
		: public SpGame
	{
	public:

		/// <summary>
		/// Default Constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpGame2D()
			: SpGame()
		{
		}

		API_INTERFACE inline sp_int gameType() const override
		{
			return 2;
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose() override
		{
		}

	};

}

#endif // SP_GAME_2D_HEADER