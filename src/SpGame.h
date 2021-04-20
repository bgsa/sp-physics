#ifndef SP_GAME_HEADER
#define SP_GAME_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class SpGame
	{
	public:
		
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGame()
		{
		}

		/// <summary>
		/// Get the game type Id
		/// </summary>
		/// <returns>id</returns>
		API_INTERFACE virtual sp_int gameType() const = 0;

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE virtual void dispose() = 0;

	};

}

#endif // AABB_HEADER