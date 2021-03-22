#ifndef SP_SCENE_MANAGER_HEADER
#define SP_SCENE_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpScene.h"

namespace NAMESPACE_PHYSICS
{

	class SpSceneManager
	{
	private:
		SpScene* _current;

		SpSceneManager()
		{
			_current = nullptr;
		}

	public:

		API_INTERFACE SpScene* current() const
		{
			return _current;
		}

	};

	extern SpSceneManager* SceneManagerInstance;

}

#endif // SP_SCENE_MANAGER_HEADER