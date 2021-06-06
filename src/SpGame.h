#ifndef SP_GAME_HEADER
#define SP_GAME_HEADER

#include "SpectrumPhysics.h"
#include "SpScene.h"

namespace NAMESPACE_PHYSICS
{
	class SpGame
	{
	private:
		SpVector<SpScene*>* _scenes;

	public:
		
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpGame()
		{
			_scenes = sp_mem_new(SpVector<SpScene*>);
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
		API_INTERFACE virtual void dispose()
		{
			if (_scenes != nullptr)
			{
				sp_mem_delete(_scenes, SpVector<SpScene*>);
				_scenes = nullptr;
			}
		}

		/// <summary>
		/// Get the scenes of the game
		/// </summary>
		/// <returns>Scenes</returns>
		API_INTERFACE inline SpVector<SpScene*>* scenes() const
		{
			return _scenes;
		}

		/// <summary>
		/// Check the scene is in the list by Id
		/// </summary>
		/// <param name="id">Scene ID</param>
		/// <returns>Scene if found orelse null</returns>
		API_INTERFACE inline SpScene* contaisScenes(const sp_size id)
		{
			for (SpVectorItem<SpScene*>* item = _scenes->begin(); item != nullptr; item = item->next())
				if (item->value()->id() == id)
					return item->value();

			return nullptr;
		}

		/// <summary>
		/// Add a new scene to a game
		/// </summary>
		/// <param name="name">Scene name</param>
		/// <returns>New Scene</returns>
		API_INTERFACE inline SpScene* addScenes(const char* name)
		{
			SpScene* scene = sp_mem_new(SpScene)(name);
			_scenes->add(scene);

			return scene;
		}

		/// <summary>
		/// Remove the scene by SceneItem
		/// </summary>
		/// <param name="sceneItem">SceneItem</param>
		/// <returns>void</returns>
		API_INTERFACE inline void removeScene(SpVectorItem<SpScene*>* sceneItem)
		{
			_scenes->remove(sceneItem);
			sp_mem_delete(sceneItem->value(), SpScene);
		}

	};

}

#endif // AABB_HEADER