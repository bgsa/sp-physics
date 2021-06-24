#ifndef SP_SCENE_HEADER
#define SP_SCENE_HEADER

#include "SpectrumPhysics.h"
#include "SpStringId.h"
#include "SpGameObjectManager.h"
#include "SpCameraManager.h"

namespace NAMESPACE_PHYSICS
{
	class SpScene
	{
	private:
		SpStringId _id;
		SpGameObjectManager* gameObjectManager;
		SpCameraManager* cameraManager;

		//IObjectManager** managers; // mapeia game object TYPE pelo seu gerenciador

	public:

		/// <summary>
		/// Create a new scene
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpScene()
		{
			gameObjectManager = sp_mem_new(SpGameObjectManager)(1000);
			cameraManager = sp_mem_new(SpCameraManager)();
		}

		/// <summary>
		/// Create a new scene with a name
		/// </summary>
		/// <param name="name">Scene Name</param>
		/// <returns>void</returns>
		API_INTERFACE inline SpScene(const sp_char* name)
		{
			_id = name;
			gameObjectManager = sp_mem_new(SpGameObjectManager)(1000);
			cameraManager = sp_mem_new(SpCameraManager)();
		}

		/// <summary>
		/// Get the ID of the Scene
		/// </summary>
		/// <returns>ID</returns>
		API_INTERFACE inline sp_size id() const
		{
			return _id.id;
		}

		/// <summary>
		/// Get the Name of the Scene
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_char* name() const
		{
			return _id.name;
		}

		/// <summary>
		/// Add a game object to a Scene
		/// </summary>
		/// <param name="gameObject">New Game Object</param>
		/// <returns></returns>
		API_INTERFACE inline void addGameObject(const SpGameObject& gameObject)
		{
			gameObjectManager->add(gameObject);
		}

		/// <summary>
		/// Remove a game object of the Scene by index
		/// </summary>
		/// <param name="index">Index of the game object</param>
		/// <returns></returns>
		API_INTERFACE inline void removeGameObject(const sp_uint index)
		{
			gameObjectManager->remove(index);
		}

		/// <summary>
		/// Get a game object of the Scene
		/// </summary>
		/// <param name="index">Index</param>
		/// <returns>Game Object</returns>
		API_INTERFACE inline SpGameObject& getGameObject(const sp_uint index) const
		{
			return gameObjectManager->get(index);
		}

		/// <summary>
		/// Dispose all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose()
		{
			if (cameraManager != nullptr)
			{
				sp_mem_delete(cameraManager, SpCameraManager);
				cameraManager = nullptr;
			}

			if (gameObjectManager != nullptr)
			{
				sp_mem_delete(gameObjectManager, SpGameObjectManager);
				gameObjectManager = nullptr;
			}
		}

		~SpScene()
		{
			dispose();
		}

	};
}

#endif // SP_SCENE_HEADER