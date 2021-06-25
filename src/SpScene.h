#ifndef SP_SCENE_HEADER
#define SP_SCENE_HEADER

#include "SpectrumPhysics.h"
#include "SpStringId.h"
#include "SpGameObjectManager.h"
#include "SpGameObjectType.h"
#include "SpCameraManager.h"

#define SP_SCENE_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS
{
	class SpScene
	{
	private:
		SpStringId _id;
		SpGameObjectManager* gameObjectManager;
		SpCameraManager* cameraManager;

		void initGameObjectsType()
		{
			SpGameObjectType* typeCamera = sp_mem_new(SpGameObjectType)(SP_GAME_OBJECT_TYPE_CAMERA, "Camera", cameraManager);
			gameObjectsTypeList.add(SP_GAME_OBJECT_TYPE_CAMERA, typeCamera);
		}
		
	public:
		SpMap<sp_uint, SpGameObjectType*> gameObjectsTypeList;

		/// <summary>
		/// Create a new scene
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpScene()
		{
			gameObjectManager = sp_mem_new(SpGameObjectManager)(1000);
			cameraManager = sp_mem_new(SpCameraManager)();
			initGameObjectsType();
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
			initGameObjectsType();
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
		API_INTERFACE inline SpGameObject& addGameObject(const sp_uint gameObjectType, const sp_char* name = nullptr)
		{
			SpObjectManager* manager = gameObjectsTypeList[gameObjectType]->manager();

			const sp_uint objectIndex = manager->add();
			return gameObjectManager->add(SP_GAME_OBJECT_TYPE_CAMERA, objectIndex, name);
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
		API_INTERFACE inline SpGameObject* gameObject(const sp_uint index) const
		{
			return gameObjectManager->get(index);
		}

		/// <summary>
		/// Get the game objects length
		/// </summary>
		/// <returns>Length</returns>
		API_INTERFACE inline sp_uint gameObjectsLength() const
		{
			return gameObjectManager->length();
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

			for (SpVectorItem<SpPair<sp_uint, SpGameObjectType*>>* item = gameObjectsTypeList.begin(); item != nullptr; item = item->next())
			{
				sp_mem_delete(item->value().value, SpGameObjectType);
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