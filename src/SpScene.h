#ifndef SP_SCENE_HEADER
#define SP_SCENE_HEADER

#include "SpectrumPhysics.h"
#include "SpStringId.h"
#include "SpGameObjectManager.h"
#include "SpGameObjectType.h"
#include "SpCameraManager.h"
#include "SpTransformManager.h"
#include "SpRenderableObjectManager.h"
#include "SpMeshManager.h"
#include "SpShader.h"

#define SP_SCENE_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS
{
	class SpScene
	{
	private:
		SpStringId _id;
		sp_uint _activeCamera;
		SpGameObjectManager* gameObjectManager;
		SpCameraManager* cameraManager;
		SpTransformManager* transformManager;
		SpRenderableObjectManager* _renderableObjectManager;
		SpMeshManager* _meshManager;
		
		void initGameObjectsType()
		{
			SpGameObjectType* typeCamera = sp_mem_new(SpGameObjectType)(SP_GAME_OBJECT_TYPE_CAMERA, "Camera", cameraManager);
			gameObjectsTypeList.add(SP_GAME_OBJECT_TYPE_CAMERA, typeCamera);

			SpGameObjectType* typePlane = sp_mem_new(SpGameObjectType)(SP_GAME_OBJECT_TYPE_PLANE, "Plane", _renderableObjectManager);
			gameObjectsTypeList.add(SP_GAME_OBJECT_TYPE_PLANE, typePlane);

			SpGameObjectType* typeCube = sp_mem_new(SpGameObjectType)(SP_GAME_OBJECT_TYPE_CUBE, "Cube", _renderableObjectManager);
			gameObjectsTypeList.add(SP_GAME_OBJECT_TYPE_CUBE, typeCube);
		}
		
		inline void init()
		{
			gameObjectManager = sp_mem_new(SpGameObjectManager)(1000);
			transformManager = sp_mem_new(SpTransformManager)();

			cameraManager = sp_mem_new(SpCameraManager)();
			_meshManager = sp_mem_new(SpMeshManager)();
			_renderableObjectManager = sp_mem_new(SpRenderableObjectManager)();

			initGameObjectsType();

			_activeCamera = 0;
		}

	public:
		SpMap<sp_uint, SpGameObjectType*> gameObjectsTypeList;
		SpVector<SpShader*> shaders;

		/// <summary>
		/// Create a new scene
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline SpScene()
		{
			init();
		}

		/// <summary>
		/// Create a new scene with a name
		/// </summary>
		/// <param name="name">Scene Name</param>
		/// <returns>void</returns>
		API_INTERFACE inline SpScene(const sp_char* name)
		{
			_id = name;			
			init();
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
		/// Get the mesh manager
		/// </summary>
		/// <returns>Mesh Manager</returns>
		API_INTERFACE inline SpMeshManager* meshManager() const
		{
			return _meshManager;
		}

		API_INTERFACE inline SpTransform* transform(const sp_uint index) const
		{
			return transformManager->get(index);
		}

		/// <summary>
		/// Add a game object to a Scene
		/// </summary>
		/// <param name="gameObject">New Game Object</param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject* addGameObject(const sp_uint gameObjectType, const sp_char* name = nullptr)
		{
			transformManager->add();

			SpObjectManager* manager = gameObjectsTypeList[gameObjectType]->manager();

			const sp_uint objectIndex = manager->add();
			return gameObjectManager->add(gameObjectType, objectIndex, name);
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
		/// Get the game objects by type
		/// </summary>
		/// <param name="gameObjectsIndexes">Indexes of Game Objects</param>
		/// <param name="gameObjectsIndexesLength">Length Indexes of Game Objects</param>
		/// <returns>gameObjectsIndexes parameters</returns>
		API_INTERFACE inline void gameObjectByType(const sp_uint gameObjectType, sp_size* gameObjectsIndexes, sp_size& gameObjectsIndexesLength) const
		{
			gameObjectsIndexesLength = 0;

			for (sp_uint i = 0; i < gameObjectManager->length(); i++)
				if (gameObjectManager->get(i)->type() == gameObjectType)
					gameObjectsIndexes[gameObjectsIndexesLength++] = i;
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
		/// Get the index of active camera
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint activeCameraIndex() const
		{
			return _activeCamera;
		}

		/// <summary>
		/// Get the active camera
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpCamera* activeCamera() const
		{
			return cameraManager->get(_activeCamera);
		}

		API_INTERFACE inline SpCameraManager* cameras() const
		{
			return cameraManager;
		}

		API_INTERFACE inline SpRenderableObjectManager* renderableObjectManager() const
		{
			return _renderableObjectManager;
		}

		/// <summary>
		/// Active a camera by index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline void activeCamera(const sp_uint index)
		{
			sp_assert(index <= cameraManager->length(), "IndexOutOfRangeException");
			_activeCamera = index;
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

			for (SpVectorItem<SpShader*>* item = shaders.begin(); item != nullptr; item = item->next())
			{
				item->value()->dispose();
				sp_mem_release(item->value());
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