#ifndef SP_SCENE_HEADER
#define SP_SCENE_HEADER

#include "SpectrumPhysics.h"
#include "SpStringId.h"
#include "SpGameObjectManager.h"
#include "SpGameObjectType.h"
#include "SpCameraManager.h"
#include "SpTransformManager.h"
#include "SpRenderableObjectManager.h"
#include "SpLightingManager.h"
#include "SpMeshManager.h"
#include "SpShader.h"
#include "SpGameObjectFactoryPlane.h"
#include "SpGameObjectFactoryCube.h"

#define SP_SCENE_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS
{
	class SpScene
	{
	private:
		SpStringId _id;
		sp_bool _loaded;
		sp_uint _activeCamera;
		SpGameObjectManager* _gameObjectManager;
		SpCameraManager* cameraManager;
		SpTransformManager* _transformManager;
		SpRenderableObjectManager* _renderableObjectManager;
		SpMeshManager* _meshManager;
		SpLightingManager* _lightingManager;
		
		void initGameObjectsType()
		{
			SpGameObjectFactory* planeFactory = sp_mem_new(SpGameObjectFactoryPlane)();
			planeFactory->init(this);

			SpGameObjectType* typePlane = sp_mem_new(SpGameObjectType)(SP_GAME_OBJECT_TYPE_PLANE, "Plane", planeFactory);
			gameObjectsTypeList.add(SP_GAME_OBJECT_TYPE_PLANE, typePlane);

			SpGameObjectFactory* cubeFactory = sp_mem_new(SpGameObjectFactoryCube)();
			cubeFactory->init(this);

			SpGameObjectType* typeCube = sp_mem_new(SpGameObjectType)(SP_GAME_OBJECT_TYPE_CUBE, "Cube", cubeFactory);
			gameObjectsTypeList.add(SP_GAME_OBJECT_TYPE_CUBE, typeCube);

			// load custom game object types from plug-ins
		}
		
		inline void init()
		{
			_gameObjectManager = sp_mem_new(SpGameObjectManager)();
			_transformManager = sp_mem_new(SpTransformManager)();

			cameraManager = sp_mem_new(SpCameraManager)();
			_meshManager = sp_mem_new(SpMeshManager)();
			_renderableObjectManager = sp_mem_new(SpRenderableObjectManager)();
			_lightingManager = sp_mem_new(SpLightingManager)();

			initGameObjectsType();

			_activeCamera = 0;
			_loaded = false;
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
		/// Get the lighting manager
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpLightingManager* lightingManager() const
		{
			return _lightingManager;
		}

		/// <summary>
		/// Get the mesh manager
		/// </summary>
		/// <returns>Mesh Manager</returns>
		API_INTERFACE inline SpMeshManager* meshManager() const
		{
			return _meshManager;
		}

		/// <summary>
		/// Get transform by game object index
		/// </summary>
		/// <param name="index">Game Object Index</param>
		/// <returns>Transform</returns>
		API_INTERFACE inline SpTransform* transform(const sp_uint index) const
		{
			return _transformManager->get(index);
		}

		/// <summary>
		/// Get transform manager
		/// </summary>
		/// <returns>Transform Manager</returns>
		API_INTERFACE inline SpTransformManager* transformManager() const
		{
			return _transformManager;
		}

		/// <summary>
		/// Add a game object to a Scene
		/// </summary>
		/// <param name="gameObject">New Game Object</param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject* addGameObject(const sp_uint gameObjectType, const sp_char* name = nullptr)
		{
			_transformManager->add();

			const sp_uint gameObjIndex = _gameObjectManager->add();
			SpGameObject* gameObj = _gameObjectManager->get(gameObjIndex);
			gameObj->type(gameObjectType);

			if (name != nullptr)
				_gameObjectManager->name(gameObjIndex, name, std::strlen(name));
			else
			{
				sp_char tempName[10];
				sp_uint length1;
				convert(gameObjIndex, tempName, length1);
				tempName[length1 + 1] = END_OF_STRING;

				_gameObjectManager->name(gameObjIndex, tempName, length1 + 1);
			}

			return gameObj;
		}

		/// <summary>
		/// Remove a game object of the Scene by index
		/// </summary>
		/// <param name="index">Index of the game object</param>
		/// <returns></returns>
		API_INTERFACE inline void removeGameObject(const sp_uint index)
		{
			SpGameObject* gameObj = gameObject(index);

			if (gameObj->isRenderableObject())
			{
				const sp_uint renderableObjIndex = gameObj->renderableObjectIndex();

				renderableObjectManager()->remove(renderableObjIndex);

				// fix the objects indexes
				for (sp_uint i = index; i < _renderableObjectManager->length(); i++)
					_renderableObjectManager->get(i)->gameObject(_renderableObjectManager->get(i)->gameObject() - 1);

				for (sp_uint i = 0; i < _gameObjectManager->length(); i++)
				{
					const sp_uint currentRenderableObjIndex = _gameObjectManager->get(i)->renderableObjectIndex();

					if (currentRenderableObjIndex > renderableObjIndex)
						_gameObjectManager->get(i)->renderableObjectIndex(currentRenderableObjIndex - 1);
				}
			}

			_transformManager->remove(index);
			_gameObjectManager->remove(index);
		}

		/// <summary>
		/// Get a game object manager
		/// </summary>
		/// <returns>Game Object Manager</returns>
		API_INTERFACE inline SpGameObjectManager* gameObjectManager() const
		{
			return _gameObjectManager;
		}

		/// <summary>
		/// Get a game object of the Scene
		/// </summary>
		/// <param name="index">Index</param>
		/// <returns>Game Object</returns>
		API_INTERFACE inline SpGameObject* gameObject(const sp_uint index) const
		{
			return _gameObjectManager->get(index);
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

			for (sp_uint i = 0; i < _gameObjectManager->length(); i++)
				if (_gameObjectManager->get(i)->type() == gameObjectType)
					gameObjectsIndexes[gameObjectsIndexesLength++] = i;
		}

		/// <summary>
		/// Get the game objects length
		/// </summary>
		/// <returns>Length</returns>
		API_INTERFACE inline sp_uint gameObjectsLength() const
		{
			return _gameObjectManager->length();
		}

		/// <summary>
		/// Add a camera to a Scene
		/// </summary>
		/// <param name="name">name (optional)</param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint addCamera(const sp_char* name = nullptr, const sp_size nameLength = 0)
		{
			const sp_uint cameraId = cameraManager->add();

			cameraManager->name(cameraId, name, nameLength);

			return cameraId;
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

		API_INTERFACE inline SpCameraManager* camerasManager() const
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
		/// Update this scene
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void update()
		{
			_transformManager->updateGpuBuffer(); // update transform in GPU
			_lightingManager->updateGpuBuffer(); // update lighting in GPU
			_renderableObjectManager->updateGpuBuffer(); // update materials in GPU
		}

		/// <summary>
		/// Check the scene is loaded
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isLoaded() const
		{
			return _loaded;
		}

		/// <summary>
		/// Load this scene
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void load()
		{
			sp_assert(!_loaded, "InvalidOperationException");
			_loaded = true;
		}

		/// <summary>
		/// Load this scene
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void unload()
		{
			sp_assert(_loaded, "InvalidOperationException");
			_loaded = false;
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

			if (_lightingManager != nullptr)
			{
				sp_mem_delete(_lightingManager, SpLightingManager);
				_lightingManager = nullptr;
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

			if (_gameObjectManager != nullptr)
			{
				sp_mem_delete(_gameObjectManager, SpGameObjectManager);
				_gameObjectManager = nullptr;
			}
		}

		~SpScene()
		{
			dispose();
		}

	};
}

#endif // SP_SCENE_HEADER