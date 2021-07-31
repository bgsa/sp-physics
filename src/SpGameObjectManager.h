#ifndef SP_GAME_OBJECT_MANAGER_HEADER
#define SP_GAME_OBJECT_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObject.h"
#include "SpTransform.h"

#ifdef OPENCL_ENABLED
	#include "GpuBufferOpenCL.h"
#endif

#define SP_GAME_OBJECT_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS 
{

	class SpGameObjectManager
	{
	private:
		sp_uint _length;
		SpGameObject* _gameObjects;
		sp_char* _names;

#ifdef OPENCL_ENABLED
		GpuBufferOpenCL* _gpuGameObjects;
#endif

		inline void addName()
		{
			if (_names != nullptr)
			{
				const sp_size namesSize = sizeof(sp_char) * (_length - 1) * SP_GAME_OBJECT_NAME_MAX_LENGTH;

				void* temp = ALLOC_SIZE(namesSize);
				std::memcpy(temp, _names, namesSize);

				sp_mem_release(_names);

				_names = sp_mem_new_array(sp_char, _length * SP_GAME_OBJECT_NAME_MAX_LENGTH);

				std::memcpy(_names, temp, namesSize);
				ALLOC_RELEASE(temp);
			}
			else
			{
				_names = sp_mem_new_array(sp_char, SP_GAME_OBJECT_NAME_MAX_LENGTH);
			}

			std::memset(&_names[(_length - 1) * SP_GAME_OBJECT_NAME_MAX_LENGTH], 0, SP_GAME_OBJECT_NAME_MAX_LENGTH);
		}

		inline void removeName(const sp_uint index)
		{
			if (_length != 0)
			{
				void* previousNames = nullptr, *nextNames = nullptr;

				const sp_size previousNamesSize = sizeof(sp_char) * index * SP_GAME_OBJECT_NAME_MAX_LENGTH;
				if (previousNamesSize != 0)
				{
					previousNames = ALLOC_SIZE(previousNamesSize);
					std::memcpy(previousNames, _names, previousNamesSize);
				}

				const sp_size nextNamesSize = sizeof(sp_char) * (_length - index) * SP_GAME_OBJECT_NAME_MAX_LENGTH;
				if (nextNamesSize != 0)
				{
					nextNames = ALLOC_SIZE(nextNamesSize);
					std::memcpy(nextNames, &_names[(index + 1) * SP_GAME_OBJECT_NAME_MAX_LENGTH], nextNamesSize);
				}

				sp_mem_release(_names);

				_names = sp_mem_new_array(sp_char, _length * SP_GAME_OBJECT_NAME_MAX_LENGTH);

				if (previousNamesSize != 0)
					std::memcpy(_names, previousNames, previousNamesSize);

				if (nextNamesSize != 0)
					std::memcpy(&_names[index * SP_GAME_OBJECT_NAME_MAX_LENGTH], nextNames, nextNamesSize);

				if (previousNamesSize != 0)
					ALLOC_RELEASE(previousNames);
				else if (nextNamesSize != 0)
					ALLOC_RELEASE(nextNames);
			}
			else
			{
				sp_mem_release(_names);
				_names = nullptr;
			}
		}

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObjectManager()
		{
			_length = ZERO_UINT;
			_gameObjects = nullptr;
			_names = nullptr;

#ifdef OPENCL_ENABLED
			_gpuGameObjects = sp_mem_new(GpuBufferOpenCL);
#endif
		}

		/// <summary>
		/// Length of game objects in list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint length() const
		{
			return _length;
		}

		/// <summary>
		/// Get a game object
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject* get(const sp_uint index) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");
			return &_gameObjects[index];
		}

		/// <summary>
		/// Get the name of game object by index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_char* name(const sp_uint index) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");
			return &_names[index * SP_GAME_OBJECT_NAME_MAX_LENGTH];
		}

		/// <summary>
		/// Set the name of game object by index
		/// </summary>
		/// <param name="index"></param>
		/// <param name="newName"></param>
		/// <param name="newNameLength"></param>
		/// <returns></returns>
		API_INTERFACE inline void name(const sp_uint index, const sp_char* newName, const sp_size newNameLength) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");

			std::memcpy(&_names[index * SP_GAME_OBJECT_NAME_MAX_LENGTH], newName, newNameLength);
			_names[index * SP_GAME_OBJECT_NAME_MAX_LENGTH + newNameLength] = END_OF_STRING;
		}

		/// <summary>
		/// Add new game object to list
		/// </summary>
		/// <param name="gameObject"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add()
		{
			if (_length > 0)
			{
				const sp_size gameObjsSize = sizeof(SpGameObject) * _length;
				void* temp = ALLOC_SIZE(gameObjsSize);
				std::memcpy(temp, _gameObjects, gameObjsSize);

				sp_mem_release(_gameObjects);

				_length++;
				_gameObjects = sp_mem_new_array(SpGameObject, _length);

				std::memcpy(_gameObjects, temp, gameObjsSize);

				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
				_gameObjects = sp_mem_new_array(SpGameObject, _length);
			}

			_gameObjects[_length - 1]._index = _length - 1;

			addName();

#ifdef OPENCL_ENABLED
			_gpuGameObjects->init(sizeof(SpGameObject) * _length, _gameObjects, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
#endif
			return _length - 1;
		}

		/// <summary>
		/// Remove a game object from list
		/// </summary>
		/// <param name="index">Index of Game Object</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)
		{
			sp_assert(index < _length, "IndexOutOfRangeException");

			void* previousGameObjs = nullptr, * nextGameObjs = nullptr;

			const sp_size previousGameObjsSize = sizeof(SpGameObject) * index;
			if (previousGameObjsSize != 0)
			{
				previousGameObjs = ALLOC_SIZE(previousGameObjsSize);
				std::memcpy(previousGameObjs, _gameObjects, previousGameObjsSize);
			}

			const sp_size nextGameObjsSize = sizeof(SpGameObject) * (_length - index - 1);
			if (nextGameObjsSize != 0)
			{
				nextGameObjs = ALLOC_SIZE(nextGameObjsSize);
				std::memcpy(nextGameObjs, &_gameObjects[index + 1], nextGameObjsSize);
			}

			sp_mem_release(_gameObjects);
			_length--;

			if (_length == 0)
			{
				_gameObjects = nullptr;
				removeName(index);
				return;
			}

			_gameObjects = sp_mem_new_array(SpGameObject, _length);

			if (previousGameObjsSize != 0)
				std::memcpy(_gameObjects, previousGameObjs, previousGameObjsSize);

			if (nextGameObjsSize != 0)
				std::memcpy(&_gameObjects[index], nextGameObjs, nextGameObjsSize);

			if (previousGameObjsSize != 0)
				ALLOC_RELEASE(previousGameObjs);
			else if (nextGameObjsSize != 0)
				ALLOC_RELEASE(nextGameObjs);

			removeName(index);

#ifdef OPENCL_ENABLED
			_gpuGameObjects->init(sizeof(SpGameObject) * _length, _gameObjects, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
#endif
		}

#ifdef OPENCL_ENABLED
		/// <summary>
		/// Get the gpu buffer of game objects
		/// </summary>
		/// <returns>Gpu Buffer</returns>
		API_INTERFACE inline GpuBufferOpenCL* gpuGameObjects() const
		{
			return _gpuGameObjects;
		}
#endif

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose()
		{
#ifdef OPENCL_ENABLED
			if (_gpuGameObjects != nullptr)
			{
				sp_mem_delete(_gpuGameObjects, GpuBufferOpenCL);
				_gpuGameObjects = nullptr;
			}
#endif
			if (_gameObjects != nullptr)
			{
				sp_mem_release(_gameObjects);
				_gameObjects = nullptr;
			}
		}

		~SpGameObjectManager()
		{
			dispose();
		}

	};

}

#endif // SP_GAME_OBJECT_MANAGER_HEADER