#ifndef SP_GAME_OBJECT_MANAGER_HEADER
#define SP_GAME_OBJECT_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObject.h"

#ifdef OPENCL_ENABLED
	#include "GpuBufferOpenCL.h"
#endif

namespace NAMESPACE_PHYSICS 
{

	class SpGameObjectManager
	{
	private:
		sp_uint _length;
		sp_uint _maxLength;
		SpGameObject* _gameObjects;

#ifdef OPENCL_ENABLED
		GpuBufferOpenCL* _gpuBuffer;
#endif

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObjectManager(const sp_uint maxLength)
		{
			sp_assert(maxLength > ZERO_UINT, "InvalidArgumentException");

			_length = ZERO_UINT;
			_maxLength = maxLength;
			_gameObjects = sp_mem_new_array(SpGameObject, _maxLength);

#ifdef OPENCL_ENABLED
			_gpuBuffer = sp_mem_new(GpuBufferOpenCL);
			_gpuBuffer->init(sizeof(SpGameObject) * _maxLength, _gameObjects, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
#endif
		}

		/// <summary>
		/// Max of game object allowed in this list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint maxLength()
		{
			return _maxLength;
		}

		/// <summary>
		/// Length of game objects in list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint length()
		{
			return _length;
		}

		/// <summary>
		/// Get a game object
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject& get(const sp_uint index) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");
			return _gameObjects[index];
		}

		/// <summary>
		/// Add game object to list
		/// </summary>
		/// <param name="gameObject"></param>
		/// <returns></returns>
		API_INTERFACE inline SpGameObject& add(const sp_uint gameObjectType, const sp_uint index, const sp_char* name = nullptr)
		{
			sp_assert(_length < _maxLength, "InvalidOperationException");

			_gameObjects[_length].type(gameObjectType);
			_gameObjects[_length]._index = index;

			if (name != nullptr)
				_gameObjects[_length]._name = name;
			else
			{
				sp_char newName[10];
				sp_uint length1, length2;
				convert(gameObjectType, newName, length1);
				convert(index, &newName[length1], length2);
				newName[length1 + length2 + 1] = END_OF_STRING;

				_gameObjects[_length]._name = newName;
			}

			_length++;

#ifdef OPENCL_ENABLED
			_gpuBuffer->update(_gameObjects, ZERO_UINT, nullptr, nullptr);
#endif

			return _gameObjects[_length - 1];
		}

		/// <summary>
		/// Remove a game object from list
		/// </summary>
		/// <param name="index">Index of Game Object</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)
		{
			sp_assert(index < _length, "IndexOutOfRangeException");

			const sp_size sizeToMove = (_length - index - 1u) * sizeof(SpGameObject);

			if (sizeToMove != ZERO_SIZE)
				std::memcpy(&_gameObjects[index], &_gameObjects[index + 1u], sizeToMove);

			_length--;
		}

#ifdef OPENCL_ENABLED
		/// <summary>
		/// Get the gpu buffer of game objects
		/// </summary>
		/// <returns>Gpu Buffer</returns>
		API_INTERFACE inline GpuBufferOpenCL* gpuBuffer() const
		{
			return _gpuBuffer;
		}
#endif

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose()
		{
#ifdef OPENCL_ENABLED
			if (_gpuBuffer != nullptr)
			{
				sp_mem_delete(_gpuBuffer, GpuBufferOpenCL);
				_gpuBuffer = nullptr;
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