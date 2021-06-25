#ifndef SP_GAME_OBJECT_MANAGER_HEADER
#define SP_GAME_OBJECT_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpGameObject.h"
#include "SpTransform.h"

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
		SpTransform* _transforms;

#ifdef OPENCL_ENABLED
		GpuBufferOpenCL* _gpuGameObjects;
		GpuBufferOpenCL* _gpuTransforms;
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
			_transforms = sp_mem_new_array(SpTransform, _maxLength);

#ifdef OPENCL_ENABLED
			_gpuGameObjects = sp_mem_new(GpuBufferOpenCL);
			_gpuGameObjects->init(sizeof(SpGameObject) * _maxLength, _gameObjects, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);

			_gpuTransforms = sp_mem_new(GpuBufferOpenCL);
			_gpuTransforms->init(sizeof(SpTransform) * _maxLength, _transforms, CL_MEM_USE_HOST_PTR | CL_MEM_READ_ONLY);
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
		API_INTERFACE inline SpGameObject* get(const sp_uint index) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");
			return &_gameObjects[index];
		}

		/// <summary>
		/// Get a transformation from index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpTransform& transform(const sp_uint index) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");
			return _transforms[index];
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

			_transforms[_length].reset();

			_length++;

#ifdef OPENCL_ENABLED
			_gpuGameObjects->update(_gameObjects, ZERO_UINT, nullptr, nullptr);
			_gpuTransforms->update(_transforms, ZERO_UINT, nullptr, nullptr);
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

			const sp_size sizeToMoveTransform = (_length - index - 1u) * sizeof(SpTransform);
			if (sizeToMoveTransform != ZERO_SIZE)
				std::memcpy(&_transforms[index], &_transforms[index + 1u], sizeToMoveTransform);

#ifdef OPENCL_ENABLED
			_gpuGameObjects->update(_gameObjects, ZERO_UINT, nullptr, nullptr);
			_gpuTransforms->update(_transforms, ZERO_UINT, nullptr, nullptr);
#endif

			_length--;
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

		/// <summary>
		/// Get the gpu buffer of transforms
		/// </summary>
		/// <returns>Gpu Buffer</returns>
		API_INTERFACE inline GpuBufferOpenCL* gpuTransforms() const
		{
			return _gpuTransforms;
		}
#endif

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose()
		{
#ifdef OPENCL_ENABLED
			if (_gpuTransforms != nullptr)
			{
				sp_mem_delete(_gpuTransforms, GpuBufferOpenCL);
				_gpuTransforms = nullptr;
			}

			if (_gpuGameObjects != nullptr)
			{
				sp_mem_delete(_gpuGameObjects, GpuBufferOpenCL);
				_gpuGameObjects = nullptr;
			}
#endif
			if (_transforms != nullptr)
			{
				sp_mem_release(_transforms);
				_transforms = nullptr;

			}
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