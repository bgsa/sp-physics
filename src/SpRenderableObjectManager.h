#ifndef SP_RENDERABLE_OBJECT_MANAGER_HEADER
#define SP_RENDERABLE_OBJECT_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpRenderableObject.h"

namespace NAMESPACE_PHYSICS 
{

	class SpRenderableObjectManager
		: public SpObjectManager
	{
	private:
		SpRenderableObject* _renderableObjects;

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpRenderableObjectManager()
			: SpObjectManager()
		{
		}

		/// <summary>
		/// Add a camera in list
		/// </summary>
		/// <param name="camera"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add() override
		{			
			if (_length > 0)
			{
				const sp_size objectSize = sizeof(SpRenderableObject) * _length;
				void* temp = ALLOC_SIZE(objectSize);
				std::memcpy(temp, _renderableObjects, objectSize);

				sp_mem_release(_renderableObjects);

				_length++;
				_renderableObjects = sp_mem_new_array(SpRenderableObject, _length);

				std::memcpy(_renderableObjects, temp, objectSize);

				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
				_renderableObjects = sp_mem_new_array(SpRenderableObject, _length);
			}

			return _length - 1;
		}

		/// <summary>
		/// Remove a renderable object from list
		/// </summary>
		/// <param name="index">Index of Renderable Object</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)  override
		{
			sp_assert(index < _length, "IndexOutOfRangeException");

			const sp_size objectSize = sizeof(SpRenderableObject) * _length;

			SpRenderableObject* temp = (SpRenderableObject*)ALLOC_SIZE(objectSize);
			std::memcpy(temp, _renderableObjects, objectSize);

			sp_mem_release(_renderableObjects);
			_renderableObjects = sp_mem_new_array(SpRenderableObject, ++_length);

			const sp_size objectSizeBegin = sizeof(SpRenderableObject) * (index + ONE_UINT);
			std::memcpy(_renderableObjects, temp, objectSizeBegin);

			const sp_size objectSizeEnd = sizeof(SpRenderableObject) * (_length - index);
			std::memcpy(&_renderableObjects[index + ONE_UINT], &temp[_length - index], objectSizeEnd);

			ALLOC_RELEASE(temp);
		}

		/// <summary>
		/// Get camera from index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpRenderableObject* get(const sp_uint index) const
		{
			return &_renderableObjects[index];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
			if (_renderableObjects != nullptr)
			{
				sp_mem_release(_renderableObjects);
				_renderableObjects = nullptr;
			}

			SpObjectManager::dispose();
		}

		~SpRenderableObjectManager()
		{
			dispose();
		}

	};

}

#endif // SP_RENDERABLE_OBJECT_MANAGER_HEADER