#ifndef SP_RENDERABLE_OBJECT_MANAGER_HEADER
#define SP_RENDERABLE_OBJECT_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpRenderableObject.h"
#include "SpMaterial.h"
#include "SpGpuTextureBuffer.h"

namespace NAMESPACE_PHYSICS 
{

	class SpRenderableObjectManager
		: public SpObjectManager
	{
	private:
		SpRenderableObject* _renderableObjects;
		SpMaterial* _materials;
		SpGpuTextureBuffer* _materialsBuffer;
		sp_int usageType;

		inline void addMaterial()
		{
			if (_materials != nullptr)
			{
				const sp_size objectSize = sizeof(SpMaterial) * (_length - 1);
				void* temp = ALLOC_SIZE(objectSize);
				std::memcpy(temp, _materials, objectSize);

				sp_mem_release(_materials);

				_materials = sp_mem_new_array(SpMaterial, _length);

				std::memcpy(_materials, temp, objectSize);

				ALLOC_RELEASE(temp);
			}
			else
				_materials = sp_mem_new_array(SpMaterial, _length);
		}

		inline void removeMaterial(const sp_uint index)
		{
			if (_length != 0)
			{
				void* previousMaterials = nullptr, * nextMaterials = nullptr;

				const sp_size previousMaterialsSize = sizeof(SpMaterial) * index;
				if (previousMaterialsSize != 0)
				{
					previousMaterials = ALLOC_SIZE(previousMaterialsSize);
					std::memcpy(previousMaterials, _materials, previousMaterialsSize);
				}

				const sp_size nextMaterialsSize = sizeof(SpMaterial) * (_length - index);
				if (nextMaterialsSize != 0)
				{
					nextMaterials = ALLOC_SIZE(nextMaterialsSize);
					std::memcpy(nextMaterials, &_materials[index + 1], nextMaterialsSize);
				}

				sp_mem_release(_materials);

				_materials = sp_mem_new_array(SpMaterial, _length);

				if (previousMaterialsSize != 0)
					std::memcpy(_materials, previousMaterials, previousMaterialsSize);

				if (nextMaterialsSize != 0)
					std::memcpy(&_materials[index], nextMaterials, nextMaterialsSize);

				if (previousMaterialsSize != 0)
					ALLOC_RELEASE(previousMaterials);
				else if (nextMaterialsSize != 0)
					ALLOC_RELEASE(nextMaterials);
			}
			else
			{
				sp_mem_release(_materials);
				_materials = nullptr;
			}
		}

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE SpRenderableObjectManager();

		/// <summary>
		/// Add a camera in list
		/// </summary>
		/// <param name="camera"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add() override
		{			
			if (_length > 0)
			{
				sp_size objectSize = sizeof(SpRenderableObject) * _length;
				void* temp = ALLOC_SIZE(objectSize);
				std::memcpy(temp, _renderableObjects, objectSize);

				sp_mem_release(_renderableObjects);

				_length++;
				_renderableObjects = sp_mem_new_array(SpRenderableObject, _length);

				std::memcpy(_renderableObjects, temp, objectSize);

				ALLOC_RELEASE(temp);

				addMaterial();
			}
			else
			{
				_length++;
				_renderableObjects = sp_mem_new_array(SpRenderableObject, _length);
				_materials = sp_mem_new_array(SpMaterial, _length);
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

			void* previousRenderables = nullptr, * nextRenderables = nullptr;

			const sp_size previousRenderablesSize = sizeof(SpRenderableObject) * index;
			if (previousRenderablesSize != 0)
			{
				previousRenderables = ALLOC_SIZE(previousRenderablesSize);
				std::memcpy(previousRenderables, _renderableObjects, previousRenderablesSize);
			}

			const sp_size nextRenderablesSize = sizeof(SpRenderableObject) * (_length - index - 1);
			if (nextRenderablesSize != 0)
			{
				nextRenderables = ALLOC_SIZE(nextRenderablesSize);
				std::memcpy(nextRenderables, &_renderableObjects[index + 1], nextRenderablesSize);
			}

			sp_mem_release(_renderableObjects);
			_length--;

			if (_length == 0)
			{
				_renderableObjects = nullptr;
				removeMaterial(index);
				return;
			}

			_renderableObjects = sp_mem_new_array(SpRenderableObject, _length);

			if (previousRenderablesSize != 0)
				std::memcpy(_renderableObjects, previousRenderables, previousRenderablesSize);

			if (nextRenderablesSize != 0)
				std::memcpy(&_renderableObjects[index], nextRenderables, nextRenderablesSize);

			if (previousRenderablesSize != 0)
				ALLOC_RELEASE(previousRenderables);
			else if (nextRenderablesSize != 0)
				ALLOC_RELEASE(nextRenderables);

			removeMaterial(index);

			updateGpuBuffer();
		}

		/// <summary>
		/// Get renderable object byindex
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpRenderableObject* get(const sp_uint index) const
		{
			return &_renderableObjects[index];
		}

		/// <summary>
		/// Get material from index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpMaterial* material(const sp_uint index) const
		{
			return &_materials[index];
		}

		/// <summary>
		/// Get the materials buffer in GPU
		/// </summary>
		/// <returns>Texture Buffer</returns>
		API_INTERFACE inline SpGpuTextureBuffer* gpuBufferMaterials() const
		{
			return _materialsBuffer;
		}

		/// <summary>
		/// Update texture buffer with the materials
		/// </summary>
		API_INTERFACE inline void updateGpuBuffer()
		{
			sp_size bufferSize = sizeof(SpMaterial) * _length;

			_materialsBuffer
				->use()
				->updateData(bufferSize, _materials, usageType);
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
			if (_materialsBuffer != nullptr)
			{
				_materialsBuffer->dispose();
				sp_mem_release(_materialsBuffer);
				_materialsBuffer = nullptr;
			}

			if (_materials != nullptr)
			{
				sp_mem_release(_materials);
				_materials = nullptr;
			}

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