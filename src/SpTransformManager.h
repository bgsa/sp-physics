#ifndef SP_TRANSFORM_MANAGER_HEADER
#define SP_TRANSFORM_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpTransform.h"
#include "SpGpuTextureBuffer.h"

namespace NAMESPACE_PHYSICS 
{

	class SpTransformManager
		: public SpObjectManager
	{
	private:
		SpTransform* _transforms;
		SpGpuTextureBuffer* _textureBuffer;
		sp_int usageType;

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE SpTransformManager();

		/// <summary>
		/// Add a camera in list
		/// </summary>
		/// <param name="camera"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add(const sp_uint length = 1) override
		{			
			if (_length > 0)
			{
				const sp_size transformsSize = sizeof(SpTransform) * _length;
				void* temp = ALLOC_SIZE(transformsSize);
				std::memcpy(temp, _transforms, transformsSize);

				sp_mem_release(_transforms);

				_length += length;
				_transforms = sp_mem_new_array(SpTransform, _length);

				std::memcpy(_transforms, temp, transformsSize);

				ALLOC_RELEASE(temp);
			}
			else
			{
				_length = length;
				_transforms = sp_mem_new_array(SpTransform, _length);
			}

			updateGpuBuffer();

			return _length - length;
		}

		/// <summary>
		/// Remove a camera from list
		/// </summary>
		/// <param name="index">Index of Camera</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)  override
		{
			sp_assert(index < _length, "IndexOutOfRangeException");

			void* previousTransforms = nullptr, * nextTransforms = nullptr;

			const sp_size previousTransformsSize = sizeof(SpTransform) * index;
			if (previousTransformsSize != 0)
			{
				previousTransforms = ALLOC_SIZE(previousTransformsSize);
				std::memcpy(previousTransforms, _transforms, previousTransformsSize);
			}

			const sp_size nextTransformsSize = sizeof(SpTransform) * (_length - index - 1);
			if (nextTransformsSize != 0)
			{
				nextTransforms = ALLOC_SIZE(nextTransformsSize);
				std::memcpy(nextTransforms, &_transforms[index + 1], nextTransformsSize);
			}

			sp_mem_release(_transforms);
			_length--;

			if (_length == 0)
			{
				_transforms = nullptr;
				return;
			}

			_transforms = sp_mem_new_array(SpTransform, _length);

			if (previousTransformsSize != 0)
				std::memcpy(_transforms, previousTransforms, previousTransformsSize);

			if (nextTransformsSize != 0)
				std::memcpy(&_transforms[index], nextTransforms, nextTransformsSize);

			if (previousTransformsSize != 0)
				ALLOC_RELEASE(previousTransforms);
			else if (nextTransformsSize != 0)
				ALLOC_RELEASE(nextTransforms);

			updateGpuBuffer();
		}

		/// <summary>
		/// Get camera from index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpTransform* get(const sp_uint index) const
		{
			return &_transforms[index];
		}

		/// <summary>
		/// Get the transform texture buffer in GPU
		/// </summary>
		/// <returns>Texture Buffer</returns>
		API_INTERFACE inline SpGpuTextureBuffer* gpuBuffer() const
		{
			return _textureBuffer;
		}

		/// <summary>
		/// Update texture buffer with the transform
		/// </summary>
		API_INTERFACE inline void updateGpuBuffer()
		{
			sp_size bufferSize = sizeof(SpTransform) * _length;

			_textureBuffer
				->use()
				->updateData(bufferSize, _transforms, usageType);
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
			if (_textureBuffer != nullptr)
			{
				_textureBuffer->dispose();
				sp_mem_release(_textureBuffer);
				_textureBuffer = nullptr;
			}

			if (_transforms != nullptr)
			{
				sp_mem_release(_transforms);
				_transforms = nullptr;
			}

			SpObjectManager::dispose();
		}

		~SpTransformManager()
		{
			dispose();
		}

	};

}

#endif // SP_TRANSFORM_MANAGER_HEADER