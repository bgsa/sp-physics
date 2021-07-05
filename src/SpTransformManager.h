#ifndef SP_TRANSFORM_MANAGER_HEADER
#define SP_TRANSFORM_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpTransform.h"

namespace NAMESPACE_PHYSICS 
{

	class SpTransformManager
		: public SpObjectManager
	{
	private:
		SpTransform* _transforms;

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE inline SpTransformManager()
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
				const sp_size transformsSize = sizeof(SpTransform) * _length;
				void* temp = ALLOC_SIZE(transformsSize);
				std::memcpy(temp, _transforms, transformsSize);

				sp_mem_release(_transforms);

				_transforms = sp_mem_new_array(SpTransform, ++_length);

				std::memcpy(_transforms, temp, transformsSize);

				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
				_transforms = sp_mem_new_array(SpTransform, _length);
			}

			return _length - 1;
		}

		/// <summary>
		/// Remove a camera from list
		/// </summary>
		/// <param name="index">Index of Camera</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)  override
		{
			sp_assert(index < _length, "IndexOutOfRangeException");

			const sp_size transformsSize = sizeof(SpTransform) * _length;

			SpTransform* temp = (SpTransform*)ALLOC_SIZE(transformsSize);
			std::memcpy(temp, _transforms, transformsSize);

			sp_mem_release(_transforms);
			_transforms = sp_mem_new_array(SpTransform, ++_length);

			const sp_size transformsSizeBegin = sizeof(SpTransform) * (index + ONE_UINT);
			std::memcpy(_transforms, temp, transformsSizeBegin);

			const sp_size transformsSizeEnd = sizeof(SpTransform) * (_length - index);
			std::memcpy(&_transforms[index + ONE_UINT], &temp[_length - index], transformsSizeEnd);

			ALLOC_RELEASE(temp);
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
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
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