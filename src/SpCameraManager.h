#ifndef SP_CAMERA_MANAGER_HEADER
#define SP_CAMERA_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpCamera.h"

namespace NAMESPACE_PHYSICS 
{

	class SpCameraManager
		: public SpObjectManager
	{
	private:
		SpCamera* _cameras;

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE inline SpCameraManager()
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
				const sp_size camerasSize = sizeof(SpCamera) * _length;
				void* temp = ALLOC_SIZE(camerasSize);
				std::memcpy(temp, _cameras, camerasSize);

				sp_mem_release(_cameras);

				_cameras = sp_mem_new_array(SpCamera, ++_length);

				std::memcpy(_cameras, temp, camerasSize);

				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
				_cameras = sp_mem_new_array(SpCamera, _length);
			}
			_cameras[_length - 1].init();

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

			const sp_size camerasSize = sizeof(SpCamera) * _length;

			SpCamera* temp = (SpCamera*)ALLOC_SIZE(camerasSize);
			std::memcpy(temp, _cameras, camerasSize);

			sp_mem_release(_cameras);
			_cameras = sp_mem_new_array(SpCamera, ++_length);

			const sp_size camerasSizeBegin = sizeof(SpCamera) * (index + ONE_UINT);
			std::memcpy(_cameras, temp, camerasSizeBegin);

			const sp_size camerasSizeEnd = sizeof(SpCamera) * (_length - index);
			std::memcpy(&_cameras[index + ONE_UINT], &temp[_length - index], camerasSizeEnd);

			ALLOC_RELEASE(temp);
		}

		/// <summary>
		/// Get camera from index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpCamera* get(const sp_uint index) const
		{
			return &_cameras[index];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
			if (_cameras != nullptr)
			{
				sp_mem_release(_cameras);
				_cameras = nullptr;
			}

			SpObjectManager::dispose();
		}

		~SpCameraManager()
		{
			dispose();
		}

	};

}

#endif // SP_CAMERA_MANAGER_HEADER