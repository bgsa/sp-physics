#ifndef SP_CAMERA_MANAGER_HEADER
#define SP_CAMERA_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpCamera.h"

namespace NAMESPACE_PHYSICS 
{

	class SpCameraManager
	{
	private:
		sp_uint _length;
		SpCamera* _cameras;

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE inline SpCameraManager()
		{
			_length = ZERO_UINT;
			_cameras = nullptr;
		}

		/// <summary>
		/// Get the length of cameras
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint length()
		{
			return _length;
		}

		/// <summary>
		/// Get a camera
		/// </summary>
		/// <param name="index">Index</param>
		/// <returns>Camera</returns>
		API_INTERFACE inline SpCamera& get(const sp_uint index) const
		{
			sp_assert(index < _length, "IndexOutOfRangeException");
			return _cameras[index];
		}

		/// <summary>
		/// Add a camera in list
		/// </summary>
		/// <param name="camera"></param>
		/// <returns></returns>
		API_INTERFACE inline void add(const SpCamera& camera)
		{
			const sp_size camerasSize = sizeof(SpCamera) * _length;

			void* temp = ALLOC_SIZE(camerasSize);
			std::memcpy(temp, _cameras, camerasSize);

			sp_mem_release(_cameras);
			_cameras = sp_mem_new_array(SpCamera, ++_length);

			std::memcpy(_cameras, temp, camerasSize);

			_cameras[_length - 1] = camera;

			ALLOC_RELEASE(temp);
		}

		/// <summary>
		/// Remove a camera from list
		/// </summary>
		/// <param name="index">Index of Camera</param>
		/// <returns>void</returns>
		API_INTERFACE inline void remove(const sp_uint index)
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
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose()
		{
			if (_cameras != nullptr)
			{
				sp_mem_release(_cameras);
				_cameras = nullptr;
			}

			_length = ZERO_UINT;
		}

		~SpCameraManager()
		{
			dispose();
		}

	};

}

#endif // SP_CAMERA_MANAGER_HEADER