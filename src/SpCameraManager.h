#ifndef SP_CAMERA_MANAGER_HEADER
#define SP_CAMERA_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpCamera.h"
#include "SpGpuTextureBuffer.h"

#define SP_CAMERA_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS 
{

	class SpCameraManager
		: public SpObjectManager
	{
	private:
		SpCamera* _cameras;
		SpGpuTextureBuffer* _textureBuffer;
		sp_uint usageType;
		sp_char* _cameraNames;

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE SpCameraManager();

		/// <summary>
		/// Get the camera texture buffer in GPU
		/// </summary>
		/// <returns>Texture Buffer</returns>
		API_INTERFACE inline SpGpuTextureBuffer* gpuBuffer() const
		{
			return _textureBuffer;
		}

		/// <summary>
		/// Update texture buffer with the cameras
		/// </summary>
		API_INTERFACE inline void updateGpuBuffer()
		{
			sp_size bufferSize = sizeof(SpCamera) * _length;

			_textureBuffer
				->use()
				->updateData(bufferSize, _cameras, usageType);
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

				_length++;
				_cameras = sp_mem_new_array(SpCamera, _length);

				std::memcpy(_cameras, temp, camerasSize);
				ALLOC_RELEASE(temp);


				const sp_size camerasNameSize = sizeof(sp_char) * SP_CAMERA_NAME_MAX_LENGTH * (_length - 1);
				temp = ALLOC_SIZE(camerasNameSize);
				std::memcpy(temp, _cameraNames, camerasNameSize);

				sp_mem_release(_cameraNames);
				_cameraNames = sp_mem_new_array(sp_char, SP_CAMERA_NAME_MAX_LENGTH * _length);
				std::memcpy(_cameraNames, temp, camerasNameSize);
				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
				_cameras = sp_mem_new_array(SpCamera, _length);
				_cameraNames = sp_mem_new_array(sp_char, _length * SP_CAMERA_NAME_MAX_LENGTH);
			}
			_cameras[_length - 1].init();

			updateGpuBuffer();

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
		/// Get the name to camera[index]
		/// </summary>
		/// <param name="index">Camera Index</param>
		/// <returns></returns>
		API_INTERFACE inline sp_char* name(const sp_uint index)
		{
			return &_cameraNames[index * SP_CAMERA_NAME_MAX_LENGTH];
		}

		/// <summary>
		/// Set a name to camera[index]
		/// </summary>
		/// <param name="index">Camera Index</param>
		/// <param name="name">New Name</param>
		/// <returns></returns>
		API_INTERFACE inline void name(const sp_uint index, const sp_char* name, const sp_size nameLength)
		{
			std::memcpy(&_cameraNames[index * SP_CAMERA_NAME_MAX_LENGTH], name, nameLength + 1);
			_cameraNames[index * SP_CAMERA_NAME_MAX_LENGTH + nameLength] = END_OF_STRING;
		}

		/// <summary>
		/// Find a camera by name.
		/// Returns SP_UINT_MAX if not found.
		/// </summary>
		/// <param name="name">Name</param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint find(const sp_char* name) const
		{
			for (sp_size i = 0; i < _length; i++)
				if (strcmp(name, &_cameraNames[i * SP_CAMERA_NAME_MAX_LENGTH]) == 0)
					return i;
			
			return SP_UINT_MAX;
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