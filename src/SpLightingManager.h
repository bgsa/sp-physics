#ifndef SP_LIGHTING_MANAGER_HEADER
#define SP_LIGHTING_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpLightSource.h"
#include "SpGpuTextureBuffer.h"

#define SP_LIGHT_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS 
{
	class SpLightingManager
		: public SpObjectManager
	{
	private:
		SpLightSource* _lights;
		SpGpuTextureBuffer* _textureBuffer;
		sp_int usageType;
		sp_char* _names;

		inline void addDefaultAmbientLight()
		{
			const sp_uint index = add();

			_lights[index].type(SP_LIGHT_SOURCE_TYPE_AMBIENT);
			_lights[index].color(SpColorRGB(0.1f, 0.1f, 0.1f));
			_lights[index].position(Vec3Zeros);
			_lights[index].direction(Vec3Zeros);
			_lights[index].lightSwitch(true);
			_lights[index].staticLight(true);

			name(index, "Global Ambient", 14);
		}

		inline void addDefaultDiffuseLight()
		{
			const sp_uint index = add();

			_lights[index].type(SP_LIGHT_SOURCE_TYPE_DIFFUSE);
			_lights[index].color(SpColorRGB(1.0f, 1.0f, 1.0f));
			_lights[index].position(Vec3(0.0f, 10.0f, 0.0f));
			_lights[index].direction(Vec3Zeros);
			_lights[index].lightSwitch(true);
			_lights[index].staticLight(true);

			name(index, "Global Diffuse", 14);
		}

		inline void addName()
		{
			const sp_size namesSize = sizeof(sp_char) * (_length-1) * SP_LIGHT_NAME_MAX_LENGTH;
			void* temp = ALLOC_SIZE(namesSize);
			std::memcpy(temp, _names, namesSize);

			sp_mem_release(_names);

			_names = sp_mem_new_array(sp_char, _length * SP_LIGHT_NAME_MAX_LENGTH);
			std::memcpy(_names, temp, namesSize);

			ALLOC_RELEASE(temp);

			std::memset(&_names[(_length - 1) * SP_LIGHT_NAME_MAX_LENGTH], 0, SP_LIGHT_NAME_MAX_LENGTH);
		}

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE SpLightingManager();

		/// <summary>
		/// Add a camera in list
		/// </summary>
		/// <param name="camera"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add() override
		{			
			if (_length > 0)
			{
				const sp_size objectSize = sizeof(SpLightSource) * _length;
				void* temp = ALLOC_SIZE(objectSize);
				std::memcpy(temp, _lights, objectSize);

				sp_mem_release(_lights);

				_length++;
				_lights = sp_mem_new_array(SpLightSource, _length);

				std::memcpy(_lights, temp, objectSize);

				ALLOC_RELEASE(temp);

				addName();
			}
			else
			{
				_length++;
				_lights = sp_mem_new_array(SpLightSource, _length);

				_names = sp_mem_new_array(sp_char, _length * SP_LIGHT_NAME_MAX_LENGTH);
				std::memset(_names, 0, SP_LIGHT_NAME_MAX_LENGTH);
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

			const sp_size objectSize = sizeof(SpLightSource) * _length;

			SpLightSource* temp = (SpLightSource*)ALLOC_SIZE(objectSize);
			std::memcpy(temp, _lights, objectSize);

			sp_mem_release(_lights);
			_length++;
			_lights = sp_mem_new_array(SpLightSource, _length);

			const sp_size objectSizeBegin = sizeof(SpLightSource) * (index + ONE_UINT);
			std::memcpy(_lights, temp, objectSizeBegin);

			const sp_size objectSizeEnd = sizeof(SpLightSource) * (_length - index);
			std::memcpy(&_lights[index + ONE_UINT], &temp[_length - index], objectSizeEnd);

			ALLOC_RELEASE(temp);
		}

		/// <summary>
		/// Get light by index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpLightSource* get(const sp_uint index) const
		{
			return &_lights[index];
		}

		/// <summary>
		/// Get name og the light
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_char* name(const sp_uint index) const
		{
			return &_names[index * SP_LIGHT_NAME_MAX_LENGTH];
		}

		/// <summary>
		/// Set name og the light
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline void name(const sp_uint index, const sp_char* newName, const sp_size newNameLength)
		{
			std::memcpy(&_names[index * SP_LIGHT_NAME_MAX_LENGTH], newName, newNameLength);
			_names[index * SP_LIGHT_NAME_MAX_LENGTH + newNameLength] = END_OF_STRING;
		}

		/// <summary>
		/// Update texture buffer with the light sources
		/// </summary>
		API_INTERFACE inline void updateGpuBuffer()
		{
			const sp_size bufferSize = sizeof(SpLightSource) * _length;

			_textureBuffer
				->use()
				->updateData(bufferSize, _lights, usageType);
		}

		/// <summary>
		/// Get the lights texture buffer in GPU
		/// </summary>
		/// <returns>Texture Buffer</returns>
		API_INTERFACE inline SpGpuTextureBuffer* gpuBuffer() const
		{
			return _textureBuffer;
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
			if (_lights != nullptr)
			{
				sp_mem_release(_lights);
				_lights = nullptr;
			}

			SpObjectManager::dispose();
		}

		~SpLightingManager()
		{
			dispose();
		}

	};

}

#endif // SP_LIGHTING_MANAGER_HEADER