#ifndef SP_MESH_MANAGER_HEADER
#define SP_MESH_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpMeshData.h"

#define SP_MESH_NAME_MAX_LENGTH (100)

namespace NAMESPACE_PHYSICS 
{
	class SpMeshManager
		: public SpObjectManager
	{
	private:
		sp_char* _meshesName;
		SpMeshData* _meshesData;

		inline sp_size meshesDataSize() const
		{
			return _meshesData[_length - 1].lastAddress() - (sp_size)_meshesData;
		}

	public:
	
		/// <summary>
		/// Default constructor
		/// </summary>
		/// <param name="maxLength">Max game object allowed</param>
		/// <returns></returns>
		API_INTERFACE inline SpMeshManager()
			: SpObjectManager()
		{
			_length = 0;
			_meshesName = nullptr;
			_meshesData = nullptr;
		}
		
		/// <summary>
		/// Add a mesh in list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add() override
		{			
			if (_length > 0)
			{
				// realloc meshes names
				void* temp = ALLOC_SIZE(_length * SP_MESH_NAME_MAX_LENGTH);
				std::memcpy(temp, _meshesName, _length * SP_MESH_NAME_MAX_LENGTH);

				sp_mem_release(_meshesName);
				_meshesName = sp_mem_new_array(sp_char, (_length + 1) * SP_MESH_NAME_MAX_LENGTH);

				std::memcpy(_meshesName, temp, _length * SP_MESH_NAME_MAX_LENGTH);
				ALLOC_RELEASE(temp);

				// realloc meshes data
				const sp_size _meshesDataSize = meshesDataSize();

				temp = ALLOC_SIZE(_meshesDataSize);
				std::memcpy(temp, _meshesData, _meshesDataSize);

				sp_mem_release(_meshesData);

				_length++;
				_meshesData = sp_mem_new_array(SpMeshData, _length);

				std::memcpy(_meshesData, temp, _meshesDataSize);
				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
				_meshesName = sp_mem_new_array(sp_char, _length * SP_MESH_NAME_MAX_LENGTH);
				_meshesData = sp_mem_new_array(SpMeshData, _length);
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

			const sp_size _meshesDataSize = meshesDataSize();

			SpMeshData* temp = (SpMeshData*)ALLOC_SIZE(_meshesDataSize);
			std::memcpy(temp, _meshesData, _meshesDataSize);

			sp_mem_release(_meshesData);
			_meshesData = sp_mem_new_array(SpMeshData, ++_length);

			const sp_size meshesDataSizeBegin = _meshesData[index - 1].lastAddress() - (sp_size)_meshesData;

			std::memcpy(_meshesData, temp, meshesDataSizeBegin);

			const sp_size meshesDataSizeEnd = _meshesData[index - 1].lastAddress() - (sp_size)&_meshesData[index];
			std::memcpy(&_meshesData[index + ONE_UINT], &temp[_length - index], meshesDataSizeEnd);

			ALLOC_RELEASE(temp);
		}

		/// <summary>
		/// Get name of mesh by index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline sp_char* name(const sp_uint index) const
		{
			return &_meshesName[index * SP_MESH_NAME_MAX_LENGTH];
		}

		/// <summary>
		/// Set a mesh name to mesh index
		/// </summary>
		/// <param name="index">Mesh Index</param>
		/// <param name="newName">New name</param>
		/// <param name="newNameLength">New name length</param>
		/// <returns></returns>
		API_INTERFACE inline void name(const sp_uint index, const sp_char* newName, const sp_size newNameLength) const
		{
			sp_assert(newNameLength + 1 > SP_MESH_NAME_MAX_LENGTH, "IndexOutOfRangeException");

			std::memcpy(&_meshesName[index * SP_MESH_NAME_MAX_LENGTH], newName, sizeof(sp_char) * newNameLength);
			_meshesName[index * SP_MESH_NAME_MAX_LENGTH + newNameLength] = END_OF_STRING;
		}

		/// <summary>
		/// Get camera from index
		/// </summary>
		/// <param name="index"></param>
		/// <returns></returns>
		API_INTERFACE inline SpMeshData* get(const sp_uint index) const
		{
			return &_meshesData[index];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void dispose() override
		{
			if (_meshesData != nullptr)
			{
				sp_mem_release(_meshesData);
				_meshesData = nullptr;
			}

			SpObjectManager::dispose();
		}

		~SpMeshManager()
		{
			dispose();
		}

	};

}

#endif // SP_MESH_MANAGER_HEADER