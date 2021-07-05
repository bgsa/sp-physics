#ifndef SP_MESH_MANAGER_HEADER
#define SP_MESH_MANAGER_HEADER

#include "SpectrumPhysics.h"
#include "SpObjectManager.h"
#include "SpMeshData.h"

#define SP_MESH_INDEX_PLANE (0)
#define SP_MESH_INDEX_CUBE  (1)

namespace NAMESPACE_PHYSICS 
{
	class SpMeshManager
		: public SpObjectManager
	{
	private:
		sp_char** _meshesName;
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
			_length = 2;

			_meshesName = sp_mem_new_array(sp_char*, _length);

			_meshesName[0] = sp_mem_new_array(sp_char, 6);
			std::memcpy(_meshesName[0], "Plane\0", sizeof(sp_char) * 6);

			_meshesName[1] = sp_mem_new_array(sp_char, 5);
			std::memcpy(_meshesName[1], "Cube\0", sizeof(sp_char) * 5);

			_meshesData = sp_mem_new_array(SpMeshData, _length);

			// create plane mesh
			sp_size idx = 0;
			_meshesData[idx].attributesLength = 4;
			_meshesData[idx].attributes = sp_mem_new_array(SpMeshAttribute, 4);
			_meshesData[idx].attributes[0].vertex = Vec3(-0.5f, 1.0f, 0.5f);
			_meshesData[idx].attributes[1].vertex = Vec3(0.5f, 1.0f, 0.5f);
			_meshesData[idx].attributes[2].vertex = Vec3(0.5f, 1.0f, -0.5f);
			_meshesData[idx].attributes[3].vertex = Vec3(-0.5f, 1.0f, -0.5f);

			_meshesData[idx].attributes[0].normal = Vec3Up;
			_meshesData[idx].attributes[1].normal = Vec3Up;
			_meshesData[idx].attributes[2].normal = Vec3Up;
			_meshesData[idx].attributes[3].normal = Vec3Up;

			_meshesData[idx].facesLength = 2u;
			_meshesData[idx].faceIndexes = sp_mem_new_array(sp_size, 2 * 3);
			_meshesData[idx].faceIndexes[0] = 0;
			_meshesData[idx].faceIndexes[1] = 1;
			_meshesData[idx].faceIndexes[2] = 2;
			_meshesData[idx].faceIndexes[3] = 2;
			_meshesData[idx].faceIndexes[4] = 3;
			_meshesData[idx].faceIndexes[5] = 0;

			// create cube mesh
			idx = 1;
			_meshesData[idx].attributesLength = 8;
			_meshesData[idx].attributes = sp_mem_new_array(SpMeshAttribute, 8);
			_meshesData[idx].attributes[0].vertex = Vec3(0.5f, -0.5f, -0.5f);  // left-bottom-front
			_meshesData[idx].attributes[1].vertex = Vec3(-0.5f, -0.5f, -0.5f); // right-bottom-front
			_meshesData[idx].attributes[2].vertex = Vec3(-0.5f, 0.5f, -0.5f);  // right-top-front
			_meshesData[idx].attributes[3].vertex = Vec3(0.5f, 0.5f, -0.5f);   // left-top-front

			_meshesData[idx].attributes[4].vertex = Vec3(-0.5f, -0.5f, 0.5f); // left-bottom-back
			_meshesData[idx].attributes[5].vertex = Vec3(0.5f, -0.5f, 0.5f);  // right-bottom-back
			_meshesData[idx].attributes[6].vertex = Vec3(0.5f, 0.5f, 0.5f);   // right-top-back
			_meshesData[idx].attributes[7].vertex = Vec3(-0.5f, 0.5f, 0.5f);  // left-top-back

			_meshesData[idx].attributes[0].normal = Vec3(0.577350259f, -0.577350259f, -0.577350259f);  //bottom-right-back
			_meshesData[idx].attributes[1].normal = Vec3(-0.577350259f, -0.577350259f, -0.577350259f); //bottom-left-back
			_meshesData[idx].attributes[2].normal = Vec3(-0.577350259f, 0.577350259f, -0.577350259f);  //top-left-back
			_meshesData[idx].attributes[3].normal = Vec3(0.577350259f, 0.577350259f, -0.577350259f);   //top-right-back
			_meshesData[idx].attributes[4].normal = Vec3(-0.577350259f, -0.577350259f, 0.577350259f);  //bottom-left-front
			_meshesData[idx].attributes[5].normal = Vec3(0.577350259f, -0.577350259f, 0.577350259f);   //bottom-right-front
			_meshesData[idx].attributes[6].normal = Vec3(0.577350259f, 0.577350259f, 0.577350259f);    //top-right-front
			_meshesData[idx].attributes[7].normal = Vec3(-0.577350259f, 0.577350259f, 0.577350259f);    //top-left-front

			const sp_uint cubeIndices[36] = {
				0,1,2, //face frontal
				2,3,0,
				4,5,6, //face-traseira
				6,7,4,
				0,5,4, //fundo
				4,1,0,
				3,2,7, //topo
				7,6,3,
				6,5,0,
				0,3,6,
				1,4,7, //face direita
				7,2,1
			};
			_meshesData[idx].facesLength = 12u;
			_meshesData[idx].faceIndexes = sp_mem_new_array(sp_size, _meshesData[idx].facesLength * 3);
			std::memcpy(_meshesData[idx].faceIndexes, cubeIndices, 36 * sizeof(sp_float));
		}
		
		/// <summary>
		/// Add a mesh in list
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_uint add() override
		{			
			if (_length > 0)
			{
				const sp_size _meshesDataSize = meshesDataSize();

				void* temp = ALLOC_SIZE(_meshesDataSize);
				std::memcpy(temp, _meshesData, _meshesDataSize);

				sp_mem_release(_meshesData);

				_meshesData = sp_mem_new_array(SpMeshData, ++_length);

				std::memcpy(_meshesData, temp, _meshesDataSize);

				ALLOC_RELEASE(temp);
			}
			else
			{
				_length++;
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
			return _meshesName[index];
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