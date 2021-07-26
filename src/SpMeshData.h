#ifndef SP_MESH_DATA_HEADER
#define SP_MESH_DATA_HEADER

#include "SpectrumPhysics.h"
#include "SpMeshAttributes.h"

namespace NAMESPACE_PHYSICS
{
	class SpMeshData
	{
	public:
		sp_size attributesLength;
		sp_size facesLength;
		SpMeshAttribute* attributes;
		sp_size* faceIndexes;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline SpMeshData()
		{
			attributesLength = facesLength = ZERO_SIZE;
			attributes = nullptr;
			faceIndexes = nullptr;
		}

		/// <summary>
		/// Size of this mesh data in bytes
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_size size() const
		{
			return 
				sizeof(SpMeshAttribute) * attributesLength +
				sizeof(sp_size) * 3 * facesLength;
		}

		/// <summary>
		/// Get the last address of face index
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_size lastAddress() const
		{
			return (sp_size)&faceIndexes[facesLength * 3 - 1];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose()
		{
			if (attributes != nullptr)
			{
				sp_mem_release(attributes);
				attributes = nullptr;
			}

			if (faceIndexes != nullptr)
			{
				sp_mem_release(faceIndexes);
				faceIndexes = nullptr;
			}
		}

		~SpMeshData()
		{
			dispose();
		}
	};
}

#endif // SP_MESH_DATA_HEADER