#ifndef SP_MESH_CACHE_HEADER
#define SP_MESH_CACHE_HEADER

#include "SpectrumPhysics.h"
#include "SpMesh.h"

namespace NAMESPACE_PHYSICS
{
	class SpMeshCache
	{
	public:
		Vec3* vertexes;

		API_INTERFACE inline SpMeshCache(const sp_uint vertexesLength)
		{
			vertexes = sp_mem_new_array(Vec3, vertexesLength);
		}

		API_INTERFACE SpMeshCache(const SpMesh* mesh, SpTransform* transformation);

		API_INTERFACE void update(const SpMesh* mesh, SpTransform* transformation);

		inline ~SpMeshCache()
		{
			if (vertexes != nullptr)
			{
				sp_mem_release(vertexes);
				vertexes = nullptr;
			}
		}

	};
}

#endif // SP_MESH_CACHE_HEADER