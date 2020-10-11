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
			vertexes = ALLOC_NEW_ARRAY(Vec3, vertexesLength);
		}

		API_INTERFACE inline SpMeshCache(const SpMesh* mesh, const SpTransform& transformation);

		API_INTERFACE inline void update(const SpMesh* mesh, const SpTransform& transformation);

		inline ~SpMeshCache()
		{
			if (vertexes != nullptr)
			{
				ALLOC_RELEASE(vertexes);
				vertexes = nullptr;
			}
		}

	};
}

#endif // SP_MESH_CACHE_HEADER