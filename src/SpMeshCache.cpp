#include "SpMeshCache.h"

namespace NAMESPACE_PHYSICS
{

	void SpMeshCache::update(const SpMesh* mesh, SpTransform* transformation)
	{
		for (sp_uint i = 0; i < mesh->vertexLength(); i++)
			transformation->transform(mesh->vertexesMesh->get(i)->value(), vertexes[i]);
	}

	SpMeshCache::SpMeshCache(const SpMesh* mesh, SpTransform* transformation)
	{
		vertexes = ALLOC_NEW_ARRAY(Vec3, mesh->vertexLength());
		update(mesh, transformation);
	}


}