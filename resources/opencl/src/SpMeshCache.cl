#ifndef SP_MESH_CACHE_HEADER
#define SP_MESH_CACHE_HEADER

#include "OpenCLBase.cl"
#include "Vec3.cl"
#include "Quat.cl"
#include "SpTransformation.cl"

__kernel void update (
    __global   sp_uint * meshCacheLength,
    __constant sp_float* meshes,
    __constant sp_uint * meshesIndexes,
    __constant sp_uint * meshesIndexesLength,
    __global   sp_float* transformations,
    __constant sp_uint * meshCacheIndexes,
    __global   sp_float* meshCache
)
{
    if (THREAD_ID + 1u > *meshCacheLength)
        return;

    const sp_uint transformationIndex = THREAD_ID * SP_TRANSFORMATION_STRIDER;

    const sp_uint vertexLength = meshesIndexesLength[THREAD_ID];
    sp_uint meshVertexIndex = meshesIndexes[THREAD_ID * 3u] + 1u;
    sp_uint meshCacheIndex = meshCacheIndexes[THREAD_ID];

    for (sp_uint i = 0; i < vertexLength; i++)
    {
        Vec3 vertex;
        vertex.x = meshes[meshVertexIndex     ];
        vertex.y = meshes[meshVertexIndex + 1u];
        vertex.z = meshes[meshVertexIndex + 2u];


        Vec3 outputVertex;
        sp_transformation_transform(transformations, transformationIndex, vertex, &outputVertex);

        meshCache[meshCacheIndex    ] = outputVertex.x;
        meshCache[meshCacheIndex + 1] = outputVertex.y;
        meshCache[meshCacheIndex + 2] = outputVertex.z;

        meshVertexIndex += 26u; // 26 = VERTEX_STRIDE
        meshCacheIndex += 3u;
    }
}

#endif // SP_MESH_CACHE_HEADER