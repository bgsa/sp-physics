#ifndef SP_MESH_CACHE_HEADER
#define SP_MESH_CACHE_HEADER

#include "OpenCLBase.cl"
#include "Vec3.cl"
#include "Quat.cl"
#include "SpTransformation.cl"

__kernel void update (
    __global   sp_uint * meshCacheLength,
    __constant sp_float* meshes,
    __constant sp_size * meshesIndexes,      // Indexes of Mesh (Vertex Index, Face Index, Edge Index) for each mesh
    __constant sp_uint * meshesStrides,      // Strides of Mesh (Vertex, Face and Edge) - 3 strides for each mesh
    __constant sp_uint * meshesVertexLength, // Vertexes Length for each mesh
    __global   sp_float* transformations,
    __constant sp_uint * meshCacheIndexes,   // Output Index
    __global   sp_float* meshCache
)
{
    if (THREAD_ID + 1u > *meshCacheLength)
        return;

    const sp_uint transformationIndex = THREAD_ID * SP_TRANSFORMATION_STRIDER;

    const sp_uint vertexLength = meshesVertexLength[THREAD_ID];
    sp_size meshVertexIndex = meshesIndexes[THREAD_ID * 3u];
    sp_uint meshCacheIndex = meshCacheIndexes[THREAD_ID];

    const sp_uint vertexStride = meshesStrides[THREAD_ID * 3u];

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

        meshVertexIndex += vertexStride;
        meshCacheIndex += 3u;
    }
}

#endif // SP_MESH_CACHE_HEADER