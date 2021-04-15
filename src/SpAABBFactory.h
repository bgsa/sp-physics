#ifndef SP_AABB_FACTORY_HEADER
#define SP_AABB_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "SpMesh.h"
#include "SpBoundingVolumeFactory.h"

#ifdef OPENCL_ENABLED
	#include "GpuCommand.h"
#endif

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Factory for AABB (6-DOP) with 3 orientations and 6 Polytopos
	/// </summary>
	class SpAABBFactory
		: public SpBoundingVolumeFactory
	{
	public:

		/// <summary>
		/// Create a AABB using the mesh provided and the cache
		/// </summary>
		/// <param name="mesh">Geometry</param>
		/// <param name="cache">Transformed vertexes</param>
		/// <param name="position">Center of object</param>
		/// <param name="output">AABB created</param>
		/// <returns>void</returns>
		API_INTERFACE static void build(SpMesh* mesh, SpMeshCache* cache, const Vec3& position, DOP18* output)
		{
			SpVertexMesh* vertex1 = mesh->vertexesMesh->get(0);

			const sp_uint indexUp = mesh->support(Vec3Up, cache->vertexes, vertex1)->index();
			const sp_uint indexDown = mesh->support(Vec3Down, cache->vertexes, vertex1)->index();
			const sp_uint indexLeft = mesh->support(Vec3Left, cache->vertexes, vertex1)->index();
			const sp_uint indexRight = mesh->support(Vec3Right, cache->vertexes, vertex1)->index();
			const sp_uint indexFront = mesh->support(Vec3Front, cache->vertexes, vertex1)->index();
			const sp_uint indexDepth = mesh->support(Vec3Depth, cache->vertexes, vertex1)->index();

			output->max[AABB_AXIS_Y] = cache->vertexes[indexUp].y;
			output->min[AABB_AXIS_Y] = cache->vertexes[indexDown].y;
			if (output->min[AABB_AXIS_Y] == output->max[AABB_AXIS_Y])
			{
				output->max[AABB_AXIS_Y] += 0.01f;
				output->min[AABB_AXIS_Y] -= 0.01f;
			}

			output->max[AABB_AXIS_X] = cache->vertexes[indexRight].x;
			output->min[AABB_AXIS_X] = cache->vertexes[indexLeft].x;
			if (output->min[AABB_AXIS_X] == output->max[AABB_AXIS_X])
			{
				output->max[AABB_AXIS_X] += 0.01f;
				output->min[AABB_AXIS_X] -= 0.01f;
			}

			output->max[AABB_AXIS_Z] = cache->vertexes[indexFront].z;
			output->min[AABB_AXIS_Z] = cache->vertexes[indexDepth].z;
			if (output->min[AABB_AXIS_Z] == output->max[AABB_AXIS_Z])
			{
				output->max[AABB_AXIS_Z] += 0.01f;
				output->min[AABB_AXIS_Z] -= 0.01f;
			}
		}

#ifdef OPENCL_ENABLED

		API_INTERFACE void init(GpuDevice* gpu, GpuBufferOpenCL* inputLengthGPU, sp_uint inputLength, GpuBufferOpenCL* meshCacheGPU, GpuBufferOpenCL* meshCacheIndexes, GpuBufferOpenCL* meshCacheVertexesLength, cl_mem transformationsGPU)
		{
			initProgram(gpu);

			globalWorkSize[0] = inputLength;
			localWorkSize[0] = gpu->getGroupLength(inputLength, inputLength);

			command = gpu->commandManager->createCommand()
				->setInputParameter(inputLengthGPU)
				->setInputParameter(meshCacheIndexes)
				->setInputParameter(meshCacheVertexesLength)
				->setInputParameter(meshCacheGPU)
				->setInputParameter(transformationsGPU, inputLength * sizeof(SpTransform))
				->setInputParameter(_boundingVolumesGPU, inputLength * sizeof(AABB))
				->buildFromProgram(program, "buildAABB");
		}

#endif

	};

}

#endif // DOP18_HEADER