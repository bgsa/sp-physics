#ifndef SP_SPHERE_BOUNDING_VOLUME_FACTORY_HEADER
#define SP_SPHERE_BOUNDING_VOLUME_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "SpMesh.h"
#include "SpBoundingVolumeFactory.h"

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Factory for Sphere bounding volume
	/// </summary>
	class SpSphereBoundingVolumeFactory
		: public SpBoundingVolumeFactory
	{
	public:

		/// <summary>
		/// Create a Sphere using the mesh provided and the cache
		/// </summary>
		/// <param name="mesh">Geometry</param>
		/// <param name="cache">Transformed vertexes</param>
		/// <param name="position">Center of object</param>
		/// <param name="output">AABB created</param>
		/// <returns>void</returns>
		API_INTERFACE static void build(SpMesh* mesh, SpMeshCache* cache, const Vec3& position, Sphere* output)
		{
			sp_float distance = position.squaredDistance(cache->vertexes[0u]);

			for (sp_uint i = 1u; i < mesh->vertexLength(); i++)
			{
				const sp_float currentDistance = position.squaredDistance(cache->vertexes[i]);

				if (currentDistance > distance)
					distance = currentDistance;
			}

			output->center = position;
			output->ray = sqrtf(distance);
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
				->setInputParameter(_boundingVolumesGPU, inputLength * sizeof(Sphere))
				->buildFromProgram(program, "buildSphere");
		}

#endif

	};

}

#endif // DOP18_HEADER