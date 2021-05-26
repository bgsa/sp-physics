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
		API_INTERFACE inline void build(SpMesh* mesh, SpMeshCache* cache, const SpTransform& transform, void* boundingVolume) override
		{
			AABB* aabb = (AABB*)boundingVolume;

			SpVertexMesh* vertex1 = mesh->vertexesMesh->get(0);

			const sp_uint indexUp = mesh->support(Vec3Up, cache->vertexes, vertex1)->index();
			const sp_uint indexDown = mesh->support(Vec3Down, cache->vertexes, vertex1)->index();
			const sp_uint indexLeft = mesh->support(Vec3Left, cache->vertexes, vertex1)->index();
			const sp_uint indexRight = mesh->support(Vec3Right, cache->vertexes, vertex1)->index();
			const sp_uint indexFront = mesh->support(Vec3Front, cache->vertexes, vertex1)->index();
			const sp_uint indexDepth = mesh->support(Vec3Depth, cache->vertexes, vertex1)->index();

			aabb->maxPoint[AABB_AXIS_Y] = cache->vertexes[indexUp].y;
			aabb->minPoint[AABB_AXIS_Y] = cache->vertexes[indexDown].y;
			if (aabb->minPoint[AABB_AXIS_Y] == aabb->maxPoint[AABB_AXIS_Y])
			{
				aabb->maxPoint[AABB_AXIS_Y] += 0.01f;
				aabb->minPoint[AABB_AXIS_Y] -= 0.01f;
			}

			aabb->maxPoint[AABB_AXIS_X] = cache->vertexes[indexRight].x;
			aabb->minPoint[AABB_AXIS_X] = cache->vertexes[indexLeft].x;
			if (aabb->minPoint[AABB_AXIS_X] == aabb->maxPoint[AABB_AXIS_X])
			{
				aabb->maxPoint[AABB_AXIS_X] += 0.01f;
				aabb->minPoint[AABB_AXIS_X] -= 0.01f;
			}

			aabb->maxPoint[AABB_AXIS_Z] = cache->vertexes[indexFront].z;
			aabb->minPoint[AABB_AXIS_Z] = cache->vertexes[indexDepth].z;
			if (aabb->minPoint[AABB_AXIS_Z] == aabb->maxPoint[AABB_AXIS_Z])
			{
				aabb->maxPoint[AABB_AXIS_Z] += 0.01f;
				aabb->minPoint[AABB_AXIS_Z] -= 0.01f;
			}
		}

		API_INTERFACE inline BoundingVolumeType boundingVolumeType() const override
		{
			return BoundingVolumeType::AABB;
		}

#ifdef OPENCL_ENABLED

		API_INTERFACE inline void init(GpuDevice* gpu, GpuBufferOpenCL* inputLengthGPU, sp_uint inputLength, GpuBufferOpenCL* meshCacheGPU, GpuBufferOpenCL* meshCacheIndexes, GpuBufferOpenCL* meshCacheVertexesLength, cl_mem transformationsGPU) override
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

		API_INTERFACE inline void initOutput(GpuDevice* gpu, const sp_uint objectsLength) override
		{
			this->gpu = gpu;
			_boundingVolumesGPU = gpu->createBuffer(sizeof(AABB) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
			_releaseBoundingVolumeGPU = true;
		}

#endif

	};

}

#endif // DOP18_HEADER