#ifndef SP_DOP18_FACTORY_HEADER
#define SP_DOP18_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "DOP18.h"
#include "SpBoundingVolumeFactory.h"
#include "SpMesh.h"

#ifdef OPENCL_ENABLED
	#include "FileSystem.h"
	#include "GpuCommand.h"
#endif

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Factory for k-DOP with 9 orientations and 18 Polytopos
	/// </summary>
	class SpDOP18Factory
		: public SpBoundingVolumeFactory
	{
	public:

		/// <summary>
		/// Create a 18-DOP using the mesh provided and the cache
		/// </summary>
		/// <param name="mesh">Geometry</param>
		/// <param name="cache">Transformed vertexes</param>
		/// <param name="position">Center of object</param>
		/// <param name="output">18-DOP created</param>
		/// <returns>void</returns>
		API_INTERFACE inline void build(SpMesh* mesh, SpMeshCache* cache, const SpTransform& transform, void* boundingVolume) override
		{
			DOP18* dop18 = (DOP18*)boundingVolume;
			SpVertexMesh* vertex1 = mesh->vertexesMesh->get(0);

			const sp_uint indexUp = mesh->support(Vec3Up, cache->vertexes, vertex1)->index();
			const sp_uint indexDown = mesh->support(Vec3Down, cache->vertexes, vertex1)->index();
			const sp_uint indexLeft = mesh->support(Vec3Left, cache->vertexes, vertex1)->index();
			const sp_uint indexRight = mesh->support(Vec3Right, cache->vertexes, vertex1)->index();
			const sp_uint indexFront = mesh->support(Vec3Front, cache->vertexes, vertex1)->index();
			const sp_uint indexDepth = mesh->support(Vec3Depth, cache->vertexes, vertex1)->index();

			dop18->max[DOP18_AXIS_Y] = cache->vertexes[indexUp].y;
			dop18->min[DOP18_AXIS_Y] = cache->vertexes[indexDown].y;
			if (dop18->min[DOP18_AXIS_Y] == dop18->max[DOP18_AXIS_Y])
			{
				dop18->max[DOP18_AXIS_Y] += 0.01f;
				dop18->min[DOP18_AXIS_Y] -= 0.01f;
			}

			dop18->max[DOP18_AXIS_X] = cache->vertexes[indexRight].x;
			dop18->min[DOP18_AXIS_X] = cache->vertexes[indexLeft].x;
			if (dop18->min[DOP18_AXIS_X] == dop18->max[DOP18_AXIS_X])
			{
				dop18->max[DOP18_AXIS_X] += 0.01f;
				dop18->min[DOP18_AXIS_X] -= 0.01f;
			}

			dop18->max[DOP18_AXIS_Z] = cache->vertexes[indexFront].z;
			dop18->min[DOP18_AXIS_Z] = cache->vertexes[indexDepth].z;
			if (dop18->min[DOP18_AXIS_Z] == dop18->max[DOP18_AXIS_Z])
			{
				dop18->max[DOP18_AXIS_Z] += 0.01f;
				dop18->min[DOP18_AXIS_Z] -= 0.01f;
			}

			const sp_uint indexUpLeft = mesh->support(DOP18_NORMALS[DOP18_PLANES_UP_LEFT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexDownRight = mesh->support(DOP18_NORMALS[DOP18_PLANES_DOWN_RIGHT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexUpRight = mesh->support(DOP18_NORMALS[DOP18_PLANES_UP_RIGHT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexDownLeft = mesh->support(DOP18_NORMALS[DOP18_PLANES_DOWN_LEFT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexUpFront = mesh->support(DOP18_NORMALS[DOP18_PLANES_UP_FRONT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexDownDepth = mesh->support(DOP18_NORMALS[DOP18_PLANES_DOWN_DEPTH_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexUpDepth = mesh->support(DOP18_NORMALS[DOP18_PLANES_UP_DEPTH_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexDownFront = mesh->support(DOP18_NORMALS[DOP18_PLANES_DOWN_FRONT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexLeftDepth = mesh->support(DOP18_NORMALS[DOP18_PLANES_LEFT_DEPTH_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexRightFront = mesh->support(DOP18_NORMALS[DOP18_PLANES_RIGHT_FRONT_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexRightDepth = mesh->support(DOP18_NORMALS[DOP18_PLANES_RIGHT_DEPTH_INDEX], cache->vertexes, vertex1)->index();
			const sp_uint indexLeftFront = mesh->support(DOP18_NORMALS[DOP18_PLANES_LEFT_FRONT_INDEX], cache->vertexes, vertex1)->index();

			Vec3 vertex = cache->vertexes[indexUpLeft];
			dop18->min[DOP18_AXIS_UP_LEFT] = vertex.x - (vertex.y - transform.position.y);

			vertex = cache->vertexes[indexDownRight];
			dop18->max[DOP18_AXIS_UP_LEFT] = vertex.x + (transform.position.y - vertex.y);

			vertex = cache->vertexes[indexUpRight];
			dop18->max[DOP18_AXIS_UP_RIGHT] = vertex.x + (vertex.y - transform.position.y);

			vertex = cache->vertexes[indexDownLeft];
			dop18->min[DOP18_AXIS_UP_RIGHT] = vertex.x + (vertex.y - transform.position.y);

			vertex = cache->vertexes[indexUpFront];
			dop18->max[DOP18_AXIS_UP_FRONT] = vertex.z + (vertex.y - transform.position.y);

			vertex = cache->vertexes[indexDownDepth];
			dop18->min[DOP18_AXIS_UP_FRONT] = vertex.z + (vertex.y - transform.position.y);

			vertex = cache->vertexes[indexUpDepth];
			dop18->min[DOP18_AXIS_UP_DEPTH] = vertex.z + (transform.position.y - vertex.y);

			vertex = cache->vertexes[indexDownFront];
			dop18->max[DOP18_AXIS_UP_DEPTH] = vertex.z + (transform.position.y - vertex.y);

			vertex = cache->vertexes[indexLeftDepth];
			dop18->min[DOP18_AXIS_LEFT_DEPTH] = vertex.x + (vertex.z - transform.position.z);

			vertex = cache->vertexes[indexRightFront];
			dop18->max[DOP18_AXIS_LEFT_DEPTH] = vertex.x + (vertex.z - transform.position.z);
		
			vertex = cache->vertexes[indexRightDepth];
			dop18->max[DOP18_AXIS_RIGHT_DEPTH] = vertex.x + (transform.position.z - vertex.z);

			vertex = cache->vertexes[indexLeftFront];
			dop18->min[DOP18_AXIS_RIGHT_DEPTH] = vertex.x + (transform.position.z - vertex.z);
		}

		API_INTERFACE inline BoundingVolumeType boundingVolumeType() const override
		{
			return BoundingVolumeType::DOP18;
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
				->setInputParameter(_boundingVolumesGPU, inputLength * sizeof(DOP18))
				->buildFromProgram(program, "buildDOP18");
		}

		API_INTERFACE inline void initOutput(GpuDevice* gpu, const sp_uint objectsLength) override
		{
			this->gpu = gpu;
			_boundingVolumesGPU = gpu->createBuffer(sizeof(DOP18) * objectsLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
			_releaseBoundingVolumeGPU = true;
		}

#endif

	};

}

#endif // DOP18_HEADER