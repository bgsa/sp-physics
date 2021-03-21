#ifndef SP_AABB_FACTORY_HEADER
#define SP_AABB_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "AABB.h"
#include "DOP18.h"
#include "SpMesh.h"
#include "FileSystem.h"

#ifdef OPENCL_ENABLED
	#include "GpuCommand.h"
#endif

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Factory for AABB (6-DOP) with 3 orientations and 6 Polytopos
	/// </summary>
	class SpAABBFactory
	{
	private:

#ifdef OPENCL_ENABLED
		GpuCommand* command;
		sp_size globalWorkSize[3] = { 0, 0, 0 };
		sp_size localWorkSize[3] = { 0, 0, 0 };
		cl_program program;

		void initProgram(GpuDevice* gpu)
		{
			SpDirectory* filename = SpDirectory::currentDirectory()
				->add(SP_DIRECTORY_OPENCL_SOURCE)
				->add("BoundingVolumeFactory.cl");

			SP_FILE file;
			file.open(filename->name()->data(), std::ios::in);
			sp_mem_delete(filename, SpDirectory);
			sp_size fileSize = file.length();
			sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
			file.read(source, fileSize);
			file.close();

			const sp_char* buildOptions = nullptr;

			sp_uint programIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions);
			ALLOC_RELEASE(source);
			program = gpu->commandManager->cachedPrograms[programIndex];
		}
#endif

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

			output->max[DOP18_AXIS_Y] = cache->vertexes[indexUp].y;
			output->min[DOP18_AXIS_Y] = cache->vertexes[indexDown].y;
			if (output->min[DOP18_AXIS_Y] == output->max[DOP18_AXIS_Y])
			{
				output->max[DOP18_AXIS_Y] += 0.01f;
				output->min[DOP18_AXIS_Y] -= 0.01f;
			}

			output->max[DOP18_AXIS_X] = cache->vertexes[indexRight].x;
			output->min[DOP18_AXIS_X] = cache->vertexes[indexLeft].x;
			if (output->min[DOP18_AXIS_X] == output->max[DOP18_AXIS_X])
			{
				output->max[DOP18_AXIS_X] += 0.01f;
				output->min[DOP18_AXIS_X] -= 0.01f;
			}

			output->max[DOP18_AXIS_Z] = cache->vertexes[indexFront].z;
			output->min[DOP18_AXIS_Z] = cache->vertexes[indexDepth].z;
			if (output->min[DOP18_AXIS_Z] == output->max[DOP18_AXIS_Z])
			{
				output->max[DOP18_AXIS_Z] += 0.01f;
				output->min[DOP18_AXIS_Z] -= 0.01f;
			}
		}

#ifdef OPENCL_ENABLED

		API_INTERFACE void init(GpuDevice* gpu, GpuBufferOpenCL* inputLengthGPU, sp_uint inputLength, GpuBufferOpenCL* meshCacheGPU, GpuBufferOpenCL* meshCacheIndexes, GpuBufferOpenCL* meshCacheVertexesLength, cl_mem transformationsGPU, cl_mem output)
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
				->setInputParameter(output, inputLength * sizeof(DOP18))
				->buildFromProgram(program, "buildAABB");
		}

		API_INTERFACE void buildGPU() const
		{
			command->execute(1u, globalWorkSize, localWorkSize);
		}

#endif

	};

}

#endif // DOP18_HEADER