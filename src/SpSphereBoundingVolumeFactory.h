#ifndef SP_SPHERE_BOUNDING_VOLUME_FACTORY_HEADER
#define SP_SPHERE_BOUNDING_VOLUME_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "SpMesh.h"
#include "FileSystem.h"

#ifdef OPENCL_ENABLED
	#include "GpuCommand.h"
#endif

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Factory for Sphere bounding volume
	/// </summary>
	class SpSphereBoundingVolumeFactory
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
				->setInputParameter(output, inputLength * sizeof(Sphere))
				->buildFromProgram(program, "buildSphere");
		}

		API_INTERFACE void buildGPU() const
		{
			command->execute(1u, globalWorkSize, localWorkSize);
		}

#endif

	};

}

#endif // DOP18_HEADER