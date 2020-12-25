#ifndef SP_DOP18_FACTORY_HEADER
#define SP_DOP18_FACTORY_HEADER

#include "SpectrumPhysics.h"
#include "DOP18.h"
#include "SpMesh.h"

#ifdef OPENCL_ENABLED
	#include "GpuCommand.h"
#endif

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Factory for k-DOP with 9 orientations and 18 Polytopos
	/// </summary>
	class SpDOP18Factory
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
		cl_event lastEvent = nullptr;

		/// <summary>
		/// Create a 18-DOP using the mesh provided and the cache
		/// </summary>
		/// <param name="mesh">Geometry</param>
		/// <param name="cache">Transformed vertexes</param>
		/// <param name="position">Center of object</param>
		/// <param name="output">18-DOP created</param>
		/// <returns>void</returns>
		API_INTERFACE static void build(SpMesh* mesh, SpMeshCache* cache, const Vec3& position, DOP18* output)
		{
			SpVertexMesh* vertex1 = mesh->vertexesMesh->get(0);

			const sp_uint indexUp = mesh->findExtremeVertexDirection(Vec3Up, cache, position, vertex1)->index();
			const sp_uint indexDown = mesh->findExtremeVertexDirection(Vec3Down, cache, position, vertex1)->index();
			const sp_uint indexLeft = mesh->findExtremeVertexDirection(Vec3Left, cache, position, vertex1)->index();
			const sp_uint indexRight = mesh->findExtremeVertexDirection(Vec3Right, cache, position, vertex1)->index();
			const sp_uint indexFront = mesh->findExtremeVertexDirection(Vec3Front, cache, position, vertex1)->index();
			const sp_uint indexDepth = mesh->findExtremeVertexDirection(Vec3Depth, cache, position, vertex1)->index();

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

			const sp_uint indexUpLeft = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_UP_LEFT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexDownRight = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_DOWN_RIGHT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexUpRight = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_UP_RIGHT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexDownLeft = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_DOWN_LEFT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexUpFront = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_UP_FRONT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexDownDepth = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_DOWN_DEPTH_INDEX], cache, position, vertex1)->index();
			const sp_uint indexUpDepth = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_UP_DEPTH_INDEX], cache, position, vertex1)->index();
			const sp_uint indexDownFront = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_DOWN_FRONT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexLeftDepth = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_LEFT_DEPTH_INDEX], cache, position, vertex1)->index();
			const sp_uint indexRightFront = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_RIGHT_FRONT_INDEX], cache, position, vertex1)->index();
			const sp_uint indexRightDepth = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_RIGHT_DEPTH_INDEX], cache, position, vertex1)->index();
			const sp_uint indexLeftFront = mesh->findExtremeVertexDirection(DOP18_NORMALS[DOP18_PLANES_LEFT_FRONT_INDEX], cache, position, vertex1)->index();

			Vec3 vertex = cache->vertexes[indexUpLeft];
			output->min[DOP18_AXIS_UP_LEFT] = vertex.x - (vertex.y - position.y);

			vertex = cache->vertexes[indexDownRight];
			output->max[DOP18_AXIS_UP_LEFT] = vertex.x + (position.y - vertex.y);

			vertex = cache->vertexes[indexUpRight];
			output->max[DOP18_AXIS_UP_RIGHT] = vertex.x + (vertex.y - position.y);

			vertex = cache->vertexes[indexDownLeft];
			output->min[DOP18_AXIS_UP_RIGHT] = vertex.x + (vertex.y - position.y);

			vertex = cache->vertexes[indexUpFront];
			output->max[DOP18_AXIS_UP_FRONT] = vertex.z + (vertex.y - position.y);

			vertex = cache->vertexes[indexDownDepth];
			output->min[DOP18_AXIS_UP_FRONT] = vertex.z + (vertex.y - position.y);

			vertex = cache->vertexes[indexUpDepth];
			output->min[DOP18_AXIS_UP_DEPTH] = vertex.z + (position.y - vertex.y);

			vertex = cache->vertexes[indexDownFront];
			output->max[DOP18_AXIS_UP_DEPTH] = vertex.z + (position.y - vertex.y);

			vertex = cache->vertexes[indexLeftDepth];
			output->min[DOP18_AXIS_LEFT_DEPTH] = vertex.x + (vertex.z - position.z);

			vertex = cache->vertexes[indexRightFront];
			output->max[DOP18_AXIS_LEFT_DEPTH] = vertex.x + (vertex.z - position.z);
		
			vertex = cache->vertexes[indexRightDepth];
			output->max[DOP18_AXIS_RIGHT_DEPTH] = vertex.x + (position.z - vertex.z);

			vertex = cache->vertexes[indexLeftFront];
			output->min[DOP18_AXIS_RIGHT_DEPTH] = vertex.x + (position.z - vertex.z);
		}

#ifdef OPENCL_ENABLED

		API_INTERFACE void init(GpuDevice* gpu, GpuBufferOpenCL* inputLengthGPU, GpuBufferOpenCL* bodyMapperGPU, GpuBufferOpenCL* rigidBodiesGPU, GpuBufferOpenCL* softBodiesGPU, GpuBufferOpenCL* softBodyIndexesGPU, sp_uint inputLength, GpuBufferOpenCL* meshCacheGPU, GpuBufferOpenCL* meshCacheIndexes, GpuBufferOpenCL* meshCacheVertexesLength, cl_mem transformationsGPU, cl_mem output)
		{
			initProgram(gpu);

			globalWorkSize[0] = inputLength;
			localWorkSize[0] = gpu->getGroupLength(inputLength, inputLength);

			command = gpu->commandManager->createCommand()
				->setInputParameter(inputLengthGPU)
				->setInputParameter(bodyMapperGPU)
				->setInputParameter(rigidBodiesGPU)
				->setInputParameter(softBodiesGPU)
				->setInputParameter(softBodyIndexesGPU)
				->setInputParameter(meshCacheIndexes)
				->setInputParameter(meshCacheVertexesLength)
				->setInputParameter(meshCacheGPU)
				->setInputParameter(transformationsGPU, inputLength * sizeof(SpTransform))
				->setInputParameter(output, inputLength * sizeof(DOP18))
				->buildFromProgram(program, "buildDOP18");
		}

		API_INTERFACE void buildGPU(GpuDevice* gpu, cl_mem transformationsGPU)
		{
			command->execute(1u, globalWorkSize, localWorkSize);

			lastEvent = command->lastEvent;

			/* test
			sp_uint* p = ALLOC_ARRAY(sp_uint, 10000);
			command->fetchInOutParameter(p, 0u);

			command->fetchInOutParameter(p, 1u);

			sp_float* c = ALLOC_ARRAY(sp_float, 10000);
			command->fetchInOutParameter(c, 9u);
			*/
		}

#endif

	};

}

#endif // DOP18_HEADER