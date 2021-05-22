#include "SpectrumPhysicsTest.h"
#include <ObjModel.h>
#include <SpWorldManager.h>

#define CLASS_NAME SpMeshCacheTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

#ifdef OPENCL_ENABLED
		SP_TEST_METHOD_DEF(updateGPU_1);
#endif

	};

#ifdef OPENCL_ENABLED
	SP_TEST_METHOD(CLASS_NAME, updateGPU_1)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		ObjModel model;
		model.load("resources\\models\\rocks\\obj.obj");

		SpMesh* mesh1 = sp_mem_new(SpMesh)();
		model.buildMesh(mesh1);
		SpWorldManagerInstance->current()->mesh(0, mesh1);
		SpWorldManagerInstance->current()->collisionFeatures(0, 0);
		SpWorldManagerInstance->current()->transforms(0)->position = Vec3(0.321708f, -35.486710f, 0.365792f);
		//SpWorldManagerInstance->current()->transforms(0)->orientation = Quat(0.836536825f, 0.158000097f, 0.348789155f, 0.391903371f);
		SpWorldManagerInstance->current()->transforms(0)->orientation = QUAT_UNIT;
		SpWorldManagerInstance->current()->transforms(0)->scaleVector = Vec3(2.0f, 2.0f, 2.0f);

		SpEdgeMesh* lastEdge = mesh1->edges->data()[mesh1->edges->length() - 1u];
		sp_size lastMemoryAddress = (sp_size)&lastEdge->faces.data()[lastEdge->faces.length() - 1u];
		sp_uint mesh1Size = lastMemoryAddress - ((sp_size)mesh1);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("SpMeshCache.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_mem_delete(filename, SpDirectory);

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size globalWorkSize[3] = { 1, 0, 0 };
		const sp_size localWorkSize[3] = { 1, 0, 0 };

		sp_uint indexesLength = 1u;
		sp_uint vertexesLength = mesh1->vertexLength();
		sp_size vertexMeshIndex = 36928u;
		sp_uint meshCacheIndex = 0u;

		const sp_size initialMemoryIndex = (sp_size)mesh1;
		sp_size vertexMemoryIndex = (sp_size)mesh1->vertexesMesh->data()[0];
		vertexMeshIndex = divideBy4(vertexMemoryIndex - initialMemoryIndex) + 1;
		sp_size vertexMemoryIndex2 = (sp_size)mesh1->vertexesMesh->data()[1];
		sp_size meshVertexStride = divideBy4(vertexMemoryIndex2 - vertexMemoryIndex);

		sp_uint meshesStride[3] = { meshVertexStride, 0u, 0u };

		cl_mem meshesGPU = gpu->createBuffer(mesh1, mesh1Size, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem indexesLengthGPU = gpu->createBuffer(&indexesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshIndexGPU = gpu->createBuffer(&vertexMeshIndex, sizeof(sp_size), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshStrideGPU = gpu->createBuffer(meshesStride, sizeof(sp_uint) * 3, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshIndexesLengthGPU = gpu->createBuffer(&vertexesLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem transformsGPU = gpu->createBuffer(SpWorldManagerInstance->current()->transforms(0), sizeof(SpTransform), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem meshCacheIndexesGPU = gpu->createBuffer(&meshCacheIndex, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		sp_uint meshCacheSize = sizeof(Vec3) * mesh1->vertexLength();
		cl_mem meshCacheGPU = gpu->createBuffer(meshCacheSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* command = gpu->commandManager
			->createCommand()
			->setInputParameter(indexesLengthGPU, sizeof(sp_uint))
			->setInputParameter(meshesGPU, mesh1Size)
			->setInputParameter(meshIndexGPU, sizeof(sp_size))
			->setInputParameter(meshStrideGPU, sizeof(sp_uint) * 3)
			->setInputParameter(meshIndexesLengthGPU, sizeof(sp_uint))
			->setInputParameter(transformsGPU, sizeof(SpTransform))
			->setInputParameter(meshCacheIndexesGPU, sizeof(sp_uint))
			->setInputParameter(meshCacheGPU, meshCacheSize)
			->buildFromProgram(program, "update");

		command->execute(1, globalWorkSize, localWorkSize, 0, 0, NULL, NULL);

		sp_float* vertexes = ALLOC_ARRAY(sp_float, mesh1->vertexLength() * 3);
		command->fetchInOutParameter(vertexes, 7);

		SpTransform transform = *SpWorldManagerInstance->current()->transforms(0);
		sp_float* expected = ALLOC_ARRAY(sp_float, mesh1->vertexLength() * 3);
		for (sp_uint i = 0; i < mesh1->vertexLength(); i++)
		{
			Vec3 output;
			mesh1->vertex(i, transform, &output);
			std::memcpy(&expected[i * 3], output, sizeof(Vec3));
		}

		for (sp_uint i = 0; i < mesh1->vertexLength() * 3; i++)
			Assert::AreEqual(vertexes[i], expected[i], L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(vertexes);
	}
#endif
	
}

#undef CLASS_NAME
