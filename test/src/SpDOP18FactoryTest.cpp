#include "SpectrumPhysicsTest.h"
#include <SpPhysicSimulator.h>

#define CLASS_NAME SpDOP18FactoryTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:

		void createMeshes2(const sp_uint index)
		{
			const sp_uint vertexesLength = 24;
			SpArray<Vec3> vertexes(vertexesLength);
			vertexes.add({ -0.179303f, -1.000000f, -0.179303f });
			vertexes.add({ 0.179303f, -1.000000f, -0.179303f });
			vertexes.add({ 0.179303f, -1.000000f,  0.179303f });
			vertexes.add({ -0.179303f, -1.000000f,  0.179303f });
			vertexes.add({ -1.000000f, -0.179303f,  0.179303f });
			vertexes.add({ -1.000000f,  0.179303f,  0.179303f });
			vertexes.add({ -1.000000f,  0.179303f, -0.179303f });
			vertexes.add({ -1.000000f, -0.179303f, -0.179303f });
			vertexes.add({ 0.179303f, -0.179303f,  1.000000f });
			vertexes.add({ 0.179303f,  0.179303f,  1.000000f });
			vertexes.add({ -0.179303f,  0.179303f,  1.000000f });
			vertexes.add({ -0.179303f, -0.179303f,  1.000000f });
			vertexes.add({ 0.179303f,  1.000000f, -0.179303f });
			vertexes.add({ -0.179303f,  1.000000f, -0.179303f });
			vertexes.add({ -0.179303f,  1.000000f,  0.179303f });
			vertexes.add({ 0.179303f,  1.000000f,  0.179303f });
			vertexes.add({ 1.000000f, -0.179303f, -0.179303f });
			vertexes.add({ 1.000000f,  0.179303f, -0.179303f });
			vertexes.add({ 1.000000f,  0.179303f,  0.179303f });
			vertexes.add({ 1.000000f, -0.179303f,  0.179303f });
			vertexes.add({ 0.179303f,  0.179303f, -1.000000f });
			vertexes.add({ 0.179303f, -0.179303f, -1.000000f });
			vertexes.add({ -0.179303f,  0.179303f, -1.000000f });
			vertexes.add({ -0.179303f, -0.179303f, -1.000000f });

			const sp_uint facesLength = 44;
			SpPoint3<sp_uint>* facesIndexes = sp_mem_new_array(SpPoint3<sp_uint>, facesLength);
			facesIndexes[0] = SpPoint3<sp_uint>(1, 2, 3);
			facesIndexes[1] = SpPoint3<sp_uint>(3, 4, 1);
			facesIndexes[2] = SpPoint3<sp_uint>(5, 6, 7);
			facesIndexes[3] = SpPoint3<sp_uint>(7, 8, 5);
			facesIndexes[4] = SpPoint3<sp_uint>(9, 10, 11);
			facesIndexes[5] = SpPoint3<sp_uint>(11, 12, 9);
			facesIndexes[6] = SpPoint3<sp_uint>(13, 14, 15);
			facesIndexes[7] = SpPoint3<sp_uint>(15, 16, 13);
			facesIndexes[8] = SpPoint3<sp_uint>(17, 18, 19);
			facesIndexes[9] = SpPoint3<sp_uint>(19, 20, 17);
			facesIndexes[10] = SpPoint3<sp_uint>(21, 13, 18);
			facesIndexes[11] = SpPoint3<sp_uint>(2, 22, 17);
			facesIndexes[12] = SpPoint3<sp_uint>(19, 16, 10);
			facesIndexes[13] = SpPoint3<sp_uint>(20, 9, 3);
			facesIndexes[14] = SpPoint3<sp_uint>(23, 7, 14);
			facesIndexes[15] = SpPoint3<sp_uint>(8, 24, 1);
			facesIndexes[16] = SpPoint3<sp_uint>(6, 11, 15);
			facesIndexes[17] = SpPoint3<sp_uint>(4, 12, 5);
			facesIndexes[18] = SpPoint3<sp_uint>(1, 4, 5);
			facesIndexes[19] = SpPoint3<sp_uint>(5, 8, 1);
			facesIndexes[20] = SpPoint3<sp_uint>(2, 1, 24);
			facesIndexes[21] = SpPoint3<sp_uint>(24, 22, 2);
			facesIndexes[22] = SpPoint3<sp_uint>(18, 17, 22);
			facesIndexes[23] = SpPoint3<sp_uint>(22, 21, 18);
			facesIndexes[24] = SpPoint3<sp_uint>(12, 11, 6);
			facesIndexes[25] = SpPoint3<sp_uint>(6, 5, 12);
			facesIndexes[26] = SpPoint3<sp_uint>(10, 9, 20);
			facesIndexes[27] = SpPoint3<sp_uint>(20, 19, 10);
			facesIndexes[28] = SpPoint3<sp_uint>(23, 24, 8);
			facesIndexes[29] = SpPoint3<sp_uint>(8, 7, 23);
			facesIndexes[30] = SpPoint3<sp_uint>(16, 15, 11);
			facesIndexes[31] = SpPoint3<sp_uint>(11, 10, 16);
			facesIndexes[32] = SpPoint3<sp_uint>(13, 16, 19);
			facesIndexes[33] = SpPoint3<sp_uint>(19, 18, 13);
			facesIndexes[34] = SpPoint3<sp_uint>(4, 3, 9);
			facesIndexes[35] = SpPoint3<sp_uint>(9, 12, 4);
			facesIndexes[36] = SpPoint3<sp_uint>(15, 14, 7);
			facesIndexes[37] = SpPoint3<sp_uint>(7, 6, 15);
			facesIndexes[38] = SpPoint3<sp_uint>(14, 13, 21);
			facesIndexes[39] = SpPoint3<sp_uint>(21, 23, 14);
			facesIndexes[40] = SpPoint3<sp_uint>(3, 2, 17);
			facesIndexes[41] = SpPoint3<sp_uint>(17, 20, 3);
			facesIndexes[42] = SpPoint3<sp_uint>(24, 23, 21);
			facesIndexes[43] = SpPoint3<sp_uint>(21, 22, 24);

			SpMesh* mesh = sp_mem_new(SpMesh)();

			mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexesLength);
			for (sp_uint i = 0; i < vertexesLength; i++)
				mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes.get(i)));


			mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(facesLength);
			for (sp_uint i = 0; i < facesLength; i++)
				mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, i, facesIndexes[i].x - 1, facesIndexes[i].y - 1, facesIndexes[i].z - 1));

			mesh->init();

			SpPhysicSimulator::instance()->mesh(index, mesh);
		}

		void resetObject(const sp_uint index)
		{
			SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

			simulator->transforms(index)->reset();

			simulator->rigidBodies(index)->currentState.reset();
			simulator->rigidBodies(index)->previousState.reset();
			simulator->rigidBodies(index)->mass(8.0f);
		}

	public:
		SP_TEST_METHOD_DEF(build);
	};

	SP_TEST_METHOD(CLASS_NAME, build)
	{
		TestPhysic::lock();

		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();

		createMeshes2(0u);
		resetObject(0u);

		SpTransform* transform = simulator->transforms(0u);
		transform->scaleVector *= 2.0f;
		transform->position = Vec3(2.0f, 0.0f, 0.0f);

		SpMesh* mesh = simulator->mesh(0u);
		SpMeshCache* cache = ALLOC_NEW(SpMeshCache)(mesh->vertexesMesh->length());
		cache->update(mesh, transform);

		DOP18* result = simulator->boundingVolumes(0u);

		SpDOP18Factory::build(mesh, cache, transform->position, result);
		
		Assert::IsTrue(isCloseEnough(result->max[0], 4.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[1], 2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[2], 2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_LEFT], 4.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_RIGHT], 4.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_FRONT], 2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_UP_DEPTH], 2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_LEFT_DEPTH], 4.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->max[DOP18_AXIS_RIGHT_DEPTH], 4.3586f), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(isCloseEnough(result->min[0], 0.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[1], -2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[2], -2.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_LEFT], -0.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_RIGHT], -0.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_FRONT], -2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_UP_DEPTH], -2.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_LEFT_DEPTH], -0.3586f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result->min[DOP18_AXIS_RIGHT_DEPTH], -0.3586f), L"Wrong value.", LINE_INFO());

		TestPhysic::unlock();
	}

}

#undef CLASS_NAME
