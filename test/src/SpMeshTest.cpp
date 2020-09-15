#include "SpectrumPhysicsTest.h"
#include "SpMesh.h"

#define CLASS_NAME SpMeshTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:
	
		void createMesh(SpMesh* mesh)
		{
			SpArray<Vec3> vertexes(24);
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

			mesh->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexes.length());
			for (sp_uint i = 0; i < vertexes.length(); i++)
				mesh->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh, i, vertexes.get(i)));

			SpPoint3<sp_uint>* facesIndexes = sp_mem_new_array(SpPoint3<sp_uint>, 42);
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
			facesIndexes[11] = SpPoint3<sp_uint>(19, 16, 10);
			facesIndexes[12] = SpPoint3<sp_uint>(20, 9, 3);
			facesIndexes[13] = SpPoint3<sp_uint>(23, 7, 14);
			facesIndexes[14] = SpPoint3<sp_uint>(6, 11, 15);
			facesIndexes[15] = SpPoint3<sp_uint>(4, 12, 5);
			facesIndexes[16] = SpPoint3<sp_uint>(1, 4, 5);
			facesIndexes[17] = SpPoint3<sp_uint>(5, 8, 1);
			facesIndexes[18] = SpPoint3<sp_uint>(2, 1, 24);
			facesIndexes[19] = SpPoint3<sp_uint>(24, 22, 2);
			facesIndexes[20] = SpPoint3<sp_uint>(18, 17, 22);
			facesIndexes[21] = SpPoint3<sp_uint>(22, 21, 18);
			facesIndexes[22] = SpPoint3<sp_uint>(12, 11, 6);
			facesIndexes[23] = SpPoint3<sp_uint>(6, 5, 12);
			facesIndexes[24] = SpPoint3<sp_uint>(10, 9, 20);
			facesIndexes[25] = SpPoint3<sp_uint>(20, 19, 10);
			facesIndexes[26] = SpPoint3<sp_uint>(23, 24, 8);
			facesIndexes[27] = SpPoint3<sp_uint>(8, 7, 23);
			facesIndexes[28] = SpPoint3<sp_uint>(16, 15, 11);
			facesIndexes[29] = SpPoint3<sp_uint>(11, 10, 16);
			facesIndexes[30] = SpPoint3<sp_uint>(13, 16, 19);
			facesIndexes[31] = SpPoint3<sp_uint>(19, 18, 13);
			facesIndexes[32] = SpPoint3<sp_uint>(4, 3, 9);
			facesIndexes[33] = SpPoint3<sp_uint>(9, 12, 4);
			facesIndexes[34] = SpPoint3<sp_uint>(15, 14, 7);
			facesIndexes[35] = SpPoint3<sp_uint>(7, 6, 15);
			facesIndexes[36] = SpPoint3<sp_uint>(14, 13, 21);
			facesIndexes[37] = SpPoint3<sp_uint>(21, 23, 14);
			facesIndexes[38] = SpPoint3<sp_uint>(3, 2, 17);
			facesIndexes[39] = SpPoint3<sp_uint>(17, 20, 3);
			facesIndexes[40] = SpPoint3<sp_uint>(24, 23, 21);
			facesIndexes[41] = SpPoint3<sp_uint>(21, 22, 24);

			mesh->faces = sp_mem_new(SpArray<SpFaceMesh*>)(42);
			for (sp_uint i = 0; i < 42; i++)
				mesh->faces->add(sp_mem_new(SpFaceMesh)(mesh, i, facesIndexes[i].x - 1, facesIndexes[i].y - 1, facesIndexes[i].z - 1));

			mesh->init();
		}

	public:
		SP_TEST_METHOD_DEF(init);
	};

	SP_TEST_METHOD(CLASS_NAME, init)
	{
		SpMesh mesh = SpMesh();
		createMesh(&mesh);

		for (sp_uint i = 0; i < mesh.vertexLength(); i++)
		{
			const sp_uint edgesLength = mesh.vertexesMesh->get(i)->edgeLength();
			Assert::IsTrue(edgesLength >= 4 || edgesLength <= 7, L"Wrong value");
		}
	}
}

#undef CLASS_NAME
