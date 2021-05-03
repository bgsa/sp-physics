#include "SpectrumPhysicsTest.h"
#include "Asserts.h"
#include <SpGJK.h>

#include <SpMatlabExporter.h>

#define CLASS_NAME SpGJKTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(epa1);
		SP_TEST_METHOD_DEF(epa2);
	};

	SP_TEST_METHOD(CLASS_NAME, epa1)
	{
		const sp_uint vertexLength = 8;
		const sp_uint faceLength = 12;

		Vec3 vertexesMesh1[vertexLength] = {
			Vec3(10.0f, 0.0f, 0.0f),
			Vec3(12.0f, 0.0f, 0.0f),
			Vec3(12.0f, 2.0f, 0.0f),
			Vec3(10.0f, 2.0f, 0.0f),
			Vec3(10.0f, 0.0f, 2.0f),
			Vec3(12.0f, 0.0f, 2.0f),
			Vec3(12.0f, 2.0f, 2.0f),
			Vec3(10.0f, 2.0f, 2.0f)
		};
		Vec3 vertexesMesh2[vertexLength] = {
			Vec3(8.5f, 0.0f, 0.0f),
			Vec3(10.5f, 0.0f, 0.0f),
			Vec3(10.5f, 2.0f, 0.0f),
			Vec3(8.5f, 2.0f, 0.0f),
			Vec3(8.5f, 0.0f, 2.0f),
			Vec3(10.5f, 0.0f, 2.0f),
			Vec3(10.5f, 2.0f, 2.0f),
			Vec3(8.5f, 2.0f, 2.0f)
		};

		sp_uint meshIndexes[36] = {
			0, 1, 2, // front1
			2, 3, 0, // front2
			0, 3, 7, // left1
			7, 4, 0, // left2
			0, 4, 5, // bottom1
			5, 1, 0, // bottom2
			6, 7, 3, // top1
			3, 2, 6, // top2
			6, 2, 1, // right1
			1, 5, 6, // right2
			6, 5, 4, // back1
			4, 7, 6  // back2
		};

		SpMesh* mesh1 = ALLOC_NEW(SpMesh)();
		mesh1->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexLength);
		mesh1->faces = sp_mem_new(SpArray<SpFaceMesh*>)(faceLength);

		for (sp_uint i = 0; i < vertexLength; i++)
			mesh1->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh1, i, vertexesMesh1[i]));

		for (sp_uint i = 0; i < faceLength; i++)
		{
			sp_uint faceIndex = 3u * i;

			mesh1->faces->add(sp_mem_new(SpFaceMesh)(mesh1, i,
				meshIndexes[faceIndex],
				meshIndexes[faceIndex + 1u],
				meshIndexes[faceIndex + 2u])
			);
		}
		mesh1->init();

		SpMesh* mesh2 = ALLOC_NEW(SpMesh)();
		mesh2->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexLength);
		mesh2->faces = sp_mem_new(SpArray<SpFaceMesh*>)(faceLength);

		for (sp_uint i = 0; i < vertexLength; i++)
			mesh2->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh2, i, vertexesMesh2[i]));

		for (sp_uint i = 0; i < faceLength; i++)
		{
			sp_uint faceIndex = 3u * i;

			mesh2->faces->add(sp_mem_new(SpFaceMesh)(mesh2, i,
				meshIndexes[faceIndex],
				meshIndexes[faceIndex + 1u],
				meshIndexes[faceIndex + 2u])
			);
		}
		mesh2->init();

		Vec3 tetrahedron[4];
		gjk(mesh1, vertexesMesh1, mesh2, vertexesMesh2, tetrahedron);

		Vec3 normal;
		sp_float depth;
		epa(tetrahedron, mesh1, vertexesMesh1, mesh2, vertexesMesh2, normal, depth);

		Assert::AreEqual(Vec3(1.0f, 0.0f, 0.0f), normal, L"Wrong value.", LINE_INFO());
		Asserts::isCloseEnough(0.5f, depth, SP_EPSILON_TWO_DIGITS, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, epa2)
	{
		const sp_uint vertexLength = 8;
		const sp_uint faceLength = 12;

		Vec3 vertexesMesh1[vertexLength] = {
			Vec3(10.0f, 0.0f, 0.0f),
			Vec3(12.0f, 0.0f, 0.0f),
			Vec3(12.0f, 2.0f, 0.0f),
			Vec3(10.0f, 2.0f, 0.0f),
			Vec3(10.0f, 0.0f, 2.0f),
			Vec3(12.0f, 0.0f, 2.0f),
			Vec3(12.0f, 2.0f, 2.0f),
			Vec3(10.0f, 2.0f, 2.0f)
		};
		Vec3 vertexesMesh2[vertexLength] = {
			Vec3(9.5f, 0.0f, 0.0f),
			Vec3(11.5f, 0.0f, 0.0f),
			Vec3(11.5f, 2.0f, 0.0f),
			Vec3(9.5f, 2.0f, 0.0f),
			Vec3(9.5f, 0.0f, 2.0f),
			Vec3(11.5f, 0.0f, 2.0f),
			Vec3(11.5f, 2.0f, 2.0f),
			Vec3(9.5f, 2.0f, 2.0f)
		};

		sp_uint meshIndexes[36] = {
			0, 1, 2, // front1
			2, 3, 0, // front2
			0, 3, 7, // left1
			7, 4, 0, // left2
			0, 4, 5, // bottom1
			5, 1, 0, // bottom2
			6, 7, 3, // top1
			3, 2, 6, // top2
			6, 2, 1, // right1
			1, 5, 6, // right2
			6, 5, 4, // back1
			4, 7, 6  // back2
		};

		SpMesh* mesh1 = ALLOC_NEW(SpMesh)();
		mesh1->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexLength);
		mesh1->faces = sp_mem_new(SpArray<SpFaceMesh*>)(faceLength);

		for (sp_uint i = 0; i < vertexLength; i++)
			mesh1->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh1, i, vertexesMesh1[i]));

		for (sp_uint i = 0; i < faceLength; i++)
		{
			sp_uint faceIndex = 3u * i;

			mesh1->faces->add(sp_mem_new(SpFaceMesh)(mesh1, i,
				meshIndexes[faceIndex],
				meshIndexes[faceIndex + 1u],
				meshIndexes[faceIndex + 2u])
			);
		}
		mesh1->init();

		SpMesh* mesh2 = ALLOC_NEW(SpMesh)();
		mesh2->vertexesMesh = sp_mem_new(SpArray<SpVertexMesh*>)(vertexLength);
		mesh2->faces = sp_mem_new(SpArray<SpFaceMesh*>)(faceLength);

		for (sp_uint i = 0; i < vertexLength; i++)
			mesh2->vertexesMesh->add(sp_mem_new(SpVertexMesh)(mesh2, i, vertexesMesh2[i]));

		for (sp_uint i = 0; i < faceLength; i++)
		{
			sp_uint faceIndex = 3u * i;

			mesh2->faces->add(sp_mem_new(SpFaceMesh)(mesh2, i,
				meshIndexes[faceIndex],
				meshIndexes[faceIndex + 1u],
				meshIndexes[faceIndex + 2u])
			);
		}
		mesh2->init();

		/*
		sp_char text[5000];
		sp_uint index = ZERO_UINT, index2 = ZERO_UINT, index3 = ZERO_UINT;
		Matlab::convert(*mesh1, vertexesMesh1, "mesh1", "red", text, &index);
		Matlab::convert(*mesh2, vertexesMesh2, "mesh2", "blue", &text[index-1], &index2);
		Matlab::display(8, 14, -2, 4, -2, 4, &text[index + index2 - 2u], &index3);
		index = index + index2 + index3 - 3u;
		Vec3* vMink = ALLOC_ARRAY(Vec3, vertexLength * vertexLength);
		minkowsky(vertexLength, vertexesMesh1, vertexLength, vertexesMesh2, vMink);
		Matlab::convert(vertexLength * vertexLength, vMink, "mink", text, index);
		*/

		Vec3 tetrahedron[4];
		gjk(mesh1, vertexesMesh1, mesh2, vertexesMesh2, tetrahedron);

		Vec3 normal;
		sp_float depth;
		epa(tetrahedron, mesh1, vertexesMesh1, mesh2, vertexesMesh2, normal, depth);

		Asserts::isCloseEnough(Vec3(1.0f, 0.0f, 0.0f), normal, SP_EPSILON_TWO_DIGITS, L"Wrong value.", LINE_INFO());
		Asserts::isCloseEnough(1.5f, depth, SP_EPSILON_TWO_DIGITS, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
