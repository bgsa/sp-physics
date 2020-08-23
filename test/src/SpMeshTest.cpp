#include "SpectrumPhysicsTest.h"
#include "SpMesh.h"

#define CLASS_NAME SpMeshTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(init_Edges_Test);
	};

	SP_TEST_METHOD(CLASS_NAME, init_Edges_Test)
	{
		SpArray<Vec3> vertexes(3u);
		vertexes.add(Vec3(0.0f, 0.0f, 0.0f));
		vertexes.add(Vec3(10.0f, 0.0f, 0.0f));
		vertexes.add(Vec3(5.0f, 10.0f, 0.0f));

		SpArray<SpPoint3<sp_uint>> facesIndex(1u);
		facesIndex.add(SpPoint3<sp_uint>(0u, 1u, 2u));
		
		SpMesh mesh = SpMesh();
		mesh.vertexes = &vertexes;
		mesh.facesIndexes = &facesIndex;
		mesh.init();

		sp_uint vertexesIndexLength = mesh.edgesIndexes->length();
		SpPoint2<sp_uint>* edges = mesh.edgesIndexes->data();

		Assert::AreEqual(3u, vertexesIndexLength, L"Wring value");
		Assert::AreEqual(0u, edges[0].x, L"Wring value");
		Assert::AreEqual(1u, edges[0].y, L"Wring value");
		Assert::AreEqual(1u, edges[1].x, L"Wring value");
		Assert::AreEqual(2u, edges[1].y, L"Wring value");
		Assert::AreEqual(2u, edges[2].x, L"Wring value");
		Assert::AreEqual(0u, edges[2].y, L"Wring value");
	}
}

#undef CLASS_NAME
