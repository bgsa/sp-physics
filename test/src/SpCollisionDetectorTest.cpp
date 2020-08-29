#include "SpectrumPhysicsTest.h"
#include <SpCollisionDetector.h>

#define CLASS_NAME SpCollisionDetectorTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(areMovingAway);
		SP_TEST_METHOD_DEF(collisionStatus);
	};

	SP_TEST_METHOD(CLASS_NAME, areMovingAway)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;

		simulator->physicProperties(0u)->position(Vec3(1.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->position(Vec3(0.0f, 0.0f, 0.0f));

		simulator->physicProperties(0u)->velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->velocity(Vec3(-10.0f, 0.0f, 0.0f));

		sp_bool result = collisionDetector.areMovingAway(0u, 1u);

		Assert::IsTrue(result, L"Wrong value.", LINE_INFO());

		simulator->physicProperties(0u)->velocity(Vec3(10.0f, 0.0f, 0.0f));
		simulator->physicProperties(1u)->velocity(Vec3(1.0f, 0.0f, 0.0f));

		result = collisionDetector.areMovingAway(0u, 1u);
		
		Assert::IsFalse(result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus)
	{
		SpPhysicSimulator* simulator = SpPhysicSimulator::instance();
		SpCollisionDetector collisionDetector;
		SpCollisionDetails details;

		const sp_uint vertexesLength = 8u;
		Vec3 vertexes[vertexesLength] = {
			Vec3(-1.0f, -1.0f, -1.0f), Vec3(-1.0f, -1.0f, 1.0f),
			Vec3(-1.0f, 1.0f, 1.0f), Vec3(-1.0f, 1.0f, -1.0f),
			Vec3(1.0f, -1.0f, -1.0f), Vec3(1.0f, 1.0f, -1.0f),
			Vec3(1.0f, 1.0f, 1.0f), Vec3(1.0f, -1.0f, 1.0f)
		};

		const sp_uint facesLength = 12u;
		SpPoint3<sp_uint> facesIndex[facesLength] = {
			SpPoint3<sp_uint>(0, 1, 2), SpPoint3<sp_uint>(2, 3, 0),
			SpPoint3<sp_uint>(4, 5, 6), SpPoint3<sp_uint>(6, 7, 4),
			SpPoint3<sp_uint>(1, 7, 2), SpPoint3<sp_uint>(7, 6, 2),
			SpPoint3<sp_uint>(4, 0, 5), SpPoint3<sp_uint>(5, 0, 3),
			SpPoint3<sp_uint>(2, 6, 5), SpPoint3<sp_uint>(2, 5, 3),
			SpPoint3<sp_uint>(0, 4, 7), SpPoint3<sp_uint>(0, 7, 1)
		};

		for (sp_uint i = 0; i < 2; i++)
		{
			SpMesh* mesh = sp_mem_new(SpMesh)();
			mesh->vertexes = sp_mem_new(SpArray<Vec3>)(vertexesLength);
			mesh->facesIndexes = sp_mem_new(SpArray<SpPoint3<sp_uint>>)(facesLength);

			for (sp_uint i = 0; i < vertexesLength; i++)
				mesh->vertexes->add(vertexes[i]);

			for (sp_uint i = 0; i < facesLength; i++)
				mesh->facesIndexes->add(facesIndex[i]);

			mesh->init();
			
			SpPhysicSimulator::instance()->mesh(i, mesh);
		}

		simulator->translate(0u, Vec3(1.0f, 0.0f, 0.0f));
		simulator->translate(1u, Vec3(0.0f, 0.0f, 0.0f));

		CollisionStatus result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		simulator->translate(0u, Vec3(1.5f, 0.0f, 0.0f));
		result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::OUTSIDE, L"Wrong value.", LINE_INFO());
	
		simulator->translate(0u, Vec3(-1.5f, 2.0f, 0.0f));
		result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		simulator->translate(0u, Vec3(0.0f, 0.2f, 0.0f));
		result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::OUTSIDE, L"Wrong value.", LINE_INFO());

		simulator->translate(0u, Vec3(0.0f, -2.2f, 2.0f));
		result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		simulator->translate(0u, Vec3(0.0f, 0.0f, -0.2f));
		result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());

		simulator->translate(0u, Vec3(-1.0f, 0.0f, -1.8f));
		simulator->scale(0u, Vec3(0.2f, 0.2f, 0.2f));
		simulator->translate(0u, Vec3(0.5f, 0.5f, 0.5f));
		result = collisionDetector.collisionStatus(0u, 1u, &details);
		Assert::IsTrue(result == CollisionStatus::INSIDE, L"Wrong value.", LINE_INFO());
	}
}

#undef CLASS_NAME
