#include "SpectrumPhysicsTest.h"
#include <DOP18.h>

#define CLASS_NAME DOP18Test

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(constructor_empty);
		SP_TEST_METHOD_DEF(collisionStatus_outside);
		SP_TEST_METHOD_DEF(collisionStatus_inside);
		SP_TEST_METHOD_DEF(collisionStatus_plane);
		SP_TEST_METHOD_DEF(collisionStatus_point);
		SP_TEST_METHOD_DEF(collisionStatus_pointWithDetails);
	};

	SP_TEST_METHOD(CLASS_NAME, constructor_empty)
	{
		DOP18 kdop = DOP18();

		for (int i = 0; i < 3; i++)
		{
			Assert::AreEqual(-0.5f, kdop.min[i], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(0.5f, kdop.max[i], L"Wrong value.", LINE_INFO());
		}

		for (int i = 3; i < DOP18_ORIENTATIONS; i++)
		{
			Assert::AreEqual(-0.75f, kdop.min[i], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(0.75f, kdop.max[i], L"Wrong value.", LINE_INFO());
		}
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus_outside)
	{
		DOP18 kdop1 = DOP18();
		DOP18 kdop2 = DOP18();

		kdop2.translate(10.0f, 3.0f, 1.0f);
		CollisionStatus result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		kdop2 = DOP18();
		kdop2.translate(1.1f, 0.0f, 0.0f);
		result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		kdop2 = DOP18();
		kdop2.translate(0.0f, 1.1f, 0.0f);
		result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		kdop2 = DOP18();
		kdop2.translate(0.0f, -1.1f, 0.0f);
		result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		kdop2 = DOP18();
		kdop2.translate(0.0f, 0.0f, -1.1f);
		result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		kdop2 = DOP18();
		kdop2.translate(1.0f, 1.0f, 1.0f);
		result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus_inside)
	{
		DOP18 kdop1 = DOP18();
		DOP18 kdop2 = DOP18();

		CollisionStatus result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());

		kdop2 = DOP18();
		kdop2.translate(0.5f, 0.0f, 0.0f);
		result = kdop1.collisionStatus(kdop2);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus_plane)
	{
		DOP18 kdop = DOP18();
		Plane3D plane = Plane3D(Vec3(0.0f), Vec3(0.0f, 1.0f, 0.0f));

		CollisionStatus result = kdop.collisionStatus(plane);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());

		kdop = DOP18();
		kdop.translate(0.5f, 0.0f, 0.0f);
		result = kdop.collisionStatus(plane);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus_point)
	{
		DOP18 kdop = DOP18();
		Vec3 point = Vec3(0.0f);

		CollisionStatus result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(1.0f, 0.0f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(-1.0f, 0.0f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(0.0f, 1.0f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(0.0f, -1.0f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(0.0f, 0.0f, 1.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(0.5f, 0.5f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(0.35f, 0.35f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());

		point = Vec3(0.40f, 0.40f, 0.0f);
		result = kdop.collisionStatus(point);
		Assert::AreEqual(CollisionStatus::OUTSIDE, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, collisionStatus_pointWithDetails)
	{
		DOP18 kdop = DOP18();
		Vec3 point = Vec3(0.3f, 0.3f, 0.0f);

		sp_bool minPlane;
		sp_uint planeIndex;
		sp_float distanceFromPlane;

		CollisionStatus result = kdop.collisionStatus(point, &minPlane, &planeIndex, &distanceFromPlane);
		Assert::AreEqual(CollisionStatus::INSIDE, result, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME