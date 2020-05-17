#include "SpectrumPhysicsTest.h"
#include <DOP18.h>

#define CLASS_NAME DOP18Test

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(DOP18_constructor_empty);

		SP_TEST_METHOD_DEF(DOP18_collisionStatus_outside);

		SP_TEST_METHOD_DEF(DOP18_collisionStatus_inside);
		
	};

	SP_TEST_METHOD(CLASS_NAME, DOP18_constructor_empty)
	{
		DOP18 kdop = DOP18();

		for (int i = 0; i < 3; i++)
		{
			Assert::AreEqual(-0.5f, kdop.min[i], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(0.5f, kdop.max[i], L"Wrong value.", LINE_INFO());
		}

		for (int i = 3; i < DOP18_ORIENTATIONS; i++)
		{
			Assert::AreEqual(-0.375f, kdop.min[i], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(0.375f, kdop.max[i], L"Wrong value.", LINE_INFO());
		}
	}

	SP_TEST_METHOD(CLASS_NAME, DOP18_collisionStatus_outside)
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

	SP_TEST_METHOD(CLASS_NAME, DOP18_collisionStatus_inside)
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

}

#undef CLASS_NAME