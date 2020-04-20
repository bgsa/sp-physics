#include "SpectrumPhysicsTest.h"
#include <OBB.h>

#define CLASS_NAME OBBTest 

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(OBB_constructor_empty_Test);

		SP_TEST_METHOD_DEF(OBB_constructor_withCenter_Test);
		
	};

	SP_TEST_METHOD(CLASS_NAME, OBB_constructor_empty_Test)
	{
		OBB obb = OBB();

		Assert::AreEqual(Vec3f(0.0f), obb.center, L"Wrong value.", LINE_INFO());
		Assert::AreEqual(Vec3f(0.5f), obb.halfWidth, L"Wrong value.", LINE_INFO());
		Assert::IsTrue(Mat3f::identity() == obb.orientation, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, OBB_constructor_withCenter_Test)
	{
		Vec3f center(1.0f, 2.0f, 3.0f);

		OBB obb = OBB(center);

		Assert::AreEqual(center, obb.center, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
