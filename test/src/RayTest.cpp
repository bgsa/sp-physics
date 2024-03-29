#include "SpectrumPhysicsTest.h"
#include "Ray.h"

#define CLASS_NAME RayTest 

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(intersection);
		SP_TEST_METHOD_DEF(intersection_plane);

	};

	SP_TEST_METHOD(CLASS_NAME, intersection_plane)
	{
		Ray r(Vec3(1.019f, -1.0f, -1.0f), Vec3(0.0f, -0.0f, -1.0f));
		Plane plane(Vec3(-1.0f, -1.0f, 1.0f), Vec3(0.0f, 0.0f, 1.0f));

		Vec3 contact;
		const sp_bool result = r.intersection(plane, contact);

		Assert::IsTrue(result);
		Assert::IsTrue(isCloseEnough(Vec3(1.019f, -1.0f, 1.0f), contact, ERROR_MARGIN_PHYSIC));
	}

	SP_TEST_METHOD(CLASS_NAME, intersection)
	{
		Ray ray1(Vec3(-0.18f, 1.0f, -0.18f), Vec3(1.0f, 0.0f, 0.0f));
		Ray ray2(Vec3(-0.72f, 0.32f, -0.32f), Vec3(0.7070f, 0.7070f, 0.0f));

		Vec3 contact;
		sp_bool result = ray1.intersection(ray2, &contact, ERROR_MARGIN_PHYSIC);

		Assert::IsFalse(result, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME