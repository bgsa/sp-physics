#include "SpectrumPhysicsTest.h"
#include <SpTransform.h>

#define CLASS_NAME SpTransformTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(transform);
	};

	SP_TEST_METHOD(CLASS_NAME, transform)
	{
		Vec3 result;
		SpTransform transform;
		transform.position = Vec3(0.0f, 0.0f, -10.0f);
		transform.orientation = Quat::createRotationAxisZ(degreesToRadians(45));

		PerformanceCounter counter;

		for (sp_uint i = 0; i < 100000; i++)
			transform.transform(Vec3(1.0f, 1.0f, 0.0f), result);

		counter.logElapsedTime("SpTransform::transform: ");

		Assert::IsTrue(isCloseEnough(Vec3(0.0f, 1.4142f, -10.0f), result), L"Wrong value.", LINE_INFO());
	}
}

#undef CLASS_NAME