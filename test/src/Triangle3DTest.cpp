#include "SpectrumPhysicsTest.h"
#include <Triangle3D.h>

#define CLASS_NAME Triangle3DTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		
		SP_TEST_METHOD_DEF(Triangle3D_barycentric_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, Triangle3D_barycentric_Test)
	{
		Vec3f point = Vec3f(4.0f, 2.0f, 0.0f);
		Triangle3D triangle = Triangle3D(
			{ 0.0f, 0.0f, 0.0f },
			{ 10.0f, 0.0f, 0.0f },
			{ 5.0f, 10.0f, 0.0f }
		);

		Vec3f result = triangle.barycentric(point);
		
		//ABC = Triangle
		//P = u*A + v*B + w*C
		//P = u*Point1 + v*Point2 + w*Point3
		Vec3f p = triangle.point1 * result[0] + triangle.point2 * result[1]  + triangle.point3 * result[2];

		for (size_t i = 0; i < 3; i++)
			Assert::IsTrue(isCloseEnough(p[i], point[i]), L"Wrong value.", LINE_INFO());

		//Must be true: u + v + w = 1
		Assert::IsTrue(isCloseEnough(1.0f, result[0] + result[1] + result[2]), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME