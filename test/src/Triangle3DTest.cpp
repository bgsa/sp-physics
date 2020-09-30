#include "SpectrumPhysicsTest.h"
#include <Triangle3D.h>

#define CLASS_NAME Triangle3DTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(area);
		SP_TEST_METHOD_DEF(barycentric);
		SP_TEST_METHOD_DEF(convert_lines);
		SP_TEST_METHOD_DEF(project);
		SP_TEST_METHOD_DEF(isInside_1);
		SP_TEST_METHOD_DEF(isInside_2);
	};

	SP_TEST_METHOD(CLASS_NAME, area)
	{
		Triangle3D triangle = Triangle3D(
			Vec3(0.0f, 0.0f, 0.0f),
			Vec3(10.0f, 0.0f, 0.0f),
			Vec3(5.0f, 10.0f, 0.0f)
		);

		sp_float result;
		PerformanceCounter counter;

		for (sp_uint i = 0; i < 100000; i++)
			result = triangle.area();

		counter.logElapsedTime("Triangle3D::area: ");

		Assert::IsTrue(isCloseEnough(50.0f, result, ERROR_MARGIN_PHYSIC), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, barycentric)
	{
		Triangle3D triangle = Triangle3D(
			Vec3(0.0f, 0.0f, 0.0f),
			Vec3(10.0f, 0.0f, 0.0f),
			Vec3(5.0f, 10.0f, 0.0f)
		);
		Vec3 point = Vec3(4.0f, 2.0f, 0.0f);

		Vec3 result;
		triangle.barycentric(point, &result);

		//Must be true: u + v + w = 1
		Assert::IsTrue(isCloseEnough(1.0f, result[0] + result[1] + result[2]), L"Wrong value.", LINE_INFO());

		//ABC = Triangle
		//P = u*A + v*B + w*C
		Vec3 p = triangle.point1 * result[0] + triangle.point2 * result[1]  + triangle.point3 * result[2];

		for (size_t i = 0; i < 3; i++)
			Assert::IsTrue(isCloseEnough(p[i], point[i]), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, convert_lines)
	{
		const Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		const Vec3 point2 = Vec3(10.0f, 0.0f, 0.0f);
		const Vec3 point3 = Vec3(5.0f, 10.0f, 0.0f);
		Triangle3D triangle = Triangle3D(point1, point2, point3);
		
		Line3D result[3];
		triangle.convert(result);

		Assert::IsTrue(result[0].point1 == point1 && result[0].point2 == point2, L"Wrong value.");
		Assert::IsTrue(result[1].point1 == point2 && result[1].point2 == point3, L"Wrong value.");
		Assert::IsTrue(result[2].point1 == point3 && result[2].point2 == point1, L"Wrong value.");
	}

	SP_TEST_METHOD(CLASS_NAME, project)
	{
		const Vec3 point1 (0.0f, 0.0f, 0.0f);
		const Vec3 point2 (10.0f, 0.0f, 0.0f);
		const Vec3 point3 (5.0f, 10.0f, 0.0f);
		Triangle3D triangle (point1, point2, point3);

		Vec3 result;
		Vec3 target(3.0f, 4.0f, -5.0f);
		Vec3 expected(3.0f, 4.0f, 0.0f);
		triangle.project(target, &result);
	
		Assert::IsTrue(expected == result, L"Wrong value.");

		target = Vec3(30.0f, 4.0f, -5.0f);
		expected = Vec3(30.0f, 4.0f, 0.0f);
		triangle.project(target, &result);

		Assert::IsTrue(expected == result, L"Wrong value.");
	}

	SP_TEST_METHOD(CLASS_NAME, isInside_1)
	{
		const Vec3 point1(0.0f, 0.0f, 0.0f);
		const Vec3 point2(10.0f, 0.0f, 0.0f);
		const Vec3 point3(5.0f, 10.0f, 0.0f);
		Triangle3D triangle(point1, point2, point3);

		Vec3 target(3.0f, 1.0f, 0.0f);
		sp_bool result;
		
		PerformanceCounter counter;

		for (sp_uint i = 0; i < 100000; i++)
			result = triangle.isInside(target);

		counter.logElapsedTime("Triangle3D::isInside: ");

		Assert::IsTrue(result, L"Wrong value.");

		target = Vec3(-3.0f, 1.0f, 0.0f);
		result = triangle.isInside(target);

		Assert::IsFalse(result, L"Wrong value.");
	}

	SP_TEST_METHOD(CLASS_NAME, isInside_2)
	{
		Triangle3D triangle(
			Vec3(1.4f, 1.0f, 0.0f),
			Vec3(1.4f, -1.0f, 0.0f),
			Vec3(0.0f, -1.0f, 1.4f)
		);

		Vec3 target(1.83f, 0.0f, 1.25f);
		sp_bool result = triangle.isInside(target);

		Assert::IsFalse(result, L"Wrong value.");
	}
}

#undef CLASS_NAME