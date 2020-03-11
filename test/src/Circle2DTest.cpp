#include "TestHeader.h"
#include <Vec2.h>
#include <Vec3.h>
#include <Line2D.h>
#include <Circle2D.h>

#define CLASS_NAME Circle2DTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(Circle2D_constructorWithThreePoints_Test);

		SP_TEST_METHOD_DEF(Circle2D_area_Test);

		SP_TEST_METHOD_DEF(Circle2D_circumference_Test);

		SP_TEST_METHOD_DEF(Circle2D_hasIntersection1_Test);

		SP_TEST_METHOD_DEF(Circle2D_hasIntersection2_Test);

		SP_TEST_METHOD_DEF(Circle2D_findIntersection1_Test);

		SP_TEST_METHOD_DEF(Circle2D_findIntersection2_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, Circle2D_constructorWithThreePoints_Test)
	{
		Circle2Df circle = Circle2Df(
			Vec2f(-3.0f, 4.0f),
			Vec2f(4.0f, 5.0f),
			Vec2f(1.0f, -4.0f)
		);

		Vec2f center = circle.center;
		float ray = circle.ray;

		Assert::AreEqual(1.0f, center.x, L"Wrong value.", LINE_INFO());
		Assert::AreEqual(1.0f, center.y, L"Wrong value.", LINE_INFO());
		Assert::AreEqual(5.0f, ray, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Circle2D_area_Test)
	{
		Vec2f center = Vec2f(7.0f, 5.0f);
		float ray = 3.0f;
		Circle2Df circle = Circle2Df(center, ray);

		float result = circle.area();
		float expected = 28.27f;

		Assert::AreEqual(expected, round(result, 2), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Circle2D_circumference_Test)
	{
		Vec2f center = Vec2f(7.0f, 5.0f);
		float ray = 3.0f;
		Circle2Df circle = Circle2Df(center, ray);

		float result = circle.circumference();
		float expected = 18.85f;

		Assert::AreEqual(expected, round(result, 2), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Circle2D_hasIntersection1_Test)
	{
		Circle2Df circle1 = Circle2Df(Vec2f(-4.0f, 4.0f), 3.0f);
		Circle2Df circle2 = Circle2Df(Vec2f(0.0f, 9.0f), 5.0f);

		bool result = circle1.hasIntersection(circle2);

		Assert::IsTrue(result, L"Circles should intersect.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Circle2D_hasIntersection2_Test)
	{
		Circle2Df circle1 = Circle2Df(Vec2f(4.0f, -3.0f), 5.0f);
		Circle2Df circle2 = Circle2Df(Vec2f(-4.0f, -3.0f), 3.0f);

		bool result = circle1.hasIntersection(circle2);

		Assert::IsTrue(result, L"Circles should intersect.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Circle2D_findIntersection1_Test)
	{
		Circle2Df circle1 = Circle2Df(Vec2f(-4.0f, 4.0f), 3.0f);
		Circle2Df circle2 = Circle2Df(Vec2f(0.0f, 9.0f), 5.0f);

		Vec2f* result = circle1.findIntersection(circle2);

		Vec2f intersection1 = Vec2f(-1.0017f, 4.1013f);
		Vec2f intersection2 = Vec2f(-4.5592f, 6.9474f);
		
		Assert::IsNotNull(result, L"Circles should intersect.", LINE_INFO());
		
		Assert::IsTrue(isCloseEnough(result[0].x, intersection1.x), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result[0].y, intersection1.y), L"Wrong value.", LINE_INFO());

		Assert::IsTrue(isCloseEnough(intersection2.x, result[1].x), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(intersection2.y, result[1].y), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Circle2D_findIntersection2_Test)
	{
		Circle2Df circle1 = Circle2Df(Vec2f(4.0f, -3.0f), 5.0f);
		Circle2Df circle2 = Circle2Df(Vec2f(-4.0f, -3.0f), 3.0f);

		Vec2f* result = circle1.findIntersection(circle2);

		Vec2f intersection = Vec2f(-1.0f, -3.0f);

		Assert::IsNotNull(result, L"Circles should intersect.", LINE_INFO());

		Assert::AreEqual(intersection.x, round(result[0].x, 3), L"Wrong value.", LINE_INFO());
		Assert::AreEqual(intersection.y, round(result[0].y, 3), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
