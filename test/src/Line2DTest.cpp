#include "TestHeader.h"
#include <Vec2.h>
#include <Vec3.h>
#include <Line2D.h>
#include <Circle2D.h>

#define CLASS_NAME Line2DTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(Line2D_isOnTheLine_Test);

		SP_TEST_METHOD_DEF(Line2D_isOnTheLeft_Test);

		SP_TEST_METHOD_DEF(Line2D_isOnTheRight_Test);

		SP_TEST_METHOD_DEF(Line2D_angle1_Test);

		SP_TEST_METHOD_DEF(Line2D_angle2_Test);

		SP_TEST_METHOD_DEF(Line2D_angle3_Test);

		SP_TEST_METHOD_DEF(Line2D_slope_Test);

		SP_TEST_METHOD_DEF(Line2D_getParametricEquation1_Test);

		SP_TEST_METHOD_DEF(Line2D_getParametricEquation2_Test);

		SP_TEST_METHOD_DEF(Line2D_getDistance_Test);

		SP_TEST_METHOD_DEF(Line2D_deltaX_Test);

		SP_TEST_METHOD_DEF(Line2D_deltaY_Test);

		SP_TEST_METHOD_DEF(Line2D_findIntersection_Test);

		SP_TEST_METHOD_DEF(Line2D_hasIntersection_Inside_Test);

		SP_TEST_METHOD_DEF(Line2D_hasIntersection_Inline_Test);

		SP_TEST_METHOD_DEF(Line2D_hasIntersection_Outside_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, Line2D_isOnTheLine_Test)
	{
		Vec2f point = Vec2f(3.0f, 3.0f);
		Line2Df line = Line2Df({ 0.0f, 0.0f }, { 10.0f, 10.0f });

		bool result = line.isOnTheLine(point);

		Assert::IsTrue(result, L"Point should be on the line.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_isOnTheLeft_Test)
	{
		Vec2f point = Vec2f(3.0f, 5.0f);
		Line2Df line = Line2Df({ 0.0f, 0.0f }, { 10.0f, 10.0f });

		bool result = line.isOnTheLeft(point);

		Assert::IsTrue(result, L"Point should be on the left.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_isOnTheRight_Test)
	{
		Vec2f point = Vec2f(3.0f, 1.0f);
		Line2Df line = Line2Df({ 0.0f, 0.0f }, { 10.0f, 10.0f });

		bool result = line.isOnTheRight(point);

		Assert::IsTrue(result, L"Point should be on the Right.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_angle1_Test)
	{
		Line2Df line = Line2Df({ 10.0f, 10.0f }, { 100.0f, 100.0f });
		float expected = 45.00f;

		float result = (float) radiansToDegrees(line.angle());

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_angle2_Test)
	{
		Line2Df line = Line2Df({ 10.0f, 10.0f }, { -20.0f, 100.0f });
		float expected = -71.56f;
		float result = (float)radiansToDegrees(line.angle());
		
		Assert::IsTrue(isCloseEnough(result, expected, 0.09f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_angle3_Test)
	{
		Line2Df line = Line2Df({ 9.0f, 3.0f }, { -1.0f, 5.67949192f });
		float expected = -15.0f;

		float result = round((float)radiansToDegrees(line.angle()), 2);

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_slope_Test)
	{
		Line2Df line = Line2Df({ 10.0f, 10.0f }, { -20.0f, 100.0f });
		float expected = -3.0f;

		float result = line.slope();

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_getParametricEquation1_Test)
	{
		Line2Df line = Line2Df({ 10.0f, 10.0f }, { -20.0f, 100.0f });
		Vec2f expected = Vec2f(-3.0f, 40.0f);

		Vec2f result = line.getParametricEquation();

		Assert::AreEqual(expected[0], result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_getParametricEquation2_Test)
	{
		Line2Df line = Line2Df({ 2.6666f, 6.0f }, { 4.6666f, 12.0f });
		Vec2f expected = Vec2f(3.0f, -2.0f);

		Vec2f result = line.getParametricEquation();
		
		Assert::IsTrue(isCloseEnough(result[0], expected[0], 0.0009f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(result[1], expected[1], 0.0009f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_getDistance_Test)
	{
		Vec2f point = { 3.0f , 3.0f };
		Line2Df line = Line2Df({ 0.0f, 13.0f }, { 1.0f, 10.0f });
		float expected = 0.316227764f;

		float result = line.getDistance(point);

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_deltaX_Test)
	{
		Line2Df line = Line2Df({ 1.0f, 5.0f }, { 7.0f, 13.0f });
		float expected = 6.0f;

		float result = line.deltaX();

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_deltaY_Test)
	{
		Line2Df line = Line2Df({ 1.0f, 5.0f }, { 7.0f, 13.0f });
		float expected = 8.0f;

		float result = line.deltaY();

		Assert::AreEqual(expected, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_findIntersection_Test)
	{
		Line2Df line1 = Line2Df({ 0.0f, 10.0f }, { 10.0f, 10.0f });
		Line2Df line2 = Line2Df({ 5.0f, 0.0f }, { 5.0f, 10.0f });
		Vec2f expected = Vec2f(5.0f, 10.0f);

		Vec2f* result = line1.findIntersection(line2);

		Assert::IsNotNull(result, L"Result shoud not be null.", LINE_INFO());
		Assert::AreEqual(expected.x, result->x, L"Wrong value.", LINE_INFO());
		Assert::AreEqual(expected.y, result->y, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_hasIntersection_Inside_Test)
	{
		Line2Df line1 = Line2Df({ 0.0f, 10.0f }, { 10.0f, 10.0f });
		Circle2Df circle = Circle2Df(Vec2f(10.0f, 10.0f), 10.0f);

		CollisionStatus collision = line1.hasIntersections(circle);

		if (collision != CollisionStatus::INSIDE)
			Assert::Fail(L"There should be an inline intersection.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_hasIntersection_Inline_Test)
	{
		Line2Df line1 = Line2Df({ 90.0f, 0.0f }, { 90.0f, 200.0f });
		Circle2Df circle = Circle2Df(Vec2f(100.0f, 100.0f), 10.0f);

		CollisionStatus collision = line1.hasIntersections(circle);

		if (collision != CollisionStatus::INLINE)
			Assert::Fail(L"There should be an INLINE intersection.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Line2D_hasIntersection_Outside_Test)
	{
		Line2Df line1 = Line2Df({ 80.0f, 0.0f }, { 80.0f, 50.0f });
		Circle2Df circle = Circle2Df(Vec2f(100.0f, 100.0f), 10.0f);

		CollisionStatus collision = line1.hasIntersections(circle);

		if (collision != CollisionStatus::OUTSIDE)
			Assert::Fail(L"There should be an INLINE intersection.", LINE_INFO());
	}
}

#undef CLASS_NAME