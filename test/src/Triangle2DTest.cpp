#include "SpectrumPhysicsTest.h"
#include <Triangle2D.h>

#define CLASS_NAME Triangle2DTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(Triangle2D_area_Test);

		SP_TEST_METHOD_DEF(Triangle2D_perimeter_Test);

		SP_TEST_METHOD_DEF(Triangle2D_height_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getCollisionStatus_Point_Inline_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getCollisionStatus_Point_Outside_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getCollisionStatus_Point_Inside_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Line_True_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Line_False_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Circle_True_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Circle_False_Test);
		
	};

	SP_TEST_METHOD(CLASS_NAME, Triangle2D_area_Test)
	{
		Triangle2Df triangle = Triangle2Df(
			{ 2.0f, 5.0f }, 
			{ 10.0f, 10.0f },
			{ 3.0f, 6.0f }
		);

		float result = triangle.area();

		Assert::AreEqual(1.5f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Triangle2D_perimeter_Test)
	{
		Triangle2Df triangle = Triangle2Df(
			{ 2.0f, 5.0f },
			{ 10.0f, 10.0f },
			{ 3.0f, 6.0f }
		);

		float result = triangle.perimeter();

		Assert::AreEqual(18.9104519f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Triangle2D_height_Test)
	{
		Triangle2Df triangle = Triangle2Df(
			{ 2.0f, 5.0f },
			{ 10.0f, 10.0f },
			{ 3.0f, 6.0f }
		);

		float result = triangle.height();

		Assert::AreEqual(6.41016722f, result, L"Wrong value.", LINE_INFO());
	}
	
	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getCollisionStatus_Point_Inline_Test)
	{
		Vec2f point = Vec2f(10.0f, 10.0f);
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		CollisionStatus result = triangle.getCollisionStatus(point);

		if (result != CollisionStatus::INLINE)
			Assert::Fail(L"Point should be inline the triangle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getCollisionStatus_Point_Inside_Test)
	{
		Vec2f point = Vec2f(10.0f, 20.0f);
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		CollisionStatus result = triangle.getCollisionStatus(point);

		if (result != CollisionStatus::INSIDE)
			Assert::Fail(L"Point should be inside the triangle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getCollisionStatus_Point_Outside_Test)
	{
		Vec2f point = Vec2f(10.0f, 5.0f);
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		CollisionStatus result = triangle.getCollisionStatus(point);

		if (result != CollisionStatus::OUTSIDE)
			Assert::Fail(L"Point should be outside the triangle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Line_True_Test)
	{
		Line2Df line = Line2Df(Vec2f(10.0f, 0.0f), Vec2f(10.0f, 200.0f));
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = triangle.hasIntersection(line);

		Assert::IsTrue(result, L"Line should cross the triangle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Line_False_Test)
	{
		Line2Df line = Line2Df(Vec2f(110.0f, 0.0f), Vec2f(110.0f, 200.0f));
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = triangle.hasIntersection(line);

		Assert::IsFalse(result, L"Line should NOT cross the triangle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Circle_True_Test)
	{
		Circle2Df circle = Circle2Df(Vec2f(50.0f, 105.0f), 10.0f);
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = triangle.hasIntersection(circle);

		Assert::IsTrue(result, L"Triangle should intersect the circle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Circle_False_Test)
	{
		Circle2Df circle = Circle2Df(Vec2f(50.0f, 111.0f), 10.0f);
		Triangle2Df triangle = Triangle2Df(
			{ 0.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = triangle.hasIntersection(circle);

		Assert::IsFalse(result, L"Triangle should NOT intersect the circle.", LINE_INFO());
	}

}

#undef CLASS_NAME