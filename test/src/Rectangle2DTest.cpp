#include "SpectrumPhysicsTest.h"
#include "Vec2List.h"
#include "Rectangle2D.h"

#define CLASS_NAME Rectangle2DTest 

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(Rectangle2D_width_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_height_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_area_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_perimeter_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_diagonalLength_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getCollisionStatus_Inside1_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getCollisionStatus_Inside2_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getCollisionStatus_Outside_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Line_True_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Line_False_Test);
		
		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Triangle_True_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Triangle_False_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Circle_True_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_hasIntersection_Circle_False_Test);

		SP_TEST_METHOD_DEF(Rectangle2D_getBoundingBox_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_width_Test)
	{
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 0.0f, 100.0f },
			{ 20.0f, 100.0f },
			{ 20.0f, 0.0f }
		);

		float result = square.width();

		Assert::AreEqual(20.0f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_height_Test)
	{
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 0.0f, 100.0f },
			{ 20.0f, 100.0f },
			{ 20.0f, 0.0f }
		);

		float result = square.height();

		Assert::AreEqual(100.0f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_area_Test)
	{
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 0.0f, 100.0f },
			{ 20.0f, 100.0f },
			{ 20.0f, 0.0f }
		);

		float result = square.area();

		Assert::AreEqual(2000.0f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_perimeter_Test)
	{
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 0.0f, 100.0f },
			{ 20.0f, 100.0f },
			{ 20.0f, 0.0f }
		);

		float result = square.perimeter();

		Assert::AreEqual(240.0f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_diagonalLength_Test)
	{
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 0.0f, 100.0f },
			{ 20.0f, 100.0f },
			{ 20.0f, 0.0f }
		);

		float result = square.diagonalLength();

		Assert::AreEqual(101.980392f, result, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getCollisionStatus_Inside1_Test)
	{
		Vec2 point = Vec2(10.0f, 10.0f);
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f }, 
			{ 0.0f, 100.0f },
			{ 100.0f, 100.0f },
			{ 100.0f, 0.0f }
		);

		CollisionStatus result = square.getSatusCollision(point);

		if (result != CollisionStatus::INSIDE)
			Assert::Fail(L"Point should be inside the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getCollisionStatus_Inside2_Test)
	{
		Vec2 point = Vec2(100.0f, 60.0f);
		Rectangle2D square = Rectangle2D(
			{ 10.0f, 10.0f },
			{ 210.0f, 10.0f },
			{ 210.0f, 110.0f },
			{ 10.0f, 110.0f }
		);

		CollisionStatus result = square.getSatusCollision(point);

		if (result != CollisionStatus::INSIDE)
			Assert::Fail(L"Point should be inside the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getCollisionStatus_Outside_Test)
	{
		Vec2 point = Vec2(200.0f, 200.0f);
		Rectangle2D rect = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 0.0f, 100.0f },
			{ 100.0f, 100.0f },
			{ 100.0f, 0.0f }
		);

		CollisionStatus result = rect.getSatusCollision(point);

		if (result != CollisionStatus::OUTSIDE)
			Assert::Fail(L"Point should be inside the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Line_True_Test)
	{
		Line2D line = Line2D(Vec2(50.0f, 0.0f), Vec2(50.0f, 200.0f));
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 100.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }				
		);

		bool result = square.hasIntersection(line);

		Assert::IsTrue(result, L"Line should cross the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Line_False_Test)
	{
		Line2D line = Line2D(Vec2(110.0f, 0.0f), Vec2(110.0f, 200.0f));
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 100.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = square.hasIntersection(line);

		Assert::IsFalse(result, L"Line should cross the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Triangle_True_Test)
	{
		Triangle2D triangle(Vec2(50.0f, 10.0f), Vec2(150.0f, 200.0f), Vec2(200.0f, 200.0f));
		Rectangle2D square(
			{ 0.0f, 0.0f },
			{ 100.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = square.hasIntersection(triangle);

		Assert::IsTrue(result, L"Triangle should intersect the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Triangle_False_Test)
	{
		Triangle2D triangle(Vec2(120.0f, 110.0f), Vec2(150.0f, 200.0f), Vec2(200.0f, 200.0f));
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 100.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = square.hasIntersection(triangle);

		Assert::IsFalse(result, L"Triangle should intersect the square.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Circle_True_Test)
	{
		Circle2D circle(Vec2(105.0f, 50.0f), 10.0f);
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 100.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		sp_bool result = square.hasIntersection(circle);

		Assert::IsTrue(result, L"Rectangle should intersect the circle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_hasIntersection_Circle_False_Test)
	{
		Circle2D circle(Vec2(111.0f, 50.0f), 10.0f);
		Rectangle2D square = Rectangle2D(
			{ 0.0f, 0.0f },
			{ 100.0f, 0.0f },
			{ 100.0f, 100.0f },
			{ 0.0f, 100.0f }
		);

		bool result = square.hasIntersection(circle);

		Assert::IsFalse(result, L"Rectangle should NOT intersect the circle.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Rectangle2D_getBoundingBox_Test)
	{
		Vec2List points;
		points.add(Vec2(0.3f, 0.0f));
		points.add(Vec2(1.0f, 0.3f));
		points.add(Vec2(0.7f, 1.0f));
		points.add(Vec2(0.0f, 0.7f));

		Rectangle2D result = Rectangle2D::getBoundingBox(points);

		Vec2 point1Expected = { 0.0f, 0.0f };
		Vec2 point2Expected = { 1.0f, 0.0f };
		Vec2 point3Expected = { 1.0f, 1.0f };
		Vec2 point4Expected = { 0.0f, 1.0f };

		Assert::AreEqual(point1Expected, result.point1, L"Wring value.", LINE_INFO());
		Assert::AreEqual(point2Expected, result.point2, L"Wring value.", LINE_INFO());
		Assert::AreEqual(point3Expected, result.point3, L"Wring value.", LINE_INFO());
		Assert::AreEqual(point4Expected, result.point4, L"Wring value.", LINE_INFO());
	}

}

#undef CLASS_NAME