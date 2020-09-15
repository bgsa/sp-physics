#include "SpectrumPhysicsTest.h"
#include <Plane3D.h>
#include <Vec3.h>
#include <Vec4.h>
#include "Asserts.h"

#define CLASS_NAME Plane3DTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(contructorWithThreePoints_Test);
		SP_TEST_METHOD_DEF(equation_Test1);
		SP_TEST_METHOD_DEF(equation_Test2);
		SP_TEST_METHOD_DEF(equation_Test3);
		SP_TEST_METHOD_DEF(intersection_line);
		SP_TEST_METHOD_DEF(intersection_ray);
		SP_TEST_METHOD_DEF(intersection_plane);
		SP_TEST_METHOD_DEF(angle_Test1);
		SP_TEST_METHOD_DEF(angle_Test2);
		SP_TEST_METHOD_DEF(distance_point_Test1);
		SP_TEST_METHOD_DEF(distance_point_Test2);
		SP_TEST_METHOD_DEF(distance_point_Test3);
		SP_TEST_METHOD_DEF(distance_point_Test4);
		SP_TEST_METHOD_DEF(distance_plane_Test1);
		SP_TEST_METHOD_DEF(closestPointOnThePlane_point_Test1);
		SP_TEST_METHOD_DEF(constructor_equation_Test);
		SP_TEST_METHOD_DEF(orientation1);
		SP_TEST_METHOD_DEF(orientation2);
		SP_TEST_METHOD_DEF(orientation3);
		SP_TEST_METHOD_DEF(isParallel_Test1);
		SP_TEST_METHOD_DEF(isParallel_Test2);
		SP_TEST_METHOD_DEF(isParallel_Test3);
		SP_TEST_METHOD_DEF(isPerpendicular_Test1);
		SP_TEST_METHOD_DEF(isPerpendicular_Test2);
	};

	SP_TEST_METHOD(CLASS_NAME, intersection_ray)
	{
		Ray r(Vec3(1.019f, -1.0f, -1.0f), Vec3(0.0f, -0.0f, -1.0f));
		Plane3D plane(Vec3(-1.0f, -1.0f, 1.0f), Vec3(0.0f, 0.0f, 1.0f));

		Vec3 contact;
		sp_bool result = plane.intersection(r, &contact);
		
		Assert::IsTrue(result);
		Assert::IsTrue(isCloseEnough(Vec3(1.019, -1.0f, 1.0f), contact, ERROR_MARGIN_PHYSIC));
	}

	SP_TEST_METHOD(CLASS_NAME, contructorWithThreePoints_Test)
	{
		Vec3 point1 = { -10.0f, 0.0f, 0.0f };
		Vec3 point2 = { 10.0f, 0.0f, 0.0f };
		Vec3 point3 = { 0.0f, 10.0f, 0.0f };
		
		Vec3 normalVectorExpected = Vec3(0.0f, 0.0f, 1.0f);

		Plane3D plane = Plane3D(point1, point2, point3);

		Asserts::isCloseEnough(plane.normalVector[0], normalVectorExpected[0], ERROR_MARGIN_PHYSIC, L"Wrong value", LINE_INFO());
		Asserts::isCloseEnough(plane.normalVector[1], normalVectorExpected[1], ERROR_MARGIN_PHYSIC, L"Wrong value", LINE_INFO());
		Asserts::isCloseEnough(plane.normalVector[2], normalVectorExpected[2], ERROR_MARGIN_PHYSIC, L"Wrong value", LINE_INFO());

		Assert::AreEqual(point1.x, plane.point.x, L"Wrong value.", LINE_INFO());
		Assert::AreEqual(point1.y, plane.point.y, L"Wrong value.", LINE_INFO());
		Assert::AreEqual(point1.z, plane.point.z, L"Wrong value.", LINE_INFO());
	}
	
	SP_TEST_METHOD(CLASS_NAME, equation_Test1)
	{
		Vec3 point1 = { 2.0f, 1.0f, -1.0f };
		Vec3 point2 = { 0.0f, 1.0f, 2.0f };
		Vec3 point3 = { -1.0f, -1.0f, 3.0f };

		Vec4 expected = Vec4(0.8241f, -0.1373f, 0.5494f, 0.96152f);

		Plane3D plane = Plane3D(point1, point2, point3);
		Vec4 components;
		plane.equation(&components);

		for (size_t i = 0; i < 4; i++)
			Asserts::isCloseEnough(expected[i], components[i], 0.0009f, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, equation_Test2)
	{
		Vec3 point = { 2.0f, 1.0f, -1.0f };
		Vec3 vector = { 1.0f, -2.0f, 3.0f };
		normalize(&vector);

		Vec4 expected = Vec4(0.267f, -0.534f, 0.801f, -0.801f);

		Plane3D plane = Plane3D(point, vector);
		Vec4 components;
		plane.equation(&components);

		for (size_t i = 0; i < 4; i++)
			Asserts::isCloseEnough(expected[i], components[i], 0.0009f, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, equation_Test3)
	{
		Vec3 point1 = { 1.0f, 2.0f, 0.0f };
		Vec3 point2 = { 2.0f, 0.0f, -1.0f };
		Vec3 point3 = { 3.0f, -2.0f, -1.0f };

		Vec4 expected = Vec4(-0.8944f, -0.4472f, 0.0f, -1.7888f);

		Plane3D plane = Plane3D(point1, point2, point3);
		Vec4 components;
		plane.equation(&components);

		for (size_t i = 0; i < 4; i++)
			Asserts::isCloseEnough(expected[i], components[i], 0.0009f, L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, intersection_line)
	{
		Plane3D plane(
			Vec3(-1.0f, -1.0f, 1.0f),
			Vec3(-1.0f, 1.0f, 1.0f),
			Vec3(1.0f, 1.0f, 1.0f)
		);
		Line3D line(
			Vec3(-0.5f, 1.0f, -1.0f),
			Vec3(-0.5f, 1.0f, 1.0f)
		);

		Vec3 contact;
		sp_bool result = plane.intersection(line, &contact);
		Vec3 expected(-0.5f, 1.0f, 1.0f);
		Assert::IsTrue(result, L"Wrong value");
		Assert::IsTrue(isCloseEnough(contact, expected, 0.01f), L"Wrong value");

		plane = Plane3D(
			Vec3(-1.0f, -1.0f, -1.0f),
			Vec3(-1.0f, 1.0f, -1.0f),
			Vec3(1.0f, 1.0f, -1.0f)
		);
		result = plane.intersection(line, &contact);
		expected = Vec3(-0.5f, 1.0f, -1.0f);
		Assert::IsTrue(result, L"Wrong value");
		Assert::IsTrue(isCloseEnough(contact, expected, 0.01f), L"Wrong value");


		line = Line3D(Vec3(), Vec3{ 4.0f, 0.0f, 0.0f });
		plane = Plane3D(
			Vec3(1.0f, 0.0f, -2.0f),
			Vec3(1.0f, 2.0f, -2.0f),
			Vec3(1.0f, 0.0f, 2.0f)
		);
		expected = Vec3(1.0f, 0.0f, 0.0f);
		result = plane.intersection(line, &contact);
		Assert::IsTrue(result, L"There should be an intersection", LINE_INFO());
		Assert::IsTrue(isCloseEnough(expected, contact, 0.0009f), L"Wrong value", LINE_INFO());

		line.point2.y = 10.0f;
		expected.y = 2.5f;
		result = plane.intersection(line, &contact);
		Assert::IsTrue(result, L"There should be an intersection", LINE_INFO());
		Assert::IsTrue(isCloseEnough(expected, contact, 0.0009f), L"Wrong value", LINE_INFO());

		line.point2.y = 0.0f;
		line.point2.x = 0.5f;
		result = plane.intersection(line, &contact);
		Assert::IsFalse(result, L"There should be an intersection", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, angle_Test1)
	{
		Plane3D plane1 = Plane3D(Vec3(1.0f, 1.0f, 1.0f), Vec3(1.0f, 1.0f, 1.0f).normalize());
		Plane3D plane2 = Plane3D(Vec3(1.0f, 1.0f, 1.0f), Vec3(1.0f, -2.0f, 3.0f).normalize());

		Vec4 a, b;
		plane1.equation(&a);
		plane2.equation(&b);

		sp_float expected = 0.308606714f;
		sp_float result = plane1.angle(plane2);

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, angle_Test2)
	{
		Plane3D plane1 = Plane3D(2.0f, 4.0f, 1.0f, 3.0f);
		Plane3D plane2 = Plane3D(-1.0f, 3.0f, 2.0f, 1.0f);

		sp_float expected = 0.6998542122f;
		sp_float result = plane1.angle(plane2);

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, distance_point_Test1)
	{
		Plane3D plane = Plane3D(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f));
		Vec3 point = Vec3(0.0f, 0.0f, 10.0f);

		sp_float expected = 10.0f;
		sp_float result = plane.distance(point);

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, distance_point_Test2)
	{
		Plane3D plane = Plane3D(Vec3(0.0f, 0.0f, 8.0f), Vec3(0.0f, 0.0f, 1.0f));
		Vec3 point = Vec3(0.0f, 0.0f, 10.0f);

		sp_float expected = 2.0f;
		sp_float result = plane.distance(point);

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, distance_point_Test3)
	{
		Plane3D plane = Plane3D(Vec3(0.0f, 0.0f, -4.0f), Vec3(0.0f, 0.0f, 1.0f));
		Vec3 point = Vec3(0.0f, 0.0f, 10.0f);

		sp_float expected = 14.0f;
		sp_float result = plane.distance(point);

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, distance_point_Test4)
	{
		Vec3 normal(0.0f, 1.0f, 1.0f);
		normalize(&normal);

		Plane3D plane = Plane3D(Vec3(0.0f, 0.0f, 0.0f), normal);
		Vec3 point = Vec3(0.0f, 0.0f, 10.0f);

		sp_float expected = 7.07106781f;
		sp_float result = plane.distance(point);

		Assert::IsTrue(isCloseEnough(expected, result), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, distance_plane_Test1)
	{
		Plane3D plane1 = Plane3D(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f));
		Plane3D plane2 = Plane3D(Vec3(0.0f, 0.0f, 1.25f), Vec3(0.0f, 0.0f, 1.0f));
		
		sp_float result = plane1.distance(plane2);
		sp_float expected = 1.25f;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, closestPointOnThePlane_point_Test1)
	{
		Plane3D plane = Plane3D(Vec3(0.0f, 0.0f, 0.0f), Vec3(0.0f, 0.0f, 1.0f));
		Vec3 point = Vec3(10.0f, 5.0f, -10.0f);

		Vec3 expected = Vec3(10.0f, 5.0f, 0.0f);;

		Vec3 result = plane.closestPointOnThePlane(point);

		for (int i = 0; i < 3; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, constructor_equation_Test)
	{
		Plane3D plane = Plane3D(1.0f, 5.0f, 3.0f, 13.0f);
		Vec3 point = Vec3(2.0f, 4.0f, 1.0f);

		float expected = 6.42317247f;
		float result = plane.distance(point);
		Asserts::isCloseEnough(expected, result, DefaultErrorMargin, L"Wrong value", LINE_INFO());

		plane = Plane3D(2.0f, -2.0f, 5.0f, 8.0f);
		point = Vec3(4.0f, -4.0f, 3.0f);
		expected = 6.78902864f;
		result = plane.distance(point);
		Asserts::isCloseEnough(expected, result, DefaultErrorMargin, L"Wrong value", LINE_INFO());

		plane = Plane3D(2.0f, -2.0f, -1.0f, 3.0f);
		point = Vec3(2.0f, -1.0f, 2.0f);			
		expected = 2.33333325f;
		result = plane.distance(point);
		Asserts::isCloseEnough(expected, result, DefaultErrorMargin, L"Wrong value", LINE_INFO());

		plane = Plane3D(1.0f, 1.0f, 1.0f, 0.0f);
		point = Vec3(3.0f, -1.0f, 4.0f);
		expected = 3.46410179f;
		result = plane.distance(point);
		Asserts::isCloseEnough(expected, result, DefaultErrorMargin, L"Wrong value", LINE_INFO());

		plane = Plane3D(4.0f, -1.0f, 1.0f, 5.0f);
		point = Vec3(1.0f, 3.0f, -6.0f);
		expected = 0.0f;
		result = plane.distance(point);
		Asserts::isCloseEnough(expected, result, DefaultErrorMargin, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, orientation1)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane = Plane3D(point1, point2, point3);
		Vec3 targetPoint = Vec3(0.0f, 0.0f, 10.0f);

		sp_float result = plane.orientation(targetPoint);

		Assert::IsTrue(result > ZERO_FLOAT, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, orientation2)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane = Plane3D(point1, point2, point3);
		Vec3 targetPoint = Vec3(0.0f, 0.0f, -10.0f);

		sp_float result = plane.orientation(targetPoint);

		Assert::IsTrue(result < ZERO_FLOAT, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, orientation3)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane = Plane3D(point1, point2, point3);
		Vec3 targetPoint = Vec3(-10.0f, 10.0f, 0.0f);

		sp_float result = plane.orientation(targetPoint);

		Assert::IsTrue(result == ZERO_FLOAT, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isParallel_Test1)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane1 = Plane3D(point1, point2, point3);

		point1 = Vec3(0.0f, 0.0f, 10.0f);
		point2 = Vec3(1.0f, 0.0f, 10.0f);
		point3 = Vec3(1.0f, 1.0f, 10.0f);

		Plane3D plane2 = Plane3D(point1, point2, point3);

		bool result = plane1.isParallel(plane2);

		Assert::IsTrue(result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isParallel_Test2)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane1 = Plane3D(point1, point2, point3);

		point1 = Vec3(0.0f, 0.0f, -10.0f);
		point2 = Vec3(3.0f, 10.0f, -10.0f);
		point3 = Vec3(-4.0f, -1.0f, -10.0f);

		Plane3D plane2 = Plane3D(point1, point2, point3);

		bool result = plane1.isParallel(plane2);

		Assert::IsTrue(result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isParallel_Test3)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane1 = Plane3D(point1, point2, point3);

		point1 = Vec3(0.0f, 0.0f, -10.0f);
		point2 = Vec3(3.0f, 10.0f, 5.0f);
		point3 = Vec3(-4.0f, -1.0f, 10.0f);

		Plane3D plane2 = Plane3D(point1, point2, point3);

		sp_bool result = plane1.isParallel(plane2);

		Assert::IsFalse(result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isPerpendicular_Test1)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane1 = Plane3D(point1, point2, point3);

		point1 = Vec3(0.0f, 0.0f, 0.0f);
		point2 = Vec3(1.0f, 0.0f, 0.0f);
		point3 = Vec3(1.0f, 0.0f, -1.0f);

		Plane3D plane2 = Plane3D(point1, point2, point3);

		sp_bool result = plane1.isPerpendicular(plane2);

		Assert::IsTrue(result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isPerpendicular_Test2)
	{
		Vec3 point1 = Vec3(0.0f, 0.0f, 0.0f);
		Vec3 point2 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 point3 = Vec3(1.0f, 1.0f, 0.0f);

		Plane3D plane1 = Plane3D(point1, point2, point3);

		point1 = Vec3(0.0f, 0.0f, 0.0f);
		point2 = Vec3(1.0f, 1.0f, 0.0f);
		point3 = Vec3(1.0f, 0.0f, 1.0f);

		Plane3D plane2 = Plane3D(point1, point2, point3);

		bool result = plane1.isPerpendicular(plane2);

		Assert::IsFalse(result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, intersection_plane)
	{
		Plane3D plane1 = Plane3D(Vec3(), Vec3(1.0f, 0.0f, 0.0f));
		Plane3D plane2 = Plane3D(Vec3(), Vec3(0.0f, 1.0f, 0.0f));

		Line3D result;
		plane1.intersection(plane2, &result);

		Assert::IsTrue(result.point1 != result.point2, L"Line should not be null!", LINE_INFO());

		Vec3 expectedPoint1 = Vec3();
		Vec3 expectedPoint2 = Vec3(0.0f, 0.0f, 1.0f);
		
		for (int i = 0; i < 3; i++)
		{
			Assert::AreEqual(result.point1[i], expectedPoint1[i], L"Wrong value", LINE_INFO());
			Assert::AreEqual(result.point2[i], expectedPoint2[i], L"Wrong value", LINE_INFO());
		}
	}
}

#undef CLASS_NAME
