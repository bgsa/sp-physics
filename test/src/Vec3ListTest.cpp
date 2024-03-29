#include "SpectrumPhysicsTest.h"
#include <Vec3List.h>
#include <vector>

#define CLASS_NAME Vec3ListTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(Vec3List_constructorDefault_Test);

		SP_TEST_METHOD_DEF(Vec3List_findExtremePointsAlongAxisX_Test);

		SP_TEST_METHOD_DEF(Vec3List_findExtremePointsAlongAxisY_Test);

		SP_TEST_METHOD_DEF(Vec3List_findExtremePointsAlongAxisZ_Test);

		SP_TEST_METHOD_DEF(Vec3List_findExtremePointsAlongAxisXYZ_Test);

		SP_TEST_METHOD_DEF(Vec3List_covarianceOnAxis_Test);

		SP_TEST_METHOD_DEF(Vec3List_covariance_Test);

		SP_TEST_METHOD_DEF(Vec3List_closetPoint_UsingBruteForce_Test);

	};


	SP_TEST_METHOD(CLASS_NAME, Vec3List_constructorDefault_Test)
	{
		Vec3* points = ALLOC_ARRAY(Vec3, 2);
		points[0] = { 0.0f, 2.0f, -3.3f };
		points[1] = { -5.1f, 2.0f, 3.0f };

		Vec3List list = Vec3List(points, 2);

		Assert::AreEqual(TWO_UINT, list.count, L"Wrong value.", LINE_INFO());

		for (int i = 0; i < 2; i++)
			for (int j = 0; j < 3; j++)
				Assert::AreEqual(points[i][j], list.points[i][j], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_findExtremePointsAlongAxisX_Test)
	{
		int pointsCount = 6;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 0.0f, 0.0f, 0.3f };
		points[1] = { 10.0f, 10.0f, 10.0f };
		points[2] = { 5.0f, 5.0f, 5.0f };
		points[3] = { -1.0f, 1.0f, 1.0f }; //min point (index 3)
		points[4] = { 12.0f, 10.0f, 10.0f }; // max point (index 4)
		points[5] = { 8.0f, 1.0f, 1.0f };

		Vec3List list = Vec3List(points, pointsCount);

		int* result = list.findExtremePointsAlongAxisX();

		Assert::AreEqual(3, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(4, result[1], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_findExtremePointsAlongAxisY_Test)
	{
		int pointsCount = 6;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 0.0f, 0.0f, 0.3f };
		points[1] = { 10.0f, 10.0f, 10.0f };
		points[2] = { 5.0f, 5.0f, 5.0f };
		points[3] = { -1.0f, 1.0f, 1.0f };
		points[4] = { 12.0f, 10.0f, 10.0f };
		points[5] = { 8.0f, 1.0f, 1.0f };

		Vec3List list = Vec3List(points, pointsCount);

		int* result = list.findExtremePointsAlongAxisY();

		Assert::AreEqual(0, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(1, result[1], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_findExtremePointsAlongAxisZ_Test)
	{
		int pointsCount = 6;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 0.0f, 0.0f, 0.3f };
		points[1] = { 10.0f, 10.0f, 10.0f };
		points[2] = { 5.0f, 5.0f, 5.0f };
		points[3] = { -1.0f, 1.0f, 1.0f };
		points[4] = { 12.0f, 10.0f, 10.0f };
		points[5] = { 8.0f, 1.0f, 17.0f };

		Vec3List list = Vec3List(points, pointsCount);

		int* result = list.findExtremePointsAlongAxisZ();

		Assert::AreEqual(0, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(5, result[1], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_findExtremePointsAlongAxisXYZ_Test)
	{
		int pointsCount = 6;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 0.0f, 0.0f, 0.3f };
		points[1] = { 10.0f, 10.0f, 10.0f };
		points[2] = { 5.0f, 5.0f, 5.0f };
		points[3] = { -1.0f, 1.0f, 1.0f };
		points[4] = { 12.0f, 10.0f, 10.0f };
		points[5] = { 8.0f, 1.0f, 17.0f };

		Vec3List list = Vec3List(points, pointsCount);

		int* result = list.findExtremePointsAlongAxisXYZ();

		Assert::AreEqual(3, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(4, result[1], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(0, result[2], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(1, result[3], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(0, result[4], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(5, result[5], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_covarianceOnAxis_Test)
	{
		int pointsCount = 3;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 1.0f, 3.0f, -7.0f };
		points[1] = { 3.0f, 9.0f, 2.0f };
		points[2] = { -5.0f, 4.0f, 6.0f };

		Vec3List list = Vec3List(points, pointsCount);

		float result = list.covarianceOnAxis(0);
		float expectedOnAxisX = 11.5556f;
		Assert::IsTrue(isCloseEnough(expectedOnAxisX, result), L"Wrong value.", LINE_INFO());

		result = list.covarianceOnAxis(1);
		float expectedOnAxisY = 6.8889f;
		Assert::IsTrue(isCloseEnough(expectedOnAxisY, result), L"Wrong value.", LINE_INFO());

		result = list.covarianceOnAxis(2);
		float expectedOnAxisZ = 29.5556f;
		Assert::IsTrue(isCloseEnough(expectedOnAxisZ, result), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_covariance_Test)
	{
		int pointsCount = 3;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 1.0f, 3.0f, -7.0f };
		points[1] = { 3.0f, 9.0f, 2.0f };
		points[2] = { -5.0f, 4.0f, 6.0f };

		Vec3List list = Vec3List(points, pointsCount);

		Mat3 result = list.covariance();
		Mat3 expected = Mat3(
			11.5556f, 5.1111f, -10.2222f,
			5.1111f, 6.8889f, 5.2222f,
			-10.2222f, 5.2222f, 29.5556f
		);
		
		for (int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Vec3List_closetPoint_UsingBruteForce_Test)
	{
		int pointsCount = 5;
		Vec3* points = ALLOC_ARRAY(Vec3, pointsCount);
		points[0] = { 1.0f, 1.0f, 1.0f };
		points[1] = { 10.0f, 0.0f, 0.0f };
		points[2] = { 0.0f, 5.0f, 5.0f };
		points[3] = { 3.0f, 4.0f, 10.0f };
		points[4] = { 0.0f, 4.0f, 4.0f };

		Vec3List list = Vec3List(points, pointsCount);

		int* result = list.closestPoint_UsingBruteForce();
		int expected[2] = { 2,4 };

		Assert::AreEqual(expected[0], result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(expected[1], result[1], L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME