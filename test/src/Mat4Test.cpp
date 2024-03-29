#include "SpectrumPhysicsTest.h"
#include <Mat4.h>

#define CLASS_NAME Mat4Test

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(Mat4_constructor_4vec4_Test);
		SP_TEST_METHOD_DEF(Mat4_getValues_Test);
		SP_TEST_METHOD_DEF(get);
		SP_TEST_METHOD_DEF(Mat4_xAxis_Test);
		SP_TEST_METHOD_DEF(Mat4_yAxis_Test);
		SP_TEST_METHOD_DEF(Mat4_zAxis_Test);
		SP_TEST_METHOD_DEF(Mat4_wAxis_Test);
		SP_TEST_METHOD_DEF(Mat4_primaryDiagonal_Test);
		SP_TEST_METHOD_DEF(Mat4_secondaryDiagonal_Test);
		SP_TEST_METHOD_DEF(Mat4_identity_Test);
		SP_TEST_METHOD_DEF(Mat4_determinant_Test);
		SP_TEST_METHOD_DEF(Mat4_determinantIJ_Test);
		SP_TEST_METHOD_DEF(Mat4_cofactorIJ_Test);
		SP_TEST_METHOD_DEF(Mat4_transpose_Test);
		SP_TEST_METHOD_DEF(Mat4_multiply_MajorColumnOrder1_Test);
		SP_TEST_METHOD_DEF(Mat4_createScaled_Test);
		SP_TEST_METHOD_DEF(Mat4_scale_Test);
		SP_TEST_METHOD_DEF(Mat4_createRotate_Test);
		SP_TEST_METHOD_DEF(Mat4_createTranslate_Test);
		SP_TEST_METHOD_DEF(Mat4_toMat3_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorMultiplyValue_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorPlusMatrix_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorMinusMatrix_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorEqualValue_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorEqualMatrix_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorNotEqualMatrix_Test);
		SP_TEST_METHOD_DEF(Mat4_operatorIndex_Test);
		SP_TEST_METHOD_DEF(Mat4_divide_operator_Test);
		SP_TEST_METHOD_DEF(Mat4_multiply_MajorColumnOrder2_Test);
		SP_TEST_METHOD_DEF(polyname);
		SP_TEST_METHOD_DEF(tridiagonal);
	};

	SP_TEST_METHOD(CLASS_NAME, tridiagonal)
	{
		Mat4 matrix = {
			4.0f, 1.0f, -2.0f, 2.0f,
			1.0f, 2.0f, 0.0f, 1.0f,
			-2.0f, 0.0f, 3.0f, -2.0f,
			2.0f, 1.0f, -2.0f, -1.0f
		};

		Mat4 result;
		matrix.tridiagonal(&result);

		Mat4 expected = {
			4.0f, -3.0f, 0.0f, 0.0f,
			-3.0f, 3.33f, -1.66f, 0.0f,
			0.0f, -1.66f, -1.32f, 0.9f,
			0.0f ,0.0f, 0.9f, 1.986f
		};

		for (sp_uint i = 0; i < MAT4_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], SP_EPSILON_TWO_DIGITS), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, polyname)
	{
		Mat4 matrix = {
			1.0f, 2.0f, 3.0f, 4.0f,
			2.0f, 1.0f, 2.0f, 3.0f,
			3.0f, 2.0f, 1.0f, 2.0f,
			4.0f ,3.0f, 2.0f, 1.0f
		};

		sp_float result[5];
		matrix.polyname(result);

		Assert::IsTrue(isCloseEnough(-1.0f, result[0]), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(4.0f, result[1]), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(40.0f, result[2]), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(56.0f, result[3]), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(20.0f, result[4]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_constructor_4vec4_Test)
	{
		Vec4 vector1 = Vec4(1.0f, 2.0f, 3.0f, 4.0f);
		Vec4 vector2 = Vec4(5.0f, 6.0f, 7.0f, 8.0f);
		Vec4 vector3 = Vec4(9.0f, 10.0f, 11.0f, 12.0f);
		Vec4 vector4 = Vec4(13.0f, 14.0f, 15.0f, 16.0f);

		Mat4 matrix = Mat4(vector1, vector2, vector3, vector4);

#if MAJOR_COLUMN_ORDER
		Mat4 expected = Mat4(
			1.0f, 5.0f, 9.0f, 13.0f,
			2.0f, 6.0f, 10.0f, 14.0f,
			3.0f, 7.0f, 11.0f, 15.0f,
			4.0f, 8.0f, 12.0f, 16.0f
		);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], matrix[i], L"Value shoud be 0", LINE_INFO());
#else
		for (sp_int i = 0; i < 4; i++)
			Assert::AreEqual(vector1[i], matrix[i], L"Value shoud be 0", LINE_INFO());

		for (sp_int i = 4; i < 8; i++)
			Assert::AreEqual(vector2[i], matrix[i], L"Value shoud be 0", LINE_INFO());

		for (sp_int i = 8; i < 12; i++)
			Assert::AreEqual(vector3[i], matrix[i], L"Value shoud be 0", LINE_INFO());

		for (sp_int i = 12; i < 16; i++)
			Assert::AreEqual(vector4[i], matrix[i], L"Value shoud be 0", LINE_INFO());
#endif
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_getValues_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		sp_float* result = matrix.values();

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(matrix[i], result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, get)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Assert::AreEqual(1.0f, matrix.get(0, 0), L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, matrix.get(0, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(3.0f, matrix.get(0, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, matrix.get(0, 3), L"Wrong value", LINE_INFO());

		Assert::AreEqual(5.0f, matrix.get(1, 0), L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, matrix.get(1, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, matrix.get(1, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, matrix.get(1, 3), L"Wrong value", LINE_INFO());

		Assert::AreEqual(9.0f, matrix.get(2, 0), L"Wrong value", LINE_INFO());
		Assert::AreEqual(10.0f, matrix.get(2, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(11.0f, matrix.get(2, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(12.0f, matrix.get(2, 3), L"Wrong value", LINE_INFO());

		Assert::AreEqual(13.0f, matrix.get(3, 0), L"Wrong value", LINE_INFO());
		Assert::AreEqual(14.0f, matrix.get(3, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(15.0f, matrix.get(3, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(16.0f, matrix.get(3, 3), L"Wrong value", LINE_INFO());
	}

#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_xAxis_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, xAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, xAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(13.0f, xAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_xAxis_MajorRowOrder_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, xAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(3.0f, xAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, xAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_yAxis_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 yAxis = matrix.yAxis();

		Assert::AreEqual(2.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, yAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(10.0f, yAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(14.0f, yAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_yAxis_MajorRowOrder_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 yAxis = matrix.yAxis();

		Assert::AreEqual(5.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, yAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, yAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, yAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_zAxis_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 zAxis = matrix.zAxis();

		Assert::AreEqual(3.0f, zAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, zAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(11.0f, zAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(15.0f, zAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_zAxis_MajorRowOrder_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 zAxis = matrix.zAxis();

		Assert::AreEqual(9.0f, zAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(10.0f, zAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(11.0f, zAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(12.0f, zAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_wAxis_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 wAxis = matrix.wAxis();

		Assert::AreEqual(4.0f, wAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, wAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(12.0f, wAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(16.0f, wAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_wAxis_MajorRowOrder_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 wAxis = matrix.wAxis();

		Assert::AreEqual(13.0f, wAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(14.0f, wAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(15.0f, wAxis[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(16.0f, wAxis[3], L"Wrong value", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat4_primaryDiagonal_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 primaryDiagonal = matrix.primaryDiagonal();

		Assert::AreEqual(1.0f, primaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, primaryDiagonal[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(11.0f, primaryDiagonal[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(16.0f, primaryDiagonal[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_secondaryDiagonal_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Vec4 secondaryDiagonal = matrix.secondaryDiagonal();

		Assert::AreEqual(4.0f, secondaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, secondaryDiagonal[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(10.0f, secondaryDiagonal[2], L"Wrong value", LINE_INFO());
		Assert::AreEqual(13.0f, secondaryDiagonal[3], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_identity_Test)
	{
		Mat4 result = Mat4Identity;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (i % (MAT4_ROW_LENGTH+1) == 0)
				Assert::AreEqual(1.0f, result[i], L"Value shoud be 0", LINE_INFO());
			else
				Assert::AreEqual(0.0f, result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_determinant_Test)
	{
		Mat4 matrix = {
			2.0f,  1.0f,  6.0f,  0.0f,
			5.0f,  0.0f,  1.0f,  3.0f,
			-2.0f,  5.0f,  1.0f,  8.0f,
			11.0f,  4.0f, -2.0f, -7.0f
		};
		sp_float result = matrix.determinant();
		sp_float expected = -3192.0f;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_determinantIJ_Test)
	{
		Mat4 matrix = {
			2.0f,  1.0f,  6.0f,  0.0f,
			5.0f,  0.0f,  1.0f,  3.0f,
			-2.0f,  5.0f,  1.0f,  8.0f,
			11.0f,  4.0f, -2.0f, -7.0f
		};
		sp_float result1 = matrix.determinantIJ(1,0);
		sp_float expected1 = 411.0f;
		Assert::AreEqual(expected1, result1, L"Wrong value", LINE_INFO());

		sp_float result2 = matrix.determinantIJ(1, 1);
		sp_float expected2 = 462.0f;
		Assert::AreEqual(expected2, result2, L"Wrong value", LINE_INFO());

		Mat3 matrix3x3 = {
			0.0f,  1.0f,  3.0f,
			5.0f,  1.0f,  8.0f,
			4.0f, -2.0f, -7.0f
		};
		sp_float determinant3x3 = matrix3x3.determinant();

		sp_float result3 = matrix.determinantIJ(0, 0);
		Assert::AreEqual(determinant3x3, result3, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_cofactorIJ_Test)
	{
		Mat4 matrix = {
			2.0f,  1.0f,  6.0f,  0.0f,
			5.0f,  0.0f,  1.0f,  3.0f,
			-2.0f,  5.0f,  1.0f,  8.0f,
			11.0f,  4.0f, -2.0f, -7.0f
		};
		sp_float result1 = matrix.cofactorIJ(1, 0);
		sp_float expected1 = -411.0f;
		Assert::AreEqual(expected1, result1, L"Wrong value", LINE_INFO());

		sp_float result2 = matrix.cofactorIJ(1, 1);
		sp_float expected2 = 462.0f;
		Assert::AreEqual(expected2, result2, L"Wrong value", LINE_INFO());

		sp_float result3 = matrix.cofactorIJ(1, 2);
		sp_float expected3 = 60.0f;
		Assert::AreEqual(expected3, result3, L"Wrong value", LINE_INFO());

		sp_float result4 = matrix.cofactorIJ(1, 3);
		sp_float expected4 = -399.0f;
		Assert::AreEqual(expected4, result4, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_transpose_Test)
	{
		Mat4 matrix = {
			1.0f,  2.0f,  3.0f,  4.0f,
			5.0f,  6.0f,  7.0f,  8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};
		Mat4 result;
		matrix.transpose(result);

		sp_float expected[MAT4_LENGTH] = {
			1.0f,  5.0f,  9.0f,  13.0f,
			2.0f,  6.0f,  10.0f,  14.0f,
			3.0f, 7.0f, 11.0f, 15.0f,
			4.0f, 8.0f, 12.0f, 16.0f
		};

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value", LINE_INFO());
	}

#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_multiply_MajorColumnOrder1_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			2.0f, 5.0f, -2.0f, 4.0f,
			1.0f, -3.0f, 8.0f, 8.0f,
			0.0f, 7.0f, -4.0f, 7.0f,
			4.0f, 4.0f, 5.0f, 2.0f
		};
		Mat4 expected = {
			-4.0f, 72.0f, 22.0f, 2.0f,
			42.0f, -29.0f, -2.0f, 39.0f,
			-19.0f, 104.0f, 24.0f, -26.0f,
			25.0f, 27.0f, 29.0f, 43.0f
		};

		Mat4 result;
		matrixA.multiply(matrixB, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_multiply_MajorColumnOrder2_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			2.0f, 5.0f, -2.0f, 4.0f,
			1.0f, -3.0f, 8.0f, 8.0f,
			0.0f, 7.0f, -4.0f, 7.0f,
			4.0f, 4.0f, 5.0f, 2.0f
		};
		Mat4 expected = {
			-4.0f, 72.0f, 22.0f, 2.0f,
			42.0f, -29.0f, -2.0f, 39.0f,
			-19.0f, 104.0f, 24.0f, -26.0f,
			25.0f, 27.0f, 29.0f, 43.0f
		};

		Mat4 result;
		matrixA.multiply(matrixB, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_multiply_MajorRowOrder1_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			2.0f, 5.0f, -2.0f, 4.0f,
			1.0f, -3.0f, 8.0f, 8.0f,
			0.0f, 7.0f, -4.0f, 7.0f,
			4.0f, 4.0f, 5.0f, 2.0f
		};
		Mat4 expected = {
			47.0f, 61.0f, 53.0f, 72.0f,
			-4.0f, -14.0f, 42.0f, 80.0f,
			13.0f, 49.0f, -35.0f, -15.0f,
			6.0f, -7.0f, 30.0f, 36.0f
		};

		Mat4 result = matrixA.multiply(matrixB);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_multiply_MajorRowOrder2_Test)
	{
		Mat4 matrixA = {
			1.0f, 0.0f,  0.0f,  0.0f,
			3.0f, -3.0f,  0.0f,  0.0f,
			-1.0f, 0.0f,  2.0f,  0.0f,
			-3.0f, 4.0f, -2.0f,  1.0f
		};

		Mat4 matrixB = {
			1.0f, -2.0f, -2.0f, -3.0f,
			0.0f, 1.0f,  6.0f,  0.0f,
			0.0f,  0.0f,  1.0f,  4.0f,
			0.0f,  0.0f,  0.0f,  1.0f
		};

		Mat4 expected = {
			1.0f, -2.0f, -2.0f, -3.0f,
			3.0f, -9.0f, -24.0f, -9.0f,
			-1.0f, 2.0f, 4.0f, 11.0f,
			-3.0f, 10.0f, 28.0f, 2.0f
		};

		Mat4 result = matrixA * matrixB;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat4_createScaled_Test)
	{
		Mat4 expected = {
			2.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 4.0f, 0.0f, 0.0f,
			0.0f, 0.0f, -3.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};

		Mat4 result;
		createScale(2.0f, 4.0f, -3.0f, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_scale_Test)
	{
		Mat4 matrixA = {
			2.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 4.0f, 0.0f, 0.0f,
			0.0f, 0.0f, -3.0f, 0.0f,
			1.0f, -10.0f, 5.0f, 1.0f
		};
		Mat4 expected = {
			4.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 16.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 9.0f, 0.0f,
			1.0f, -10.0f, 5.0f, 1.0f
		};

		matrixA.scale(2.0f, 4.0f, -3.0f);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_createRotate_Test)
	{
		Mat4 expected = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 0.866025388f, 0.5f, 0.0f,
			0.0f, -0.5f, 0.866025388f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};

		sp_float angle = (sp_float)radians(30);
		
		Mat4 result;
		createRotate(angle, 1.0f, 0.0f, 0.0f, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(result[i], expected[i]), L"Wrong number", LINE_INFO());
	}

#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_createTranslate_Test)
	{
		Mat4 expected = {
			1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 2.0f, -3.0f, 1.0f
		};

		Mat4 result;
		createTranslate(0.0f, 2.0f, -3.0f, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_createTranslate_MajorRowOrder_Test)
	{
		Mat4 expected = {
			1.0f, 0.0f, 0.0f, 4.0f,
			0.0f, 1.0f, 0.0f, 2.0f,
			0.0f, 0.0f, 1.0f, -3.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};

		Mat4 result = Mat4::createTranslate(4.0f, 2.0f, -3.0f);

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat4_toMat3_Test)
	{
		Mat4 matrix = {
			2.0f, 5.0f, -2.0f, 4.0f,
			1.0f, -3.0f, 8.0f, 8.0f,
			0.0f, 7.0f, -4.0f, 7.0f,
			4.0f, 4.0f, 5.0f, 2.0f
		};
		Mat3 expected = {
			2.0f, 5.0f, -2.0f,
			1.0f, -3.0f, 8.0f,
			0.0f, 7.0f, -4.0f
		};

		Mat3 result;
		matrix.toMat3(result);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
	
#ifdef MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorMultiplyValue_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 expected = {
			8.0f, 6.0f, 4.0f, 18.0f,
			-4.0f, 16.0f, 8.0f, -4.0f,
			6.0f, -10.0f, 2.0f, 6.0f,
			2.0f, 8.0f, 0.0f, 0.0f
		};

		Mat4 result;
		multiply(matrixA, 2.0f, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorPlusMatrix_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			2.0f, 5.0f, -2.0f, 4.0f,
			1.0f, -3.0f, 8.0f, 8.0f,
			0.0f, 7.0f, -4.0f, 7.0f,
			4.0f, 4.0f, 5.0f, 2.0f
		};
		Mat4 expected = {
			6.0f, 8.0f, 0.0f, 13.0f,
			-1.0f, 5.0f, 12.0f, 6.0f,
			3.0f, 2.0f, -3.0f, 10.0f,
			5.0f, 8.0f, 5.0f, 2.0f
		};

		Mat4 result;
		add(matrixA, matrixB, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorMinusMatrix_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			2.0f, 5.0f, -2.0f, 4.0f,
			1.0f, -3.0f, 8.0f, 8.0f,
			0.0f, 7.0f, -4.0f, 7.0f,
			4.0f, 4.0f, 5.0f, 2.0f
		};
		Mat4 expected = {
			2.0f, -2.0f, 4.0f, 5.0f,
			-3.0f, 11.0f, -4.0f, -10.0f,
			3.0f, -12.0f, 5.0f, -4.0f,
			-3.0f, 0.0f, -5.0f, -2.0f
		};

		Mat4 result;
		diff(matrixA, matrixB, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorEqualValue_Test)
	{
		Mat4 matrixA = {
			1.0f,  1.0f,  1.0f, 1.0f,
			1.0f,  1.0f,  1.0f, 1.0f,
			1.0f,  1.0f,  1.0f, 1.0f,
			1.0f,  1.0f,  1.0f, 1.0f
		};

		sp_bool result = matrixA == 1.0f;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixA[4] = 0.0f;

		result = matrixA == 1.0f;

		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorEqualMatrix_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};

		sp_bool result = matrixA == matrixB;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixB[5] = 10.0f;
		result = matrixA == matrixB;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorNotEqualMatrix_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		Mat4 matrixB = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};

		sp_bool result = matrixA != matrixB;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());

		matrixB.m21 = 10.0f;
		result = matrixA != matrixB;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat4_operatorIndex_Test)
	{
		Mat4 matrixA = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};
		sp_float expected[16] = {
			4.0f,  3.0f,  2.0f, 9.0f,
			-2.0f,  8.0f, 4.0f, -2.0f,
			3.0f,  -5.0f,  1.0f, 3.0f,
			1.0f, 4.0f,  0.0f, 0.0f
		};

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_decomposeLU_MajorRowOrder_Test)
	{
		Mat4 matrix = {
			1.0f, -2.0f, -2.0f, -3.0f,
			3.0f, -9.0f,  0.0f, -9.0f,
			-1.0f,  2.0f,  4.0f,  7.0f,
			-3.0f, -6.0f,  26.0f, 2.0f
		};

		Mat4 lowerMatrixExpected = {
			1.0f, 0.0f,  0.0f,  0.0f,
			3.0f, -3.0f,  0.0f,  0.0f,
			-1.0f, 0.0f,  2.0f,  0.0f,
			-3.0f, -12.0f, -4.0f,  1.0f
		};

		Mat4 upperMatrixExpected = {
			1.0f, -2.0f, -2.0f, -3.0f,
			0.0f, 1.0f,  -2.0f,  0.0f,
			0.0f,  0.0f,  1.0f,  2.0f,
			0.0f,  0.0f,  0.0f,  1.0f
		};

		Mat4* decomposeLU = matrix.decomposeLU();
		Mat4 lowerMatrixResult = decomposeLU[0];
		Mat4 upperMatrixResult = decomposeLU[1];
		Mat4 result = lowerMatrixResult * upperMatrixResult;

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());
	}
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat4_decomposeLDU_MajorRowOrder_Test)
	{
		Mat4 matrix = {
				1.0f, -2.0f, -2.0f, -3.0f,
				3.0f, -9.0f,  0.0f, -9.0f,
			-1.0f,  2.0f,  4.0f,  7.0f,
			-3.0f, -6.0f,  26.0f, 2.0f
		};

		Mat4 lowerMatrixExpected = {
				1.0f, 0.0f,  0.0f,  0.0f,
				3.0f, -3.0f,  0.0f,  0.0f,
			-1.0f, 0.0f,  2.0f,  0.0f,
			-3.0f, -12.0f, -4.0f,  1.0f
		};

		Mat4 diagonalMatrixExpected = {
			1.0f, 0.0f,  0.0f, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f,  0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};

		Mat4 upperMatrixExpected = {
			1.0f, -2.0f, -2.0f, -3.0f,
			0.0f, 1.0f, -2.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 2.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		};

		Mat4* decomposeLDU = matrix.decomposeLDU();
		Mat4 lowerMatrixResult = decomposeLDU[0];
		Mat4 diagonalMatrixResult = decomposeLDU[1];
		Mat4 upperMatrixResult = decomposeLDU[2];
		Mat4 result = lowerMatrixResult * diagonalMatrixResult * upperMatrixResult;

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (sp_size i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(diagonalMatrixResult[i], diagonalMatrixExpected[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat4_divide_operator_Test)
	{
		Mat4 matrixA = {
			2.0f, 1.0f, 3.0f, -6.0f,
			0.0f, 4.0f, -2.0f, 4.0f,
			7.0f, 1.0f, 0.0f, 4.0f,
			7.0f, 1.0f, 0.0f, 4.0f
		};
		Mat4 expected = {
			1.0f, 0.5f, 1.5f, -3.0f,
			0.0f, 2.0f, -1.0f, 2.0f,
			3.5f, 0.5f, 0.0f, 2.0f,
			3.5f, 0.5f, 0.0f, 2.0f
		};

		Mat4 result;
		divide(matrixA, 2.0f, result);

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

}

#undef CLASS_NAME