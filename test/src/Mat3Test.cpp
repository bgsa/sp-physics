#include "SpectrumPhysicsTest.h"
#include "Asserts.h"
#include <Mat3.h>

#define CLASS_NAME Mat3Test

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(get);
		SP_TEST_METHOD_DEF(adjoint);
		SP_TEST_METHOD_DEF(Mat3_getAxisX_Test);
		SP_TEST_METHOD_DEF(Mat3_getAxisY_Test);
		SP_TEST_METHOD_DEF(Mat3_getAxisZ_Test);
		SP_TEST_METHOD_DEF(Mat3_xAxis_Test);
		SP_TEST_METHOD_DEF(Mat3_yAxis_Test);
		SP_TEST_METHOD_DEF(Mat3_zAxis_Test);
		SP_TEST_METHOD_DEF(Mat3_createTranslate_Test);
		SP_TEST_METHOD_DEF(Mat3_primaryDiagonal_Test);
		SP_TEST_METHOD_DEF(Mat3_secondaryDiagonal_Test);
		SP_TEST_METHOD_DEF(transpose);
		SP_TEST_METHOD_DEF(Mat3_createScaled_Test);
		SP_TEST_METHOD_DEF(Mat3_createRotate_Test);
		SP_TEST_METHOD_DEF(Mat3_scale_Test);
		SP_TEST_METHOD_DEF(Mat3_determinant_Test);
		SP_TEST_METHOD_DEF(Mat3_sizeInBytes_Test);
		SP_TEST_METHOD_DEF(Mat3_clone_Test);
		SP_TEST_METHOD_DEF(eigenValueAndVectorMax);
		SP_TEST_METHOD_DEF(eigenValues);
		SP_TEST_METHOD_DEF(eigenValuesAndVectors);
		SP_TEST_METHOD_DEF(Mat3_operatorMinus_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorMinus_matrix3_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorSum_matrix3_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorDivide_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorDivideEqual_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorEqual_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorNotEqual_Test);
		SP_TEST_METHOD_DEF(Mat3_operatorEqual_Value_Test);
		SP_TEST_METHOD_DEF(isIdentity);
		SP_TEST_METHOD_DEF(isHermitian);
		SP_TEST_METHOD_DEF(isOrthogonal);
		SP_TEST_METHOD_DEF(multiply_Vec3xVec3);
		SP_TEST_METHOD_DEF(multiply_Mat3xMat3);
		SP_TEST_METHOD_DEF(diagonalize);
		SP_TEST_METHOD_DEF(isSymetric);
		SP_TEST_METHOD_DEF(isDiagonallyDominant);
		SP_TEST_METHOD_DEF(isPositiveDefinite);
		SP_TEST_METHOD_DEF(polyname_1);
		SP_TEST_METHOD_DEF(polyname_2);
		SP_TEST_METHOD_DEF(sqrt);
		SP_TEST_METHOD_DEF(decomposeLU);
		SP_TEST_METHOD_DEF(decomposeLDU);
		SP_TEST_METHOD_DEF(decomposeLLt);
		SP_TEST_METHOD_DEF(convert);
	};

	SP_TEST_METHOD(CLASS_NAME, adjoint)
	{
		Mat3 matrix(
			1.0f, 3.0f, -1.0f,
			0.0f, 2.0f, -4.0f,
			1.0f, 0.0f, 5.0f
		);
		Mat3 expected(
			10.0f, -15.0f, -10.0f,
			-4.0f, 6.0f, 4.0f,
			-2.0f, 3.0f, 2.0f
		);

		Mat3 result;
		matrix.adjoint(result);

		for (sp_uint i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isHermitian)
	{
		Mat3 matrix(
			1.0f, 0.0f, 1.0f,
			0.0f, 2.0f, 0.0f,
			1.0f, 0.0f, 1.0f
		);
		
		Assert::IsTrue(matrix.isHermitian(), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isOrthogonal)
	{
		Mat3 matrix(
			0.4285f, 0.2857f, 0.8571f,
			-0.8571f, 0.4285f, 0.2857f,
			0.2857f, 0.8571f, -0.4285f
		);

		Assert::IsTrue(matrix.isOrthogonal(), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, sqrt)
	{
		Mat3 matrix(
			2.0f, 3.0f, 3.0f,
			3.0f, 2.0f, 3.0f,
			3.0f, 3.0f, 1.0f
		);
		Mat3 expected(
			sqrtf(2.0f), sqrtf(3.0f), sqrtf(3.0f),
			sqrtf(3.0f), sqrtf(2.0f), sqrtf(3.0f),
			sqrtf(3.0f), sqrtf(3.0f), sqrtf(1.0f)
		);

		Mat3 result;
		matrix.sqrt(result);

		for (sp_uint i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, polyname_1)
	{
		Mat3 matrix(
			3.0f, 1.0f, 5.0f,
			3.0f, 3.0f, 1.0f,
			4.0f, 6.0f, 4.0f
		);

		Vec4 result;
		matrix.polyname(&result);

		Assert::IsTrue(isCloseEnough(-1.0f, result.x), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(10.0f, result.y), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(-4.0f, result.z), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(40.0f, result.w), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, polyname_2)
	{
		Mat3 matrix(
			5.0f, 3.0f, 2.0f,
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f
		);

		Vec4 result;
		matrix.polyname(&result);

		Assert::IsTrue(isCloseEnough(-1.0f, result.x), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(5.0f, result.y), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(3.0f, result.z), L"Wrong value", LINE_INFO());
		Assert::IsTrue(isCloseEnough(2.0f, result.w), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isSymetric)
	{
		Mat3 matrix(
			4.0f, 2.0f, 0.0f,
			2.0f, 5.0f, 3.0f,
			0.0f, 3.0f, 6.0f
		);
		Assert::IsTrue(matrix.isSymetric(), L"Value shoud be 0", LINE_INFO());

		matrix[0] = 1.0f;
		Assert::IsTrue(matrix.isSymetric(), L"Value shoud be 0", LINE_INFO());

		matrix[1] = 1.0f;
		Assert::IsFalse(matrix.isSymetric(), L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isDiagonallyDominant)
	{
		Mat3 matrix(
			7.0f, 2.0f, 0.0f,
			3.0f, 5.0f, -1.0f,
			0.0f, 5.0f, -6.0f
		);
		Assert::IsTrue(matrix.isDiagonallyDominant(), L"", LINE_INFO());

		matrix = {
			 6.0f,  4.0f, -3.0f,
			 4.0f, -2.0f,  0.0f,
			-3.0f,  0.0f,  1.0f
		};
		Assert::IsFalse(matrix.isDiagonallyDominant(), L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isPositiveDefinite)
	{
		Mat3 matrix(
			2.0f, -1.0f, 0.0f,
			-1.0f, 2.0f, -1.0f,
			0.0f, -1.0f, 2.0f
		);
		Assert::IsTrue(matrix.isPositiveDefinite(), L"", LINE_INFO());

		matrix = {
			 6.0f,  4.0f, -3.0f,
			 4.0f, -2.0f,  0.0f,
			-3.0f,  0.0f,  1.0f
		};
		Assert::IsFalse(matrix.isPositiveDefinite(), L"Value shoud be 0", LINE_INFO());
	}
	
	SP_TEST_METHOD(CLASS_NAME, diagonalize)
	{
		Mat3 matrix(
			4.0f, 2.0f, 0.0f,
			2.0f, 5.0f, 3.0f,
			0.0f, 3.0f, 6.0f
		);

		Vec3 expected(4.63f, 8.90f, 1.45f);

		sp_uint iterations;
		Mat3 result;
		matrix.diagonalize(result, iterations, SP_EPSILON_TWO_DIGITS);

		Vec3 temp;
		result.primaryDiagonal(temp);

		for (sp_uint i = 0; i < VEC3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], temp[i], SP_EPSILON_TWO_DIGITS), L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, multiply_Vec3xVec3)
	{
		Vec3 v1 = { 1.0f, 2.0f, 3.0f };
		Vec3 v2 = { 4.0f, 5.0f, 6.0f };
		Mat3 result;
		
		multiply(v1, v2, result);

		Mat3 expected = {
			4.0f, 5.0f, 6.0f,
			8.0f, 10.0f, 12.0f,
			12.0f, 15.0f, 18.0f
		};

		for (sp_uint i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(expected[i] == result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, get)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Assert::AreEqual(1.0f, matrix.get(1, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, matrix.get(1, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, matrix.get(1, 3), L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, matrix.get(2, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, matrix.get(2, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, matrix.get(2, 3), L"Wrong value", LINE_INFO());
		Assert::AreEqual(3.0f, matrix.get(3, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, matrix.get(3, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, matrix.get(3, 3), L"Wrong value", LINE_INFO());
	}

#ifdef MAJOR_COLUMN_ORDER

	SP_TEST_METHOD(CLASS_NAME, Mat3_getAxisX_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 axis = matrix.axis(0);

		Assert::AreEqual(1.0f, axis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, axis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, axis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_getAxisY_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 axis = matrix.axis(1);

		Assert::AreEqual(2.0f, axis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, axis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, axis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_getAxisZ_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 axis = matrix.axis(2);

		Assert::AreEqual(3.0f, axis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, axis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, axis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_xAxis_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, xAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, xAxis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_yAxis_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 yAxis = matrix.yAxis();

		Assert::AreEqual(2.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, yAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, yAxis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_zAxis_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 zAxis = matrix.zAxis();

		Assert::AreEqual(3.0f, zAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, zAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, zAxis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, multiply_Mat3xMat3)
	{
		Mat3 matrixA = {
			2.0f, 3.0f, 4.0f,
			3.0f, 4.0f, 5.0f,
			4.0f, 5.0f, 6.0f
		};
		Mat3 matrixB = {
			0.0f, 1.0f, 2.0f,
			-1.0f, 0.0f, 1.0f,
			-2.0f, -1.0f, 0.0f
		};
		Mat3 expected = {
			11.0f, 14.0f, 17.0f,
			2.0f, 2.0f, 2.0f,
			-7.0f, -10.0f, -13.0f
		};

		Mat3 result;
		multiply(matrixA, matrixB, result);

		for (sp_int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
	
	SP_TEST_METHOD(CLASS_NAME, Mat3_createTranslate_Test)
	{
		Mat3 expected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			5.0f, 2.0f, -3.0f
		};

		Mat3 result = Mat3::createTranslate(5.0f, 2.0f, -3.0f);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, decomposeLU)
	{
		Mat3 matrix = {
			6.0f, -2.0f,  0.0f,
			9.0f, -1.0f,  1.0f,
			3.0f,  7.0f,  5.0f
		};

		Mat3 upperExpected = {
			1.0f,  -0.3333f,  0.0f,
			0.0f,  1.0f,  0.5f,
			0.0f, 0.0f,  1.0f
		};
		
		Mat3 lowerExpected = {
			6.0f, 0.0f, 0.0f,
			9.0f, 2.0f, 0.0f,
			3.0f, 8.0f, 1.0f
		};

		Mat3 L, U;
		matrix.decomposeLU(L, U);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(lowerExpected[i], L[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(upperExpected[i], U[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		Mat3 result = U * L;
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(matrix[i], result[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, decomposeLDU)
	{
		Mat3 matrix = {
			6.0f, -2.0f,  0.0f,
			9.0f, -1.0f,  1.0f,
			3.0f,  7.0f,  5.0f
		};

		Mat3 upperExpected = {
			1.0f,  -0.3333f,  0.0f,
			0.0f,  1.0f,  0.5f,
			0.0f, 0.0f,  1.0f
		};

		Mat3 diagonalExpected = {
			6.0f, 0.0f, 0.0f,
			0.0f, 2.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3 lowerExpected = {
			1.0f, 0.0f, 0.0f,
			1.5f, 1.0f, 0.0f,
			0.5f, 4.0f, 1.0f
		};

		Mat3 L, D, U;
		matrix.decomposeLDU(L, D, U);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(lowerExpected[i], L[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(upperExpected[i], U[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(diagonalExpected[i], D[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		Mat3 result = U * D * L;
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(matrix[i], result[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, decomposeLLt)
	{
		Mat3 matrix = {
			 4.0f, -1.00f,  1.00f,
			-1.0f,  4.25f,  2.75f,
			 1.0f,  2.75f,  3.50f
		};

		Mat3 lowerExpected = {
			 2.0f, 0.0f, 0.0f,
			-0.5f, 2.0f, 0.0f,
			 0.5f, 1.5f, 1.0f
		};
		Mat3 lowerTransposed;
		lowerExpected.transpose(lowerTransposed);

		Mat3 L, Lt;
		matrix.decomposeLLt(L, Lt);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(lowerExpected[i], L[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(lowerTransposed[i], Lt[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		Mat3 result = Lt * L;
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Asserts::isCloseEnough(matrix[i], result[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}
	
#endif

#ifdef MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_xAxis_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, xAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(3.0f, xAxis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_yAxis_RowMajorRow_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 yAxis = matrix.yAxis();

		Assert::AreEqual(4.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, yAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, yAxis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_zAxis_RowMajorRow_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 zAxis = matrix.zAxis();

		Assert::AreEqual(7.0f, zAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, zAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, zAxis[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_multiply_MajorRowOrder_Test)
	{
		Mat3 matrixA = {
			2.0f, 3.0f, 4.0f,
			3.0f, 4.0f, 5.0f,
			4.0f, 5.0f, 6.0f
		};
		Mat3 matrixB = {
			0.0f, 1.0f, 2.0f,
			-1.0f, 0.0f, 1.0f,
			-2.0f, -1.0f, 0.0f
		};
		Mat3 expected = {
			-11.0f, -2.0f, 7.0f,
			-14.0f, -2.0f, 10.0f,
			-17.0f, -2.0f, 13.0f
		};

		Mat3 result = matrixA.multiply(matrixB);

		for (int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_createTranslate_MajorRowOrder_Test)
	{
		Mat3 expected = {
			1.0f, 0.0f, 5.0f,
			0.0f, 1.0f, 2.0f,
			0.0f, 0.0f, -3.0f
		};

		Mat3 result = Mat3::createTranslate(5.0f, 2.0f, -3.0f);

		for (sp_uint i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_decomposeLU_MajorRowOrder_Test)
	{
		Mat3 matrix = {
			2.0f,  6.0f,  2.0f,
			-3.0f, -8.0f,  0.0f,
			4.0f,  9.0f,  2.0f
		};

		Mat3 lowerMatrixExpected = {
			2.0f,  0.0f,  0.0f,
			-3.0f,  1.0f,  0.0f,
			4.0f, -3.0f,  7.0f
		};

		Mat3 upperMatrixExpected = {
			1.0f, 3.0f, 1.0f,
			0.0f, 1.0f, 3.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3* decomposeLU = matrix.decomposeLU();
		Mat3 lowerMatrixResult = decomposeLU[0];
		Mat3 upperMatrixResult = decomposeLU[1];

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());

		Mat3 result = lowerMatrixResult * upperMatrixResult;
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_decomposeLDU_MajorRowOrder_Test)
	{
		Mat3 matrix = {
			2.0f,  6.0f,  2.0f,
			-3.0f, -8.0f,  0.0f,
			4.0f,  9.0f,  2.0f
		};

		Mat3 lowerMatrixExpected = {
			2.0f,  0.0f,  0.0f,
			-3.0f,  1.0f,  0.0f,
			4.0f, -3.0f,  7.0f
		};

		Mat3 diagonalMatrixExpected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3 upperMatrixExpected = {
			1.0f, 3.0f, 1.0f,
			0.0f, 1.0f, 3.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3* decomposeLDU = matrix.decomposeLDU();
		Mat3 lowerMatrixResult = decomposeLDU[0];
		Mat3 diagonalMatrixResult = decomposeLDU[1];
		Mat3 upperMatrixResult = decomposeLDU[2];

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(diagonalMatrixResult[i], diagonalMatrixExpected[i], L"Wrong number", LINE_INFO());

		Mat3 result = lowerMatrixResult * diagonalMatrixResult * upperMatrixResult;
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());
	}

#endif

	SP_TEST_METHOD(CLASS_NAME, Mat3_primaryDiagonal_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 primaryDiagonal;
		matrix.primaryDiagonal(primaryDiagonal);

		Assert::AreEqual(1.0f, primaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, primaryDiagonal[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, primaryDiagonal[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_secondaryDiagonal_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3 secondaryDiagonal = matrix.secondaryDiagonal();

		Assert::AreEqual(3.0f, secondaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, secondaryDiagonal[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, secondaryDiagonal[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, transpose)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};
		Mat3 result;
		matrix.transpose(result);

		sp_float expected[MAT3_LENGTH] = {
			1.0f, 4.0f, 7.0f,
			2.0f, 5.0f, 8.0f,
			3.0f, 6.0f, 9.0f
		};

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value", LINE_INFO());

		result.transpose();

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(matrix[i], result[i], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_createScaled_Test)
	{
		Mat3 expected = {
			2.0f, 0.0f, 0.0f,
			0.0f, 4.0f, 0.0f,
			0.0f, 0.0f, -3.0f
		};

		Mat3 result = Mat3::createScale(2.0f, 4.0f, -3.0f);

		for (sp_int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_scale_Test)
	{
		Mat3 matrixA = {
			2.0f, 0.0f, 0.0f,
			0.0f, 4.0f, 0.0f,
			1.0f, 10.0f, -3.0f
		};
		Mat3 expected = {
			4.0f, 0.0f, 0.0f,
			0.0f, 16.0f, 0.0f,
			1.0f, 10.0f, 9.0f
		};

		matrixA.scale(2.0f, 4.0f, -3.0f);

		for (sp_int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_createRotate_Test)
	{
		Mat3 expected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 0.866025388f, 0.5f,
			0.0f, -0.5f, 0.866025388f
		};

		sp_float angle = (sp_float) degreesToRadians(30);

		Mat3 result = Mat3::createRotate(angle, 1.0f, 0.0f, 0.0f);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(result[i], expected[i]), L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_determinant_Test)
	{
		Mat3 matrix = {
			2.0f, 5.0f, 6.0f,
			1.0f, 6.0f, 7.0f,
			-1.0f, 2.0f, 3.0f
		};
		sp_float expected = 6.0f;
		sp_float result = matrix.determinant();

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_sizeInBytes_Test)
	{
		sp_size result = Mat3Identity.sizeInBytes();
		sp_size expected = 36;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_clone_Test)
	{
		Mat3 matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		Mat3 result;
		matrix.clone(&result);

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(matrix[i], result[i], L"Wrong number", LINE_INFO());
	}
	
	SP_TEST_METHOD(CLASS_NAME, eigenValues)
	{
		Mat3 matrix = {
			3.0f, 1.0f, 0.0f,
			1.0f, 3.0f, 1.0f,
			0.0f, 1.0f, 3.0f
		};

		Vec3 result;
		sp_uint iterations;
		matrix.eigenValues(result, iterations, SP_EPSILON_THREE_DIGITS);

		Assert::IsTrue(isCloseEnough(3.0f, result.x, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
		Assert::IsTrue(isCloseEnough(1.585f, result.y, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
		Assert::IsTrue(isCloseEnough(4.414f, result.z, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

		matrix = {
			2.0f, 0.0f, 0.0f,
			0.0f, 3.0f, 4.0f,
			0.0f, 4.0f, 9.0f
		};

		matrix.eigenValues(result, iterations, SP_EPSILON_THREE_DIGITS);

		Assert::IsTrue(isCloseEnough(2.0f, result.x), L"Wrong number", LINE_INFO());
		Assert::IsTrue(isCloseEnough(11.0f, result.y), L"Wrong number", LINE_INFO());
		Assert::IsTrue(isCloseEnough(1.0f, result.z), L"Wrong number", LINE_INFO());

		matrix = {
			 2.0f, 0.0f, 0.0f,
			 1.0f, 2.0f, 1.0f,
			-1.0f, 0.0f, 1.0f
		};

		Mat3 symmetric;
		matrix.symmetric(symmetric);
		
		symmetric.eigenValues(result, iterations, SP_EPSILON_THREE_DIGITS);

		Asserts::isCloseEnough(0.535f, result.x, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(4.0f, result.y, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(7.464f, result.z, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, eigenValuesAndVectors)
	{
		Mat3 matrix = {
			 4.0f, 2.0f, 0.0f,
			 2.0f, 5.0f, 3.0f,
			 0.0f, 3.0f, 6.0f
		};

		/*
		Eigen::Map<Eigen::Matrix3Xf> m((sp_float*)matrix, 3, 3);
		Eigen::EigenSolver<Eigen::Matrix3Xf> s;
		s.compute(m, true);
		Eigen::Matrix3Xcf m2 = s.eigenvectors();
		Eigen::scomplex* o = ALLOC_ARRAY(Eigen::scomplex, 9);
		o = m2.data();
		*/

		Vec3 eigenValues;
		Mat3 eigenVectors;
		sp_uint iterations;
		matrix.eigenValuesAndVectors(eigenValues, eigenVectors, iterations, SP_UINT_MAX, SP_EPSILON_FOUR_DIGITS);

		Vec3 expectedValues(1.4516f, 8.9089f, 4.6395f);
		Mat3 expectedVectors = {
			-0.54801f, 0.69826f, -0.46056f,
			0.27285f, 0.6697f, 0.69069f,
			0.79072f, 0.25284f, -0.55753f
		};

		for (sp_uint i = 0; i < 3u; i++)
			Assert::IsTrue(isCloseEnough(expectedValues[i], eigenValues[i], SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

		for (sp_uint i = 0; i < 9u; i++)
			Assert::IsTrue(isCloseEnough(expectedVectors[i], eigenVectors[i], SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
	
		matrix = {
			1403.5f, 0.0f, 0.0f,
			0.0f, 1273.7974f, 0.1916f,
			0.0f, 0.1916f, 1403.5f
		};
		expectedValues = Vec3(1403.5f, 1273.7968f, 1403.5f);
		expectedVectors = Mat3(
			1.0f, 0.0f, 0.0f,
			0.0f, -1.0f, 0.0015f,
			0.0f, -0.0015f, -1.0f
		);

		matrix.eigenValuesAndVectors(eigenValues, eigenVectors, iterations);

		SystemOfLinearEquations system;
		sp_log_debug1snl(system.printMatrix(eigenVectors, 3, 3).c_str());

		for (sp_uint i = 0; i < 3u; i++)
			Assert::IsTrue(isCloseEnough(expectedValues[i], eigenValues[i], SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

		for (sp_uint i = 0; i < 9u; i++)
			Assert::IsTrue(isCloseEnough(expectedVectors[i], eigenVectors[i], SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

	}

	SP_TEST_METHOD(CLASS_NAME, eigenValueAndVectorMax)
	{
		Mat3 matrix = {
			528.2f, 547.6f, 156.4f,
			273.8f, 312.8f, 98.0f,
			78.2f, 98.0f, 39.0f
		};

		sp_float expectedValue = 849.1f;
		Vec3 expectedVector(1.0f, 0.54f, 0.1619f);
		
		Mat3 matrixT;
		matrix.transpose(matrixT);

		sp_float eigenValue;
		Vec3 eigenVector;
		matrixT.eigenValuesAndVectorsMax(eigenValue, eigenVector, 200);

		Asserts::isCloseEnough(expectedValue, eigenValue, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());

		for (sp_int i = 0; i < MAT3_ROW_LENGTH; i++)
			Asserts::isCloseEnough(expectedVector[i], eigenVector[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorMinus_Test)
	{
		Mat3 matrixA = {
			-2.0f, 1.0f, 3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3 expected = {
			2.0f, -1.0f, -3.0f,
			0.0f, -4.0f, +2.0f,
			-7.0f, -1.0f, 0.0f
		};

		Mat3 result = -matrixA;

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorMinus_matrix3_Test)
	{
		Mat3 matrixA = {
			-2.0f, 1.0f, -3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3 matrixB = {
			-2.0f, 10.0f, 5.0f,
			0.0f, 4.0f, -2.0f,
			-7.0f, 1.0f, 0.0f
		};
		Mat3 expected = {
			0.0f, -9.0f, -8.0f,
			0.0f, 0.0f, 0.0f,
			14.0f, 0.0f, 0.0f
		};

		Mat3 result = matrixA - matrixB;

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorSum_matrix3_Test)
	{
		Mat3 matrixA = {
			-2.0f, 1.0f, -3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3 matrixB = {
			-2.0f, 10.0f, 5.0f,
			0.0f, 4.0f, -2.0f,
			-7.0f, 1.0f, 0.0f
		};
		Mat3 expected = {
			-4.0f, 11.0f, 2.0f,
			0.0f, 8.0f, -4.0f,
			0.0f, 2.0f, 0.0f
		};

		Mat3 result = matrixA + matrixB;

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorDivide_Test)
	{
		Mat3 matrixA = {
			2.0f, 1.0f, 3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3 expected = {
			1.0f, 0.5f, 1.5f,
			0.0f, 2.0f, -1.0f,
			3.5f, 0.5f, 0.0f
		};

		Mat3 result = matrixA / 2.0f;

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorDivideEqual_Test)
	{
		Mat3 matrixA = {
			2.0f, 1.0f, 3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3 expected = {
			1.0f, 0.5f, 1.5f,
			0.0f, 2.0f, -1.0f,
			3.5f, 0.5f, 0.0f
		};

		matrixA /= 2.0f;

		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorEqual_Test)
	{
		Mat3 matrixA = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};
		Mat3 matrixB = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		sp_bool result = matrixA == matrixB;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixB[6] = 6.5f;
		result = matrixA == matrixB;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorNotEqual_Test)
	{
		Mat3 matrixA = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};
		Mat3 matrixB = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		sp_bool result = matrixA != matrixB;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());

		matrixB[6] = 6.5f;
		result = matrixA != matrixB;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorEqual_Value_Test)
	{
		Mat3 matrixA = {
			3.0f, 3.0f, 3.0f,
			3.0f, 3.0f, 3.0f,
			3.0f, 3.0f, 3.0f
		};

		sp_bool result = matrixA == 3.0f;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixA[6] = 6.5f;
		result = matrixA == 3.0f;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isIdentity)
	{
		Mat3 matrixA = Mat3Identity;

		sp_bool result = matrixA.isIdentity();
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixA[6] = 1.0f;
		result = matrixA.isIdentity();
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, convert)
	{
		Mat3 input = { 
			0.5771f, 0.4228f, -0.6986f,
			0.4228f, 0.5771f, 0.6986f,
			0.6986f, -0.6986f, 0.1542f
		};
		Quat result;

		input.convert(result);

		Quat expected = Quat(0.7596f, -0.4598f, -0.45982f, -0.0f);

		Asserts::isCloseEnough(expected.w, result.w, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(expected.x, result.x, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(expected.y, result.y, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(expected.z, result.z, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());


		input = Mat3Identity;
		input.convert(result);

		expected = Quat(1.0f, 0.0f, 0.0f, 0.0f);

		Asserts::isCloseEnough(expected.w, result.w, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(expected.x, result.x, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(expected.y, result.y, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
		Asserts::isCloseEnough(expected.z, result.z, SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

}

#undef CLASS_NAME 