#include "TestHeader.h"
#include <Mat3.h>

#define CLASS_NAME Mat3Test

namespace SP_PHYSICS_TEST_NAMESPACE
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

	};


	SP_TEST_METHOD(CLASS_NAME, Mat3_constructorEmpty_Test)
	{
		Mat3<float> result;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(0.0f, result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_constructorValues_Test)
	{
		float emptyMatrix[MAT3_SIZE] = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Mat3<float> result = Mat3<float>(emptyMatrix);

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(emptyMatrix[i], result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_getValues_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		float* result = matrix.getValues();

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(matrix[i], result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_getValue_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Assert::AreEqual(1.0f, matrix.getValue(1, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, matrix.getValue(1, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, matrix.getValue(1, 3), L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, matrix.getValue(2, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, matrix.getValue(2, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, matrix.getValue(2, 3), L"Wrong value", LINE_INFO());
		Assert::AreEqual(3.0f, matrix.getValue(3, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, matrix.getValue(3, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, matrix.getValue(3, 3), L"Wrong value", LINE_INFO());
	}

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_getAxisX_ColumnMajorOrder_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f axis = matrix.getAxis(0);

		Assert::AreEqual(1.0f, axis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, axis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, axis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_getAxisY_ColumnMajorOrder_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f axis = matrix.getAxis(1);

		Assert::AreEqual(2.0f, axis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, axis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, axis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_getAxisZ_ColumnMajorOrder_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f axis = matrix.getAxis(2);

		Assert::AreEqual(3.0f, axis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, axis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, axis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_xAxis_ColumnMajorOrder_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, xAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, xAxis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_xAxis_RowMajorOrder_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, xAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(3.0f, xAxis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_yAxis_ColumnMajorRow_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f yAxis = matrix.yAxis();

		Assert::AreEqual(2.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, yAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, yAxis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_yAxis_RowMajorRow_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f yAxis = matrix.yAxis();

		Assert::AreEqual(4.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, yAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, yAxis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_zAxis_ColumnMajorRow_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f zAxis = matrix.zAxis();

		Assert::AreEqual(3.0f, zAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(6.0f, zAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, zAxis[2], L"Wrong value", LINE_INFO());
	}
#endif

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_zAxis_RowMajorRow_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f zAxis = matrix.zAxis();

		Assert::AreEqual(7.0f, zAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(8.0f, zAxis[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, zAxis[2], L"Wrong value", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat3_primaryDiagonal_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f primaryDiagonal = matrix.primaryDiagonal();

		Assert::AreEqual(1.0f, primaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, primaryDiagonal[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(9.0f, primaryDiagonal[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_secondaryDiagonal_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};

		Vec3f secondaryDiagonal = matrix.secondaryDiagonal();

		Assert::AreEqual(3.0f, secondaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, secondaryDiagonal[1], L"Wrong value", LINE_INFO());
		Assert::AreEqual(7.0f, secondaryDiagonal[2], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_identity_Test)
	{
		Mat3<float> result = Mat3<float>::identity();
		
		for (int i = 0; i < MAT3_SIZE; i++)
			if (i % 4 == 0)
				Assert::AreEqual(1.0f, result[i], L"Value shoud be 0", LINE_INFO()); 
			else
				Assert::AreEqual(0.0f, result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_transpose_Test)
	{
		Mat3<float> matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, 9.0f
		};
		Mat3<float> result = matrix.transpose();

		float expected[MAT3_SIZE] = {
			1.0f, 4.0f, 7.0f,
			2.0f, 5.0f, 8.0f,
			3.0f, 6.0f, 9.0f
		};

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value", LINE_INFO());
	}

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_multiply_MajorColumnOrder_Test)
	{
		Mat3f matrixA = {
			2.0f, 3.0f, 4.0f,
			3.0f, 4.0f, 5.0f,
			4.0f, 5.0f, 6.0f
		};
		Mat3f matrixB = {
			0.0f, 1.0f, 2.0f,
			-1.0f, 0.0f, 1.0f,
			-2.0f, -1.0f, 0.0f
		};
		Mat3f expected = {
			11.0f, 14.0f, 17.0f,
			2.0f, 2.0f, 2.0f,
			-7.0f, -10.0f, -13.0f
		};

		Mat3f result = matrixA.multiply(matrixB);
		
		for (int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_multiply_MajorRowOrder_Test)
	{
		Mat3f matrixA = {
			2.0f, 3.0f, 4.0f,
			3.0f, 4.0f, 5.0f,
			4.0f, 5.0f, 6.0f
		};
		Mat3f matrixB = {
			0.0f, 1.0f, 2.0f,
			-1.0f, 0.0f, 1.0f,
			-2.0f, -1.0f, 0.0f
		};
		Mat3f expected = {
			-11.0f, -2.0f, 7.0f,
			-14.0f, -2.0f, 10.0f,
			-17.0f, -2.0f, 13.0f
		};

		Mat3f result = matrixA.multiply(matrixB);

		for (int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat3_createScaled_Test)
	{
		Mat3f expected = {
			2.0f, 0.0f, 0.0f,
			0.0f, 4.0f, 0.0f,
			0.0f, 0.0f, -3.0f
		};

		Mat3f result = Mat3f::createScale(2.0f, 4.0f, -3.0f);

		for (int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_scale_Test)
	{
		Mat3f matrixA = {
			2.0f, 0.0f, 0.0f,
			0.0f, 4.0f, 0.0f,
			1.0f, 10.0f, -3.0f
		};
		Mat3f expected = {
			4.0f, 0.0f, 0.0f,
			0.0f, 16.0f, 0.0f,
			1.0f, 10.0f, 9.0f
		};

		matrixA.scale(2.0f, 4.0f, -3.0f);

		for (int i = 0; i < 9; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_createRotate_Test)
	{
		Mat3f expected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 0.866025388f, 0.5f,
			0.0f, -0.5f, 0.866025388f
		};

		float angle = (float) degreesToRadians(30);

		Mat3f result = Mat3f::createRotate(angle, 1.0f, 0.0f, 0.0f);

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_createTranslate_MajorColumnOrder_Test)
	{
		Mat3f expected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			5.0f, 2.0f, -3.0f
		};

		Mat3f result = Mat3f::createTranslate(5.0f, 2.0f, -3.0f);

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_createTranslate_MajorRowOrder_Test)
	{
		Mat3f expected = {
			1.0f, 0.0f, 5.0f,
			0.0f, 1.0f, 2.0f,
			0.0f, 0.0f, -3.0f
		};

		Mat3f result = Mat3f::createTranslate(5.0f, 2.0f, -3.0f);

		for (size_t i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat3_determinant_Test)
	{
		Mat3f matrix = {
			2.0f, 5.0f, 6.0f,
			1.0f, 6.0f, 7.0f,
			-1.0f, 2.0f, 3.0f
		};
		float expected = 6.0f;
		float result = matrix.determinant();

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_sizeInBytes_Test)
	{
		size_t result = Mat3<float>::identity().sizeInBytes();
		size_t expected = 36;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_clone_Test)
	{
		Mat3f matrix = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		Mat3f result = matrix.clone();

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(matrix[i], result[i], L"Wrong number", LINE_INFO());
	}

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_decomposeLU_MajorRowOrder_Test)
	{
		Mat3f matrix = {
			2.0f,  6.0f,  2.0f,
			-3.0f, -8.0f,  0.0f,
			4.0f,  9.0f,  2.0f
		};

		Mat3f lowerMatrixExpected = {
			2.0f,  0.0f,  0.0f,
			-3.0f,  1.0f,  0.0f,
			4.0f, -3.0f,  7.0f
		};

		Mat3f upperMatrixExpected = {
			1.0f, 3.0f, 1.0f,
			0.0f, 1.0f, 3.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3f* decomposeLU = matrix.decomposeLU();
		Mat3f lowerMatrixResult = decomposeLU[0];
		Mat3f upperMatrixResult = decomposeLU[1];

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());

		Mat3f result = lowerMatrixResult * upperMatrixResult;
		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_decomposeLU_MajorColumnOrder_Test)
	{
		Mat3f matrix = {
			2.0f, -3.0f,  4.0f,
			6.0f, -8.0f,  9.0f,
			2.0f,  0.0f,  2.0f
		};

		Mat3f lowerMatrixExpected = {
			2.0f,  -3.0f,  4.0f,
			0.0f,  1.0f,  -3.0f,
			0.0f, 0.0f,  7.0f
		};

		Mat3f upperMatrixExpected = {
			1.0f, 0.0f, 0.0f,
			3.0f, 1.0f, 0.0f,
			1.0f, 3.0f, 1.0f
		};

		Mat3f* decomposeLU = matrix.decomposeLU();
		Mat3f lowerMatrixResult = decomposeLU[0];
		Mat3f upperMatrixResult = decomposeLU[1];
		Mat3f result = lowerMatrixResult * upperMatrixResult;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());
	}
#endif

#if MAJOR_ROW_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_decomposeLDU_MajorRowOrder_Test)
	{
		Mat3f matrix = {
			2.0f,  6.0f,  2.0f,
			-3.0f, -8.0f,  0.0f,
			4.0f,  9.0f,  2.0f
		};

		Mat3f lowerMatrixExpected = {
			2.0f,  0.0f,  0.0f,
			-3.0f,  1.0f,  0.0f,
			4.0f, -3.0f,  7.0f
		};

		Mat3f diagonalMatrixExpected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3f upperMatrixExpected = {
			1.0f, 3.0f, 1.0f,
			0.0f, 1.0f, 3.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3f* decomposeLDU = matrix.decomposeLDU();
		Mat3f lowerMatrixResult = decomposeLDU[0];
		Mat3f diagonalMatrixResult = decomposeLDU[1];
		Mat3f upperMatrixResult = decomposeLDU[2];

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(diagonalMatrixResult[i], diagonalMatrixExpected[i], L"Wrong number", LINE_INFO());

		Mat3f result = lowerMatrixResult * diagonalMatrixResult * upperMatrixResult;
		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());
	}
#endif

#if MAJOR_COLUMN_ORDER
	SP_TEST_METHOD(CLASS_NAME, Mat3_decomposeLDU_MajorColumnOrder_Test)
	{
		Mat3f matrix = {
			2.0f, -3.0f,  4.0f,
			6.0f, -8.0f,  9.0f,
			2.0f,  0.0f,  2.0f
		};

		Mat3f lowerMatrixExpected = {
			2.0f,  -3.0f,  4.0f,
			0.0f,  1.0f,  -3.0f,
			0.0f, 0.0f,  7.0f
		};

		Mat3f diagonalMatrixExpected = {
			1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f
		};

		Mat3f upperMatrixExpected = {
			1.0f, 0.0f, 0.0f,
			3.0f, 1.0f, 0.0f,
			1.0f, 3.0f, 1.0f
		};

		Mat3f* decomposeLDU = matrix.decomposeLDU();
		Mat3f lowerMatrixResult = decomposeLDU[0];
		Mat3f diagonalMatrixResult = decomposeLDU[1];
		Mat3f upperMatrixResult = decomposeLDU[2];
		Mat3f result = lowerMatrixResult * diagonalMatrixResult * upperMatrixResult;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(result[i], matrix[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(lowerMatrixResult[i], lowerMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(upperMatrixResult[i], upperMatrixExpected[i], L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(diagonalMatrixResult[i], diagonalMatrixExpected[i], L"Wrong number", LINE_INFO());
	}
#endif

	SP_TEST_METHOD(CLASS_NAME, Mat3_getAutovalueAndAutovector_Test)
	{
		Mat3f matrix = {
			528.2f, 547.6f, 156.4f,
			273.8f, 312.8f, 98.0f,
			78.2f, 98.0f, 39.0f
		};

		float expectedValue = 849.1f;
		Vec3f expectedVector = Vec3f{ 1.0f, 0.54f , 0.1619f };
		AutovalueAutovector3<float> result = matrix.getAutovalueAndAutovector();

		Assert::AreEqual(ceil(expectedValue), ceil(result.autovalue), L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT3_ROWSIZE; i++)
			Assert::AreEqual(ceil(expectedVector[i]), ceil(result.autovector[i]), L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorMinus_Test)
	{
		Mat3f matrixA = {
			-2.0f, 1.0f, 3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3f expected = {
			2.0f, -1.0f, -3.0f,
			0.0f, -4.0f, +2.0f,
			-7.0f, -1.0f, 0.0f
		};

		Mat3f result = -matrixA;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorMinus_matrix3_Test)
	{
		Mat3f matrixA = {
			-2.0f, 1.0f, -3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3f matrixB = {
			-2.0f, 10.0f, 5.0f,
			0.0f, 4.0f, -2.0f,
			-7.0f, 1.0f, 0.0f
		};
		Mat3f expected = {
			0.0f, -9.0f, -8.0f,
			0.0f, 0.0f, 0.0f,
			14.0f, 0.0f, 0.0f
		};

		Mat3f result = matrixA - matrixB;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorSum_matrix3_Test)
	{
		Mat3f matrixA = {
			-2.0f, 1.0f, -3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3f matrixB = {
			-2.0f, 10.0f, 5.0f,
			0.0f, 4.0f, -2.0f,
			-7.0f, 1.0f, 0.0f
		};
		Mat3f expected = {
			-4.0f, 11.0f, 2.0f,
			0.0f, 8.0f, -4.0f,
			0.0f, 2.0f, 0.0f
		};

		Mat3f result = matrixA + matrixB;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorDivide_Test)
	{
		Mat3f matrixA = {
			2.0f, 1.0f, 3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3f expected = {
			1.0f, 0.5f, 1.5f,
			0.0f, 2.0f, -1.0f,
			3.5f, 0.5f, 0.0f
		};

		Mat3f result = matrixA / 2.0f;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorDivideEqual_Test)
	{
		Mat3f matrixA = {
			2.0f, 1.0f, 3.0f,
			0.0f, 4.0f, -2.0f,
			7.0f, 1.0f, 0.0f
		};
		Mat3f expected = {
			1.0f, 0.5f, 1.5f,
			0.0f, 2.0f, -1.0f,
			3.5f, 0.5f, 0.0f
		};

		matrixA /= 2.0f;

		for (int i = 0; i < MAT3_SIZE; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorEqual_Test)
	{
		Mat3f matrixA = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};
		Mat3f matrixB = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		bool result = matrixA == matrixB;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixB[6] = 6.5f;
		result = matrixA == matrixB;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorNotEqual_Test)
	{
		Mat3f matrixA = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};
		Mat3f matrixB = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			7.0f, 8.0f, -9.0f
		};

		bool result = matrixA != matrixB;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());

		matrixB[6] = 6.5f;
		result = matrixA != matrixB;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_operatorEqual_Value_Test)
	{
		Mat3f matrixA = {
			3.0f, 3.0f, 3.0f,
			3.0f, 3.0f, 3.0f,
			3.0f, 3.0f, 3.0f
		};

		bool result = matrixA == 3.0f;
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixA[6] = 6.5f;
		result = matrixA == 3.0f;
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat3_isIdentity_Test)
	{
		Mat3f matrixA = Mat3f::identity();

		bool result = matrixA.isIdentity();
		Assert::IsTrue(result, L"Wrong number", LINE_INFO());

		matrixA[6] = 1.0f;
		result = matrixA.isIdentity();
		Assert::IsFalse(result, L"Wrong number", LINE_INFO());
	}

}

#undef CLASS_NAME 