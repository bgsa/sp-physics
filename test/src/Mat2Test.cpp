#include "SpectrumPhysicsTest.h"
#include "Mat2.h"

#define CLASS_NAME Mat2Test

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(Mat2Test)
	{
	public:

		SP_TEST_METHOD_DEF(Mat2_constructorEmpty_Test);

		SP_TEST_METHOD_DEF(Mat2_constructorValues_Test);

		SP_TEST_METHOD_DEF(Mat2_getValues_Test);

		SP_TEST_METHOD_DEF(Mat2_getValue_Test);

		SP_TEST_METHOD_DEF(Mat2_xAxis_Test);

		SP_TEST_METHOD_DEF(Mat2_yAxis_Test);

		SP_TEST_METHOD_DEF(Mat2_multiply_Test);

		SP_TEST_METHOD_DEF(Mat2_createTranslate_Test);

		SP_TEST_METHOD_DEF(Mat2_primaryDiagonal_Test);

		SP_TEST_METHOD_DEF(Mat2_secondaryDiagonal_Test);

		SP_TEST_METHOD_DEF(Mat2_identity_Test);

		SP_TEST_METHOD_DEF(Mat2_transpose_Test);
		
		SP_TEST_METHOD_DEF(Mat2_createScaled_Test);

		SP_TEST_METHOD_DEF(Mat2_scale_Test);

		SP_TEST_METHOD_DEF(Mat2_determinant_Test);

		SP_TEST_METHOD_DEF(Mat2_sizeInBytes_Test);

		SP_TEST_METHOD_DEF(Mat2_clone_Test);

		SP_TEST_METHOD_DEF(Mat2_getAutovalueAndAutovector_Test);

		SP_TEST_METHOD_DEF(Mat2_operatorMinus_Test);

		SP_TEST_METHOD_DEF(Mat2_operatorMinus_matrix2_Test);

		SP_TEST_METHOD_DEF(Mat2_operatorSum_matrix2_Test);

		SP_TEST_METHOD_DEF(Mat2_operatorDivide_scalar_Test);

		SP_TEST_METHOD_DEF(Mat2_divide_equal_operator_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, Mat2_constructorEmpty_Test)
	{
		Mat2 result;

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(0.0f, result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_constructorValues_Test)
	{
		float emptyMatrix[MAT2_LENGTH] = {
			1.0f, 2.0f, 
			4.0f, 5.0f
		};

		Mat2 result = Mat2(emptyMatrix);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(emptyMatrix[i], result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_getValues_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		float* result = matrix.getValues();

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(matrix[i], result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_getValue_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Assert::AreEqual(1.0f, matrix.getValue(1, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, matrix.getValue(1, 2), L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, matrix.getValue(2, 1), L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, matrix.getValue(2, 2), L"Wrong value", LINE_INFO());
	}

#if MAJOR_COLUMN_ORDER

	SP_TEST_METHOD(CLASS_NAME, Mat2_xAxis_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Vec2 xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, xAxis[1], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_yAxis_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Vec2 yAxis = matrix.yAxis();

		Assert::AreEqual(2.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, yAxis[1], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_multiply_Test)
	{
		Mat2 matrixA = {
			2.0f, 3.0f,
			3.0f, 4.0f
		};
		Mat2 matrixB = {
			0.0f, 1.0f,
			-1.0f, 0.0f
		};
		Mat2 expected = {
			3.0f, 4.0f,
			-2.0f, -3.0f
		};

		Mat2 result = matrixA.multiply(matrixB);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_createTranslate_Test)
	{
		Mat2 expected = {
			1.0f, 0.0f,
			5.0f, 2.0f
		};

		Mat2 result = Mat2::createTranslate(5.0f, 2.0f);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

#endif

#if MAJOR_ROW_ORDER

	SP_TEST_METHOD(CLASS_NAME, Mat2_xAxis_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Vec2 xAxis = matrix.xAxis();

		Assert::AreEqual(1.0f, xAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(2.0f, xAxis[1], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_yAxis_RowMajorRow_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Vec2 yAxis = matrix.yAxis();

		Assert::AreEqual(4.0f, yAxis[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, yAxis[1], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_multiply_MajorRowOrder_Test)
	{
		Mat2 matrixA = {
			2.0f, 3.0f,
			3.0f, 4.0f
		};
		Mat2 matrixB = {
			0.0f, 1.0f,
			-1.0f, 0.0f
		};
		Mat2 expected = {
			-3.0f, 2.0f,
			-4.0f, 3.0f
		};

		Mat2 result = matrixA.multiply(matrixB);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_createTranslate_MajorRowOrder_Test)
	{
		Mat2 expected = {
			1.0f, 5.0f,
			0.0f, 2.0f
		};

		Mat2 result = Mat2::createTranslate(5.0f, 2.0f);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

#endif

	SP_TEST_METHOD(CLASS_NAME, Mat2_primaryDiagonal_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Vec2 primaryDiagonal = matrix.primaryDiagonal();

		Assert::AreEqual(1.0f, primaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(5.0f, primaryDiagonal[1], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_secondaryDiagonal_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Vec2 secondaryDiagonal = matrix.secondaryDiagonal();

		Assert::AreEqual(2.0f, secondaryDiagonal[0], L"Wrong value", LINE_INFO());
		Assert::AreEqual(4.0f, secondaryDiagonal[1], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_identity_Test)
	{
		Mat2 result = Mat2::identity();

		for (int i = 0; i < MAT2_LENGTH; i++)
			if (i % 3 == 0)
				Assert::AreEqual(1.0f, result[i], L"Value shoud be 0", LINE_INFO());
			else
				Assert::AreEqual(0.0f, result[i], L"Value shoud be 0", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_transpose_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};
		Mat2 result = matrix.transpose();

		float expected[MAT2_LENGTH] = {
			1.0f, 4.0f,
			2.0f, 5.0f
		};

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_createScaled_Test)
	{
		Mat2 expected = {
			3.0f, 0.0f,
			0.0f, 4.0f
		};

		Mat2 result = Mat2::createScale(3.0f, 4.0f);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_scale_Test)
	{
		Mat2 matrixA = {
			2.0f, 0.0f,
			0.0f, 4.0f
		};
		Mat2 expected = {
			4.0f, 0.0f,
			0.0f, 16.0f,
		};

		matrixA.scale(2.0f, 4.0f);

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_determinant_Test)
	{
		Mat2 matrix = {
			2.0f, 9.0f,
			-1.0f, 6.0f
		};
		float expected = 21.0f;

		float result = matrix.determinant();

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_sizeInBytes_Test)
	{
		size_t result = Mat2::identity().sizeInBytes();
		size_t expected = 16;

		Assert::AreEqual(expected, result, L"Wrong value", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_clone_Test)
	{
		Mat2 matrix = {
			1.0f, 2.0f,
			4.0f, 5.0f
		};

		Mat2 result = matrix.clone();

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(matrix[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_getAutovalueAndAutovector_Test)
	{
		Mat2 matrix = {
			3.0f, 2.0f,
			2.0f, 3.0f
		};

		float expectedValue = 5.0f;
		Vec2 expectedVector = Vec2{ 1.0f, 1.0f }; // VERIFICAR no livro algebra linear !
		AutoValueAutoVector2 result = matrix.getAutovalueAndAutovector();

		Assert::AreEqual(ceilf(expectedValue), ceilf(result.autovalue), L"Wrong number", LINE_INFO());

		for (int i = 0; i < MAT2_ROW_LENGTH; i++)
			Assert::AreEqual(ceil(expectedVector[i]), ceil(result.autoVector[i]), L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_operatorMinus_Test)
	{
		Mat2 matrixA = {
			2.0f, -1.0f,
			0.0f, 4.0f
		};
		Mat2 expected = {
			-2.0f, 1.0f,
			0.0f, -4.0f,
		};

		Mat2 result = -matrixA;

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_operatorMinus_matrix2_Test)
	{
		Mat2 matrixA = {
			2.0f, -1.0f,
			0.0f, 4.0f
		};
		Mat2 matrixB = {
			4.0f, -10.0f,
			2.0f, -5.0f
		};
		Mat2 expected = {
			-2.0f, 9.0f,
			-2.0f, 9.0f,
		};
		
		Mat2 result = matrixA - matrixB;

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_operatorSum_matrix2_Test)
	{
		Mat2 matrixA = {
			2.0f, -1.0f,
			0.0f, 4.0f
		};
		Mat2 matrixB = {
			4.0f, -10.0f,
			2.0f, -5.0f
		};
		Mat2 expected = {
			6.0f, -11.0f,
			2.0f, -1.0f,
		};

		Mat2 result = matrixA + matrixB;

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_operatorDivide_scalar_Test)
	{
		Mat2 matrixA = {
			2.0f, 1.0f,
			0.0f, 4.0f
		};
		Mat2 expected = {
			1.0f, 0.5f,
			0.0f, 2.0f,
		};

		Mat2 result = matrixA / 2.0f;

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, Mat2_divide_equal_operator_Test)
	{
		Mat2 matrixA = {
			2.0f, 1.0f,
			0.0f, 4.0f
		};
		Mat2 expected = {
			1.0f, 0.5f,
			0.0f, 2.0f,
		};

		matrixA /= 2.0f;

		for (int i = 0; i < MAT2_LENGTH; i++)
			Assert::AreEqual(expected[i], matrixA[i], L"Wrong number", LINE_INFO());
	}

}

#undef CLASS_NAME