#include "SpectrumPhysicsTest.h"
#include "Mat.h"
#include "Asserts.h"

#define CLASS_NAME MatTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(MatTest)
	{
	public:
		SP_TEST_METHOD_DEF(_transpose);
		SP_TEST_METHOD_DEF(_multiply);
		SP_TEST_METHOD_DEF(householder);
		SP_TEST_METHOD_DEF(hessenberg);
	};

	SP_TEST_METHOD(CLASS_NAME, hessenberg)
	{
		/*
		sp_float matrix[9] = {
			 2.0f, -1.0f, 3.0f,
			 2.0f,  0.0f, 1.0f,
			-2.0f,  1.0f, 4.0f
		};
		*/
		sp_float matrix[16] = {
			 1.0f, 2.0f, 3.0f, 4.0f,
			 5.0f, 6.0f, 7.0f, 8.0f,
			 9.0f, 10.0f, 11.0f, 12.0f,
			 13.0f, 14.0f, 15.0f, 16.0f
		};

		sp_float expected[16] = {
			  4.0f,    -3.0f,     0.0f,    0.0f,
			 -3.0f,  3.3333f, -1.6666f,    0.0f,
			  0.0f, -1.6666f,   -1.32f, 0.9066f,
			  0.0f,     0.0f,  0.9066f, 1.9866f
		};

		sp_float result[16];
		Mat::hessenberg((sp_float*)matrix, 4u, result);

		for (sp_int i = 0; i < 3u; i++)
			Asserts::isCloseEnough(expected[i], result[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, householder)
	{
		sp_float matrix[16] = {
			 4.0f, 1.0f, -2.0f,  2.0f,
			 1.0f, 2.0f,  0.0f,  1.0f,
			-2.0f, 0.0f,  3.0f, -2.0f,
			 2.0f, 1.0f, -2.0f, -1.0f
		};
		sp_float expected[16] = {
			  4.0f,    -3.0f,     0.0f,    0.0f,
			 -3.0f,  3.3333f, -1.6666f,    0.0f,
			  0.0f, -1.6666f,   -1.32f, 0.9066f,
			  0.0f,     0.0f,  0.9066f, 1.9866f
		};

		sp_float result[16];
		Mat::householder((sp_float*)matrix, 4u, result);

		for (sp_int i = 0; i < 4u; i++)
			Asserts::isCloseEnough(expected[i], result[i], SP_EPSILON_THREE_DIGITS, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, _transpose)
	{
		sp_float matrix[16] = {
			1.0f, 5.0f, 9.0f, 13.0f,
			2.0f, 6.0f, 10.0f, 14.0f,
			3.0f, 7.0f, 11.0f, 15.0f,
			4.0f, 8.0f, 12.0f, 16.0f
		};
		sp_float expected[16] = {
			1.0f, 2.0f, 3.0f, 4.0f,
			5.0f, 6.0f, 7.0f, 8.0f,
			9.0f, 10.0f, 11.0f, 12.0f,
			13.0f, 14.0f, 15.0f, 16.0f
		};

		Mat m(4,4, matrix);
		Mat e(4,4, expected);

		Mat result(4, 4);
		transpose(m, result);
		
		Assert::IsTrue(isCloseEnough(result, e, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, _multiply)
	{
		sp_float a[10] = {
			8.0f, 5.0f, 1.0f, 8.0f, 8.0f,
			9.0f, 1.0f, 2.0f, 2.0f, 2.0f
		};

		sp_float b[10] = {
			9.0f, 4.0f, 
			3.0f, 3.0f, 
			1.0f, 8.0f, 
			2.0f, 5.0f, 
			6.0f, 5.0f
		};
		sp_float e[4] = {
			152.0f, 135.0f,
			102.0f, 75.0f
		};

		Mat A(2, 5, a);
		Mat B(5, 2, b);
		Mat expected(2, 2, e);

		Mat result(2, 2);
		multiply(A, B, result);

		Assert::IsTrue(isCloseEnough(result, expected, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
	}
}

#undef CLASS_NAME