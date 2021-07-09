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
		SP_TEST_METHOD_DEF(_multiply2);
		SP_TEST_METHOD_DEF(isSymmetric);
		SP_TEST_METHOD_DEF(maxOffDiagonal);
		SP_TEST_METHOD_DEF(maxOffDiagonalSymmetric);
		SP_TEST_METHOD_DEF(primaryDiagonal);
		SP_TEST_METHOD_DEF(svd);
		SP_TEST_METHOD_DEF(householder);
		SP_TEST_METHOD_DEF(gramSchmidt);
	};

	SP_TEST_METHOD(CLASS_NAME, primaryDiagonal)
	{
		sp_float a1[10] = {
			8.0f, 5.0f, 1.0f, 8.0f, 8.0f,
			9.0f, 1.0f, 2.0f, 2.0f, 2.0f
		};
		Mat A(2, 5, a1, SpStackMemoryAllocator::main());

		sp_float diagonal[2];
		A.primaryDiagonal(diagonal);

		Assert::AreEqual(diagonal[0], 8.0f, L"Wrong number", LINE_INFO());
		Assert::AreEqual(diagonal[1], 1.0f, L"Wrong number", LINE_INFO());

		sp_float a2[10] = {
			3.0f, 5.0f, 
			1.0f, 8.0f, 
			8.0f, 4.0f, 
			2.0f, 3.0f, 
			2.0f, 1.0f
		};
		A = Mat(5, 2, a2, SpStackMemoryAllocator::main());

		A.primaryDiagonal(diagonal);

		Assert::AreEqual(diagonal[0], 3.0f, L"Wrong number", LINE_INFO());
		Assert::AreEqual(diagonal[1], 8.0f, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, maxOffDiagonal)
	{
		sp_float a[9] = {
			8.0f, 2.0f, 3.0f,
			1.0f, 8.0f, 5.0f,
			3.0f, 4.0f, 9.0f
		};
		Mat A(3, 3, a, SpStackMemoryAllocator::main());

		sp_uint row, column;
		sp_float value;
		A.maxOffDiagonal(row, column, value);

		Assert::AreEqual(value, 5.0f, L"Wrong number", LINE_INFO());
		Assert::AreEqual(row, 1u, L"Wrong number", LINE_INFO());
		Assert::AreEqual(column, 2u, L"Wrong number", LINE_INFO());

		A.set(0u, 1u, 7.0f);
		A.maxOffDiagonal(row, column, value);

		Assert::AreEqual(value, 7.0f, L"Wrong number", LINE_INFO());
		Assert::AreEqual(row, 0u, L"Wrong number", LINE_INFO());
		Assert::AreEqual(column, 1u, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, maxOffDiagonalSymmetric)
	{
		sp_float a[9] = {
			8.0f, 2.0f, 3.0f,
			1.0f, 8.0f, 5.0f,
			3.0f, 4.0f, 9.0f
		};
		Mat A(3, 3, a, SpStackMemoryAllocator::main());

		sp_uint row, column;
		sp_float value;
		A.maxOffDiagonal(row, column, value);

		Assert::AreEqual(value, 5.0f, L"Wrong number", LINE_INFO());
		Assert::AreEqual(row, 1u, L"Wrong number", LINE_INFO());
		Assert::AreEqual(column, 2u, L"Wrong number", LINE_INFO());

		A.set(2u, 1u, 6.0f);
		A.maxOffDiagonal(row, column, value);

		Assert::AreEqual(value, 6.0f, L"Wrong number", LINE_INFO());
		Assert::AreEqual(row, 2u, L"Wrong number", LINE_INFO());
		Assert::AreEqual(column, 1u, L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, isSymmetric)
	{
		sp_float a[10] = {
			8.0f, 2.0f, 3.0f, 
			1.0f, 8.0f, 4.0f, 
			3.0f, 4.0f, 9.0f, 10.0f
		};
		Mat A(2, 5, a, SpStackMemoryAllocator::main());

		Assert::IsFalse(A.isSymmetric(), L"Wrong number", LINE_INFO());

		A = Mat(3, 3, a, SpStackMemoryAllocator::main());

		Assert::IsFalse(A.isSymmetric(), L"Wrong number", LINE_INFO());

		A.set(0, 1, 1.0f);

		Assert::IsTrue(A.isSymmetric(), L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, svd)
	{
		sp_float a[15] = {
			1.0f, 0.0f, 1.0f, 
			0.0f, 1.0f, 0.0f, 
			0.0f, 1.0f, 1.0f, 
			0.0f, 1.0f, 0.0f, 
			1.0f, 1.0f, 0.0f
		};
		Mat A(5, 3, a, SpStackMemoryAllocator::main());

		Mat u(5, 5, SpStackMemoryAllocator::main()), 
			s(5, 3, SpStackMemoryAllocator::main()), 
			v(3, 3, SpStackMemoryAllocator::main());

		sp_uint iterations;
		A.svd(u, s, v, iterations, SP_UINT_MAX, SP_EPSILON_TWO_DIGITS);

		sp_float uE[25] = {
		   -0.3651f,  0.8165f,  0.0000f,  0.1184f, -0.4313f,
		   -0.3651f, -0.4082f,  0.0000f, -0.5635f, -0.6185f,
		   -0.5477f, -0.0000f,  0.7071f, -0.1184f,  0.4313f,
		   -0.3651f, -0.4082f,  0.0000f,  0.8003f, -0.2441f,
		   -0.5477f,  0.0000f, -0.7071f, -0.1184f,  0.4313f
		};
		Mat uExpected(5, 5, uE, SpStackMemoryAllocator::main());

		sp_float sE[25] = {
		   2.23610f, 0.0f,    0.0f, 0.0f, 0.0f,
		   0.0f,     1.4142f, 0.0f, 0.0f, 0.0f,
		   0.0f,     0.0f,    1.0f, 0.0f, 0.0f,
		   0.0f,     0.0f,    0.0f, 0.0f, 0.0f,
		   0.0f,     0.0f,    0.0f, 0.0f, 0.0f
		};
		Mat sExpected(5, 5, sE, SpStackMemoryAllocator::main());

		sp_float vE[9] = {
		   -0.4082f,  0.5774f, -0.7071f,
		   -0.8165f, -0.5774f,  0.0f,
		   -0.4082f,  0.5774f,  0.7071f
		};
		Mat vExpected(3, 3, vE, SpStackMemoryAllocator::main());

		Assert::IsTrue(isCloseEnough(u, uExpected, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
		Assert::IsTrue(isCloseEnough(s, sExpected, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
		Assert::IsTrue(isCloseEnough(v, vExpected, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

		Mat vt(3, 3, SpStackMemoryAllocator::main());
		vExpected.transpose(vt);

		Mat expected(3, 3, a, SpStackMemoryAllocator::main());
		multiply(u, s, vt, expected);
		Assert::IsTrue(isCloseEnough(A, expected, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

		/*  test for PCA  (can be removed)
		sp_float aa[15] = {
			0.0f,  1.0f, 10.0f,
			0.0f, 10.0f, 10.0f,
			0.0f, 20.0f, 0.0f,
			0.0f, 30.0f, 0.0f,
			0.0f, 40.0f, 10.0f
		};
		Mat AA(5, 3, aa);
		Mat RR(5, 3);
		AA.svd(u, s, v, iterations, SP_UINT_MAX, SP_EPSILON_TWO_DIGITS);

		std::string us = u.toString();
		std::string ss = s.toString();
		std::string vs = v.toString();
		*/
	}

	SP_TEST_METHOD(CLASS_NAME, gramSchmidt)
	{
		sp_float m1[9] = {
			1.0f, 0.0f, 0.0f,
			1.0f, 1.0f, 0.0f,
			1.0f, 1.0f, 1.0f
		};
		Mat M1(3, 3, m1, SpStackMemoryAllocator::main());
		sp_float expected[9] = {
			0.5773f, -0.8163f, 0.0f,
			0.5773f, 0.4082f, -0.7071f,
			0.5773f, 0.4082f, 0.7071f
		};

		Mat result(3, 3, SpStackMemoryAllocator::main());
		M1.gramSchmidt(result);

		for (sp_uint i = 0; i < MAT3_LENGTH; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], SP_EPSILON_THREE_DIGITS), L"Wrong value", LINE_INFO());


		sp_float a[15] = {
			1.0f, 0.0f, 1.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 1.0f, 1.0f,
			0.0f, 1.0f, 0.0f,
			1.0f, 1.0f, 0.0f
		};
		Mat A(5, 3, a, SpStackMemoryAllocator::main());
		Mat R(5, 3, SpStackMemoryAllocator::main());
		A.gramSchmidt(R);

		sp_float c[15] = {
			0.7071f, -0.2673f,  0.4781f,
			0.0f,     0.5345f, -0.1195f,
			0.0f,     0.5345f,  0.7171f,
			0.0f,     0.5345f, -0.1195f,
			0.7071f,  0.2673f, -0.4781f
		};
		Mat C(5, 3, c, SpStackMemoryAllocator::main());

		Assert::IsTrue(isCloseEnough(C, R, SP_EPSILON_THREE_DIGITS), L"Wrong value", LINE_INFO());

		sp_float t[15] = {
			1.0f, 2.0f, 3.0f,
			4.0f, 5.0f, 6.0f,
			0.0f, 1.0f, 5.0f,
			6.0f, 0.0f, 7.0f,
			7.0f, 2.0f, 6.0f
		};
		Mat T(5, 3, t, SpStackMemoryAllocator::main());
		T.gramSchmidt(R);

		sp_float tr[15] = {
			0.0990f,  0.3569f,  0.1677f,
			0.3961f,  0.7776f, -0.1358f,
			0.0f,     0.2167f,  0.8662f,
			0.5941f, -0.4589f,  0.3674f,
			0.6931f, -0.1020f, -0.2612f
		};
		Mat TR(5, 3, tr, SpStackMemoryAllocator::main());

		Assert::IsTrue(isCloseEnough(TR, R, SP_EPSILON_THREE_DIGITS), L"Wrong value", LINE_INFO());
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

		Mat m(4,4, matrix, SpStackMemoryAllocator::main());
		Mat e(4,4, expected, SpStackMemoryAllocator::main());

		Mat result(4, 4, SpStackMemoryAllocator::main());
		transpose(m, result);
		
		Assert::IsTrue(isCloseEnough(result, e, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());

		sp_float a2[10] = {
			8.0f, 5.0f, 1.0f, 8.0f, 8.0f,
			9.0f, 1.0f, 2.0f, 2.0f, 2.0f
		};
		m = Mat(2, 5, matrix, SpStackMemoryAllocator::main());

		result = Mat(5, 2, SpStackMemoryAllocator::main());
		transpose(m, result);

		sp_float expected2[10] = {
			8.0f, 9.0f,
			5.0f, 1.0f,
			1.0f, 2.0f,
			8.0f, 2.0f,
			8.0f, 2.0f
		};
		Mat e2(5, 2, expected2, SpStackMemoryAllocator::main());

		Assert::IsTrue(isCloseEnough(result, e2, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
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

		Mat A(2, 5, a, SpStackMemoryAllocator::main());
		Mat B(5, 2, b, SpStackMemoryAllocator::main());
		Mat expected(2, 2, e, SpStackMemoryAllocator::main());

		Mat result(2, 2, SpStackMemoryAllocator::main());
		multiply(A, B, result);

		Assert::IsTrue(isCloseEnough(result, expected, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, _multiply2)
	{
		sp_float a[15] = {
			1.0f, 0.0f, 1.0f,
			0.0f, 1.0f, 0.0f,
			0.0f, 1.0f, 1.0f,
			0.0f, 1.0f, 0.0f,
			1.0f, 1.0f, 0.0f
		};
		Mat A(5, 3, a, SpStackMemoryAllocator::main());

		sp_float b[9] = {
			0.4082f, 0.8164f, 0.4082f,
			0.5773f, -0.5773f, 0.5773f,
			-0.7071f, 0.0f, 0.7071f
		};
		Mat B(3, 3, b, SpStackMemoryAllocator::main());

		sp_float e[15] = {
				   -0.2989f, 0.8164f, 1.1153f,
					0.5773f, -0.5773f, 0.5773f,
				   -0.1298f, -0.5773f, 1.2844f,
					0.5773f, -0.5773f, 0.5773f,
					0.9855f, 0.2391f, 0.9855f
		};
		Mat expected = Mat(5, 3, e, SpStackMemoryAllocator::main());

		Mat result = Mat(5, 3, SpStackMemoryAllocator::main());
		multiply(A, B, result);

		Assert::IsTrue(isCloseEnough(expected, result, SP_EPSILON_THREE_DIGITS), L"Wrong number", LINE_INFO());
	}
}

#undef CLASS_NAME