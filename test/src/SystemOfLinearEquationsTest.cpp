#include "SpectrumPhysicsTest.h"
#include "SystemOfLinearEquations.h"

#define CLASS_NAME SystemOfLinearEquationsTest

namespace NAMESPACE_PHYSICS_TEST
{
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(solve);
		SP_TEST_METHOD_DEF(solve_2varaibles_Test);
		SP_TEST_METHOD_DEF(solve_3varaibles_Test);
		SP_TEST_METHOD_DEF(solve_4varaibles_Test);
		SP_TEST_METHOD_DEF(getLineEquation_Test);
		SP_TEST_METHOD_DEF(getCircleEquation_Test);
		SP_TEST_METHOD_DEF(canonicalForm);
	};

	SP_TEST_METHOD(CLASS_NAME, canonicalForm)
	{
		sp_float matrix[15] = {
			2.0f, 3.0f, -2.0f, -7.0f, 1.0f,
			1.0f, 1.0f, 1.0f, 3.0f, 6.0f,
			1.0f, -1.0f, 1.0f, 5.0f, 4.0f
		};

		SystemOfLinearEquations system;
		system.canonicalForm(matrix, 3, 5);

		sp_float expected[15] = {
			1.0f, 0.0f, 0.0f, 1.0f, 2.0f,
			0.0f, 1.0f, 0.0f, -1.0f, 1.0f,
			0.0f, 0.0f, 1.0f, 3.0f, 3.0f
		};

		for (sp_uint i = 0; i < 15; i++)
			Assert::AreEqual(expected[i], matrix[i], L"Value wrong", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		SystemOfLinearEquations linearSystem;
		const sp_uint rowSize = 3;
		const sp_uint colSize = 4;

		/*
		sp_float matrix[rowSize * colSize] = {
			2.0f, 0.0f, -2.0f, 0.0f,
			-1.0f, 0.0f, -1.0f, 0.0f,
			-1.0f, 0.0f, -1.0f, 0.0f
		};
		*/
		sp_float matrix[rowSize * colSize] = {
			1.0f, 0.0f, 2.0f, 0.0f,
			-1.0f, -1.0f, -1.0f, 0.0f,
			-1.0f, 0.0f, -2.0f, 0.0f
		};

		sp_float expected[colSize - 1] = { 2.0f, 3.0f, 0.0f };

		sp_float result[3];
		sp_bool hasSolution = linearSystem.solve(matrix, rowSize, colSize, result);

		Assert::IsTrue(hasSolution, L"Value wrong", LINE_INFO());

		for (sp_uint i = 0; i < colSize - 1; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i], SP_EPSILON_THREE_DIGITS), L"Value wrong", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, solve_2varaibles_Test)
	{
		SystemOfLinearEquations linearSystem;
		const sp_uint rowSize = 2;
		const sp_uint colSize = 3;

		sp_float matrix[rowSize * colSize] {
			1.0f,  1.0f, 3.0f,
			1.0f, -1.0f, 1.0f
		};

		sp_float expected[colSize - 1] = { 2.0f, 1.0f };

		sp_float result[2];
		linearSystem.solve(matrix, rowSize, colSize, result);

		for (sp_uint i = 0; i < colSize - 1; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Value wrong", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, solve_3varaibles_Test)
	{				
		SystemOfLinearEquations linearSystem;
		const sp_uint rowSize = 3;
		const sp_uint colSize = 4;

		sp_float matrix[rowSize * colSize]{
			2.0f,  1.0f, -3.0f, -1.0f,
			-1.0f, 3.0f, 2.0f, 12.0f,
			3.0f, 1.0f, -3.0, 0.0f
		};

		sp_float expected[colSize - 1] = { 1.0f, 3.0f, 2.0f };

		sp_float result[3];
		linearSystem.solve(matrix, rowSize, colSize, result);

		for (sp_uint i = 0; i < colSize - 1; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Value wrong", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, solve_4varaibles_Test)
	{
		SystemOfLinearEquations linearSystem;
		const sp_uint rowSize = 4;
		const sp_uint colSize = 5;

		sp_float matrix[rowSize * colSize] {
			1.0f, -1.0f, 2.0f, -1.0f, -8.0f,
			2.0f, -2.0f, 3.0f, -3.0f, -20.0f,
			1.0f, 1.0f, 1.0f, 0.0f, -2.0f,
			1.0f, -1.0f, 4.0f, 3.0f, 4.0f,
		};

		sp_float expected[colSize - 1] = { -7.0f, 3.0f, 2.0f, 2.0f };

		sp_float result[4];
		linearSystem.solve(matrix, rowSize, colSize, result);

		for (sp_uint i = 0; i < colSize - 1; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Value wrong", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, getLineEquation_Test)
	{
		Vec2 point1 = { 2.0f, 1.0f };
		Vec2 point2 = { 3.0f, 7.0f };

		Vec3 expected = { -6.0f, 1.0f, 11.0f };

		Vec3 result = SystemOfLinearEquations::getLineEquation(point1, point2);

		for (sp_uint i = 0; i < 3; i++)
			Assert::AreEqual(expected[i], result[i], L"Value wrong", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, getCircleEquation_Test)
	{
		Vec2 point1 = { 1.0f, 7.0f };
		Vec2 point2 = { 6.0f, 2.0f };
		Vec2 point3 = { 4.0f, 6.0f };

		Vec4 expected = { 10.0f, -20.0f, -40.0f, -200.0f };

		Vec4 result = SystemOfLinearEquations::getCircleEquation(point1, point2, point3);

		for (sp_uint i = 0; i < 4; i++)
			Assert::AreEqual(expected[i], result[i], L"Value wrong", LINE_INFO());
	}

}

#undef CLASS_NAME