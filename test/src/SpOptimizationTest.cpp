#include "SpectrumPhysicsTest.h"
#include "SystemOfLinearEquations.h"
#include <SpOptimization.h>

#define CLASS_NAME SpOptimizationTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float func_objective(sp_float x)
	{
		return sinf(x) * TWO_FLOAT - (x * x) / 10.0f;
	}
	sp_float func_derived1_objective(sp_float x)
	{
		return cosf(x) * TWO_FLOAT - x / 5.0f;
	}
	sp_float func_derived2_objective(sp_float x)
	{
		return sinf(x) * -TWO_FLOAT - 1.0f / 5.0f;
	}

	sp_float func_objective_2(sp_float x)
	{
		return x * x + 2.0f * x - 3.0f;
	}
	sp_float func_derived1_objective_2(sp_float x)
	{
		return 2.0f * x + 2.0f;
	}
	sp_float func_derived2_objective_2(sp_float x)
	{
		return 2.0f;
	}

	sp_float func_objective_2D(sp_float x, sp_float y)
	{
		return y - x - (2.0f * (x * x)) - (2.0f * x * y) - (y * y);
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(aurea);
		SP_TEST_METHOD_DEF(interpolation);
		SP_TEST_METHOD_DEF(newton);
		SP_TEST_METHOD_DEF(scan);
		SP_TEST_METHOD_DEF(arbitrarySearch);
		SP_TEST_METHOD_DEF(simplex);
		SP_TEST_METHOD_DEF(simplex_unbouded);
		SP_TEST_METHOD_DEF(simplex_infinity);
	};

	SP_TEST_METHOD(CLASS_NAME, aurea)
	{
		SpOptimization optimization;
		sp_float x1 = 0.0f;
		sp_float x2 = 4.0f;
		sp_float fx;

		sp_float result = optimization.goldenSectionSearch(func_objective, sp_optimization_type_maximize, x1, x2, &fx);
		Assert::IsTrue(isCloseEnough(result, 1.4276f, 0.00999f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(fx, 1.7755f, 0.00999f), L"Wrong value.", LINE_INFO());

		x2 = 20.0f;
		result = optimization.goldenSectionSearch(func_objective_2, sp_optimization_type_minimize, x1, x2, &fx);
		Assert::IsTrue(isCloseEnough(result, 0.009f, 0.00999f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(fx, -2.981f, 0.00999f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, interpolation)
	{
		SpOptimization optimization;
		sp_float x1 = 0.0f;
		sp_float x2 = 1.0f;
		sp_float x3 = 4.0f;
		sp_float fx;

		sp_float result = optimization.interpolation(func_objective, sp_optimization_type_maximize, x1, x2, x3, &fx);
		Assert::IsTrue(isCloseEnough(result, 1.4276f, 0.00999f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(fx, 1.7755f, 0.00999f), L"Wrong value.", LINE_INFO());

		x2 = 5.0f;
		x3 = 20.0f;
		result = optimization.interpolation(func_objective_2, sp_optimization_type_minimize, x1, x2, x3, &fx);
		Assert::IsTrue(isCloseEnough(result, 0.009f, 0.00999f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(fx, -2.981f, 0.00999f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, newton)
	{
		SpOptimization optimization;
		sp_float x1 = 2.5f;

		sp_float result = optimization.newton(x1, func_derived1_objective, func_derived2_objective);
		Assert::IsTrue(isCloseEnough(result, 1.4276f, 0.00999f), L"Wrong value.", LINE_INFO());

		x1 = 5.0f;
		result = optimization.newton(x1, func_derived1_objective_2, func_derived2_objective_2);
		Assert::IsTrue(isCloseEnough(result, -1.0f, 0.00999f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, scan)
	{
		SpOptimization optimization;

		const sp_float xmin = -2.0f;
		const sp_float xmax = 2.0f;
		const sp_float ymin = 1.0f;
		const sp_float ymax = 3.0f;
		const sp_float stepSize = 0.1f;

		sp_uint iterations;
		sp_float xValue, yValue;
		sp_float result = optimization.scan(func_objective_2D, xmin, xmax, ymin, ymax, stepSize, stepSize, &xValue, &yValue, &iterations);

		Assert::IsTrue(isCloseEnough(result, 1.25f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(xValue, -1.0f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(yValue, 1.5f ), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, arbitrarySearch)
	{
		SpOptimization optimization;

		const sp_float xmin = -2.0f;
		const sp_float xmax = 2.0f;
		const sp_float ymin = 1.0f;
		const sp_float ymax = 3.0f;

		sp_uint iterations = 10000u;
		sp_float xValue, yValue;
		sp_float result = optimization.arbitrarySearch(func_objective_2D, xmin, xmax, ymin, ymax, iterations, &xValue, &yValue);

		// Values should be closed to F_xy = 1.25f, x = -1.0f, y = 1.5f
	}

	SP_TEST_METHOD(CLASS_NAME, simplex)
	{
		SpOptimization optimization;

		sp_float matrix[32] = {
			 2.0f,  1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2.0f,
			 2.0f, -1.0f,  5.0f, 0.0f, 1.0f, 0.0f, 0.0f, 6.0f,
			 4.0f,  1.0f,  1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 6.0f,
			-1.0f, -2.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f
		};

		SpPair<sp_uint, sp_float> result[3];

		optimization.simplex(matrix, 4u, 8u, result);

		SpPair<sp_uint, sp_float> expected[3] = {
			{1u, 4.0f},
			{2u, 2.0f},
			{5u, 0.0f}
		};

		for (sp_uint i = 0; i < 3; i++)
		{
			Assert::AreEqual(expected[i].key, result[i].key, L"Wrong value.", LINE_INFO());
			Assert::AreEqual(expected[i].value, result[i].value, L"Wrong value.", LINE_INFO());
		}
	}

	SP_TEST_METHOD(CLASS_NAME, simplex_unbouded)
	{
		SpOptimization optimization;

		sp_float matrix[18] = {
			 1.0f, -1.0f, 1.0f, 0.0f, 0.0f, 1.0f,
			 3.0f, -2.0f, 0.0f, 1.0f, 0.0f, 6.0f,
			-3.0f, -2.0f, 0.0f, 0.0f, 1.0f, 0.0f
		};

		SpPair<sp_uint, sp_float> result[2];

		optimization.simplex(matrix, 3u, 6u, result);

		SpPair<sp_uint, sp_float> expected[2] = {
			{0u, 4.0f},
			{1u, 3.0f}
		};

		for (sp_uint i = 0; i < 2; i++)
		{
			Assert::AreEqual(expected[i].key, result[i].key, L"Wrong value.", LINE_INFO());
			Assert::AreEqual(expected[i].value, result[i].value, L"Wrong value.", LINE_INFO());
		}
	}

	SP_TEST_METHOD(CLASS_NAME, simplex_infinity)
	{
		SpOptimization optimization;

		sp_float matrix[28] = {
			 10.0f,    5.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2500.0f,
			  4.0f,   10.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2000.0f,
			  2.0f,    3.0f, 0.0f, 0.0f, 1.0f, 0.0f, 900.0f,
			-40.0f, -100.0f, 0.0f, 0.0f, 0.0f, 1.0f,   0.0f,
		};

		SpPair<sp_uint, sp_float> result[3];

		optimization.simplex(matrix, 4u, 7u, result);

		SpPair<sp_uint, sp_float> expected[3] = {
			{2u, 1500.0f},
			{1u,  200.0f},
			{4u,  300.0f}
		};

		for (sp_uint i = 0; i < 2; i++)
		{
			Assert::AreEqual(expected[i].key, result[i].key, L"Wrong value.", LINE_INFO());
			Assert::AreEqual(expected[i].value, result[i].value, L"Wrong value.", LINE_INFO());
		}
	}
}

#undef CLASS_NAME