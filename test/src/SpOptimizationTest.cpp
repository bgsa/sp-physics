#include "SpectrumPhysicsTest.h"
#include <SpOptimization.h>

#define CLASS_NAME SpOptimizationTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float func_objective(sp_float x)
	{
		return sinf(x) * TWO_FLOAT - (x * x) / 10.0f;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(aurea);
		SP_TEST_METHOD_DEF(interpolation);
	};

	SP_TEST_METHOD(CLASS_NAME, aurea)
	{
		SpOptimization optimization;
		sp_float x1 = 0.0f;
		sp_float x2 = 4.0f;
		sp_float fx;

		sp_float result = optimization.aurea(func_objective, sp_optimization_type_maximize, x1, x2, &fx);

		Assert::IsTrue(isCloseEnough(result, 1.4276f, 0.00999f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(fx, 1.7755f, 0.00999f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, interpolation)
	{
		SpOptimization optimization;
		sp_float x1 = 0.0f;
		sp_float x2 = 1.0f;
		sp_float x3 = 4.0f;
		sp_float fx;

		sp_float result = optimization.interpolation(func_objective, sp_optimization_type_maximize, x1, x2, x3, &fx);

		Assert::IsTrue(isCloseEnough(result, 1.4374f, 0.00999f), L"Wrong value.", LINE_INFO());
		Assert::IsTrue(isCloseEnough(fx, 1.7756f, 0.00999f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME