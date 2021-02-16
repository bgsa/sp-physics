#include "SpectrumPhysicsTest.h"
#include <AlgorithmBisection.h>

#define CLASS_NAME AlgorithmBisectionTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float funcBisection(sp_float x)
	{
		return powf(x, 3) + 4.0f * powf(x, 2) - 10.0f;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(solve);

		SP_TEST_METHOD_DEF(maxNumberOfIteration);

	};

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		AlgorithmBisection algorithm;
		sp_float result = algorithm.solve(1.0f, 2.0f, funcBisection);
		
		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, maxNumberOfIteration)
	{
		AlgorithmBisection algorithm;
		sp_int result = algorithm.maxNumberOfIteration();

		Assert::AreEqual(result, 14, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME