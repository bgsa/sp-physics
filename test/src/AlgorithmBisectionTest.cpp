#include "TestHeader.h"
#include <AlgorithmBisection.h>

#define CLASS_NAME AlgorithmBisectionTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	float funcBisection(float x)
	{
		return powf(x, 3) + 4 * pow(x, 2) - 10;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmBisection_solve_Test);

		SP_TEST_METHOD_DEF(AlgorithmBisection_maxNumberOfIteration_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmBisection_solve_Test)
	{
		AlgorithmBisection<float> algorithm;
		float result = algorithm.solve(1.0f, 2.0f, funcBisection);
		
		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmBisection_maxNumberOfIteration_Test)
	{
		AlgorithmBisection<float> algorithm;
		int result = algorithm.maxNumberOfIteration();

		Assert::AreEqual(result, 14, L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME