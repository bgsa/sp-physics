#ifndef ALGORITHM_AITKEN_TEST_HEADER
#define ALGORITHM_AITKEN_TEST_HEADER

#include "SpectrumPhysicsTest.h"
#include <AlgorithmAitken.h>

#define CLASS_NAME AlgorithmAitkenTest

namespace NAMESPACE_PHYSICS_TEST
{
	float funcAitken(float x)
	{
		return sqrtf(10.0f / (x + 4.0f));
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmAitken_solve_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmAitken_solve_Test)
	{
		AlgorithmAitken algorithm;
		const sp_float initialApproximation = 1.5f;

		sp_float result = algorithm.solve(initialApproximation, funcAitken);

		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME

#endif // !ALGORITHM_AITKEN_TEST_HEADER
