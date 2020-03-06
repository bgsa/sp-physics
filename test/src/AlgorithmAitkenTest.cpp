#ifndef ALGORITHM_AITKEN_TEST_HEADER
#define ALGORITHM_AITKEN_TEST_HEADER

#include "TestHeader.h"
#include <AlgorithmAitken.h>

#define CLASS_NAME AlgorithmAitkenTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	float funcAitken(float x)
	{
		return sqrtf(10.0f / (x + 4.0f));
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmAitken_solve_Test)
	{
		AlgorithmAitken<float> algorithm;
		const float initialApproximation = 1.5f;

		float result = algorithm.solve(initialApproximation, funcAitken);

		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME

#endif // !ALGORITHM_AITKEN_TEST_HEADER
