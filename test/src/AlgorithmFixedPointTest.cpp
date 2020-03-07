#include "TestHeader.h"
#include <AlgorithmFixedPoint.h>

#define CLASS_NAME AlgorithmFixedPointTest

namespace SP_PHYSICS_TEST_NAMESPACE
{
	float funcFixedPoint(float x)
	{
		float numerator = powf(x, 3) + 4 * pow(x, 2) - 10;

		float numeratorDerived = 3 * pow(x, 2) + 8 * x;

		return x - (numerator / numeratorDerived);
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmFixedPoint_solve_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmFixedPoint_solve_Test)
	{
		AlgorithmFixedPoint<float> algorithm;
		const float initialApproximation = 1.5f;

		float result = algorithm.solve(initialApproximation, funcFixedPoint);

		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
