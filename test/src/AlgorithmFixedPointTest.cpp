#include "SpectrumPhysicsTest.h"
#include <AlgorithmFixedPoint.h>

#define CLASS_NAME AlgorithmFixedPointTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float funcFixedPoint(float x)
	{
		sp_float numerator = powf(x, 3) + 4.0f * powf(x, 2) - 10.0f;

		sp_float numeratorDerived = 3.0f * powf(x, 2) + 8.0f * x;

		return x - (numerator / numeratorDerived);
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(solve);

	};

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		AlgorithmFixedPoint algorithm;
		const sp_float initialApproximation = 1.5f;

		sp_float result = algorithm.solve(initialApproximation, funcFixedPoint);

		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
