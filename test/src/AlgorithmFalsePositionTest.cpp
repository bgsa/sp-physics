#include "SpectrumPhysicsTest.h"
#include <AlgorithmFalsePosition.h>

#define CLASS_NAME AlgorithmFalsePositionTest

namespace NAMESPACE_PHYSICS_TEST
{
	float funcFalsePosition(float x)
	{
		return std::cos(x) - x;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(solve);

	};

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		AlgorithmFalsePosition algorithm;
		float result = algorithm.solve(0.5f, float(PI/4.0f), funcFalsePosition);

		Assert::IsTrue(isCloseEnough(result, 0.7390f), L"Wrong value.", LINE_INFO());
	}
}

#undef CLASS_NAME
