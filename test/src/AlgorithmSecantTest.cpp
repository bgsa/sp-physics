#include "SpectrumPhysicsTest.h"
#include <AlgorithmSecant.h>

#define CLASS_NAME AlgorithmSecantTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float funcSecant(sp_float x)
	{
		return std::cosf(x) - x;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(solve);

	};

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		AlgorithmSecant algorithm;
		sp_float result = algorithm.solve(HALF_FLOAT, sp_float(PI / 4), funcSecant);

		Assert::IsTrue(isCloseEnough(result, 0.7390f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME