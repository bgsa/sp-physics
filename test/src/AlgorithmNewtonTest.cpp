#include "SpectrumPhysicsTest.h"
#include <AlgorithmNewton.h>

#define CLASS_NAME AlgorithmNewtonTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float funcNewton(sp_float x)
	{
		return std::cos(x) - x;
	}

	sp_float funcDerivatedNewton(sp_float x)
	{
		return -std::sin(x) - 1;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(solve);

	};

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		AlgorithmNewton algorithm;
		sp_float result = algorithm.solve(sp_float(PI / 4), funcNewton, funcDerivatedNewton);

		Assert::IsTrue(isCloseEnough(result, 0.7390f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME