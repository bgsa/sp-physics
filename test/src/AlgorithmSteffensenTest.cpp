#include "SpectrumPhysicsTest.h"
#include <AlgorithmSteffensen.h>

#define CLASS_NAME AlgorithmSteffensenTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float funcSteffensen(sp_float x)
	{
		//return powf(x, 3) + 4 * pow(x, 2) - 10;

		return sp_sqrt(10.0f / (x + 4.0f));
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(solve);

	};

	SP_TEST_METHOD(CLASS_NAME, solve)
	{
		AlgorithmSteffensen algorithm;
		sp_float result = algorithm.solve(1.5f, funcSteffensen);

		Assert::IsTrue(isCloseEnough(result, 1.3652f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
