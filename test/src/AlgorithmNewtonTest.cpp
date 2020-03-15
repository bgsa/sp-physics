#include "TestHeader.h"
#include <AlgorithmNewton.h>

#define CLASS_NAME AlgorithmNewtonTest

namespace NAMESPACE_PHYSICS_TEST
{
	float funcNewton(float x)
	{
		return std::cos(x) - x;
	}

	float funcDerivatedNewton(float x)
	{
		return -std::sin(x) - 1;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmNewton_solve_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmNewton_solve_Test)
	{
		AlgorithmNewton<float> algorithm;
		float result = algorithm.solve(float(PI / 4), funcNewton, funcDerivatedNewton);

		Assert::IsTrue(isCloseEnough(result, 0.7390f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME