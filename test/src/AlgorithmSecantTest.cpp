#include "SpectrumPhysicsTest.h"
#include <AlgorithmSecant.h>

#define CLASS_NAME AlgorithmSecantTest

namespace NAMESPACE_PHYSICS_TEST
{
	float funcSecant(float x)
	{
		return std::cos(x) - x;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmSecant_solve_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSecant_solve_Test)
	{
		AlgorithmSecant<float> algorithm;
		float result = algorithm.solve(0.5f, float(PI / 4), funcSecant);

		Assert::IsTrue(isCloseEnough(result, 0.7390f), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME