#include "SpectrumPhysicsTest.h"
#include <AlgorithmLagrange.h>

#define CLASS_NAME AlgorithmLagrangeTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		float funcLagrange(float x) 
		{
			return 1.0f / x;
		}

		SP_TEST_METHOD_DEF(AlgorithmLagrange_polynomialApproximation_Test1);

		SP_TEST_METHOD_DEF(AlgorithmLagrange_polynomialApproximation_Test2);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmLagrange_polynomialApproximation_Test1)
	{
		AlgorithmLagrange<float> algorithm;
		Vec2<float> points[2] = { Vec2<float>(2.0, 4.0f) , Vec2<float>(5.0f, 1.0f) };

		float result = algorithm.polynomialApproximation(points, 2, 3.0f);

		Assert::IsTrue(isCloseEnough(3.0f, result), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmLagrange_polynomialApproximation_Test2)
	{
		AlgorithmLagrange<float> algorithm;
		Vec2<float> points[3] = { Vec2<float>(2.0, funcLagrange(2.0f)), 
									Vec2<float>(2.75, funcLagrange(2.75f)), 
									Vec2<float>(4.0, funcLagrange(4.0f))
								};

		float result = algorithm.polynomialApproximation(points, 3, 3.0f);

		Assert::IsTrue(isCloseEnough(0.3295f, result), L"Wrong value.", LINE_INFO());
	}

}
