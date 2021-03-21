#include "SpectrumPhysicsTest.h"
#include <AlgorithmHorner.h>

#define CLASS_NAME AlgorithmHornerTest

namespace NAMESPACE_PHYSICS_TEST
{
	sp_float funcHorner(sp_float x)
	{
		return powf(x , 4)* 2.0f - 3 * powf(x , 2) + 3.0f * x - 4.0f;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(AlgorithmHorner_polynomialDivision_Test);

		SP_TEST_METHOD_DEF(AlgorithmHorner_findRoots_Test);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmHorner_polynomialDivision_Test)
	{
		AlgorithmHorner algorithm;
		sp_float polynomial[5] = { 2.0f, 0.0f, -3.0f, 3.0f, -4.0f };
		sp_float x0 = -2.0f;
		sp_float expected[5] = { 2.0f, -4.0f, 5.0f, -7.0f, 10.0f };

		sp_float result[5];
		algorithm.polynomialDivision(x0, polynomial, 5u, result);

		for (size_t i = 0; i < 5; i++)
			Assert::IsTrue(isCloseEnough(result[i], expected[i]), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmHorner_findRoots_Test)
	{
		AlgorithmHorner algorithm;
		sp_float polynomial[5] = { 2.0f, 0.0f, -3.0f, 3.0f, -4.0f };
		sp_float x0 = -2.0f;
		sp_float expected[5] = { -2.0f, -1.7959f, -1.7424f, -1.7389f, -1.7389f };

		sp_float result[5];
		algorithm.findRoots(x0, polynomial, 5u, result);

		for (size_t i = 0; i < 5; i++)
			Assert::IsTrue(isCloseEnough(result[i], expected[i]), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME