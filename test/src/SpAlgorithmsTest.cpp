#ifndef SP_ALGORITHM_TESTS_HEADER
#define SP_ALGORITHM_TESTS_HEADER

#include "SpectrumPhysicsTest.h"
#include <SpAlgorithms.h>

#define CLASS_NAME SpAlgorithms

namespace NAMESPACE_PHYSICS_TEST
{
	
	SP_TEST_CLASS(CLASS_NAME)
	{
	public:
		SP_TEST_METHOD_DEF(jacobi_Test);
		SP_TEST_METHOD_DEF(gaussSeidel_Test);
	};

	SP_TEST_METHOD(CLASS_NAME, jacobi_Test)
	{
		// Example taken from book: Analise Numerica 10Ed: pagina 502
		sp_float matrixA[16] = {
			10.0f, -1.0f, 2.0f, 0.0f,
			-1.0f, 11.0f, -1.0f, 3.0f,
			2.0f, -1.0f, 10.0f, -1.0f,
			0.0f, 3.0f, -1.0f, 8.0f
		};
		sp_float vectorB[4] = { 6.0f, 25.0f, -11.0f, 15.0f };
		sp_float initialApproximation[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
		
		sp_float result[4];

		jacobi(matrixA, vectorB, 4, 4, initialApproximation, result, DefaultErrorMargin);

		sp_float expected[4] = { 1.0f, 2.0f, -1.0f, 1.0f };

		for (sp_uint i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, gaussSeidel_Test)
	{
		// Example taken from book: Analise Numerica 10Ed: pagina 502
		sp_float matrixA[16] = {
			10.0f, -1.0f, 2.0f, 0.0f,
			-1.0f, 11.0f, -1.0f, 3.0f,
			2.0f, -1.0f, 10.0f, -1.0f,
			0.0f, 3.0f, -1.0f, 8.0f
		};
		sp_float vectorB[4] = { 6.0f, 25.0f, -11.0f, 15.0f };
		sp_float initialApproximation[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

		sp_float result[4];

		gaussSeidel(matrixA, vectorB, 4, 4, initialApproximation, result, DefaultErrorMargin);

		sp_float expected[4] = { 1.0f, 2.0f, -1.0f, 1.0f };

		for (sp_uint i = 0; i < 4; i++)
			Assert::IsTrue(isCloseEnough(expected[i], result[i]), L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME

#endif
