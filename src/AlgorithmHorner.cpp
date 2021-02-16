#include "AlgorithmHorner.h"

namespace NAMESPACE_PHYSICS
{
	void AlgorithmHorner::polynomialDivision(sp_float x0, sp_float* polynomial, sp_uint polynomialDegree, sp_float* output)
	{
		output[0] = polynomial[0];

		for (sp_uint i = 1u; i < polynomialDegree; i++)
			output[i] = polynomial[i] + output[i - 1u] * x0; //Bk = Ak + Bk + 1 * x0;
	}

	sp_float solvePolynomial(sp_float* coefficients, sp_uint polynomialDegree, sp_float x)
	{
		sp_float result = ZERO_FLOAT;

		for (sp_uint i = polynomialDegree - 1; i != ZERO_UINT; i--)
			result += coefficients[polynomialDegree - i - 1] * powf(x, (sp_float)i);
			
		result += coefficients[polynomialDegree - 1];

		return result;
	}

	void AlgorithmHorner::findRoots(sp_float x0, sp_float* polynomial, sp_uint polynomialDegree, sp_float* output)
	{
		output[0] = x0;
		
		for (sp_uint i = 0; i < polynomialDegree - 1; i++)
		{
			sp_float* dividedPolynomial = ALLOC_NEW_ARRAY(sp_float, polynomialDegree);
			
			polynomialDivision(output[i], polynomial, polynomialDegree, dividedPolynomial);

			const sp_float value = dividedPolynomial[polynomialDegree - 1];

			const sp_float valueDerived = solvePolynomial(dividedPolynomial, polynomialDegree - 1, output[i]);
				
			output[i + 1] = output[i] - value / valueDerived;

			ALLOC_RELEASE(dividedPolynomial);
		}
	}

}