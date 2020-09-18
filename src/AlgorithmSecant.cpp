#include "AlgorithmSecant.h"

namespace NAMESPACE_PHYSICS
{

	sp_float AlgorithmSecant::solve(sp_float approximation1, sp_float approximation2, sp_float functor(sp_float), sp_int maxOfInteration)
	{
		sp_float valueApproximation1 = functor(approximation1);
		sp_float valueApproximation2 = functor(approximation2);

		while (maxOfInteration != ZERO_INT)
		{
			sp_float newApproximation = approximation2 - valueApproximation2 * (approximation2 - approximation1) / (valueApproximation2 - valueApproximation1);

			if (isCloseEnough(newApproximation - approximation2, ZERO_FLOAT))
				return newApproximation;

			approximation1 = approximation2;
			valueApproximation1 = valueApproximation2;

			approximation2 = newApproximation;
			valueApproximation2 = functor(newApproximation);

			maxOfInteration--;
		}

		return SP_NOT_A_NUMBER;
	}

}