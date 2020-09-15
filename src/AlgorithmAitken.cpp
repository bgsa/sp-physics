#include "AlgorithmAitken.h"

namespace NAMESPACE_PHYSICS
{
	sp_float AlgorithmAitken::solve(sp_float approximation, sp_float functor(sp_float), sp_int maxOfInteration)
	{
		while (maxOfInteration != 0)
		{
			sp_float newAproximation = functor(approximation);
			sp_float newAproximation1 = functor(newAproximation);
			sp_float newAproximation2 = functor(newAproximation1);

			sp_double divisor = newAproximation2 - 2.0 * newAproximation1 + newAproximation;

			if (divisor == ZERO_DOUBLE)
				return newAproximation2;

			newAproximation = newAproximation - std::pow(newAproximation1 - newAproximation, TWO_FLOAT) / divisor;

			if (isCloseEnough(newAproximation, approximation))
				return newAproximation;

			approximation = newAproximation;

			maxOfInteration--;
		}

		return SP_NOT_A_NUMBER;
	}
}