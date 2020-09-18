#include "AlgorithmNewton.h"

namespace NAMESPACE_PHYSICS
{

	sp_float AlgorithmNewton::solve(sp_float approximation, sp_float functor(sp_float), sp_float derivedFunctor(sp_float), sp_int maxOfInteration)
	{
		while (maxOfInteration != ZERO_INT)
		{
			sp_float newApproximation = approximation - (functor(approximation) / derivedFunctor(approximation));

			if (isCloseEnough(newApproximation, approximation))
				return newApproximation;

			approximation = newApproximation;

			maxOfInteration--;
		}

		return SP_NOT_A_NUMBER;
	}

}