#include "AlgorithmSteffensen.h"

namespace NAMESPACE_PHYSICS
{

	sp_float AlgorithmSteffensen::solve(sp_float approximation, sp_float functor(sp_float), sp_int maxOfInteration)
	{
		while (maxOfInteration != ZERO_INT)
		{
			sp_float aproximation1 = functor(approximation);
			sp_float aproximation2 = functor(aproximation1);

			sp_float divisor = (aproximation2 - (TWO_FLOAT * aproximation1) + approximation);

			if (divisor == ZERO_FLOAT)
				return aproximation2;

			sp_float newAproximation = (sp_float) (approximation - ((aproximation1 - approximation) * (aproximation1 - approximation)) / divisor);

			if (isCloseEnough(newAproximation, approximation))
				return approximation;

			approximation = newAproximation;

			maxOfInteration--;
		}

		return SP_NOT_A_NUMBER;
	}

}