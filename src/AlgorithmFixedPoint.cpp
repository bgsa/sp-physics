#include "AlgorithmFixedPoint.h"

namespace NAMESPACE_PHYSICS
{
	sp_float AlgorithmFixedPoint::solve(sp_float approximation, sp_float functor(sp_float), sp_int maxOfInteration)
	{	
		while (maxOfInteration != 0)
		{
			sp_float newApproximation = functor(approximation);

			if (NAMESPACE_FOUNDATION::isCloseEnough(newApproximation, approximation))
				return newApproximation;

			approximation = newApproximation;

			maxOfInteration--;
		}

		return NAN;
	}

}