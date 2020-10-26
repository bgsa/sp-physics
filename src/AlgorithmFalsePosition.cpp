#include "AlgorithmFalsePosition.h"

namespace NAMESPACE_PHYSICS
{
	sp_float AlgorithmFalsePosition::solve(sp_float approximation1, sp_float approximation2, sp_float functor(sp_float), sp_int maxOfInteration)
	{
		sp_float valueApproximation1 = functor(approximation1);
		sp_float valueApproximation2 = functor(approximation2);

		while (maxOfInteration != 0)
		{
			sp_float newApproximation = approximation2 - valueApproximation2 * (approximation2 - approximation1) / (valueApproximation2 - valueApproximation1);

			if (NAMESPACE_FOUNDATION::isCloseEnough(newApproximation - approximation2, ZERO_FLOAT))
				return newApproximation;

			sp_float newValueApproximation = functor(newApproximation);

			if (sign(newValueApproximation) * sign(valueApproximation2) < 0)
			{
				approximation1 = approximation2;
				valueApproximation1 = valueApproximation2;
			}

			approximation2 = newApproximation;
			valueApproximation2 = newValueApproximation;

			maxOfInteration--;
		}

		return sp_float(NAN);
	}
}