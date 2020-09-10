#include "AlgorithmBisection.h"

namespace NAMESPACE_PHYSICS
{

	sp_float AlgorithmBisection::solve(sp_float intervalA, sp_float intervalB, sp_float functor(sp_float), sp_int maxOfInteration)
	{
		sp_float midPoint = intervalA + ((intervalB - intervalA) * HALF_FLOAT);
		sp_float valueMidPoint = functor(midPoint);
		sp_float valueA = functor(intervalA);
		
		while (maxOfInteration != 0)
		{
			if (isCloseEnough(valueMidPoint, ZERO_FLOAT))
				return midPoint;

			if (sign(valueA) * sign(valueMidPoint) > 0)
			{
				intervalA = midPoint;
				valueA = valueMidPoint;
			}
			else
				intervalB = midPoint;

			midPoint = intervalA + ((intervalB - intervalA) * HALF_FLOAT);
			valueMidPoint = functor(midPoint);

			maxOfInteration--;
		}

		return NAN;
	}

	sp_int AlgorithmBisection::maxNumberOfIteration()
	{
		const sp_double log2 = log10(2);
		const sp_double v = log10(DefaultErrorMargin);
		
		return (sp_int) ceil(-v / log2);
	}
}