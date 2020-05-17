#include "AlgorithmLagrange.h"

namespace NAMESPACE_PHYSICS
{

	sp_float AlgorithmLagrange::polynomialApproximation(Vec2* points, sp_uint pointsCount, sp_float x)
	{
		sp_float result = ZERO_FLOAT;

		for (sp_uint i = 0; i < pointsCount; i++)
		{
			sp_float li = 1.0f;

			for (sp_uint j = 0; j < pointsCount; j++)
			{
				if (i == j)
					continue;

				li *= (x - points[j][0]) / (points[i][0] - points[j][0]);			
			}

			result += points[i][1] * li;
		}

		return result;
	}

}