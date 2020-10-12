#include "SpOptimization.h"

namespace NAMESPACE_PHYSICS
{
	const sp_float SP_AUREA = (sqrtf(5.0f) - ONE_FLOAT) / TWO_FLOAT;

	sp_float SpOptimization::interpolation(sp_objective_function function, sp_optimization_type_function optimizationType,
		sp_float x0, sp_float x1, sp_float x2, sp_float* output_fx, const sp_float _epsilon) const
	{
		sp_float f_x0 = function(x0);
		sp_float f_x1 = function(x1);
		sp_float f_x2 = function(x2);
		sp_float x3, f_x3;

		while (true)
		{
			x3 = (f_x0 * (x1 * x1 - x2 * x2) + f_x1 * (x2 * x2 - x0 * x0) + f_x2 * (x0 * x0 - x1 * x1))
				/ (TWO_FLOAT * f_x0 * (x1 - x2) + TWO_FLOAT * f_x1 * (x2 - x0) + TWO_FLOAT * f_x2 * (x0 - x1));

			f_x3 = function(x3);

			if (optimizationType(f_x3, f_x1)) // x0 descartado
			{
				x0 = x1;
				f_x0 = f_x1;
			}
			else // x2 descartado
			{
				x2 = x1;
				f_x2 = f_x1;
			}

			if (isCloseEnough(f_x1, f_x3, _epsilon))
				break;

			x1 = x3;
			f_x1 = f_x3;
		}

		output_fx[0] = f_x3;
		return x3;
	}

	sp_float SpOptimization::aurea(sp_objective_function function, sp_optimization_type_function optimizationType,
		const sp_float minInterval, const sp_float maxInterval, 
		sp_float* output_fx, const sp_float _epsilon) const
	{
		sp_float xLower = minInterval;
		sp_float xUpper = maxInterval;

		sp_float d = (maxInterval - minInterval) * SP_AUREA;
		sp_float x1 = minInterval + d;
		sp_float x2 = maxInterval - d;

		sp_float f_x1 = function(x1);
		sp_float f_x2 = function(x2);

		sp_float xResult, fx;

		if (optimizationType(f_x1, f_x2))
		{
			xResult = x1;
			fx = f_x1;
		}
		else
		{
			xResult = x2;
			fx = f_x2;
		}
	
		while (true)
		{
			d = SP_AUREA * d;

			if (optimizationType(f_x1, f_x2))
			{
				xLower = x2;
				x2 = x1;
				x1 = xLower + d;
				f_x2 = f_x1;
				f_x1 = function(x1);
			}
			else
			{
				xUpper = x1;
				x1 = x2;
				x2 = xUpper - d;
				f_x1 = f_x2;
				f_x2 = function(x2);
			}

			if (optimizationType(f_x1, f_x2))
			{
				xResult = x1;
				fx = f_x1;
			}
			else
			{
				xResult = x2;
				fx = f_x2;
			}

			if (d < _epsilon)
				break;

		}

		output_fx[0] = fx;
		return xResult;
	}
}