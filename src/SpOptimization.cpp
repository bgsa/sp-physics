#include "SpOptimization.h"

#include "SpLogger.h"

namespace NAMESPACE_PHYSICS
{
	
	sp_float SpOptimization::newton(sp_float approximation, sp_float firstDetivedFunction(sp_float), sp_float secondDerivedFunctor(sp_float), sp_float _epsilon, sp_int maxOfInteration)
	{
		while (maxOfInteration != ZERO_INT)
		{
			sp_float newApproximation = approximation - (firstDetivedFunction(approximation) / secondDerivedFunctor(approximation));

			if (isCloseEnough(newApproximation, approximation, _epsilon))
				return newApproximation;

			approximation = newApproximation;

			maxOfInteration--;
		}

		return SP_NOT_A_NUMBER;
	}

	sp_float SpOptimization::interpolation(sp_objective_function1D function, sp_optimization_type_function optimizationType,
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

			if (optimizationType(x3, x1)) // x0 descartado
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

	sp_float SpOptimization::goldenSectionSearch(sp_objective_function1D function, sp_optimization_type_function optimizationType,
		const sp_float minInterval, const sp_float maxInterval, 
		sp_float* output_fx, const sp_float _epsilon) const
	{
		sp_float xLower = minInterval;
		sp_float xUpper = maxInterval;

		sp_float d = (maxInterval - minInterval) * SP_GOLDEN_RATIO;
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
			d = SP_GOLDEN_RATIO * d;

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


	sp_float SpOptimization::scan(sp_objective_function2D function,
		const sp_float xmin, const sp_float xmax, const sp_float ymin, const sp_float ymax,
		sp_float xStepSize, sp_float yStepSize, sp_float* outputX, sp_float* outputY, sp_uint* iterations) const
	{
		sp_float maxF_xy = SP_FLOAT_MIN;
		sp_float maxX, maxY;
		sp_uint iteratonsCounter = ZERO_UINT;

		for (sp_float x = xmin; x < xmax; x+= xStepSize)
		{
			for (sp_float y = ymin; y < ymax; y += yStepSize)
			{
				sp_float f_xy = function(x, y);

				if (f_xy > maxF_xy)
				{
					maxF_xy = f_xy;
					maxX = x;
					maxY = y;
				}

				iteratonsCounter++;
			}
		}

		outputX[0] = maxX;
		outputY[0] = maxY;
		iterations[0] = iteratonsCounter;
		return maxF_xy;
	}

	sp_float SpOptimization::arbitrarySearch(sp_objective_function2D function,
		const sp_float xmin, const sp_float xmax, const sp_float ymin, const sp_float ymax, sp_uint iterations,
		sp_float* outputX, sp_float* outputY) const
	{
		sp_float maxF_xy = SP_FLOAT_MIN;
		sp_float maxX, maxY;
		sp_uint iteratonsCounter = ZERO_UINT;
		Randomizer randomizer(ZERO_FLOAT, ONE_FLOAT);

		for (sp_float i = 0; i < iterations; i++)
		{
			const sp_float x = xmin + ((xmax - xmin) * randomizer.randFloat());
			const sp_float y = ymin + ((ymax - ymin) * randomizer.randFloat());
			const sp_float f_xy = function(x, y);

			if (f_xy > maxF_xy)
			{
				maxF_xy = f_xy;
				maxX = x;
				maxY = y;
			}
		}

		outputX[0] = maxX;
		outputY[0] = maxY;
		return maxF_xy;
	}

	/*
	void SpOptimization::simplex(sp_float* function, sp_uint functionsLength, sp_float* constraints, sp_uint constraintsLength, sp_float* output) const
	{
		const sp_uint rowLength = constraintsLength + ONE_UINT;
		const sp_uint columnLength = functionsLength + constraintsLength + ONE_UINT;
		const sp_uint elementsLength = rowLength * columnLength;

		// step 1:
		// if any equation is '>=', change to '<=' by multiplying all values by -1
		
		sp_float* matrix = ALLOC_NEW_ARRAY(sp_float, elementsLength);
		std::memset(matrix, ZERO_INT, elementsLength * SIZEOF_FLOAT);

		for (sp_uint i = 0; i < constraintsLength; i++)
		{
			const sp_uint constraintIndex = functionsLength * i;
			const sp_uint rowIndex = i * columnLength;

			std::memcpy(&matrix[rowIndex], &constraints[constraintIndex], (functionsLength - ONE_UINT) * SIZEOF_FLOAT);
			matrix[rowIndex + columnLength - ONE_UINT] = constraints[constraintIndex + functionsLength - ONE_UINT];
			matrix[rowIndex + functionsLength + i - ONE_UINT] = ONE_FLOAT;
		}

		sp_float* functionInMatrix = &matrix[elementsLength - columnLength];

		for (sp_uint i = 0; i < functionsLength; i++)
			functionInMatrix[i] = -function[i];

		matrix[elementsLength - TWO_UINT] = ONE_FLOAT;
		matrix[elementsLength - ONE_UINT] = function[functionsLength - 1];

		std::string s = Mat::toString(matrix, rowLength, columnLength);
		sp_log_info1s(s.c_str());

		sp_float minimumFunctionValue = functionInMatrix[0];
		sp_uint minimumIndex = ZERO_UINT;
		for (sp_uint i = 1u; i < functionsLength - ONE_UINT; i++)
			if (functionInMatrix[i] < minimumFunctionValue)
			{
				minimumFunctionValue = functionInMatrix[i];
				minimumIndex = i;
			}

		if (minimumFunctionValue >= ZERO_FLOAT)
			return; // TERMINADO !!!!

		sp_float minimumRestrictionValue = matrix[columnLength - ONE_UINT];
		sp_uint minimumRestrictionIndex = ZERO_UINT;
		for (sp_uint i = 1u; i < functionsLength - ONE_UINT; i++)
			if (functionInMatrix[i * columnLength + columnLength - ONE_UINT] < minimumRestrictionValue)
			{
				minimumRestrictionValue = functionInMatrix[i];
				minimumRestrictionIndex = i;
			}
	}
	*/

}