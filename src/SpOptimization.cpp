#include "SpOptimization.h"

#include "SpLogger.h"

namespace NAMESPACE_PHYSICS
{
	
	sp_float SpOptimization::newton(sp_float approximation, sp_float firstDetivedFunction(sp_float), sp_float secondDerivedFunctor(sp_float), sp_float _epsilon, sp_int maxOfInteration)
	{
		while (maxOfInteration != ZERO_INT)
		{
			sp_float newApproximation = approximation - (firstDetivedFunction(approximation) / secondDerivedFunctor(approximation));

			if (NAMESPACE_FOUNDATION::isCloseEnough(newApproximation, approximation, _epsilon))
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

			if (NAMESPACE_FOUNDATION::isCloseEnough(f_x1, f_x3, _epsilon))
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

	void SpOptimization::simplex(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength, SpPair<sp_uint, sp_float>* output) const
	{
#define isSolutionUnbounded (pivotRowIndex == SP_UINT_MAX)
		sp_float* newMatrix = ALLOC_NEW_ARRAY(sp_float, rowLength * columnLength);
		std::memcpy(newMatrix, matrix, rowLength * columnLength * sizeof(sp_float));

		for (register sp_uint row = 0; row < rowLength - ONE_UINT; row++)
		{
#define slackVariableLength (columnLength - rowLength - 1u)
			output[row].key = slackVariableLength + row;
			output[row].value = newMatrix[row * columnLength + columnLength - 1u];
#undef slackVariableLength
		}

		SystemOfLinearEquations linearEquation;

		while (!isOptimum(newMatrix, rowLength, columnLength))
		{
			sp_uint pivotRowIndex = SP_UINT_MAX, pivotColumnIndex = SP_UINT_MAX;
			pivotSelector(newMatrix, rowLength, columnLength, &pivotColumnIndex, &pivotRowIndex);

			if (isSolutionUnbounded)
				break;

			linearEquation.pivot(newMatrix, rowLength, columnLength, pivotColumnIndex, pivotRowIndex);

			output[pivotRowIndex].key = pivotColumnIndex;
			for (sp_uint row = 0; row < rowLength - 1u; row++)
				output[row].value = newMatrix[row * columnLength + columnLength - 1u];

			//isFeasible(newMatrix, rowLength, columnLength)
		}

		ALLOC_RELEASE(newMatrix);
#undef isSolutionUnbounded
	}

}