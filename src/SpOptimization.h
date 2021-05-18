#ifndef SP_OPTIMIZATION_HEADER
#define SP_OPTIMIZATION_HEADER

#include "SpectrumPhysics.h"
#include "SystemOfLinearEquations.h"
#include "Randomizer.h"
#include "SpPair.h"

namespace NAMESPACE_PHYSICS
{
	const sp_float SP_GOLDEN_RATIO = (sp_sqrt(5.0f) - ONE_FLOAT) / TWO_FLOAT;

	typedef sp_bool(*sp_optimization_type_function)(sp_float, sp_float);
	typedef sp_float(*sp_objective_function1D)(sp_float);
	typedef sp_float(*sp_objective_function2D)(sp_float, sp_float);

	API_INTERFACE inline sp_bool sp_optimization_type_maximize(sp_float x1, sp_float x2)
	{
		return x1 > x2;
	}

	API_INTERFACE inline sp_bool sp_optimization_type_minimize(sp_float x1, sp_float x2)
	{
		return x1 < x2;
	}

	class SpOptimization
	{
	public:

		/// <summary>
		/// Optimize the maximum value for X in function for only one variable
		/// </summary>
		/// <param name="function">Objective function to be optimized</param>
		/// <param name="optimizationType">Type of optimization (maximize or minimize)</param>
		/// <param name="minInterval">Min X interval</param>
		/// <param name="maxInterval">Max X interval</param>
		/// <param name="output_fx">Max value for X result value</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>X optimized for max fx value (returned in output_fx)</returns>
		API_INTERFACE sp_float goldenSectionSearch(sp_objective_function1D function, sp_optimization_type_function optimizationType,
			const sp_float minInterval, const sp_float maxInterval, sp_float* output_fx,
			const sp_float _epsilon = ERROR_MARGIN_PHYSIC) const;

		/// <summary>
		/// Optimize the maximum value for X in function for only one variable
		/// </summary>
		/// <param name="function">Objective function to be optimized</param>
		/// <param name="optimizationType">Type of optimization (maximize or minimize)</param>
		/// <param name="minInterval">Min X interval</param>
		/// <param name="maxInterval">Max X interval</param>
		/// <param name="output_fx">Max value for X result value</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>X optimized for max fx value (returned in output_fx)</returns>
		API_INTERFACE
			sp_float interpolation(sp_objective_function1D function, sp_optimization_type_function optimizationType,
			sp_float x0, sp_float x1, sp_float x2, sp_float* output_fx, const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Optimize the minimum value fox X in function for only one variable
		/// </summary>
		/// <param name="approximation">First approximation</param>
		/// <param name="firstDetivedFunction">First derived of objective function</param>
		/// <param name="secondDerivedFunctor">Second derived of objective function</param>
		/// <param name="maxOfInteration">Maximum of iteration</param>
		/// <returns>X value for minumum F(X)</returns>
		API_INTERFACE sp_float newton(sp_float approximation, sp_float firstDetivedFunction(sp_float), sp_float secondDerivedFunctor(sp_float), sp_float _epsilon = DefaultErrorMargin, sp_int maxOfInteration = 100);

		/// <summary>
		/// Scan all values for (x, y) in function given the step size
		/// </summary>
		/// <param name="function">Objective function</param>
		/// <param name="xmin">Minimum value for x</param>
		/// <param name="xmax">Mazimum value for x</param>
		/// <param name="ymin">Minimum value for y</param>
		/// <param name="ymax">Maximum value for y</param>
		/// <param name="xStepSize">Step size of X axis</param>
		/// <param name="yStepSize">Step size of Y axis</param>
		/// <param name="outputX">Maximum function value for X</param>
		/// <param name="outputY">Maximum function value for Y</param>
		/// <param name="iterations">Iterations count</param>
		/// <returns>Maximum value of the function for (outputX, outputY) values</returns>
		API_INTERFACE sp_float scan(sp_objective_function2D function,
			const sp_float xmin, const sp_float xmax, const sp_float ymin, const sp_float ymax,
			sp_float xStepSize, sp_float yStepSize, sp_float* outputX, sp_float* outputY, sp_uint* iterations) const;

		/// <summary>
		/// Randomize values for (x, y) function in order to maximize or minimize the function
		/// </summary>
		/// <param name="function">Objective function</param>
		/// <param name="xmin">Minimum value for x function parameter</param>
		/// <param name="xmax">Maximum value for x function parameter</param>
		/// <param name="ymin">Minimum value for y function parameter</param>
		/// <param name="ymax">Maximum value for y function parameter</param>
		/// <param name="iterations">Iterations count</param>
		/// <param name="outputX">Optimized x function parameter</param>
		/// <param name="outputY">Optimized y function parameter</param>
		/// <returns>Maximum function value</returns>
		API_INTERFACE sp_float arbitrarySearch(sp_objective_function2D function,
			const sp_float xmin, const sp_float xmax, const sp_float ymin, const sp_float ymax, sp_uint iterations,
			sp_float* outputX, sp_float* outputY) const;

		/// <summary>
		/// Check if the simplex solution is feasible
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <returns>True if the solution is feasible or else false</returns>
		inline sp_bool isFeasible(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength) const
		{
			for (register sp_uint row = 0; row < rowLength; row++)
				if (matrix[row * columnLength + columnLength - ONE_UINT] < ZERO_FLOAT)
					return false;

			return true;
		}

		/// <summary>
		/// Check if the simplex solution is global optimum
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <returns></returns>
		inline sp_bool isOptimum(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength) const
		{
			for (register sp_uint column = 0; column < columnLength; column++)
				if (matrix[(rowLength - 1u) * columnLength + column] < ZERO_FLOAT)
					return false;

			return true;
		}

		/// <summary>
		/// Select the pivot
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="pivotColumnIndex">Pivot Column Index Output</param>
		/// <param name="pivotRowIndex">Pivot Column Index Output</param>
		/// <returns>pivotColumnIndex, pivotRowIndex</returns>
		inline void pivotSelector(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength, sp_uint* pivotColumnIndex, sp_uint* pivotRowIndex) const
		{
			const sp_uint lastRow = (rowLength - 1u) * columnLength;
			register sp_float minimumValue = SP_FLOAT_MAX;

			for (register sp_uint column = 0u; column < columnLength; column++)
				if (matrix[lastRow + column] < minimumValue)
				{
					minimumValue = matrix[lastRow + column];
					pivotColumnIndex[0] = column;
				}

			minimumValue = SP_FLOAT_MAX;

			for (register sp_uint row = 0u; row < rowLength; row++)
			{
				if (matrix[row * columnLength + *pivotColumnIndex] == ZERO_FLOAT) // avoid zero division
					continue;

				const sp_float value = matrix[row * columnLength + columnLength - 1u] / matrix[row * columnLength + *pivotColumnIndex];
				if (value > ZERO_FLOAT && value < minimumValue)
				{
					minimumValue = value;
					pivotRowIndex[0] = row;
				}
			}
		}

		/// <summary>
		/// Check if the simplex solution is unbounded
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="pivotColumnIndex">Pivot Column Index</param>
		/// <returns>True if the solution is unbounded or else false</returns>
		inline sp_bool isUnbouded(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength, const sp_uint pivotColumnIndex) const
		{
			for (register sp_uint row = 0; row < rowLength; row++)
				if (matrix[row * columnLength + pivotColumnIndex] > ZERO_FLOAT)
					return false;

			return true;
		}

		/// <summary>
		/// Minimize a linear function in format Ax=b
		/// </summary>
		/// <param name="matrix">Function + Constraints</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="output">Pair[VariableIndex, VariableValue] where VariableIndex = x1, x4, xIndex</param>
		/// <returns>output</returns>
		API_INTERFACE void simplex(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength, SpPair<sp_uint, sp_float>* output) const;

	};
}

#endif // SP_OPTIMIZATION_HEADER