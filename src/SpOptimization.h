#ifndef SP_OPTIMIZATION_HEADER
#define SP_OPTIMIZATION_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	typedef sp_bool(*sp_optimization_type_function)(sp_float, sp_float);
	typedef sp_float(*sp_objective_function)(sp_float);

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
		/// Optimiza the maximum value for X in a function for only one variable
		/// </summary>
		/// <param name="function">Objective function to be optimized</param>
		/// <param name="optimizationType">Type of optimization (maximize or minimize)</param>
		/// <param name="minInterval">Min X interval</param>
		/// <param name="maxInterval">Max X interval</param>
		/// <param name="output_fx">Max value for X result value</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>X optimized for max fx value (returned in output_fx)</returns>
		API_INTERFACE sp_float aurea(sp_objective_function function, sp_optimization_type_function optimizationType,
			const sp_float minInterval, const sp_float maxInterval, sp_float* output_fx,
			const sp_float _epsilon = ERROR_MARGIN_PHYSIC) const;

		/// <summary>
		/// Optimiza the maximum value for X in a function for only one variable
		/// </summary>
		/// <param name="function">Objective function to be optimized</param>
		/// <param name="optimizationType">Type of optimization (maximize or minimize)</param>
		/// <param name="minInterval">Min X interval</param>
		/// <param name="maxInterval">Max X interval</param>
		/// <param name="output_fx">Max value for X result value</param>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>X optimized for max fx value (returned in output_fx)</returns>
		API_INTERFACE
			sp_float SpOptimization::interpolation(sp_objective_function function, sp_optimization_type_function optimizationType,
			sp_float x0, sp_float x1, sp_float x2, sp_float* output_fx, const sp_float _epsilon = DefaultErrorMargin) const;

	};
}

#endif // SP_OPTIMIZATION_HEADER