#ifndef ALGORITHM_BISECTION_HEADER
#define ALGORITHM_BISECTION_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS 
{

	class AlgorithmBisection
	{
	public:
	
		///<summary>
		/// Find Zero in function using Bisection Method: F(x) = 0
		/// The conversion is assured, but requires many interations (not so eficient compared with "Fixed Point" or "Newton" Method
		///</summary>
		API_INTERFACE sp_float solve(sp_float intervalA, sp_float intervalB, sp_float functor(sp_float), sp_int maxOfInteration = 100);

		///<summary>
		///Returns the maximum number of iteration to solve the function
		///</summary>
		API_INTERFACE sp_int maxNumberOfIteration();

	};

}

#endif // ALGORITHM_BISECTION_HEADER