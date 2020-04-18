#ifndef ALGORITHM_BISECTION_HEADER
#define ALGORITHM_BISECTION_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS 
{

	template <typename T>
	class AlgorithmBisection
	{
	public:
	
		///<summary>
		/// Find Zero in function using Bisection Method: F(x) = 0
		/// The conversion is assured, but requires many interations (not so eficient compared with "Fixed Point" or "Newton" Method
		///</summary>
		API_INTERFACE T solve(T intervalA, T intervalB, T functor(T), int maxOfInteration = 100);

		///<summary>
		///Returns the maximum number of iteration to solve the function
		///</summary>
		API_INTERFACE int maxNumberOfIteration();

	};

}

#endif // ALGORITHM_BISECTION_HEADER