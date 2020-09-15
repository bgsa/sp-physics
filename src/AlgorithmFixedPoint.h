#ifndef ALGORITHM_FIXED_POSITION_HEADER
#define ALGORITHM_FIXED_POSITION_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class AlgorithmFixedPoint
	{
	public:

		///<summary>
		/// Find the value of the function, given a value: F(x) = x
		/// It means: Given an X, the result of the function is X
		/// Find all values where x = y in the functions
		/// If the functions is optiomize using Newton concepts, it finds Zeros in function efficiently
		///</summary>
		API_INTERFACE sp_float solve(sp_float approximation, sp_float functor(sp_float), sp_int maxOfInteration = 100);

	};

}

#endif // ALGORITHM_FIXED_POSITION_HEADER