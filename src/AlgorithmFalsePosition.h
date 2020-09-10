#ifndef ALGORITHM_FALSE_POSITION_HEADER
#define ALGORITHM_FALSE_POSITION_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class AlgorithmFalsePosition
	{
	public:

		///<summary>
		/// Find Zero in function using False Position Method: F(x) = 0
		///</summary>
		API_INTERFACE sp_float solve(sp_float approximation1, sp_float approximation2, sp_float functor(sp_float), sp_int maxOfInteration = 100);

	};

}

#endif // !ALGORITHM_FALSE_POSITION_HEADER