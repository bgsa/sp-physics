#ifndef ALGORITHM_NEWTON_HEADER
#define ALGORITHM_NEWTON_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class AlgorithmNewton
	{
	public:

		///<summary>
		/// Find Zero in function using Newton Method: F(x) = 0
		///</summary>
		API_INTERFACE sp_float solve(sp_float approximation, sp_float functor(sp_float), sp_float derivedFunctor(sp_float), sp_int maxOfInteration = 100);

	};

}

#endif // !ALGORITHM_NEWTON_HEADER