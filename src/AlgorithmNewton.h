#ifndef ALGORITHM_NEWTON_HEADER
#define ALGORITHM_NEWTON_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	template <typename T>
	class AlgorithmNewton
	{
	public:

		///<summary>
		/// Find Zero in function using Newton Method: F(x) = 0
		///</summary>
		API_INTERFACE T solve(T approximation, T functor(T), T derivedFunctor(T), int maxOfInteration = 100);

	};

}

#endif // !ALGORITHM_NEWTON_HEADER