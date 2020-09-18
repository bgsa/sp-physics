#ifndef ALGORITHM_SECANT_HEADER
#define ALGORITHM_SECANT_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class AlgorithmSecant
	{
	public:

		///<summary>
		/// Find Zero in function using Secant Method: F(x) = 0
		///</summary>
		API_INTERFACE sp_float solve(sp_float approximation1, sp_float approximation2, sp_float functor(sp_float), sp_int maxOfInteration = 100);

	};

}

#endif // !ALGORITHM_SECANT_HEADER