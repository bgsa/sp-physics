#ifndef ALGORITHM_STEFFESEN_HEADER
#define ALGORITHM_STEFFESEN_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class AlgorithmSteffensen
	{
	public:

		///<summary>
		/// Find Zero in function using Steffensen Method
		/// This method accelerate the conversion from linear to quadratic conversion
		/// Ex.: Convert the polynomial to fixed point method and use Seteffensen Method to solve
		///</summary>
		API_INTERFACE sp_float solve(sp_float approximation1, sp_float functor(sp_float), sp_int maxOfInteration = 100);

	};

}

#endif // !ALGORITHM_STEFFESEN_HEADER