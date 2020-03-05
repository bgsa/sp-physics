#ifndef ALGORITHM_SECANT_HEADER
#define ALGORITHM_SECANT_HEADER

#include "OpenML.h"

namespace OpenML
{

	template <typename T>
	class AlgorithmSecant
	{
	public:

		///<summary>
		/// Find Zero in function using Secant Method: F(x) = 0
		///</summary>
		API_INTERFACE T solve(T approximation1, T approximation2, T functor(T), int maxOfInteration = 100);

	};

}

#endif // !ALGORITHM_SECANT_HEADER