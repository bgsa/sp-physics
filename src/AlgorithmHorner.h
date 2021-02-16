#ifndef ALGORITHM_HORNER_HEADER
#define ALGORITHM_HORNER_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class AlgorithmHorner
	{
	public:

		///<summary>
		/// Divides a plynomial by a factor
		/// x0 is the first value of coeficient values
		/// x0 can be found using Newton, Secant, Bissection methods
		/// Returns an array that contains the polynimal divided with 1 degree less
		///</summary>
		API_INTERFACE void polynomialDivision(sp_float x0, sp_float* polynomial, sp_uint polynomialDegree, sp_float* output);

		/// <summary>
		/// Find roots (zeros) from a plynomial P(x) = 0
		/// </summary>
		/// <param name="x0">First value of coeficient values. x0 can be found using Newton, Secant, Bissection methods.</param>
		/// <param name="polynomial">Polyname</param>
		/// <param name="polynomialDegree">Degree</param>
		/// <param name="output">Values</param>
		/// <returns>Returns an array that contains the polynimal divided with 1 degree lesser</returns>
		API_INTERFACE void findRoots(sp_float x0, sp_float* polynomial, sp_uint polynomialDegree, sp_float* output);

	};

}

#endif // ALGORITHM_HORNER_HEADER