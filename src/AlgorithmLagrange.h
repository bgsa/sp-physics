#ifndef ALGORITHM_LAGRANGE_HEADER
#define ALGORITHM_LAGRANGE_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	class AlgorithmLagrange
	{
	public:

		///<summary>
		///Find the value "y" from: F(x) = y where F(x) are function points
		///This method approximate the function by a polynomial where the function F(x) is unknown, 
		///so points (x, F(x)) of the function are given
		///</summary>
		API_INTERFACE sp_float polynomialApproximation(Vec2* points, sp_uint pointsCount, sp_float x);

	};

}

#endif // ALGORITHM_LAGRANGE_HEADER