#include "AlgorithmFixedPoint.h"

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	T AlgorithmFixedPoint<T>::solve(T approximation, T functor(T), int maxOfInteration)
	{	
		while (maxOfInteration != 0)
		{
			T newApproximation = functor(approximation);

			if (isCloseEnough(newApproximation, approximation))
				return newApproximation;

			approximation = newApproximation;

			maxOfInteration--;
		}

		return T(NAN);
	}

	template class AlgorithmFixedPoint<int>;
	template class AlgorithmFixedPoint<float>;
	template class AlgorithmFixedPoint<double>;
}