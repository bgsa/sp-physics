#include "AlgorithmNewton.h"

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	T AlgorithmNewton<T>::solve(T approximation, T functor(T), T derivedFunctor(T), int maxOfInteration)
	{
		while (maxOfInteration != 0)
		{
			T newApproximation = approximation - (functor(approximation) / derivedFunctor(approximation));

			if (isCloseEnough(newApproximation, approximation))
				return newApproximation;

			approximation = newApproximation;

			maxOfInteration--;
		}

		return T(NAN);
	}

	template class AlgorithmNewton<sp_int>;
	template class AlgorithmNewton<sp_float>;
	template class AlgorithmNewton<sp_double>;
}