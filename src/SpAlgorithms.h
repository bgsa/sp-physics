#ifndef SP_ALGORITHMS_HEADER
#define SP_ALGORITHMS_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{

	/// <summary>
	/// Solve linear equation Ax=b using Jacobi iterative method
	/// Note: Gauss-Seidel method is may faster
	/// </summary>
	/// <param name="A">Coeficients Matrix A</param>
	/// <param name="b"></param>
	/// <param name="rowLength">Rows of matrix A</param>
	/// <param name="columnLength">Columns of matrix A</param>
	/// <param name="initialApproximation">Initial approximation</param>
	/// <param name="output">Result x (Ax=b)</param>
	/// <param name="_epsilon">Error margin</param>
	/// <returns>void</returns>
	API_INTERFACE void jacobi(sp_float* A, sp_float* b, sp_uint rowLength, sp_uint columnLength, sp_float* approximation, sp_float* output, const sp_float _epsilon = ERROR_MARGIN_PHYSIC);

	/// <summary>
	/// Solve linear equation Ax=b using Gauss-Seidel iterative method
	/// Note: Jacobi method is may slower
	/// </summary>
	/// <param name="A">Coeficients Matrix A</param>
	/// <param name="b"></param>
	/// <param name="rowLength">Rows of matrix A</param>
	/// <param name="columnLength">Columns of matrix A</param>
	/// <param name="initialApproximation">Initial approximation</param>
	/// <param name="output">Result x (Ax=b)</param>
	/// <param name="_epsilon">Error margin</param>
	/// <returns>void</returns>
	API_INTERFACE void gaussSeidel(sp_float* A, sp_float* b, sp_uint rowLength, sp_uint columnLength, sp_float* approximation, sp_float* output, const sp_float _epsilon = ERROR_MARGIN_PHYSIC);

}

#endif // SP_ALGORITHMS_HEADER