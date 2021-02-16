#ifndef MAT_HEADER
#define MAT_HEADER

#include "SpectrumPhysics.h"
#include <iomanip>
#include <Eigen/Eigenvalues> 

namespace NAMESPACE_PHYSICS
{
	class Mat
	{
	public:

		/// <summary>
		/// Check the matrix is Hessenberg Upper format
		/// All values under subdiagonal are Zero
		/// </summary>
		/// <param name="matrix">Input Square Matrix</param>
		/// <param name="columnLength">Dimension</param>
		/// <returns>True if the format is Hessenberg Upper</returns>
		static inline sp_bool isHessenbergUpper(sp_float* matrix, const sp_uint columnLength, const sp_float _epsilon = DefaultErrorMargin)
		{
			for (sp_uint column = ZERO_UINT; column < columnLength; column++)
				for (sp_uint row = column + TWO_UINT; row < columnLength; row++)
					if (!NAMESPACE_FOUNDATION::isCloseEnough(matrix[row * columnLength + column], ZERO_FLOAT, _epsilon))
						return false;

			return true;
		}

		/// <summary>
		/// Check the square matrix is symetric
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="columnLength">Dimension</param>
		/// <returns>True if the matrix is symetric orelse False</returns>
		static inline sp_bool isSymetric(sp_float* matrix, const sp_uint columnLength)
		{
			for (sp_uint row = ZERO_UINT; row < columnLength; row++)
				for (sp_uint column = row + ONE_UINT; column < columnLength; column++)
					if (NAMESPACE_FOUNDATION::isCloseEnough(matrix[row * columnLength + column], matrix[column * columnLength + row], ZERO_FLOAT))
						return false;
			
			return true;
		}

		/// <summary>
		/// Reduce a matrix to a Hessenberg Upper
		/// If the matrix is symetrix, it will reduce to a tridiagonal matrix
		/// </summary>
		/// <param name="matrix">Input Square Matrix</param>
		/// <param name="columnLength">Dimension</param>
		/// <param name="output">Hessenberg Matrix</param>
		static void hessenberg(sp_float* matrix, register const sp_uint columnLength, sp_float* output);

		/// <summary>
		/// Householder numerical method - Reduce a matrix from symetric to tridiagonal
		/// </summary>
		/// <param name="matrix">Input Square Matrix - This matrix has to be square</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="output">Tridiagonal Matrix</param>
		static void householder(sp_float* matrix, const register sp_uint columnLength, sp_float* output);

		static std::string toString(const sp_float* values, const sp_uint rowLength, const sp_int precision = 4)
		{
			std::string content;
			const sp_uint length = rowLength * rowLength;

			for (sp_uint i = ZERO_UINT; i < length; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << values[i];
				std::string numberAsString = stream.str();

				if (values[i] >= ZERO_FLOAT)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % rowLength == ZERO_UINT)
					content += END_OF_LINE;
			}

			return content;
		}

		static std::string toString(const sp_float* values, const sp_uint rowLength, const sp_uint columnLength, const sp_int precision = 4)
		{
			std::string content;
			const sp_uint length = rowLength * columnLength;

			for (sp_uint i = ZERO_UINT; i < length; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << values[i];
				std::string numberAsString = stream.str();

				if (values[i] >= ZERO_FLOAT)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % columnLength == ZERO_UINT)
					content += END_OF_LINE;
			}

			return content;
		}

		static void gaussianElimination(sp_float *matrix, const sp_int rowSize);

		/// <summary>
		/// Get the Homography Matrix (the transformation perspective matrix from a plane to another one)
		/// <summary>
		static Mat3 getPerspectiveTransform2D(const Vec2 sourcePoints[4], const Vec2 targetPoints[4] );

	};
	
}

#endif // !MAT_HEADER