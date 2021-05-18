#ifndef SYSTEM_OF_LINEAR_EQUATIONS_HEADER
#define SYSTEM_OF_LINEAR_EQUATIONS_HEADER

#include "SpectrumPhysics.h"
#include "SpLogger.h"
#include <iomanip>

namespace NAMESPACE_PHYSICS
{
	class SystemOfLinearEquations
	{
	public:

		API_INTERFACE std::string printMatrix(sp_float* matrix, sp_uint rowLength, sp_uint columnLength, sp_int precision = 4) const
		{
			sp_uint total = rowLength * columnLength;
			std::string content;

			for (sp_uint i = 0; i < total; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << matrix[i];
				std::string numberAsString = stream.str();

				sp_bool isPositive = matrix[i] >= 0.0f;

				if (isPositive)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % columnLength == 0)
					content += END_OF_LINE;
			}

			return content;
		}

		/// <summary>
		/// Divide a row of matrix by a value.
		/// (Optional) Starting from an index column
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="colLength">Column Length</param>
		/// <param name="lineIndex">Line Index</param>
		/// <param name="value">Divisor</param>
		/// <param name="startingFromColumnIndex">Starting from this index</param>
		API_INTERFACE inline void divideRow(sp_float* matrix, const sp_uint colLength, const sp_uint lineIndex, const sp_float value, const sp_uint startingFromColumnIndex = ZERO_UINT)
		{
			const sp_float newValue = NAMESPACE_FOUNDATION::div(ONE_FLOAT, value);

			for (register sp_uint column = startingFromColumnIndex; column < colLength; column++)
				matrix[lineIndex * colLength + column] *= newValue;
		}

		/// <summary>
		/// Find the max value in a matrix column
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="columnIndex">Index of COlumn to be searched</param>
		/// <param name="maxValueIndex">Row Index</param>
		/// <param name="startingFromRowIndex">Starting from Row Index</param>
		/// <return>Max Value</return>
		API_INTERFACE inline sp_float maxValueInColumn(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength, const sp_uint columnIndex, sp_uint* maxValueIndex, const sp_uint startingFromRowIndex = ZERO_UINT)
		{
			sp_float maxValue = matrix[startingFromRowIndex * columnLength + columnIndex];
			maxValueIndex[0] = startingFromRowIndex;

			for (register sp_uint line = startingFromRowIndex; line < rowLength; line++)
				if (sp_abs(matrix[line * columnLength + columnIndex]) > sp_abs(maxValue))
				{
					maxValue = matrix[line * columnLength + columnIndex];
					maxValueIndex[0] = line;
				}

			return maxValue;
		}

		/// <summary>
		/// Swap/exchange two lines in the matrix
		/// </summary>
		/// <param name="matrix"></param>
		/// <param name="colLength">Columns Length</param>
		/// <param name="line1Index">Line 1 Index</param>
		/// <param name="line2Index">Line 2 Index</param>
		API_INTERFACE inline void swapLines(sp_float* matrix, const sp_uint rowLength, const sp_uint colLength, const sp_uint line1Index, const sp_uint line2Index) const
		{
			const sp_size rowSize = SIZEOF_FLOAT * colLength;

			sp_float* tempStorage = ALLOC_NEW_ARRAY(sp_float, colLength);

			std::memcpy(tempStorage, &matrix[line1Index * colLength], rowSize);
			std::memcpy(&matrix[line1Index * colLength], &matrix[line2Index * colLength], rowSize);
			std::memcpy(&matrix[line2Index * colLength], tempStorage, rowSize);

			ALLOC_RELEASE(tempStorage);
		}

		/// <summary>
		/// Pivot the matrix given the pivot index
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="pivotColumnIndex">Pivot Column Index</param>
		/// <param name="pivotRowIndex">Pivot Row Index</param>
		API_INTERFACE void pivot(sp_float* matrix, const sp_uint rowLength, register const sp_uint columnLength, const sp_uint pivotColumnIndex, const sp_uint pivotRowIndex) const;

		/// <summary>
		/// Solve a system of linear equations NxN
		/// Uses Gauss Elimination with Partial Pivoting in Rows
		/// </summary>
		/// <param name="matrix">Input Matrix</param>
		/// <param name="rowLength">Row Length</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="output">Values</param>
		/// <returns>True if there is any solution orelse False</returns>
		API_INTERFACE sp_bool solve(sp_float* matrix, const sp_uint rowLength, 
			const sp_uint columnLength, sp_float* output);

		/// <summary>
		/// Change the matrix to a cannonical form
		/// </summary>
		API_INTERFACE inline void canonicalForm(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength) const
		{
			for (register sp_uint pivotIndex = ZERO_UINT; pivotIndex < rowLength; pivotIndex++)
			{
				if (matrix[pivotIndex * columnLength + pivotIndex] != ZERO_FLOAT)
					pivot(matrix, rowLength, columnLength, pivotIndex, pivotIndex);
				else
				{
					swapLines(matrix, rowLength, columnLength, pivotIndex, pivotIndex + 1u);
					pivotIndex--;

					sp_log_debug1s(printMatrix(matrix, rowLength, columnLength).c_str());
					sp_log_newline();
				}
			}
		}

		/// <summary>
		/// Get the line equation in 2 points for 2D space
		/// The result is a vector with 3 components (a, b, c)
		/// Line equation: "ax + bx + c = 0"
		/// </summary>
		API_INTERFACE static Vec3 getLineEquation(const Vec2& point1, const Vec2& point2);

		/// <summary>
		/// Get the line equation in 3 points for 2D space
		/// The result is a vector with 1 components (a, b, c, d)
		/// Line equation: "a(x^2 + y) + bx + cy + d = 0"
		/// </summary>
		API_INTERFACE static Vec4 getCircleEquation(const Vec2& point1, const Vec2& point2, const Vec2& point3);

	};
}

#endif // !SYSTEM_OF_LINEAR_EQUATIONS_HEADER