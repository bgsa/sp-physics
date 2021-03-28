#include "SystemOfLinearEquations.h"

namespace NAMESPACE_PHYSICS
{

	sp_bool SystemOfLinearEquations::solve(sp_float* matrix, const sp_uint rowLength, const sp_uint columnLength, sp_float* output)
	{
		sp_assert(rowLength + 1u == columnLength, "MatrixFormatException"); // matrix must be NxN
		std::memset(output, ZERO_INT, SIZEOF_FLOAT * (columnLength - ONE_UINT));

#define pivotColumnIndex row
		sp_float* upperMatrix = ALLOC_ARRAY(sp_float, rowLength * columnLength);
		std::memcpy(upperMatrix, matrix, rowLength * columnLength * SIZEOF_FLOAT);

		for (register sp_uint row = 0u; row < rowLength - 1u; row++)
		{
			sp_log_debug1snl(printMatrix(upperMatrix, rowLength, columnLength).c_str());

			sp_uint newPivotRowIndex;
			register sp_float pivot = maxValueInColumn(upperMatrix, rowLength, columnLength, pivotColumnIndex, &newPivotRowIndex, row);

			if (NAMESPACE_FOUNDATION::isCloseEnough(pivot, ZERO_FLOAT)) // system is undertemined!!
			{
				// output[newPivotRowIndex] can have any value (system undertemined)
				output[newPivotRowIndex] = ONE_FLOAT;
				continue;
			}

			if (row != newPivotRowIndex)
				swapLines(upperMatrix, rowLength, columnLength, row, newPivotRowIndex);
		
			sp_log_debug1snl(printMatrix(upperMatrix, rowLength, columnLength).c_str());

			divideRow(upperMatrix, columnLength, row, pivot, pivotColumnIndex);

			sp_log_debug1snl(printMatrix(upperMatrix, rowLength, columnLength).c_str());

			const sp_uint lineIndex = row * columnLength;
			for (sp_uint lowerLines = row + 1u; lowerLines < rowLength; lowerLines++)
			{
				const sp_float pivotOperator = -upperMatrix[lowerLines * columnLength + pivotColumnIndex];

				for (sp_uint column = 0; column < columnLength; column++)
					upperMatrix[lowerLines * columnLength + column] += pivotOperator * upperMatrix[lineIndex + column];
			}

			sp_log_debug1snl(printMatrix(upperMatrix, rowLength, columnLength).c_str());
		}

		sp_log_debug1snl(printMatrix(upperMatrix, rowLength, columnLength).c_str());

		for (sp_uint row = rowLength - 1u; row != SP_UINT_MAX; row--)
		{			
			for (sp_int column = columnLength - 2u; column != row; column--)
				output[row] += output[column] * upperMatrix[row * columnLength + column];
			
			output[row] 
				= NAMESPACE_FOUNDATION::isCloseEnough(upperMatrix[row * columnLength + row], ZERO_FLOAT)
				? ZERO_FLOAT
				: NAMESPACE_FOUNDATION::div((upperMatrix[row * columnLength + columnLength - ONE_UINT] - output[row])
					, upperMatrix[row * columnLength + row]);
		}
			
		ALLOC_RELEASE(upperMatrix);
		return true;
#undef pivotColumnIndex
	}

	void SystemOfLinearEquations::pivot(sp_float* matrix, const sp_uint rowLength, register const sp_uint columnLength, const sp_uint pivotColumnIndex, const sp_uint pivotRowIndex) const
	{
		sp_float pivotValue = ONE_FLOAT / matrix[pivotRowIndex * columnLength + pivotColumnIndex];

		for (register sp_uint column = 0u; column < columnLength; column++)
			matrix[pivotRowIndex * columnLength + column] *= pivotValue;

		for (register sp_uint row = ZERO_UINT; row < rowLength; row++)
			if (row != pivotRowIndex)
			{
				pivotValue = matrix[row * columnLength + pivotColumnIndex];

				for (register sp_uint column = ZERO_UINT; column < columnLength; column++)
					matrix[row * columnLength + column] -= pivotValue * matrix[pivotRowIndex * columnLength + column];
			}
	}

	Vec3 SystemOfLinearEquations::getLineEquation(const Vec2& point1, const Vec2& point2)
	{
		Mat3 matrix = {
			ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
			point1.x, point1.y, ONE_FLOAT,
			point2.x, point2.y, ONE_FLOAT
		};

		return Vec3(
			matrix.cofactorIJ(0, 0), 
			matrix.cofactorIJ(0, 1),
			matrix.cofactorIJ(0, 2)
		);
	}

	Vec4 SystemOfLinearEquations::getCircleEquation(const Vec2& point1, const Vec2& point2, const Vec2& point3)
	{
		const sp_float m21 = powf(point1.x, 2) + powf(point1.y, 2);
		const sp_float m31 = powf(point2.x, 2) + powf(point2.y, 2);
		const sp_float m41 = powf(point3.x, 2) + powf(point3.y, 2);

		Mat4 matrix = {
			ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
			m21, point1.x, point1.y, ONE_FLOAT,
			m31, point2.x, point2.y, ONE_FLOAT,
			m41, point3.x, point3.y, ONE_FLOAT
		};

		return Vec4(
			matrix.cofactorIJ(0, 0),
			matrix.cofactorIJ(0, 1), 
			matrix.cofactorIJ(0, 2),
			matrix.cofactorIJ(0, 3)
		);
	}
}