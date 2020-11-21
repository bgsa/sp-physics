#include "SystemOfLinearEquations.h"

namespace NAMESPACE_PHYSICS
{

	sp_float* SystemOfLinearEquations::solve(sp_float* matrix, sp_uint rowSize, sp_uint colSize)
	{
		sp_float* result = ALLOC_ARRAY(sp_float, colSize - 1);

		sp_float* upperMatrix = ALLOC_ARRAY(sp_float, rowSize * colSize);
		std::memcpy(upperMatrix, matrix, sizeof(upperMatrix) * rowSize * colSize);

		sp_uint pivotColumnIndex = 0;

		for (sp_uint line = 0; line < rowSize; line++)
		{
			sp_float pivot = upperMatrix[line * colSize + pivotColumnIndex];

			if (pivot == 0.0f) 
			{
				for (sp_uint i = line + 1; i < rowSize; i++)
				{
					pivot = upperMatrix[i * colSize + pivotColumnIndex];

					if (pivot != 0.0f) 
					{
						//troca linha
						for (sp_uint column = 0; column < colSize; column++)
						{
							sp_float temp = upperMatrix[line * colSize + column];
							upperMatrix[line * colSize + column] = upperMatrix[i * colSize + column];
							upperMatrix[i * colSize + column] = temp;
						}

						break;
					}
				}
			}			

			sp_float pivotOperator = 1 / pivot;

			for (sp_uint column = 0; column < colSize; column++)
				upperMatrix[line * colSize + column] *= pivotOperator;
				
			for (sp_uint lowerLines = line + 1; lowerLines < rowSize; lowerLines++)
			{
				pivot = upperMatrix[lowerLines * colSize + pivotColumnIndex];
				pivotOperator = -pivot;

				for (sp_uint column = 0; column < colSize; column++)
					upperMatrix[lowerLines * colSize + column] += pivotOperator * upperMatrix[line * colSize + column];
			}

			pivotColumnIndex++;
		}

		//string content = printMatrix(upperMatrix, rowSize, colSize);

		for (sp_int row = rowSize - 1; row >= 0; row--)
		{			
			result[row] = upperMatrix[row * colSize + colSize - 1];

			for (sp_int column = colSize - 1; column > row + 1; column--)
				result[row] -= upperMatrix[row * colSize + column - ONE_UINT] * result[column - ONE_UINT];
		}
			
		ALLOC_RELEASE(upperMatrix);
		return result;
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