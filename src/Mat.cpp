#include "Mat.h"

namespace NAMESPACE_PHYSICS
{
	void Mat::gaussianElimination(sp_float *matrix, const sp_int rowSize)
	{
		sp_int       i = 0;
		sp_int       j = 0;
		const sp_int m = rowSize - 1;

		while (i < m && j < rowSize)
		{
			sp_int maxi = i;
			
			for (sp_int k = i + 1; k < m; ++k)
				if (abs(matrix[k * rowSize + j]) > abs(matrix[maxi * rowSize + j]))
					maxi = k;

			if (matrix[maxi * rowSize + j] != 0.0f)
			{
				if (i != maxi)
					for (sp_int k = 0; k < rowSize; k++)
					{
						const sp_float aux = matrix[i * rowSize + k];
						matrix[i * rowSize + k] = matrix[maxi * rowSize + k];
						matrix[maxi * rowSize + k] = aux;
					}

				const sp_float aIj = matrix[i * rowSize + j];

				for (sp_int k = 0; k < rowSize; k++)
					matrix[i * rowSize + k] /= aIj;

				for (sp_int u = i + 1; u < m; u++)
				{
					const sp_float aUj = matrix[u * rowSize + j];

					for (int k = 0; k < rowSize; k++)
						matrix[u * rowSize + k] -= aUj * matrix[i * rowSize + k];
				}

				++i;
			}

			++j;
		}

		for (i = m - 2; i >= 0; --i)
			for (j = i + 1; j < rowSize - 1; j++)
				matrix[i * rowSize + m] -= matrix[i * rowSize + j] * matrix[j * rowSize + m];
	}

	Mat3 Mat::getPerspectiveTransform2D(const Vec2 sourcePoints[4], const Vec2 targetPoints[4])
	{
		sp_float homographyMatrix[8][9] =
		{
			{ -sourcePoints[0][0], -sourcePoints[0][1], -1, 0, 0, 0, sourcePoints[0][0] * targetPoints[0][0], sourcePoints[0][1] * targetPoints[0][0], -targetPoints[0][0] }, // h11
			{ 0, 0, 0, -sourcePoints[0][0], -sourcePoints[0][1], -1, sourcePoints[0][0] * targetPoints[0][1], sourcePoints[0][1] * targetPoints[0][1], -targetPoints[0][1] }, // h12
			{ -sourcePoints[1][0], -sourcePoints[1][1], -1, 0, 0, 0, sourcePoints[1][0] * targetPoints[1][0], sourcePoints[1][1] * targetPoints[1][0], -targetPoints[1][0] }, // h13
			{ 0, 0, 0, -sourcePoints[1][0], -sourcePoints[1][1], -1, sourcePoints[1][0] * targetPoints[1][1], sourcePoints[1][1] * targetPoints[1][1], -targetPoints[1][1] }, // h21
			{ -sourcePoints[2][0], -sourcePoints[2][1], -1, 0, 0, 0, sourcePoints[2][0] * targetPoints[2][0], sourcePoints[2][1] * targetPoints[2][0], -targetPoints[2][0] }, // h22
			{ 0, 0, 0, -sourcePoints[2][0], -sourcePoints[2][1], -1, sourcePoints[2][0] * targetPoints[2][1], sourcePoints[2][1] * targetPoints[2][1], -targetPoints[2][1] }, // h23
			{ -sourcePoints[3][0], -sourcePoints[3][1], -1, 0, 0, 0, sourcePoints[3][0] * targetPoints[3][0], sourcePoints[3][1] * targetPoints[3][0], -targetPoints[3][0] }, // h31
			{ 0, 0, 0, -sourcePoints[3][0], -sourcePoints[3][1], -1, sourcePoints[3][0] * targetPoints[3][1], sourcePoints[3][1] * targetPoints[3][1], -targetPoints[3][1] }, // h32
		};

		gaussianElimination(&homographyMatrix[0][0], 9);

		Mat3 result = Mat3(
			homographyMatrix[0][8], homographyMatrix[1][8], homographyMatrix[2][8],
			homographyMatrix[3][8], homographyMatrix[4][8], homographyMatrix[5][8],
			homographyMatrix[6][8], homographyMatrix[7][8], ONE_FLOAT
		);

		return result;
	}

}