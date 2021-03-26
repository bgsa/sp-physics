#include "Mat.h"

namespace NAMESPACE_PHYSICS
{

	void Mat::svd(Mat& s, Mat& v, Mat& d) const
	{

	}

	void multiply(const Mat& a, const Mat& b, Mat& output)
	{
		sp_assert(output.length() == a.rows() * b.columns(), "InvalidArgumentException");
		sp_assert(a.rows() == b.columns() && b.rows() == a.columns(), "InvalidOperationException");

		std::memset(output, 0, sizeof(sp_float) * output.length());

		for (register sp_uint row = 0u; row < a.rows(); row++)
			for (register sp_uint column = 0u; column < b.columns(); column++)
			{
				const sp_uint outputIndex = row * a.rows() + column;
				const sp_uint rowIndex = row * a.columns();

				for (register sp_uint k = 0u; k < a.columns(); k++)
					output[outputIndex] += a[rowIndex + k] * b[k * b.columns() + column];
			}
	}

	void Mat::hessenberg(sp_float* matrix, const sp_uint columnLength, sp_float* output)
	{
		Eigen::MatrixXf m = Eigen::Map<Eigen::MatrixXf>(matrix, columnLength, columnLength);

		Eigen::HessenbergDecomposition<Eigen::MatrixXcf> hd(columnLength);
		hd.compute(m);

		Eigen::MatrixXf e = hd.packedMatrix().real();

		std::memcpy(output, e.data(), columnLength * columnLength * SIZEOF_FLOAT);

		/*
#define n columnLength
		sp_float* v = ALLOC_ARRAY(sp_float, n);
		sp_float* y = ALLOC_ARRAY(sp_float, n);
		sp_float* u = ALLOC_ARRAY(sp_float, n);
		sp_float* z = ALLOC_ARRAY(sp_float, n);

		sp_float* m = ALLOC_ARRAY(sp_float, n * n);
		std::memcpy(m, matrix, SIZEOF_FLOAT * n * n);

		for (register sp_uint k = ZERO_FLOAT; k < n - 2u; k++)
		{
			sp_float q = ZERO_FLOAT;

			for (sp_uint j = k + 1u; j < n; j++)
				q += (m[j * columnLength + k] * m[j * columnLength + k]);

			sp_float temp = m[(k + 1) * columnLength + k];
			sp_float alpha;

			if (NAMESPACE_FOUNDATION::isCloseEnough(temp, ZERO_FLOAT))
				alpha = -sqrtf(q);
			else
				alpha = -((sqrtf(q) * temp) / fabsf(temp));

			sp_float rqs = (alpha * alpha) - (alpha * temp);

			std::memset(v, ZERO_INT, SIZEOF_FLOAT * n);
			v[k + 1u] = temp - alpha;

			for (sp_uint j = k + 2u; j < n; j++)
				v[j] = m[j * columnLength + k];

			// step 6:
			std::memset(u, ZERO_INT, SIZEOF_FLOAT * n);
			std::memset(y, ZERO_INT, SIZEOF_FLOAT * n);
			for (sp_uint j = ZERO_UINT; j < n; j++)
			{
				temp = ZERO_FLOAT;
				sp_float temp2 = ZERO_FLOAT;

				for (sp_uint i = k + 1u; i < n; i++)
				{
					temp += m[j * columnLength + i] * v[i];
					temp2 += m[i * columnLength + j] * v[i];
				}

				u[j] = div(temp, rqs);
				y[j] = div(temp2, rqs);
			}

			// step 7:
			sp_float prod = ZERO_FLOAT;
			for (sp_uint i = k + 1u; i < n; i++)
				prod += v[i] * u[i];

			// step 8:
			std::memset(z, ZERO_INT, SIZEOF_FLOAT * n);
			for (sp_uint j = ZERO_UINT; j < n; j++)
				z[j] = u[j] - div(prod, rqs) * v[j];

			std::memcpy(output, m, SIZEOF_FLOAT * n * n);

			// step 9 (10, 11):
			for (register sp_uint l = k + 1u; l < n; l++)
			{
				for (sp_uint j = ZERO_UINT; j <= k; j++)
				{
					output[j * columnLength + l] = m[j * columnLength + l] - z[j] * v[l];
					output[l * columnLength + j] = m[l * columnLength + j] - y[j] * v[l];
				}

				for (sp_uint j = k + ONE_UINT; j < n; j++)
					output[j * columnLength + l]
					= m[j * columnLength + l] - z[j] * v[l] - y[l] * v[j];
			}

			// step 12:
			output[n * n - 1u] = m[n * n - 1u] - 2.0f * v[n - 1u] * z[n - 1u];

			// step 13:
			for (sp_uint j = k + 2u; j < n; j++)
				output[k * columnLength + j] = output[j * columnLength + k] = ZERO_FLOAT;

			// step 14:
			output[(k + 1) * columnLength + k] = m[(k + 1) * columnLength + k] - v[k + 1u] * z[k];
			output[k * columnLength + k + 1u] = output[(k + 1) * columnLength + k];

			std::memcpy(m, output, sizeof(SIZEOF_FLOAT) * n * n);
		}

		ALLOC_RELEASE(v);
#undef n
*/

		sp_assert(isHessenbergUpper(output, columnLength), "InvalidOperationException");
	}

	void Mat::householder(sp_float* matrix, const register sp_uint columnLength, sp_float* output)
	{
		sp_assert(NAMESPACE_PHYSICS::isSymmetric(matrix, columnLength), "InvalidArgumentException");

#define n columnLength
		sp_float* v = ALLOC_ARRAY(sp_float, n);
		sp_float* u = ALLOC_ARRAY(sp_float, n);
		sp_float* z = ALLOC_ARRAY(sp_float, n);

		sp_float* m = ALLOC_ARRAY(sp_float, n * n);
		std::memcpy(m, matrix, SIZEOF_FLOAT * n * n);

		for (register sp_uint k = ZERO_UINT; k < n - 2u; k++)
		{
			sp_float q = ZERO_FLOAT;

			for (sp_uint j = k + 1u; j < n; j++)
				q += (m[j * columnLength + k] * m[j * columnLength + k]);

			sp_float temp = m[(k + 1) * columnLength + k];
			sp_float alpha;

			if (NAMESPACE_FOUNDATION::isCloseEnough(temp, ZERO_FLOAT))
				alpha = -sqrtf(q);
			else
				alpha = -((sqrtf(q) * temp) / fabsf(temp));

			sp_float rqs = (alpha * alpha) - (alpha * temp);

			std::memset(v, ZERO_INT, SIZEOF_FLOAT * n);
			v[k + 1u] = temp - alpha;

			for (sp_uint j = k + 2u; j < n; j++)
				v[j] = m[j * columnLength + k];

			// step 6:
			std::memset(u, ZERO_INT, SIZEOF_FLOAT * n);
			for (sp_uint j = k; j < n; j++)
			{
				temp = ZERO_FLOAT;

				for (sp_uint i = k + 1u; i < n; i++)
					temp += m[j * columnLength + i] * v[i];

				u[j] = div(temp, rqs);
			}

			// step 7:
			sp_float prod = ZERO_FLOAT;
			for (sp_uint i = k + 1u; i < n; i++)
				prod += v[i] * u[i];

			// step 8:
			std::memset(z, ZERO_INT, SIZEOF_FLOAT * n);
			for (sp_uint j = k; j < n; j++)
				z[j] = u[j] - div(prod, (2.0f * rqs)) * v[j];

			std::memcpy(output, m, SIZEOF_FLOAT * n * n);

			// step 9 (10, 11):
			for (register sp_uint l = k + 1u; l < n - 1u; l++)
			{

				for (sp_uint j = l + 1u; j < n; j++)
					output[j * columnLength + l]
					= output[l * columnLength + j]
					= m[j * columnLength + l] - v[l] * z[j] - v[j] * z[l];

				output[l * columnLength + l] = m[l * columnLength + l] - 2.0f * v[l] * z[l];
			}

			// step 12:
			output[n * n - 1u] = m[n * n - 1u] - 2.0f * v[n - 1u] * z[n - 1u];

			// step 13:
			for (sp_uint j = k + 2u; j < n; j++)
				output[k * columnLength + j] = output[j * columnLength + k] = ZERO_FLOAT;

			// step 14:
			output[(k + 1) * columnLength + k] = m[(k + 1) * columnLength + k] - v[k + 1u] * z[k];
			output[k * columnLength + k + 1u] = output[(k + 1) * columnLength + k];

			std::memcpy(m, output, sizeof(SIZEOF_FLOAT) * n * n);
		}

		ALLOC_RELEASE(v);
#undef n
	}

	void Mat::gaussianElimination(sp_float *matrix, const sp_int rowSize)
	{
		sp_int       i = 0;
		sp_int       j = 0;
		const sp_int m = rowSize - 1;

		while (i < m && j < rowSize)
		{
			sp_int maxi = i;
			
			for (sp_int k = i + 1; k < m; ++k)
				if (fabsf(matrix[k * rowSize + j]) > fabsf(matrix[maxi * rowSize + j]))
					maxi = k;

			if (matrix[maxi * rowSize + j] != ZERO_FLOAT)
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