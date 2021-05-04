#include "Mat.h"

namespace NAMESPACE_PHYSICS
{

	void Mat::transpose(Mat& output) const
	{
		for (register sp_uint row = 0u; row < _rows; row++)
			for (register sp_uint column = 0u; column < _columns; column++)
				output[column * _rows + row] = _values[row * _columns + column];
	}

	sp_bool Mat::eigenValuesAndVectors(sp_float* eigenValues, Mat& eigenVectors, sp_uint& iterations, const sp_uint maxIterations, const sp_float _epsilon) const
	{
#define aqq matrix.get(columnIndex, columnIndex)
#define	app matrix.get(rowIndex, rowIndex)
#define	apq matrix.get(rowIndex, columnIndex)

		if (isSymmetric())
		{
			Mat matrix(this);
			Mat temp(_columns, _columns);

			eigenVectors.setIdentity();

			iterations = ZERO_UINT;

			sp_float offDiagonal = ZERO_FLOAT;

			// get the square sum of the elements off diagonal
			for (sp_uint row = ZERO_UINT; row < matrix.rows(); row++)
				for (sp_uint column = row + ONE_UINT; column < matrix.columns(); column++)
					offDiagonal += (matrix.get(row, column) * matrix.get(row, column));

			while (!NAMESPACE_FOUNDATION::isCloseEnough(offDiagonal, ZERO_FLOAT, _epsilon) && iterations < maxIterations)
			{
				sp_uint rowIndex, columnIndex;
				sp_float value;

				matrix.maxOffDiagonal(columnIndex, rowIndex, value);

				sp_float theta = (aqq - app) / (TWO_FLOAT * apq);

				//If theta is so large that theta2 would overflow on the computer, 
				// so we set t = 1 / (2 theta)

				if (theta < ZERO_FLOAT)
					theta = -theta;

				const sp_float tangentTheta
					= NAMESPACE_FOUNDATION::isCloseEnough(theta, ZERO_FLOAT, DefaultErrorMargin)
					? ONE_FLOAT
					: ONE_FLOAT / (theta + sign(theta) * sp_sqrt(ONE_FLOAT + theta * theta));
				//: ONE_FLOAT / (theta + sp_sqrt(ONE_FLOAT + theta * theta));
				//: sign(theta) / (fabsf(theta) + sp_sqrt(ONE_FLOAT + theta * theta));

				const sp_float cosTheta = ONE_FLOAT / sp_sqrt(ONE_FLOAT + tangentTheta * tangentTheta);
				const sp_float sinTheta = cosTheta * tangentTheta;

				Mat jacobiRotationMatrix(3, 3);
				jacobiRotation(matrix, sinTheta, cosTheta, rowIndex, columnIndex, matrix, jacobiRotationMatrix);

				multiply(eigenVectors, jacobiRotationMatrix, temp);
				std::memcpy(eigenVectors, temp, sizeof(sp_float) * temp.rows() * temp.columns());

				matrix.primaryDiagonal(eigenValues);

				offDiagonal = ZERO_FLOAT;
				for (sp_uint row = ZERO_UINT; row < matrix.rows(); row++)
					for (sp_uint column = row + ONE_UINT; column < matrix.columns(); column++)
						offDiagonal += (matrix.get(row, column) * matrix.get(row, column));

				iterations++;
			}

			if (iterations == ZERO_UINT)
			{
				matrix.primaryDiagonal(eigenValues);
			}
		}
		else
		{
			sp_assert(false, "NotImplementedException");
		}

		return iterations != maxIterations;
#undef apq
#undef app
#undef aqq
	}

	void Mat::gramSchmidt(Mat& output) const
	{
		Vec* u = ALLOC_NEW_ARRAY(Vec, rows());
		Vec* v = ALLOC_NEW_ARRAY(Vec, rows());
		Vec vLengths(rows());

		for (sp_uint column = 0; column < rows(); column++)
		{
			u[column].resize(rows());
			v[column].resize(rows());
		}

		for (sp_uint row = 0; row < rows(); row++)
		{
			const sp_uint rowIndex = row * columns();

			v[0][row] = _values[rowIndex];

			for (sp_uint column = 0; column < columns(); column++)
				u[column][row] = _values[rowIndex + column];
		}

		vLengths[0] = v[0].norm();

		Vec temp1(rows());
		Vec temp2(rows());
		Vec tempOut(rows());

		for (sp_uint row = ONE_UINT; row < rows(); row++)
		{
			tempOut.fill(ZERO_INT);

			for (sp_uint i = ZERO_UINT; i < row; i++)
			{
				multiply(v[i], u[row].dot(v[i]), temp1);

				if (NAMESPACE_FOUNDATION::isCloseEnough(vLengths[i] * vLengths[i], ZERO_FLOAT))
					temp2.fill(ZERO_INT);
				else
					div(temp1, vLengths[i] * vLengths[i], temp2);

				diff(tempOut, temp2, tempOut);
			}

			add(u[row], tempOut, v[row]);

			vLengths[row] = v[row].norm();
		}

		for (sp_uint column = ZERO_UINT; column < columns(); column++)
		{
			const sp_float inverseNorma 
				= !NAMESPACE_FOUNDATION::isCloseEnough(vLengths[column], ZERO_FLOAT)
				? NAMESPACE_FOUNDATION::div(ONE_FLOAT, vLengths[column])
				: ZERO_FLOAT;

			for (sp_uint row = ZERO_UINT; row < rows(); row++)
				output.set(row, column, v[column][row] * inverseNorma);
		}
	}

	void Mat::qr(Mat& q, Mat& r) const
	{
		gramSchmidt(q);

		r.fill(ZERO_UINT);

		for (sp_uint row = 0; row < rows(); row++)
			for (sp_uint column = row + ONE_UINT; column < columns(); column++)
				;
				//r.set(row, column, );
	}

	sp_bool Mat::svd(Mat& u, Mat& s, Mat& v, sp_uint& iterations, const sp_uint maxIterations, const sp_float _epsilon) const
	{
		Mat transposed(_columns, _rows);
		transpose(transposed);

		Mat _symmetric(_columns, _columns);
		multiply(transposed, *this, _symmetric);

		sp_float* eigenValues = ALLOC_NEW_ARRAY(sp_float, _columns);

		if (!_symmetric.eigenValuesAndVectors(eigenValues, v, iterations, maxIterations, _epsilon))
			return false;

		sortEigens(eigenValues, v, _columns);

		s.fill(ZERO_UINT);
		for (sp_uint i = 0; i < s.columns(); i++)
			s.set(i, i, sp_sqrt(eigenValues[i]));

		Mat temp(_rows, v.columns());
		multiply(*this, v, temp);

		for (sp_uint column = 0; column < temp.columns(); column++)
		{
			const sp_float value
				= ! NAMESPACE_FOUNDATION::isCloseEnough(s.get(column, column), ZERO_FLOAT)
				? NAMESPACE_FOUNDATION::div(ONE_FLOAT, s.get(column, column))
				: ZERO_FLOAT;

			for (sp_uint row = 0; row < temp.rows(); row++)
				u._values[row * u.columns() + column] = value * temp[row * temp.columns() + column];
		}

		ALLOC_RELEASE(eigenValues);
		return true;
	}

	void multiply(const Mat& a, const Mat& b, Mat& output)
	{
		sp_assert(output.length() == a.rows() * b.columns(), "InvalidArgumentException");
		sp_assert(a.columns() == b.rows(), "InvalidOperationException");

		std::memset(output, 0, sizeof(sp_float) * output.length());

		for (register sp_uint row = 0u; row < a.rows(); row++)
			for (register sp_uint column = 0u; column < b.columns(); column++)
			{
				const sp_uint outputIndex = row * output.columns() + column;
				const sp_uint rowIndex = row * a.columns();

				for (register sp_uint k = 0u; k < a.columns(); k++)
					output[outputIndex] += a[rowIndex + k] * b[k * b.columns() + column];
			}
	}

	void multiply(const Mat& a, const Mat& b, const Mat& c, Mat& output)
	{
		Mat temp(a.rows(), b.columns());
		std::memset(temp, 0, sizeof(sp_float) * temp.length());

		multiply(a, b, temp);
		multiply(temp, c, output);
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
				alpha = -sp_sqrt(q);
			else
				alpha = -((sp_sqrt(q) * temp) / fabsf(temp));

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
				alpha = -sp_sqrt(q);
			else
				alpha = -((sp_sqrt(q) * temp) / fabsf(temp));

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

				u[j] = NAMESPACE_FOUNDATION::div(temp, rqs);
			}

			// step 7:
			sp_float prod = ZERO_FLOAT;
			for (sp_uint i = k + 1u; i < n; i++)
				prod += v[i] * u[i];

			// step 8:
			std::memset(z, ZERO_INT, SIZEOF_FLOAT * n);
			for (sp_uint j = k; j < n; j++)
				z[j] = u[j] - NAMESPACE_FOUNDATION::div(prod, (2.0f * rqs)) * v[j];

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

	void givensRotation(Mat& output, const sp_uint rowIndex, const sp_uint columnIndex, const sp_float sinTheta, const sp_float cosTheta)
	{
		output.setIdentity();

		output[columnIndex * output.columns() + columnIndex]
			= output[rowIndex * output.columns() + rowIndex]
			= sinTheta;

		output[rowIndex * output.columns() + columnIndex] = -cosTheta;
		output[columnIndex * output.columns() + rowIndex] = cosTheta;
	}

	void jacobiRotation(const Mat& input, const sp_float sinTheta, const sp_float cosTheta, const sp_uint rowIndex, const sp_uint columnIndex, Mat& output, Mat& jacobiRotation)
	{
		givensRotation(jacobiRotation, rowIndex, columnIndex, sinTheta, cosTheta);

		Mat jacobiRotationT(jacobiRotation.rows(), jacobiRotation.columns());
		jacobiRotation.transpose(jacobiRotationT);

		multiply(jacobiRotationT, input, jacobiRotation, output);
	}

}