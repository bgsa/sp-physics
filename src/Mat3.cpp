#include "Mat3.h"

namespace NAMESPACE_PHYSICS
{
	
	Mat3::Mat3(const sp_float defaultValue)
	{
		static sp_float emptyMatrix[MAT3_SIZE] = {
			defaultValue, defaultValue, defaultValue,
			defaultValue, defaultValue, defaultValue,
			defaultValue, defaultValue, defaultValue
		};

		std::memcpy(&values, emptyMatrix, sizeof(values));
	}

	
	Mat3::Mat3(sp_float* values)
	{
		std::memcpy(&this->values, values, sizeof(this->values));
	}

	
	Mat3::Mat3(
		const sp_float value11, const sp_float value21, const sp_float value31,
		const sp_float value12, const sp_float value22, const sp_float value32,
		const sp_float value13, const sp_float value23, const sp_float value33)
	{
		values[0] = value11;
		values[1] = value21;
		values[2] = value31;
		values[3] = value12;
		values[4] = value22;
		values[5] = value32;
		values[6] = value13;
		values[7] = value23;
		values[8] = value33;
	}

	
	sp_float* Mat3::getValues()
	{
		return values;
	}

	
	sp_float Mat3::getValue(const sp_int x, const sp_int y) const
	{
		return values[(y-1) * MAT3_ROWSIZE + (x-1)];
	}

	
	Vec3 Mat3::getAxis(const sp_int index) const
	{
	#if MAJOR_COLUMN_ORDER

		return Vec3(
			values[index],
			values[index + MAT3_ROWSIZE],
			values[index + MAT3_ROWSIZE + MAT3_ROWSIZE]
		);
	#else

		return Vec3(
			values[index * MAT3_ROWSIZE],
			values[index * MAT3_ROWSIZE + 1],
			values[index * MAT3_ROWSIZE + 2]
		);
	#endif
	}

	
	Vec3 Mat3::xAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec3{
			values[0],
			values[3],
			values[6]
		};
	#else
		return Vec3{
			values[0],
			values[1],
			values[2]
		};
	#endif
	}

	
	Vec3 Mat3::yAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec3{
			values[1],
			values[4],
			values[7]
		};
	#else
		return Vec3{
			values[3],
			values[4],
			values[5]
		};
	#endif
	}

	
	Vec3 Mat3::zAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec3{
			values[2],
			values[5],
			values[8]
		};
	#else
		return Vec3{
			values[6],
			values[7],
			values[8]
		};
	#endif
	}

	
	Mat3 Mat3::identity()
	{
		static sp_float identityMatrix[MAT3_SIZE] = {
			ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
			ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT,
			ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
		};

		Mat3 result;
		std::memcpy(&result, identityMatrix, sizeof(values));

		return result;
	}

	
	Vec3 Mat3::primaryDiagonal() const
	{
		return Vec3 {
			values[0],
			values[4],
			values[8]
		};
	}

	
	Vec3 Mat3::secondaryDiagonal() const
	{
		return Vec3 {
			values[2],
				values[4],
				values[6]
		};
	}

	
	Mat3 Mat3::transpose() const
	{
		Mat3 result;

		//copy principal diagonal
		result[0] = values[0];
		result[4] = values[4];
		result[8] = values[8];

		//swap others numbers
		sp_float temp = values[1];
		result[1] = values[3];
		result[3] = temp;

		temp = values[2];
		result[2] = values[6];
		result[6] = temp;

		temp = values[5];
		result[5] = values[7];
		result[7] = temp;

		return result;
	}

	
	Mat3 Mat3::multiply(const Mat3& matrixB) const
	{
		Mat3 result;

	#if MAJOR_COLUMN_ORDER
		for (int line = 0; line < MAT3_ROWSIZE; line++)
		{
			sp_float ai0 = values[line];
			sp_float ai1 = values[MAT3_ROWSIZE + line];
			sp_float ai2 = values[TWO_MAT3_ROWSIZE + line];

			result[line] = ai0 * matrixB[0] + ai1 * matrixB[1] + ai2 * matrixB[2];
			result[MAT3_ROWSIZE + line] = ai0 * matrixB[MAT3_ROWSIZE] + ai1 * matrixB[MAT3_ROWSIZE + 1] + ai2 * matrixB[MAT3_ROWSIZE + 2];
			result[TWO_MAT3_ROWSIZE + line] = ai0 * matrixB[TWO_MAT3_ROWSIZE] + ai1 * matrixB[TWO_MAT3_ROWSIZE + 1] + ai2 * matrixB[TWO_MAT3_ROWSIZE + 2];
		}
	#else
		for (int column = 0; column < MAT3_ROWSIZE; column++)
		{
			sp_float ai0 = values[(column * MAT3_ROWSIZE) + 0];
			sp_float ai1 = values[(column * MAT3_ROWSIZE) + 1];
			sp_float ai2 = values[(column * MAT3_ROWSIZE) + 2];

			result[(column * MAT3_ROWSIZE) + 0] = ai0 * matrixB[(0 * MAT3_ROWSIZE) + 0] + ai1 * matrixB[(1 * MAT3_ROWSIZE) + 0] + ai2 * matrixB[(2 * MAT3_ROWSIZE) + 0];
			result[(column * MAT3_ROWSIZE) + 1] = ai0 * matrixB[(0 * MAT3_ROWSIZE) + 1] + ai1 * matrixB[(1 * MAT3_ROWSIZE) + 1] + ai2 * matrixB[(2 * MAT3_ROWSIZE) + 1];
			result[(column * MAT3_ROWSIZE) + 2] = ai0 * matrixB[(0 * MAT3_ROWSIZE) + 2] + ai1 * matrixB[(1 * MAT3_ROWSIZE) + 2] + ai2 * matrixB[(2 * MAT3_ROWSIZE) + 2];
		}
	#endif

		return result;
	}

	
	Vec3 Mat3::multiply(const Vec3& vector) const
	{
		Vec3 result;

		result[0] = values[0] * vector[0] + values[1] * vector[1] + values[2] * vector[2];
		result[1] = values[MAT3_ROWSIZE] * vector[0] + values[MAT3_ROWSIZE + 1] * vector[1] + values[MAT3_ROWSIZE + 2] * vector[2];
		result[2] = values[TWO_MAT3_ROWSIZE] * vector[0] + values[TWO_MAT3_ROWSIZE + 1] * vector[1] + values[TWO_MAT3_ROWSIZE + 2] * vector[2];

		return result;
	}

	
	sp_float Mat3::determinantIJ(const sp_size i, const sp_size j) const
	{
		sp_float* matrixValues = ALLOC_ARRAY(sp_float, 4);
		sp_size index = 0;

		for (sp_size row = 0; row < MAT3_ROWSIZE; row++)
		{
			if (i == row)
				continue;

			for (sp_size column = 0; column < MAT3_ROWSIZE; column++)
			{
				if (j == column)
					continue;

				matrixValues[index] = values[row * MAT3_ROWSIZE + column];
				index++;
			}
		}

		Mat2 matrix = Mat2(matrixValues);
		sp_float determinant = matrix.determinant();

		return determinant;
	}

	
	sp_float Mat3::cofactorIJ(const sp_size i, const sp_size j) const
	{
		sp_float determinantIJValue = determinantIJ(i, j);

		if (isOdd(i + j))
			determinantIJValue *= -1;

		return determinantIJValue;
	}

	
	Mat3 Mat3::createScale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
	{
		Mat3 result = Mat3::identity();

		result.scale(xScale, yScale, zScale);

		return result;
	}

	
	void Mat3::scale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
	{
		values[0] *= xScale;
		values[4] *= yScale;
		values[8] *= zScale;
	}

	
	Mat3 Mat3::createRotate(const sp_float angleRadians, const sp_float x, const sp_float y, const sp_float z)
	{	
		const sp_float sine = sinf(angleRadians);
		const sp_float cosine = cosf(angleRadians);

		const sp_float mag = sqrtf(x*x + y * y + z * z);

		if (mag == 0.0f)
			return Mat3::identity();

		// Rotation matrix is normalized
		const sp_float x1 = x / mag;
		const sp_float y1 = y / mag;
		const sp_float z1 = z / mag;

		const sp_float xx = x1 * x1;
		const sp_float yy = y1 * y1;
		const sp_float zz = z1 * z1;
		const sp_float xy = x1 * y1;
		const sp_float yz = y1 * z1;
		const sp_float zx = z1 * x1;
		const sp_float xs = x1 * sine;
		const sp_float ys = y1 * sine;
		const sp_float zs = z1 * sine;
		const sp_float one_c = ONE_FLOAT - cosine;

		Mat3 result;

		result[0 * MAT3_ROWSIZE + 0] = (one_c * xx) + cosine;
		result[1 * MAT3_ROWSIZE + 0] = (one_c * xy) - zs;
		result[2 * MAT3_ROWSIZE + 0] = (one_c * zx) + ys;

		result[0 * MAT3_ROWSIZE + 1] = (one_c * xy) + zs;
		result[1 * MAT3_ROWSIZE + 1] = (one_c * yy) + cosine;
		result[2 * MAT3_ROWSIZE + 1] = (one_c * yz) - xs;

		result[0 * MAT3_ROWSIZE + 2] = (one_c * zx) - ys;
		result[1 * MAT3_ROWSIZE + 2] = (one_c * yz) + xs;
		result[2 * MAT3_ROWSIZE + 2] = (one_c * zz) + cosine;

		return result;
	}

	
	Mat3 Mat3::createTranslate(const sp_float x, const sp_float y, const sp_float z)
	{
		Mat3 result = Mat3::identity();

	#if MAJOR_COLUMN_ORDER
		result[6] = x;
		result[7] = y;
		result[8] = z;
	#else
		result[2] = x;
		result[5] = y;
		result[8] = z;
	#endif

		return result;
	}

	
	Mat3 Mat3::createTranslate(const Vec3& position)
	{
		Mat3 result = Mat3::identity();

	#if MAJOR_COLUMN_ORDER
		result[6] = position.x;
		result[7] = position.y;
		result[8] = position.z;
	#else
		result[2] = position.x;
		result[5] = position.y;
		result[8] = position.z;
	#endif

		return result;
	}

	
	sp_float Mat3::determinant() const
	{
		return
			(values[0] * values[4] * values[8]
				+ values[1] * values[5] * values[6]
				+ values[2] * values[3] * values[7]
				)
			-
			(values[2] * values[4] * values[6]
				+ values[0] * values[5] * values[7]
				+ values[1] * values[3] * values[8]
				);
	}

	
	Mat3 Mat3::invert() const
	{
		Mat3 matrixInverse;
		sp_float detij;

		sp_float det = determinant();

		for (int i = 0; i < MAT3_ROWSIZE; i++)
			for (int j = 0; j < MAT3_ROWSIZE; j++)
			{
				detij = determinantIJ(j, i);

				if ((i + j) & 0x1)
					matrixInverse[i * MAT3_ROWSIZE + j] = -detij * det;
				else
					matrixInverse[i * MAT3_ROWSIZE + j] = detij * det;
			}

		return matrixInverse;
	}

	
	sp_bool Mat3::isIdentity() const
	{
		Mat3 identityMatrix = Mat3::identity();

		for (int i = 0; i < MAT3_SIZE; i++)
			if (values[i] != identityMatrix[i])
				return false;

		return true;
	}

	
	sp_size Mat3::sizeInBytes() const
	{
		return MAT3_SIZE * SIZEOF_FLOAT;
	}

	
	Mat3 Mat3::clone() const
	{
		Mat3 result;

		memcpy(&result, this, sizeof(Mat3));

		return result;
	}

	
	sp_float& Mat3::operator[](sp_int index)
	{
		sp_assert(index >= 0 && index < MAT3_SIZE);

		return values[index];
	}
	
	sp_float Mat3::operator[](sp_int index) const
	{
		sp_assert(index >= 0 && index < MAT3_SIZE);

		return values[index];
	}

	
	sp_float& Mat3::operator[](sp_uint index)
	{
		sp_assert(index >= 0 && index < MAT3_SIZE);

		return values[index];
	}
	
	sp_float Mat3::operator[](sp_uint index) const
	{
		sp_assert(index >= 0 && index < MAT3_SIZE);

		return values[index];
	}

#if defined(WINDOWS) && defined(ENV_64BITS)
	
	T& Mat3::operator[](sp_size index)
	{
		sp_assert(index >= 0 && index < MAT3_SIZE);

		return values[index];
	}
	
	T Mat3::operator[](sp_size index) const
	{
		sp_assert(index >= 0 && index < MAT3_SIZE);

		return values[index];
	}
#endif

	
	Mat3::operator void*() const
	{
		return (void*)values;
	}

	
	Mat3::operator sp_float*() const
	{
		return (sp_float*)values;
	}

	
	Mat3 Mat3::operator-() const
	{
		Mat3 result;

		for (int i = 0; i < MAT3_SIZE; i++)
			result[i] = -values[i];

		return result;
	}

	
	Mat3 Mat3::operator-(const Mat3& matrix) const
	{
		Mat3 result;

		for (int i = 0; i < MAT3_SIZE; i++)
			result[i] = values[i] - matrix[i];

		return result;
	}

	
	Mat3 Mat3::operator+(const Mat3& matrix) const
	{
		Mat3 result;

		for (int i = 0; i < MAT3_SIZE; i++)
			result[i] = values[i] + matrix[i];

		return result;
	}

	
	Mat3 Mat3::operator*(const Mat3& matrix) const
	{
		return multiply(matrix);
	}

	
	Vec3 Mat3::operator*(const Vec3& matrix) const
	{
		return multiply(matrix);
	}

	
	void Mat3::operator*=(const Mat3& matrix)
	{
		std::memcpy(&this->values, multiply(matrix).values, sizeof(this->values));
	}

	
	Mat3 Mat3::operator/(const sp_float value) const
	{
		return Mat3 {
			values[0] / value, values[1] / value, values[2] / value,
			values[3] / value, values[4] / value, values[5] / value,
			values[6] / value, values[7] / value, values[8] / value
		};
	}

	
	void Mat3::operator/=(const sp_float value)
	{
		sp_float invValue = ONE_FLOAT / value;

		values[0] *= invValue;
		values[1] *= invValue;
		values[2] *= invValue;
		values[3] *= invValue;
		values[4] *= invValue;
		values[5] *= invValue;
		values[6] *= invValue;
		values[7] *= invValue;
		values[8] *= invValue;
	}

	
	sp_bool Mat3::operator==(const Mat3& matrix) const
	{
		for (sp_int i = 0; i < MAT3_SIZE; i++)
			if (values[i] != matrix[i])
				return false;

		return true;
	}

	
	sp_bool Mat3::operator!=(const Mat3& matrix) const
	{
		for (sp_int i = 0; i < MAT3_SIZE; i++)
			if (values[i] != matrix[i])
				return true;

		return false;
	}

	
	sp_bool Mat3::operator==(const sp_float value) const
	{
		for (sp_int i = 0; i < MAT3_SIZE; i++)
			if (values[i] != value)
				return false;

		return true;
	}

	
	std::string Mat3::toString() const
	{
		return Mat::toString(values, MAT3_SIZE);
	}

	
	Mat3* Mat3::decomposeLU() const
	{
		Mat3 lowerMatrix = Mat3::identity();
		Mat3 upperMatrix = this->clone();
		Mat3* result = ALLOC_ARRAY(Mat3, 2);

		std::vector<Mat3> elementarInverseMatrixes;
		Mat3 elementarInverseMatrix;

		sp_int rowSize = MAT3_ROWSIZE;
		sp_int colSize = MAT3_ROWSIZE;

	#if MAJOR_COLUMN_ORDER
		sp_int pivotRowIndex = 0;

		for (sp_int column = 0; column < colSize; column++)
		{
			sp_float pivot = upperMatrix[pivotRowIndex * rowSize + column];
			sp_float pivotOperator = 1.0f / pivot;

			for (int row = 0; row < rowSize; row++)
				upperMatrix[row * rowSize + column] *= pivotOperator;

			elementarInverseMatrix = Mat3::identity();
			elementarInverseMatrix[pivotRowIndex * rowSize + column] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerColumns = column + 1; lowerColumns < rowSize; lowerColumns++)
			{
				pivot = upperMatrix[pivotRowIndex * rowSize + lowerColumns];
				pivotOperator = -pivot;

				for (int row = 0; row < rowSize; row++)
					upperMatrix[row * rowSize + lowerColumns] += pivotOperator * upperMatrix[row * rowSize + column];

				elementarInverseMatrix = Mat3::identity();
				elementarInverseMatrix[pivotRowIndex * rowSize + lowerColumns] = pivot;
				elementarInverseMatrixes.push_back(elementarInverseMatrix);
			}

			pivotRowIndex++;
		}

		for (sp_int i = 0; sp_size(i) < elementarInverseMatrixes.size(); i++)
			lowerMatrix *= elementarInverseMatrixes[i];
	#else
		sp_size pivotColumnIndex = 0;

		for (sp_size line = 0; line < rowSize; line++)
		{
			sp_float pivot = upperMatrix[line * rowSize + pivotColumnIndex];
			sp_float pivotOperator = 1.0f / pivot;

			for (sp_int column = 0; column < colSize; column++)
				upperMatrix[line * rowSize + column] *= pivotOperator;

			elementarInverseMatrix = Mat3::identity();
			elementarInverseMatrix[line * rowSize + pivotColumnIndex] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerLines = line + 1; lowerLines < rowSize; lowerLines++)
			{
				pivot = upperMatrix[lowerLines * rowSize + pivotColumnIndex];
				pivotOperator = -pivot;

				for (sp_int column = 0; column < colSize; column++)
					upperMatrix[lowerLines * rowSize + column] += pivotOperator * upperMatrix[line * rowSize + column];

				elementarInverseMatrix = Mat3::identity();
				elementarInverseMatrix[lowerLines * rowSize + pivotColumnIndex] = pivot;
				elementarInverseMatrixes.push_back(elementarInverseMatrix);
			}

			pivotColumnIndex++;
		}

		for (sp_int i = 0; sp_size(i) < elementarInverseMatrixes.size(); i++)
			lowerMatrix *= elementarInverseMatrixes[i];
	#endif

		result[0] = lowerMatrix;
		result[1] = upperMatrix;

		return result;
	}

	
	Mat3* Mat3::decomposeLDU() const
	{
		Mat3 diagonalMatrix = Mat3::identity();
		Mat3* result = ALLOC_ARRAY(Mat3, 3);

		Mat3* lowerAndUpperMatrixes = decomposeLU();

		Mat3 upperMatrix = lowerAndUpperMatrixes[1];
		int diagonalIndex = 0;

	#if MAJOR_COLUMN_ORDER
		for (sp_int column = 0; column < MAT3_ROWSIZE; column++)
		{
			sp_float pivot = upperMatrix[diagonalIndex * MAT3_ROWSIZE + column];

			diagonalMatrix[column * MAT3_ROWSIZE + diagonalIndex] = pivot;

			for (int row = column; row < MAT3_ROWSIZE; row++)
				upperMatrix[column * MAT3_ROWSIZE + row] /= pivot;

			diagonalIndex++;
		}
	#else
		for (sp_int row = 0; row < MAT3_ROWSIZE; row++)
		{
			T pivot = upperMatrix[row * MAT3_ROWSIZE + diagonalIndex];

			diagonalMatrix[row * MAT3_ROWSIZE + diagonalIndex] = pivot;

			for (int column = row; column < MAT3_ROWSIZE; column++)
				upperMatrix[row * MAT3_ROWSIZE + column] /= pivot;

			diagonalIndex++;
		}
	#endif

		result[0] = lowerAndUpperMatrixes[0];
		result[1] = diagonalMatrix;
		result[2] = upperMatrix;

		ALLOC_RELEASE(lowerAndUpperMatrixes);

		return result;
	}

	
	AutoValueAutoVector3 Mat3::getAutovalueAndAutovector(const sp_ushort maxIteration) const
	{
		Mat3 matrix = *this;
		Vec3 autovector = { ONE_FLOAT, ONE_FLOAT, ONE_FLOAT };
		sp_float autovalue;

		for (sp_short iterationIndex = 0; iterationIndex < maxIteration; iterationIndex++)
		{
			Vec3 ax = matrix * autovector;
			autovalue = ax.maximum();
			autovector = ax / autovalue;
		}

		return AutoValueAutoVector3{ autovalue, { autovector.x, autovector.y, autovector.z} };
	}

}