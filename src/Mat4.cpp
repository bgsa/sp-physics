#include "Mat4.h"

namespace NAMESPACE_PHYSICS
{
	
	Mat4::Mat4(const sp_float defaultValue)
	{
		for (sp_ushort i = 0; i < MAT4_LENGTH; i++)
			values[i] = defaultValue;
	}

	
	Mat4::Mat4(const Vec4& vector1, const Vec4& vector2, const Vec4& vector3, const Vec4& vector4)
	{
	#if MAJOR_COLUMN_ORDER
		values[0] = vector1[0];
		values[4] = vector1[1];
		values[8] = vector1[2];
		values[12] = vector1[3];

		values[1] = vector2[0];
		values[5] = vector2[1];
		values[9] = vector2[2];
		values[13] = vector2[3];

		values[2] = vector3[0];
		values[6] = vector3[1];
		values[10] = vector3[2];
		values[14] = vector3[3];

		values[3] = vector4[0];
		values[7] = vector4[1];
		values[11] = vector4[2];
		values[15] = vector4[3];
	#else
		values[0] = vector1[0];
		values[1] = vector1[1];
		values[2] = vector1[2];
		values[3] = vector1[3];

		values[4] = vector2[0];
		values[5] = vector2[1];
		values[6] = vector2[2];
		values[7] = vector2[3];

		values[8] = vector3[0];
		values[9] = vector3[1];
		values[10] = vector3[2];
		values[11] = vector3[3];

		values[12] = vector4[0];
		values[13] = vector4[1];
		values[14] = vector4[2];
		values[15] = vector4[3];
	#endif
	}

	
	Mat4::Mat4(sp_float* values)
	{
		std::memcpy(&this->values, values, sizeof(this->values));
	}

	
	Mat4::Mat4(
		const sp_float value11, const sp_float value21, const sp_float value31, const sp_float value41,
		const sp_float value12, const sp_float value22, const sp_float value32, const sp_float value42,
		const sp_float value13, const sp_float value23, const sp_float value33, const sp_float value43,
		const sp_float value14, const sp_float value24, const sp_float value34, const sp_float value44)
	{
		values[0] = value11;
		values[1] = value21;
		values[2] = value31;
		values[3] = value41;

		values[4] = value12;
		values[5] = value22;
		values[6] = value32;
		values[7] = value42;

		values[8] = value13;
		values[9] = value23;
		values[10] = value33;
		values[11] = value43;

		values[12] = value14;
		values[13] = value24;
		values[14] = value34;
		values[15] = value44;
	}

	
	sp_float* Mat4::getValues()
	{
		return values;
	}

	
	sp_float Mat4::getValue(const sp_int x, const sp_int y) const
	{
		return values[(y-1) * MAT4_ROW_LENGTH + (x-1)];
	}

	
	Vec4 Mat4::xAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4 {
			values[0 * MAT4_ROW_LENGTH + 0],
				values[1 * MAT4_ROW_LENGTH + 0],
				values[2 * MAT4_ROW_LENGTH + 0],
				values[3 * MAT4_ROW_LENGTH + 0]
		};
	#else
		return Vec4 {
			values[0 * MAT4_ROW_LENGTH + 0],
				values[0 * MAT4_ROW_LENGTH + 1],
				values[0 * MAT4_ROW_LENGTH + 2],
				values[0 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	
	Vec4 Mat4::yAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4 {
			values[0 * MAT4_ROW_LENGTH + 1],
				values[1 * MAT4_ROW_LENGTH + 1],
				values[2 * MAT4_ROW_LENGTH + 1],
				values[3 * MAT4_ROW_LENGTH + 1]
		};
	#else
		return Vec4 {
			values[1 * MAT4_ROW_LENGTH + 0],
				values[1 * MAT4_ROW_LENGTH + 1],
				values[1 * MAT4_ROW_LENGTH + 2],
				values[1 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	
	Vec4 Mat4::zAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4 {
			values[0 * MAT4_ROW_LENGTH + 2],
				values[1 * MAT4_ROW_LENGTH + 2],
				values[2 * MAT4_ROW_LENGTH + 2],
				values[3 * MAT4_ROW_LENGTH + 2]
		};
	#else
		return Vec4 {
			values[2 * MAT4_ROW_LENGTH + 0],
				values[2 * MAT4_ROW_LENGTH + 1],
				values[2 * MAT4_ROW_LENGTH + 2],
				values[2 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	
	Vec4 Mat4::wAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4 {
			values[0 * MAT4_ROW_LENGTH + 3],
				values[1 * MAT4_ROW_LENGTH + 3],
				values[2 * MAT4_ROW_LENGTH + 3],
				values[3 * MAT4_ROW_LENGTH + 3]
		};
	#else
		return Vec4 {
			values[3 * MAT4_ROW_LENGTH + 0],
				values[3 * MAT4_ROW_LENGTH + 1],
				values[3 * MAT4_ROW_LENGTH + 2],
				values[3 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	
	Vec4 Mat4::primaryDiagonal() const
	{
		return Vec4 {
			values[0],
				values[5],
				values[10],
				values[15]
		};
	}

	
	Vec4 Mat4::secondaryDiagonal() const
	{
		return Vec4 {
			values[3],
				values[6],
				values[9],
				values[12]
		};
	}

	
	Mat4 Mat4::identity()
	{
		static sp_float identityMatrix[MAT4_LENGTH] = {
			ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
			ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
			ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT,
			ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
		};

		Mat4 result;
		std::memcpy(&result, identityMatrix, sizeof(values));

		return result;
	}

	
	Mat4 Mat4::transpose() const
	{
		Mat4 result;

		//copy principal diagonal
		result[0] = values[0];
		result[5] = values[5];
		result[10] = values[10];
		result[15] = values[15];

		//swap others numbers
		sp_float temp = values[1];
		result[1] = values[4];
		result[4] = temp;

		temp = values[2];
		result[2] = values[8];
		result[8] = temp;

		temp = values[3];
		result[3] = values[12];
		result[12] = temp;

		temp = values[6];
		result[6] = values[9];
		result[9] = temp;

		temp = values[7];
		result[7] = values[13];
		result[13] = temp;

		temp = values[11];
		result[11] = values[14];
		result[14] = temp;

		return result;
	}

	
	sp_float Mat4::determinantIJ(sp_int i, sp_int j) const
	{
		sp_int x, y, ii, jj;
		sp_float ret, mat[3][3];

		x = 0;
		for (ii = 0; ii < 4; ii++)
		{
			if (ii == i)
				continue;

			y = 0;

			for (jj = 0; jj < 4; jj++)
			{
				if (jj == j)
					continue;

				mat[x][y] = values[(ii * 4) + jj];
				y++;
			}

			x++;
		}

		ret = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]);
		ret -= mat[0][1] * (mat[1][0] * mat[2][2] - mat[2][0] * mat[1][2]);
		ret += mat[0][2] * (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]);

		return ret;
	}

	
	sp_float Mat4::cofactorIJ(const sp_int i, const sp_int j) const
	{
		sp_float determinantIJValue = determinantIJ(i, j);

		if (isOdd(i + j))
			determinantIJValue *= -1;

		return determinantIJValue;
	}

	
	sp_float Mat4::determinant() const
	{
		sp_float det = ZERO_FLOAT;

		for (sp_int i = 0; i < MAT4_ROW_LENGTH; i++)
		{
			det += (i & 0x1) ? 
				(-values[i] * determinantIJ(0, i)) 
				: (values[i] * determinantIJ(0, i));
		}

		return det;
	}

	
	Mat4 Mat4::multiply(const Mat4 &matrixB) const
	{
		Mat4 result;

	#if MAJOR_COLUMN_ORDER
		for (int line = 0; line < MAT4_ROW_LENGTH; line++)
		{
			const sp_float ai0 = values[(0 * MAT4_ROW_LENGTH) + line];
			const sp_float ai1 = values[(1 * MAT4_ROW_LENGTH) + line];
			const sp_float ai2 = values[(2 * MAT4_ROW_LENGTH) + line];
			const sp_float ai3 = values[(3 * MAT4_ROW_LENGTH) + line];

			sp_float a = matrixB[0];

			result[(0 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(0 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(0 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(0 * MAT4_ROW_LENGTH) + 3];
			result[(1 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(1 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(1 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(1 * MAT4_ROW_LENGTH) + 3];
			result[(2 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(2 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(2 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(2 * MAT4_ROW_LENGTH) + 3];
			result[(3 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(3 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(3 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(3 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 3];
		}
	#else 
		for (int column = 0; column < MAT4_ROW_LENGTH; column++)
		{
			sp_float ai0 = values[(column * MAT4_ROW_LENGTH) + 0];
			sp_float ai1 = values[(column * MAT4_ROW_LENGTH) + 1];
			sp_float ai2 = values[(column * MAT4_ROW_LENGTH) + 2];
			sp_float ai3 = values[(column * MAT4_ROW_LENGTH) + 3];

			result[(column * MAT4_ROW_LENGTH) + 0] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 0] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 0] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 0];
			result[(column * MAT4_ROW_LENGTH) + 1] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 1] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 1] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 1];
			result[(column * MAT4_ROW_LENGTH) + 2] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 2] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 2] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 2];
			result[(column * MAT4_ROW_LENGTH) + 3] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 3] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 3] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 3] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 3];
		}
	#endif

		return result;
	}

	
	Vec4 Mat4::multiply(const Vec4 &vector) const
	{
		Vec4 result;

	#if MAJOR_COLUMN_ORDER
		result[0]
			= vector[0] * values[0 * MAT4_ROW_LENGTH + 0]
			+ vector[1] * values[1 * MAT4_ROW_LENGTH + 0]
			+ vector[2] * values[2 * MAT4_ROW_LENGTH + 0]
			+ vector[3] * values[3 * MAT4_ROW_LENGTH + 0];

		result[1]
			= vector[0] * values[0 * MAT4_ROW_LENGTH + 1]
			+ vector[1] * values[1 * MAT4_ROW_LENGTH + 1]
			+ vector[2] * values[2 * MAT4_ROW_LENGTH + 1]
			+ vector[3] * values[3 * MAT4_ROW_LENGTH + 1];

		result[2]
			= vector[0] * values[0 * MAT4_ROW_LENGTH + 2]
			+ vector[1] * values[1 * MAT4_ROW_LENGTH + 2]
			+ vector[2] * values[2 * MAT4_ROW_LENGTH + 2]
			+ vector[3] * values[3 * MAT4_ROW_LENGTH + 2];

		result[3]
			= vector[0] * values[0 * MAT4_ROW_LENGTH + 3]
			+ vector[1] * values[1 * MAT4_ROW_LENGTH + 3]
			+ vector[2] * values[2 * MAT4_ROW_LENGTH + 3]
			+ vector[3] * values[3 * MAT4_ROW_LENGTH + 3];

	#else

		result[0]
			= values[0 * MAT4_ROW_LENGTH + 0] * vector[0]
			+ values[0 * MAT4_ROW_LENGTH + 1] * vector[1]
			+ values[0 * MAT4_ROW_LENGTH + 2] * vector[2]
			+ values[0 * MAT4_ROW_LENGTH + 3] * vector[3];

		result[1]
			= values[1 * MAT4_ROW_LENGTH + 0] * vector[0]
			+ values[1 * MAT4_ROW_LENGTH + 1] * vector[1]
			+ values[1 * MAT4_ROW_LENGTH + 2] * vector[2]
			+ values[1 * MAT4_ROW_LENGTH + 3] * vector[3];

		result[2]
			= values[2 * MAT4_ROW_LENGTH + 0] * vector[0]
			+ values[2 * MAT4_ROW_LENGTH + 1] * vector[1]
			+ values[2 * MAT4_ROW_LENGTH + 2] * vector[2]
			+ values[2 * MAT4_ROW_LENGTH + 3] * vector[3];

		result[3]
			= values[3 * MAT4_ROW_LENGTH + 0] * vector[0]
			+ values[3 * MAT4_ROW_LENGTH + 1] * vector[1]
			+ values[3 * MAT4_ROW_LENGTH + 2] * vector[2]
			+ values[3 * MAT4_ROW_LENGTH + 3] * vector[3];
	#endif

		return result;
	}

	
	Mat4 Mat4::invert() const
	{
		Mat4 mInverse;

		sp_int i, j;
		sp_float det = 0.0f;
		sp_float detij;

		// calculate 4x4 determinant
		for (i = 0; i < 4; i++)
		{
			det += (i & 0x1) ? (-values[i] * determinantIJ(0, i)) : (values[i] * determinantIJ(0, i));
		}
		det = ONE_FLOAT / det;

		// calculate inverse
		for (i = 0; i < 4; i++)
		{
			for (j = 0; j < 4; j++)
			{
				detij = determinantIJ(j, i);
				mInverse[(i * 4) + j] = ((i + j) & 0x1) ? (-detij * det) : (detij *det);
			}
		}

		return mInverse;
	}

	
	Mat4 Mat4::createScale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
	{
		return Mat4(
			xScale, 0.0f, 0.0f, 0.0f,
			0.0f, yScale, 0.0f, 0.0f,
			0.0f, 0.0f, zScale, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
			);
	}

	
	Mat4 Mat4::createScale(const Vec3& scale)
	{
		return Mat4(
			ONE_FLOAT + scale.x, 0.0f, 0.0f, 0.0f,
			0.0f, ONE_FLOAT + scale.y, 0.0f, 0.0f,
			0.0f, 0.0f, ONE_FLOAT + scale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		);
	}

	
	void Mat4::scale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
	{
		values[0] *= xScale;
		values[5] *= yScale;
		values[10] *= zScale;
	}

	
	Mat4 Mat4::createRotate(const sp_float angleRadians, const sp_float x, const sp_float y, const sp_float z)
	{
		const sp_float sineAngle = sinf(angleRadians);
		const sp_float cosineAngle = cosf(angleRadians);

		const sp_float mag = sqrtf(x*x + y * y + z * z);

		if (mag == 0.0f)
			return Mat4::identity();

		// Rotation matrix is normalized
		const sp_float x1 = x / mag;
		const sp_float y1 = y / mag;
		const sp_float z1 = z / mag;

		const sp_float xx = x * x;
		const sp_float yy = y * y;
		const sp_float zz = z * z;
		const sp_float xy = x * y;
		const sp_float yz = y * z;
		const sp_float zx = z * x;
		const sp_float xs = x * sineAngle;
		const sp_float ys = y * sineAngle;
		const sp_float zs = z * sineAngle;
		const sp_float one_c = 1.0f - cosineAngle;

		Mat4 result;

	#define M(row,col)  result[col * MAT4_ROW_LENGTH + row]
		M(0, 0) = (one_c * xx) + cosineAngle;
		M(0, 1) = (one_c * xy) - zs;
		M(0, 2) = (one_c * zx) + ys;
		M(0, 3) = ZERO_FLOAT;

		M(1, 0) = (one_c * xy) + zs;
		M(1, 1) = (one_c * yy) + cosineAngle;
		M(1, 2) = (one_c * yz) - xs;
		M(1, 3) = ZERO_FLOAT;

		M(2, 0) = (one_c * zx) - ys;
		M(2, 1) = (one_c * yz) + xs;
		M(2, 2) = (one_c * zz) + cosineAngle;
		M(2, 3) = ZERO_FLOAT;

		M(3, 0) = ZERO_FLOAT;
		M(3, 1) = ZERO_FLOAT;
		M(3, 2) = ZERO_FLOAT;
		M(3, 3) = ONE_FLOAT;
	#undef M

		return result;
	}

	
	Mat4 Mat4::createTranslate(const sp_float x, const sp_float y, const sp_float z)
	{
		Mat4 result = Mat4::identity();

	#if MAJOR_COLUMN_ORDER
		result[12] = x;
		result[13] = y;
		result[14] = z;
	#else
		result[3] = x;
		result[7] = y;
		result[11] = z;
	#endif

		return result;
	}

	
	Mat4 Mat4::createTranslate(const Vec3& position)
	{
		Mat4 result = Mat4::identity();

#if MAJOR_COLUMN_ORDER
		result[12] = position.x;
		result[13] = position.y;
		result[14] = position.z;
#else
		result[3] = position.x;
		result[7] = position.y;
		result[11] = position.z;
#endif

		return result;
	}

	
	Mat4 Mat4::createOrthographicMatrix(const sp_float xMin, const sp_float xMax, const sp_float yMin, const sp_float yMax, const sp_float zMin, const sp_float zMax)
	{
		Mat4 projectionMatrix = Mat4::identity();

		projectionMatrix[0] = TWO_FLOAT / (xMax - xMin);
		projectionMatrix[5] = TWO_FLOAT / (yMax - yMin);
		projectionMatrix[10] = -TWO_FLOAT / (zMax - zMin);
		projectionMatrix[12] = -((xMax + xMin) / (xMax - xMin));
		projectionMatrix[13] = -((yMax + yMin) / (yMax - yMin));
		projectionMatrix[14] = -((zMax + zMin) / (zMax - zMin));
		projectionMatrix[15] = ONE_FLOAT;

		return projectionMatrix;
	}

	
	sp_size Mat4::sizeInBytes() const
	{
		return MAT4_LENGTH * sizeof(sp_float);
	}

	
	Mat4 Mat4::clone() const
	{
		Mat4 result;

		std::memcpy(&result, this, sizeof(Mat4));

		return result;
	}

	
	Mat4 Mat4::operator*(const sp_float value)  const
	{
		Mat4 result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] * value;

		return result;
	}

	
	void Mat4::operator*=(const Mat4 &matrix)
	{
		std::memcpy(&this->values, multiply(matrix).values, sizeof(this->values));
	}

	
	Mat4 Mat4::operator*(const Mat4 &matrix) const
	{
		return multiply(matrix);
	}

	
	Vec4 Mat4::operator*(const Vec4 &vector) const
	{
		return multiply(vector);
	}

	
	Mat4 Mat4::operator/(const sp_float value) const
	{
		return Mat4 {
			values[0] / value, values[1] / value, values[2] / value, values[3] / value,
			values[4] / value, values[5] / value, values[6] / value, values[7] / value,
			values[8] / value, values[9] / value, values[10] / value, values[11] / value,
			values[12] / value, values[13] / value, values[14] / value, values[15] / value
		};
	}

	
	void Mat4::operator/=(const sp_float value)
	{
		for (size_t i = 0; i < MAT4_LENGTH; i++)
			values[i] /= value;
	}

	
	Mat4 Mat4::operator-() const
	{
		Mat4 result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = -values[i];

		return result;
	}

	
	Mat4 Mat4::operator+(const Mat4& matrix) const
	{
		Mat4 result;

		for (int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] + matrix[i];

		return result;
	}

	
	Mat4 Mat4::operator+(const sp_float value) const
	{
		Mat4 result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] + value;

		return result;
	}

	
	Mat4 Mat4::operator-(const Mat4& matrix) const
	{
		Mat4 result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] - matrix[i];

		return result;
	}

	
	Mat4 Mat4::operator-(const sp_float value) const
	{
		Mat4 result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] - value;

		return result;
	}

	
	sp_bool Mat4::operator==(const Mat4& matrix) const
	{
		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (values[i] != matrix[i])
				return false;

		return true;
	}

	
	sp_bool Mat4::operator==(const sp_float value) const
	{
		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (values[i] != value)
				return false;

		return true;
	}

	
	sp_bool Mat4::operator!=(const Mat4& matrix) const
	{
		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (values[i] != matrix[i])
				return true;

		return false;
	}

	
	sp_float& Mat4::operator[](const sp_int index)
	{
		sp_assert(index >= 0 && index < MAT4_LENGTH);

		return values[index];
	}

	
	sp_float Mat4::operator[](const sp_int index) const
	{
		sp_assert(index >= 0 && index < MAT4_LENGTH);

		return values[index];
	}

	
	Mat4::operator void*() const
	{
		return (void*) values;
	}

	
	Mat4::operator sp_float*()
	{
		return values;
	}

	
	Mat4::operator sp_float*() const
	{
		return(sp_float*)values;
	}

	
	Mat3 Mat4::toMat3() const
	{
		return Mat3 {
			values[0], values[1], values[2],
				values[4], values[5], values[6],
				values[8], values[9], values[10]
		};
	}

	
	std::string Mat4::toString() const
	{
		return Mat::toString(values, MAT4_LENGTH);
	}

	
	Mat4* Mat4::decomposeLU() const
	{
		Mat4 lowerMatrix = Mat4::identity();
		Mat4 upperMatrix = this->clone();
		Mat4* result = ALLOC_ARRAY(Mat4, 2);

		std::vector<Mat4> elementarInverseMatrixes;
		Mat4 elementarInverseMatrix;

		sp_int rowSize = MAT4_ROW_LENGTH;
		sp_int colSize = MAT4_ROW_LENGTH;

	#if MAJOR_COLUMN_ORDER
		sp_int pivotRowIndex = 0;

		for (sp_int column = 0; column < colSize; column++)
		{
			sp_float pivot = upperMatrix[pivotRowIndex * rowSize + column];
			sp_float pivotOperator = 1 / pivot;

			for (sp_int row = 0; row < rowSize; row++)
				upperMatrix[row * rowSize + column] *= pivotOperator;

			elementarInverseMatrix = Mat4::identity();
			elementarInverseMatrix[pivotRowIndex * rowSize + column] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerColumns = column + 1; lowerColumns < rowSize; lowerColumns++)
			{
				pivot = upperMatrix[pivotRowIndex * rowSize + lowerColumns];
				pivotOperator = -pivot;

				for (int row = 0; row < rowSize; row++)
					upperMatrix[row * rowSize + lowerColumns] += pivotOperator * upperMatrix[row * rowSize + column];

				elementarInverseMatrix = Mat4::identity();
				elementarInverseMatrix[pivotRowIndex * rowSize + lowerColumns] = pivot;
				elementarInverseMatrixes.push_back(elementarInverseMatrix);
			}

			pivotRowIndex++;
		}

		for (sp_int i = 0; sp_size(i) < elementarInverseMatrixes.size(); i++)
			lowerMatrix *= elementarInverseMatrixes[i];
	#else
		sp_int pivotColumnIndex = 0;

		for (sp_int line = 0; line < rowSize; line++)
		{
			sp_float pivot = upperMatrix[line * rowSize + pivotColumnIndex];
			sp_float pivotOperator = 1 / pivot;

			for (sp_int column = 0; column < colSize; column++)
				upperMatrix[line * rowSize + column] *= pivotOperator;

			elementarInverseMatrix = Mat4::identity();
			elementarInverseMatrix[line * rowSize + pivotColumnIndex] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerLines = line + 1; lowerLines < rowSize; lowerLines++)
			{
				pivot = upperMatrix[lowerLines * rowSize + pivotColumnIndex];
				pivotOperator = -pivot;

				for (sp_int column = 0; column < colSize; column++)
					upperMatrix[lowerLines * rowSize + column] += pivotOperator * upperMatrix[line * rowSize + column];

				elementarInverseMatrix = Mat4::identity();
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

	
	Mat4* Mat4::decomposeLDU() const
	{
		Mat4 diagonalMatrix = Mat4::identity();
		Mat4* result = ALLOC_ARRAY(Mat4, 3);

		Mat4* lowerAndUpperMatrixes = decomposeLU();

		Mat4 upperMatrix = lowerAndUpperMatrixes[1];
		unsigned short diagonalIndex = 0;

	#if MAJOR_COLUMN_ORDER
		for (sp_int column = 0; column < MAT4_ROW_LENGTH; column++)
		{
			sp_float pivot = upperMatrix[diagonalIndex * MAT4_ROW_LENGTH + column];

			diagonalMatrix[column * MAT4_ROW_LENGTH + diagonalIndex] = pivot;

			for (sp_int row = column; row < MAT4_ROW_LENGTH; row++)
				upperMatrix[column * MAT4_ROW_LENGTH + row] /= pivot;

			diagonalIndex++;
		}
	#else
		for (sp_int row = 0; row < MAT4_ROW_LENGTH; row++)
		{
			sp_float pivot = upperMatrix[row * MAT4_ROW_LENGTH + diagonalIndex];

			diagonalMatrix[row * MAT4_ROW_LENGTH + diagonalIndex] = pivot;

			for (sp_int column = row; column < MAT4_ROW_LENGTH; column++)
				upperMatrix[row * MAT4_ROW_LENGTH + column] /= pivot;

			diagonalIndex++;
		}
	#endif

		result[0] = lowerAndUpperMatrixes[0];
		result[1] = diagonalMatrix;
		result[2] = upperMatrix;

		ALLOC_RELEASE(lowerAndUpperMatrixes);

		return result;
	}

	
	AutoValueAutoVector4 Mat4::getAutovalueAndAutovector(const sp_short maxIteration) const
	{
		Mat4 matrix = *this;
		Vec4 autovector(ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT);
		sp_float autovalue;

		for (sp_short iterationIndex = 0; iterationIndex < maxIteration; iterationIndex++)
		{
			Vec4 ax = matrix * autovector;
			autovalue = ax.maximum();
			autovector = ax / autovalue;
		}

		return AutoValueAutoVector4{ autovalue, { autovector.x, autovector.y, autovector.z, autovector.w} };
	}

	/*
	Mat3 Mat4::toNormalMatrix()
	{
		return toMat3().transpose().invert();
	}
	*/

}