#include "Mat4.h"

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	Mat4<T>::Mat4(const T defaultValue)
	{
		for (sp_ushort i = 0; i < MAT4_LENGTH; i++)
			values[i] = defaultValue;
	}

	template <typename T>
	Mat4<T>::Mat4(const Vec4<T>& vector1, const Vec4<T>& vector2, const Vec4<T>& vector3, const Vec4<T>& vector4)
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

	template <typename T>
	Mat4<T>::Mat4(T* values)
	{
		std::memcpy(&this->values, values, sizeof(this->values));
	}

	template <typename T>
	Mat4<T>::Mat4(
		const T value11, const T value21, const T value31, const T value41,
		const T value12, const T value22, const T value32, const T value42,
		const T value13, const T value23, const T value33, const T value43,
		const T value14, const T value24, const T value34, const T value44)
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

	template <typename T>
	T* Mat4<T>::getValues()
	{
		return values;
	}

	template <typename T>
	T Mat4<T>::getValue(const sp_int x, const sp_int y) const
	{
		return values[(y-1) * MAT4_ROW_LENGTH + (x-1)];
	}

	template <typename T>
	Vec4<T> Mat4<T>::xAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4<T> {
			values[0 * MAT4_ROW_LENGTH + 0],
				values[1 * MAT4_ROW_LENGTH + 0],
				values[2 * MAT4_ROW_LENGTH + 0],
				values[3 * MAT4_ROW_LENGTH + 0]
		};
	#else
		return Vec4<T> {
			values[0 * MAT4_ROW_LENGTH + 0],
				values[0 * MAT4_ROW_LENGTH + 1],
				values[0 * MAT4_ROW_LENGTH + 2],
				values[0 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	template <typename T>
	Vec4<T> Mat4<T>::yAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4<T> {
			values[0 * MAT4_ROW_LENGTH + 1],
				values[1 * MAT4_ROW_LENGTH + 1],
				values[2 * MAT4_ROW_LENGTH + 1],
				values[3 * MAT4_ROW_LENGTH + 1]
		};
	#else
		return Vec4<T> {
			values[1 * MAT4_ROW_LENGTH + 0],
				values[1 * MAT4_ROW_LENGTH + 1],
				values[1 * MAT4_ROW_LENGTH + 2],
				values[1 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	template <typename T>
	Vec4<T> Mat4<T>::zAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4<T> {
			values[0 * MAT4_ROW_LENGTH + 2],
				values[1 * MAT4_ROW_LENGTH + 2],
				values[2 * MAT4_ROW_LENGTH + 2],
				values[3 * MAT4_ROW_LENGTH + 2]
		};
	#else
		return Vec4<T> {
			values[2 * MAT4_ROW_LENGTH + 0],
				values[2 * MAT4_ROW_LENGTH + 1],
				values[2 * MAT4_ROW_LENGTH + 2],
				values[2 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	template <typename T>
	Vec4<T> Mat4<T>::wAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec4<T> {
			values[0 * MAT4_ROW_LENGTH + 3],
				values[1 * MAT4_ROW_LENGTH + 3],
				values[2 * MAT4_ROW_LENGTH + 3],
				values[3 * MAT4_ROW_LENGTH + 3]
		};
	#else
		return Vec4<T> {
			values[3 * MAT4_ROW_LENGTH + 0],
				values[3 * MAT4_ROW_LENGTH + 1],
				values[3 * MAT4_ROW_LENGTH + 2],
				values[3 * MAT4_ROW_LENGTH + 3]
		};
	#endif
	}

	template <typename T>
	Vec4<T> Mat4<T>::primaryDiagonal() const
	{
		return Vec4<T> {
			values[0],
				values[5],
				values[10],
				values[15]
		};
	}

	template <typename T>
	Vec4<T> Mat4<T>::secondaryDiagonal() const
	{
		return Vec4<T> {
			values[3],
				values[6],
				values[9],
				values[12]
		};
	}

	template <typename T>
	Mat4<T> Mat4<T>::identity()
	{
		static T identityMatrix[MAT4_LENGTH] = {
			T(1), T(0), T(0), T(0),
			T(0), T(1), T(0), T(0),
			T(0), T(0), T(1), T(0),
			T(0), T(0), T(0), T(1)
		};

		Mat4<T> result;
		std::memcpy(&result, identityMatrix, sizeof(values));

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::transpose() const
	{
		Mat4<T> result;

		//copy principal diagonal
		result[0] = values[0];
		result[5] = values[5];
		result[10] = values[10];
		result[15] = values[15];

		//swap others numbers
		T temp = values[1];
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

	template <typename T>
	T Mat4<T>::determinantIJ(int i, int j) const
	{
		sp_int x, y, ii, jj;
		T ret, mat[3][3];

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

	template <typename T>
	T Mat4<T>::cofactorIJ(const sp_int i, const sp_int j) const
	{
		T determinantIJValue = determinantIJ(i, j);

		if (isOdd(i + j))
			determinantIJValue *= -1;

		return determinantIJValue;
	}

	template <typename T>
	T Mat4<T>::determinant() const
	{
		T det = T(0);

		for (sp_int i = 0; i < MAT4_ROW_LENGTH; i++)
		{
			det += (i & 0x1) ? 
				(-values[i] * determinantIJ(0, i)) 
				: (values[i] * determinantIJ(0, i));
		}

		return det;
	}

	template <typename T>
	Mat4<T> Mat4<T>::multiply(const Mat4<T> &matrixB) const
	{
		Mat4<T> result;

	#if MAJOR_COLUMN_ORDER
		for (int line = 0; line < MAT4_ROW_LENGTH; line++)
		{
			const T ai0 = values[(0 * MAT4_ROW_LENGTH) + line];
			const T ai1 = values[(1 * MAT4_ROW_LENGTH) + line];
			const T ai2 = values[(2 * MAT4_ROW_LENGTH) + line];
			const T ai3 = values[(3 * MAT4_ROW_LENGTH) + line];

			T a = matrixB[0];

			result[(0 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(0 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(0 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(0 * MAT4_ROW_LENGTH) + 3];
			result[(1 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(1 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(1 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(1 * MAT4_ROW_LENGTH) + 3];
			result[(2 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(2 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(2 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(2 * MAT4_ROW_LENGTH) + 3];
			result[(3 * MAT4_ROW_LENGTH) + line] = ai0 * matrixB[(3 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(3 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(3 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 3];
		}
	#else 
		for (int column = 0; column < MAT4_ROW_LENGTH; column++)
		{
			T ai0 = values[(column * MAT4_ROW_LENGTH) + 0];
			T ai1 = values[(column * MAT4_ROW_LENGTH) + 1];
			T ai2 = values[(column * MAT4_ROW_LENGTH) + 2];
			T ai3 = values[(column * MAT4_ROW_LENGTH) + 3];

			result[(column * MAT4_ROW_LENGTH) + 0] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 0] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 0] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 0];
			result[(column * MAT4_ROW_LENGTH) + 1] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 1] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 1] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 1] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 1];
			result[(column * MAT4_ROW_LENGTH) + 2] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 2] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 2] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 2] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 2];
			result[(column * MAT4_ROW_LENGTH) + 3] = ai0 * matrixB[(0 * MAT4_ROW_LENGTH) + 3] + ai1 * matrixB[(1 * MAT4_ROW_LENGTH) + 3] + ai2 * matrixB[(2 * MAT4_ROW_LENGTH) + 3] + ai3 * matrixB[(3 * MAT4_ROW_LENGTH) + 3];
		}
	#endif

		return result;
	}

	template <typename T>
	Vec4<T> Mat4<T>::multiply(const Vec4<T> &vector) const
	{
		Vec4<T> result;

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

	template <typename T>
	Mat4<T> Mat4<T>::invert() const
	{
		Mat4<T> mInverse;

		sp_int i, j;
		T det = T(0);
		T detij;

		// calculate 4x4 determinant
		for (i = 0; i < 4; i++)
		{
			det += (i & 0x1) ? (-values[i] * determinantIJ(0, i)) : (values[i] * determinantIJ(0, i));
		}
		det = T(1) / det;

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

	template <typename T>
	Mat4<T> Mat4<T>::createScale(const T xScale, const T yScale, const T zScale)
	{
		return Mat4<T>(
			xScale, 0.0f, 0.0f, 0.0f,
			0.0f, yScale, 0.0f, 0.0f,
			0.0f, 0.0f, zScale, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
			);
	}

	template <typename T>
	Mat4<T> Mat4<T>::createScale(const Vec3<T>& scale)
	{
		return Mat4<T>(
			ONE_FLOAT + scale.x, 0.0f, 0.0f, 0.0f,
			0.0f, ONE_FLOAT + scale.y, 0.0f, 0.0f,
			0.0f, 0.0f, ONE_FLOAT + scale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
		);
	}

	template <typename T>
	void Mat4<T>::scale(const T xScale, const T yScale, const T zScale)
	{
		values[0] *= xScale;
		values[5] *= yScale;
		values[10] *= zScale;
	}

	template <typename T>
	Mat4<T> Mat4<T>::createRotate(const T angleRadians, const T x, const T y, const T z)
	{
		const T sineAngle = T(sin(angleRadians));
		const T cosineAngle = T(cos(angleRadians));

		const T mag = T(sqrt(x*x + y * y + z * z));

		if (mag == 0.0f)
			return Mat4::identity();

		// Rotation matrix is normalized
		const T x1 = x / mag;
		const T y1 = y / mag;
		const T z1 = z / mag;

		const T xx = x * x;
		const T yy = y * y;
		const T zz = z * z;
		const T xy = x * y;
		const T yz = y * z;
		const T zx = z * x;
		const T xs = x * sineAngle;
		const T ys = y * sineAngle;
		const T zs = z * sineAngle;
		const T one_c = T(1) - cosineAngle;

		Mat4<T> result;

	#define M(row,col)  result[col * MAT4_ROW_LENGTH + row]
		M(0, 0) = (one_c * xx) + cosineAngle;
		M(0, 1) = (one_c * xy) - zs;
		M(0, 2) = (one_c * zx) + ys;
		M(0, 3) = T(0);

		M(1, 0) = (one_c * xy) + zs;
		M(1, 1) = (one_c * yy) + cosineAngle;
		M(1, 2) = (one_c * yz) - xs;
		M(1, 3) = T(0);

		M(2, 0) = (one_c * zx) - ys;
		M(2, 1) = (one_c * yz) + xs;
		M(2, 2) = (one_c * zz) + cosineAngle;
		M(2, 3) = T(0);

		M(3, 0) = T(0);
		M(3, 1) = T(0);
		M(3, 2) = T(0);
		M(3, 3) = T(1);
	#undef M

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::createTranslate(const T x, const T y, const T z)
	{
		Mat4<T> result = Mat4<T>::identity();

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

	template <typename T>
	Mat4<T> Mat4<T>::createTranslate(const Vec3<T>& position)
	{
		Mat4<T> result = Mat4<T>::identity();

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

	template <typename T>
	Mat4<T> Mat4<T>::createOrthographicMatrix(const T xMin, const T xMax, const T yMin, const T yMax, const T zMin, const T zMax)
	{
		Mat4<T> projectionMatrix = Mat4<T>::identity();

		projectionMatrix[0] = T(2) / (xMax - xMin);
		projectionMatrix[5] = T(2) / (yMax - yMin);
		projectionMatrix[10] = T(-2) / (zMax - zMin);
		projectionMatrix[12] = -((xMax + xMin) / (xMax - xMin));
		projectionMatrix[13] = -((yMax + yMin) / (yMax - yMin));
		projectionMatrix[14] = -((zMax + zMin) / (zMax - zMin));
		projectionMatrix[15] = T(1);

		return projectionMatrix;
	}

	template <typename T>
	sp_size Mat4<T>::sizeInBytes() const
	{
		return MAT4_LENGTH * sizeof(T);
	}

	template <typename T>
	Mat4<T> Mat4<T>::clone() const
	{
		Mat4<T> result;

		std::memcpy(&result, this, sizeof(Mat4<T>));

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator*(const T value)  const
	{
		Mat4<T> result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] * value;

		return result;
	}

	template <typename T>
	void Mat4<T>::operator*=(const Mat4<T> &matrix)
	{
		std::memcpy(&this->values, multiply(matrix).values, sizeof(this->values));
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator*(const Mat4<T> &matrix) const
	{
		return multiply(matrix);
	}

	template <typename T>
	Vec4<T> Mat4<T>::operator*(const Vec4<T> &vector) const
	{
		return multiply(vector);
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator/(const T value) const
	{
		return Mat4<T> {
			values[0] / value, values[1] / value, values[2] / value, values[3] / value,
			values[4] / value, values[5] / value, values[6] / value, values[7] / value,
			values[8] / value, values[9] / value, values[10] / value, values[11] / value,
			values[12] / value, values[13] / value, values[14] / value, values[15] / value
		};
	}

	template <typename T>
	void Mat4<T>::operator/=(const T value)
	{
		for (size_t i = 0; i < MAT4_LENGTH; i++)
			values[i] /= value;
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator-() const
	{
		Mat4<T> result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = -values[i];

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator+(const Mat4<T>& matrix) const
	{
		Mat4<T> result;

		for (int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] + matrix[i];

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator+(const T value) const
	{
		Mat4<T> result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] + value;

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator-(const Mat4<T>& matrix) const
	{
		Mat4<T> result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] - matrix[i];

		return result;
	}

	template <typename T>
	Mat4<T> Mat4<T>::operator-(const T value) const
	{
		Mat4<T> result;

		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			result[i] = values[i] - value;

		return result;
	}

	template <typename T>
	sp_bool Mat4<T>::operator==(const Mat4<T>& matrix) const
	{
		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (values[i] != matrix[i])
				return false;

		return true;
	}

	template <typename T>
	sp_bool Mat4<T>::operator==(const T value) const
	{
		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (values[i] != value)
				return false;

		return true;
	}

	template <typename T>
	sp_bool Mat4<T>::operator!=(const Mat4<T>& matrix) const
	{
		for (sp_int i = 0; i < MAT4_LENGTH; i++)
			if (values[i] != matrix[i])
				return true;

		return false;
	}

	template <typename T>
	T& Mat4<T>::operator[](const sp_int index)
	{
		sp_assert(index >= 0 && index < MAT4_LENGTH);

		return values[index];
	}

	template <typename T>
	T Mat4<T>::operator[](const sp_int index) const
	{
		sp_assert(index >= 0 && index < MAT4_LENGTH);

		return values[index];
	}

	template <typename T>
	Mat4<T>::operator void*() const
	{
		return (void*) values;
	}

	template <typename T>
	Mat4<T>::operator T*()
	{
		return values;
	}

	template <typename T>
	Mat4<T>::operator T*() const
	{
		return(T*)values;
	}

	template <typename T>
	Mat3<T> Mat4<T>::toMat3() const
	{
		return Mat3<T> {
			values[0], values[1], values[2],
				values[4], values[5], values[6],
				values[8], values[9], values[10]
		};
	}

	template <typename T>
	std::string Mat4<T>::toString() const
	{
		return Mat<T>::toString(values, MAT4_LENGTH);
	}

	template <typename T>
	Mat4<T>* Mat4<T>::decomposeLU() const
	{
		Mat4<T> lowerMatrix = Mat4<T>::identity();
		Mat4<T> upperMatrix = this->clone();
		Mat4<T>* result = ALLOC_ARRAY(Mat4<T>, 2);

		std::vector<Mat4<T>> elementarInverseMatrixes;
		Mat4<T> elementarInverseMatrix;

		int rowSize = MAT4_ROW_LENGTH;
		int colSize = MAT4_ROW_LENGTH;

	#if MAJOR_COLUMN_ORDER
		sp_int pivotRowIndex = 0;

		for (sp_int column = 0; column < colSize; column++)
		{
			T pivot = upperMatrix[pivotRowIndex * rowSize + column];
			T pivotOperator = 1 / pivot;

			for (sp_int row = 0; row < rowSize; row++)
				upperMatrix[row * rowSize + column] *= pivotOperator;

			elementarInverseMatrix = Mat4<T>::identity();
			elementarInverseMatrix[pivotRowIndex * rowSize + column] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerColumns = column + 1; lowerColumns < rowSize; lowerColumns++)
			{
				pivot = upperMatrix[pivotRowIndex * rowSize + lowerColumns];
				pivotOperator = -pivot;

				for (int row = 0; row < rowSize; row++)
					upperMatrix[row * rowSize + lowerColumns] += pivotOperator * upperMatrix[row * rowSize + column];

				elementarInverseMatrix = Mat4<T>::identity();
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
			T pivot = upperMatrix[line * rowSize + pivotColumnIndex];
			T pivotOperator = 1 / pivot;

			for (sp_int column = 0; column < colSize; column++)
				upperMatrix[line * rowSize + column] *= pivotOperator;

			elementarInverseMatrix = Mat4<T>::identity();
			elementarInverseMatrix[line * rowSize + pivotColumnIndex] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerLines = line + 1; lowerLines < rowSize; lowerLines++)
			{
				pivot = upperMatrix[lowerLines * rowSize + pivotColumnIndex];
				pivotOperator = -pivot;

				for (int column = 0; column < colSize; column++)
					upperMatrix[lowerLines * rowSize + column] += pivotOperator * upperMatrix[line * rowSize + column];

				elementarInverseMatrix = Mat4<T>::identity();
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

	template <typename T>
	Mat4<T>* Mat4<T>::decomposeLDU() const
	{
		Mat4<T> diagonalMatrix = Mat4<T>::identity();
		Mat4<T>* result = ALLOC_ARRAY(Mat4<T>, 3);

		Mat4<T>* lowerAndUpperMatrixes = decomposeLU();

		Mat4<T> upperMatrix = lowerAndUpperMatrixes[1];
		unsigned short diagonalIndex = 0;

	#if MAJOR_COLUMN_ORDER
		for (sp_int column = 0; column < MAT4_ROW_LENGTH; column++)
		{
			T pivot = upperMatrix[diagonalIndex * MAT4_ROW_LENGTH + column];

			diagonalMatrix[column * MAT4_ROW_LENGTH + diagonalIndex] = pivot;

			for (sp_int row = column; row < MAT4_ROW_LENGTH; row++)
				upperMatrix[column * MAT4_ROW_LENGTH + row] /= pivot;

			diagonalIndex++;
		}
	#else
		for (sp_int row = 0; row < MAT4_ROW_LENGTH; row++)
		{
			T pivot = upperMatrix[row * MAT4_ROW_LENGTH + diagonalIndex];

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

	template <typename T>
	AutovalueAutovector4<T> Mat4<T>::getAutovalueAndAutovector(const sp_short maxIteration) const
	{
		Mat4<T> matrix = *this;
		Vec4<T> autovector = { T(1), T(1), T(1), T(1) };
		T autovalue;

		for (sp_short iterationIndex = 0; iterationIndex < maxIteration; iterationIndex++)
		{
			Vec4<T> ax = matrix * autovector;
			autovalue = ax.maximum();
			autovector = ax / autovalue;
		}

		return AutovalueAutovector4<T>{ autovalue, autovector };
	}

	/*
	template <typename T>
	Mat3<T> Mat4<T>::toNormalMatrix()
	{
		return toMat3().transpose().invert();
	}
	*/

	template class Mat4<sp_float>;
	template class Mat4<sp_double>;
}