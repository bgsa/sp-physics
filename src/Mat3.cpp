#include "Mat3.h"

#define M11 (0)
#define M12 (1)
#define M13 (2)

#define M21 (3)
#define M22 (4)
#define M23 (5)

#define M31 (6)
#define M32 (7)
#define M33 (8)

namespace NAMESPACE_PHYSICS
{
	
	Mat3::Mat3()
	{
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

	sp_float Mat3::index(const sp_int x, const sp_int y) const
	{
		return values[(y-1u) * MAT3_ROW_LENGTH + (x-1u)];
	}
	
	Vec3 Mat3::axis(const sp_int index) const
	{
	#if MAJOR_COLUMN_ORDER

		return Vec3(
			values[index],
			values[index + MAT3_ROW_LENGTH],
			values[index + MAT3_ROW_LENGTH + MAT3_ROW_LENGTH]
		);
	#else

		return Vec3(
			values[index * MAT3_ROW_LENGTH],
			values[index * MAT3_ROW_LENGTH + 1],
			values[index * MAT3_ROW_LENGTH + 2]
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
		static sp_float identityMatrix[MAT3_LENGTH] = {
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

	void Mat3::primaryDiagonal(Vec3* output) const
	{
		output[0].x = values[0];
		output[0].y = values[4];
		output[0].z = values[8];
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

	sp_bool Mat3::isPositiveDefinite() const
	{
		return (values[M33] > ZERO_FLOAT)
			&& (values[M22] * values[M33] - (values[M23] * values[M32])) > ZERO_FLOAT
			&& (determinant() > ZERO_FLOAT);
	}

	Mat3 Mat3::multiply(const Mat3& matrixB) const
	{
		Mat3 result;

	#if MAJOR_COLUMN_ORDER
		for (int line = 0; line < MAT3_ROW_LENGTH; line++)
		{
			sp_float ai0 = values[line];
			sp_float ai1 = values[MAT3_ROW_LENGTH + line];
			sp_float ai2 = values[MAT3_TWO_ROW_LENGTH + line];

			result[line] = ai0 * matrixB[0] + ai1 * matrixB[1] + ai2 * matrixB[2];
			result[MAT3_ROW_LENGTH + line] = ai0 * matrixB[MAT3_ROW_LENGTH] + ai1 * matrixB[MAT3_ROW_LENGTH + 1] + ai2 * matrixB[MAT3_ROW_LENGTH + 2];
			result[MAT3_TWO_ROW_LENGTH + line] = ai0 * matrixB[MAT3_TWO_ROW_LENGTH] + ai1 * matrixB[MAT3_TWO_ROW_LENGTH + 1] + ai2 * matrixB[MAT3_TWO_ROW_LENGTH + 2];
		}
	#else
		for (int column = 0; column < MAT3_ROW_LENGTH; column++)
		{
			sp_float ai0 = values[(column * MAT3_ROW_LENGTH) + 0];
			sp_float ai1 = values[(column * MAT3_ROW_LENGTH) + 1];
			sp_float ai2 = values[(column * MAT3_ROW_LENGTH) + 2];

			result[(column * MAT3_ROW_LENGTH) + 0] = ai0 * matrixB[(0 * MAT3_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT3_ROW_LENGTH) + 0] + ai2 * matrixB[(2 * MAT3_ROW_LENGTH) + 0];
			result[(column * MAT3_ROW_LENGTH) + 1] = ai0 * matrixB[(0 * MAT3_ROW_LENGTH) + 1] + ai1 * matrixB[(1 * MAT3_ROW_LENGTH) + 1] + ai2 * matrixB[(2 * MAT3_ROW_LENGTH) + 1];
			result[(column * MAT3_ROW_LENGTH) + 2] = ai0 * matrixB[(0 * MAT3_ROW_LENGTH) + 2] + ai1 * matrixB[(1 * MAT3_ROW_LENGTH) + 2] + ai2 * matrixB[(2 * MAT3_ROW_LENGTH) + 2];
		}
	#endif

		return result;
	}

	void Mat3::multiply(const Vec3& vector, Vec3& output) const
	{
		output.x = values[0] * vector[0] + values[1] * vector[1] + values[2] * vector[2];
		output.y = values[MAT3_ROW_LENGTH] * vector[0] + values[MAT3_ROW_LENGTH + 1] * vector[1] + values[MAT3_ROW_LENGTH + 2] * vector[2];
		output.z = values[MAT3_TWO_ROW_LENGTH] * vector[0] + values[MAT3_TWO_ROW_LENGTH + 1] * vector[1] + values[MAT3_TWO_ROW_LENGTH + 2] * vector[2];
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

		result[0 * MAT3_ROW_LENGTH + 0] = (one_c * xx) + cosine;
		result[1 * MAT3_ROW_LENGTH + 0] = (one_c * xy) - zs;
		result[2 * MAT3_ROW_LENGTH + 0] = (one_c * zx) + ys;

		result[0 * MAT3_ROW_LENGTH + 1] = (one_c * xy) + zs;
		result[1 * MAT3_ROW_LENGTH + 1] = (one_c * yy) + cosine;
		result[2 * MAT3_ROW_LENGTH + 1] = (one_c * yz) - xs;

		result[0 * MAT3_ROW_LENGTH + 2] = (one_c * zx) - ys;
		result[1 * MAT3_ROW_LENGTH + 2] = (one_c * yz) + xs;
		result[2 * MAT3_ROW_LENGTH + 2] = (one_c * zz) + cosine;

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

	Mat3 Mat3::invert() const
	{
		const sp_float det = ONE_FLOAT / determinant();
		Mat3 matrixInverse;

		for (int i = 0; i < MAT3_ROW_LENGTH; i++)
			for (int j = 0; j < MAT3_ROW_LENGTH; j++)
			{
				const sp_float detij = determinantIJ(j, i);

				if ((i + j) & 0x1)
					matrixInverse[i * MAT3_ROW_LENGTH + j] = -detij * det;
				else
					matrixInverse[i * MAT3_ROW_LENGTH + j] = detij * det;
			}

		return matrixInverse;
	}

	sp_bool Mat3::isIdentity() const
	{
		Mat3 identityMatrix = Mat3::identity();

		for (int i = 0; i < MAT3_LENGTH; i++)
			if (values[i] != identityMatrix[i])
				return false;

		return true;
	}

	sp_size Mat3::sizeInBytes() const
	{
		return MAT3_LENGTH * SIZEOF_FLOAT;
	}

#if defined(WINDOWS) && defined(ENV_64BITS)
	
	T& Mat3::operator[](sp_size index)
	{
		sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");

		return values[index];
	}
	
	T Mat3::operator[](sp_size index) const
	{
		sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");

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

		for (int i = 0; i < MAT3_LENGTH; i++)
			result[i] = -values[i];

		return result;
	}

	
	Mat3 Mat3::operator-(const Mat3& matrix) const
	{
		Mat3 result;

		for (int i = 0; i < MAT3_LENGTH; i++)
			result[i] = values[i] - matrix[i];

		return result;
	}

	
	Mat3 Mat3::operator+(const Mat3& matrix) const
	{
		Mat3 result;

		for (int i = 0; i < MAT3_LENGTH; i++)
			result[i] = values[i] + matrix[i];

		return result;
	}

	
	Mat3 Mat3::operator*(const Mat3& matrix) const
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
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			if (values[i] != matrix[i])
				return false;

		return true;
	}

	
	sp_bool Mat3::operator!=(const Mat3& matrix) const
	{
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			if (values[i] != matrix[i])
				return true;

		return false;
	}

	
	sp_bool Mat3::operator==(const sp_float value) const
	{
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			if (values[i] != value)
				return false;

		return true;
	}

	
	std::string Mat3::toString() const
	{
		return Mat::toString(values, MAT3_LENGTH);
	}

	
	void Mat3::decomposeLU(Mat3* lower, Mat3* upper) const
	{
#define lowerMatrix lower[0]
#define upperMatrix upper[0]

		std::memcpy(lower, Mat3Zeros, sizeof(Mat3));
		std::memcpy(upper, this, sizeof(Mat3));

		lowerMatrix[M11] = values[M11];
		lowerMatrix[M21] = values[M21];
		lowerMatrix[M31] = values[M31];

		upperMatrix[M12] = div(values[M12], upperMatrix[M11]);
		upperMatrix[M13] = div(values[M13], upperMatrix[M11]);
		upperMatrix[M11] = ONE_FLOAT;

		upperMatrix[M22] = -upperMatrix[M21] * upperMatrix[M12] + upperMatrix[M22];
		upperMatrix[M23] = -upperMatrix[M21] * upperMatrix[M13] + upperMatrix[M23];
		upperMatrix[M21] = ZERO_FLOAT;

		upperMatrix[M32] = -upperMatrix[M31] * upperMatrix[M12] + upperMatrix[M32];
		upperMatrix[M33] = -upperMatrix[M31] * upperMatrix[M13] + upperMatrix[M33];
		upperMatrix[M31] = ZERO_FLOAT;

		lowerMatrix[M22] = upperMatrix[M22];
		upperMatrix[M23] = div(upperMatrix[M23], upperMatrix[M22]);
		upperMatrix[M22] = ONE_FLOAT;

		lowerMatrix[M32] = upperMatrix[M32];
		upperMatrix[M33] = (-upperMatrix[M32] * upperMatrix[M23]) + upperMatrix[M33];
		lowerMatrix[M33] = upperMatrix[M33];
		upperMatrix[M32] = ZERO_FLOAT;
		upperMatrix[M33] = ONE_FLOAT;

		sp_assert(upperMatrix * lowerMatrix == *this, "ApplicationException");

#undef upperMatrix
#undef lowerMatrix
	}
	
	void Mat3::decomposeLDU(Mat3* lower, Mat3* diagonal, Mat3* upper) const
	{
		decomposeLU(lower, upper);

		std::memcpy(diagonal, Mat3Zeros, sizeof(Mat3));

		diagonal[0][M11] = lower[0][M11];
		diagonal[0][M22] = lower[0][M22];
		diagonal[0][M33] = lower[0][M33];

		lower[0][M21] = div(lower[0][M21], lower[0][M11]);
		lower[0][M31] = div(lower[0][M31], lower[0][M11]);

		lower[0][M32] = div(lower[0][M32], lower[0][M22]);

		lower[0][M11] = lower[0][M22] = lower[0][M33] = ONE_FLOAT;

		sp_assert(upper[0] * diagonal[0] * lower[0] == *this, "ApplicationException");
	}

	void Mat3::decomposeLLt(Mat3* lower, Mat3* lowerTransposed) const
	{
#define m lower[0]
		std::memcpy(lower, Mat3Zeros, sizeof(Mat3));
	
		m[M11] = sqrtf(values[M11]);
		m[M21] = div(values[M21], m[M11]);
		m[M31] = div(values[M31], m[M11]);

		m[M22] = sqrtf(values[M22] - m[M21] * m[M21]);

		m[M32] = (values[M32] - m[M21] * m[M31]) / m[M22];

		m[M33] = sqrtf(values[M33] - m[M31] * m[M31] - m[M32] * m[M32]);

		NAMESPACE_PHYSICS::transpose(m, lowerTransposed);

		sp_assert(lowerTransposed[0] * lower[0] == *this, "ApplicationException");
#undef m
	}

	void Mat3::polyname(Vec4* output) const
	{
		const sp_float s1 = trace();
		const sp_float a1 = s1;
		
		Mat3 AxA = multiply(*this);
		
		const sp_float s2 = AxA.trace();
		const sp_float a2 = HALF_FLOAT * (s2 - a1 * s1);

		Mat3 AxAxA = this->multiply(AxA);

		const sp_float s3 = AxAxA.trace();
		const sp_float a3 = ONE_OVER_THREE * (s3 - a1 * s2 - a2 * s1);
		
		output[0].x = -ONE_FLOAT;
		output[0].y = a1;
		output[0].z = a2;
		output[0].w = a3;
	}

	void Mat3::sqrt(Mat3* output) const
	{
		const Eigen::MatrixXf m = Eigen::Map<Eigen::MatrixXf>(*this, MAT3_ROW_LENGTH, MAT3_ROW_LENGTH);

		Eigen::MatrixXf mr = m.sqrt();

		std::memcpy(output, mr.data(), sizeof(Mat3));

		/*
		sp_uint iterations;

		Mat3 temp = *this * transpose();

		std::string content = Mat::toString((sp_float*)&temp, 3);
		sp_log_debug1snl(content.c_str());

		Mat3 diag;
		temp.diagonalize(&diag, &iterations, SP_EPSILON_THREE_DIGITS);

		content = Mat::toString((sp_float*)&diag, 3);
		sp_log_debug1snl(content.c_str());


		Vec3 _autoValues;
		diag.autoValues(&_autoValues, &iterations, SP_EPSILON_THREE_DIGITS);

		Vec3 _autoVectors[3];
		diag.autoVectors(_autoValues, _autoVectors);

		Mat3 autoVectorMatrix = {
			_autoVectors[0].x, _autoVectors[1].x, _autoVectors[2].x,
			_autoVectors[0].y, _autoVectors[1].y, _autoVectors[2].y,
			_autoVectors[0].z, _autoVectors[1].z, _autoVectors[2].z
		};

		Mat3 autoVectorMatrixInverse = autoVectorMatrix.invert();

		Mat3 sqrtDiagonalMatrix = autoVectorMatrixInverse* * this* autoVectorMatrix;
		sqrtDiagonalMatrix[M11] = sqrtf(sqrtDiagonalMatrix[M11]);
		sqrtDiagonalMatrix[M22] = sqrtf(sqrtDiagonalMatrix[M22]);
		sqrtDiagonalMatrix[M33] = sqrtf(sqrtDiagonalMatrix[M33]);

		Mat3 result = autoVectorMatrix* sqrtDiagonalMatrix * autoVectorMatrixInverse;
		*/
	}

	void Mat3::eigenValues(Vec3& output, sp_uint& iterations, const sp_float _epsilon) const
	{
		Mat3 Q, R, A;
		decomposeQR(Q, R, iterations, _epsilon);

		NAMESPACE_PHYSICS::multiply(Q, R, A);

		output.x = A[M11];
		output.y = A[M22];
		output.z = A[M33];
	}

	void Mat3::eigenValuesAndVectors(Vec3& eigenValues, Mat3& eigenVectors, sp_uint& iterations, const sp_uint maxIterations, const sp_float _epsilon) const
	{
		//sp_assert(isTridiagonal(), "InvalidOperationException");

		iterations = ZERO_UINT;

		Mat3 A, Q, R;
		std::memcpy(&A, this, sizeof(Mat3));
		std::memcpy(eigenVectors, Mat3Identity, sizeof(Mat3));

		sp_float r, cos0, sin0;

		do
		{
			Mat3 P1, P2;
			std::memcpy(&P1, &Mat3Identity, sizeof(Mat3));
			std::memcpy(&P2, &Mat3Identity, sizeof(Mat3));

			if (A[M21] != ZERO_FLOAT)
			{
				r = ONE_FLOAT / sqrtf(A.values[M21] * A.values[M21] + A.values[M11] * A.values[M11]);
				cos0 = A.values[M11] * r;
				sin0 = A.values[M21] * r;

				P1[M11] = P1[M22] = cos0;
				P1[M12] = sin0;
				P1[M21] = -sin0;

				NAMESPACE_PHYSICS::multiply(A, P1, A);
			}

			if (A[M32] != ZERO_FLOAT)
			{
				r = ONE_FLOAT / sqrtf(A.values[M32] * A.values[M32] + A.values[M22] * A.values[M22]);
				cos0 = A.values[M22] * r;
				sin0 = A.values[M32] * r;

				P2[M22] = P2[M33] = cos0;
				P2[M23] = sin0;
				P2[M32] = -sin0;
			}

			NAMESPACE_PHYSICS::multiply(A, P2, R);

			NAMESPACE_PHYSICS::multiply(P2.transpose(), P1.transpose(), Q); // Matrix Q = P(i) * P(i+1) * P(i+2) * ... * P(i+N)

			NAMESPACE_PHYSICS::multiply(Q, R, A);

			NAMESPACE_PHYSICS::multiply(eigenVectors, Q, eigenVectors);
			
			iterations++;
		} while ((!NAMESPACE_FOUNDATION::isCloseEnough(A[M21], ZERO_FLOAT, _epsilon)
			|| !NAMESPACE_FOUNDATION::isCloseEnough(A[M32], ZERO_FLOAT, _epsilon))
			&& iterations < maxIterations);

		R.primaryDiagonal(&eigenValues);

		sp_assert(isCloseEnough(Q.transpose() * Q, Mat3Identity), "ApplicationException");
	}

	void Mat3::eigenValuesAndVectorsMax(sp_float& eigenValue, Vec3& eigenVector, const sp_ushort maxIteration) const
	{
		Mat3 matrix;
		std::memcpy(&matrix, this, sizeof(Mat3));

		std::memcpy(&eigenVector, Vec3Ones, sizeof(Vec3));

		for (sp_short iterationIndex = 0; iterationIndex < maxIteration; iterationIndex++)
		{
			Vec3 ax;
			matrix.multiply(eigenVector, ax);

			eigenValue = ax.maximum();
			eigenVector = ax / eigenVector;
		}
	}

	void Mat3::decomposeQR(Mat3& Q, Mat3& R, sp_uint& iterations, const sp_float _epsilon) const
	{
		sp_assert(isTridiagonal(), "InvalidOperationException");
		iterations = ZERO_UINT;

		Mat3 A;
		std::memcpy(&A, this, sizeof(Mat3));

		sp_float r, cos0, sin0;

		do
		{
			Mat3 P1, P2;
			std::memcpy(&P1, &Mat3Identity, sizeof(Mat3));
			std::memcpy(&P2, &Mat3Identity, sizeof(Mat3));

			if (!NAMESPACE_FOUNDATION::isCloseEnough(A[M21], ZERO_FLOAT, _epsilon))
			{
				r = ONE_FLOAT / sqrtf(A.values[M21] * A.values[M21] + A.values[M11] * A.values[M11]);
				cos0 = A.values[M11] * r;
				sin0 = A.values[M21] * r;

				P1[M11] = P1[M22] = cos0;
				P1[M12] = sin0;
				P1[M21] = -sin0;

				NAMESPACE_PHYSICS::multiply(A, P1, A);
			}

			if (!NAMESPACE_FOUNDATION::isCloseEnough(A[M32], ZERO_FLOAT, _epsilon))
			{
				r = ONE_FLOAT / sqrtf(A.values[M32] * A.values[M32] + A.values[M22] * A.values[M22]);
				cos0 = A.values[M22] * r;
				sin0 = A.values[M32] * r;

				P2[M22] = P2[M33] = cos0;
				P2[M23] = sin0;
				P2[M32] = -sin0;
			}

			NAMESPACE_PHYSICS::multiply(A, P2, R);

			NAMESPACE_PHYSICS::multiply(P2.transpose(), P1.transpose(), Q); // Matrix Q = P(i) * P(i+1) * P(i+2) * ... * P(i+N)

			NAMESPACE_PHYSICS::multiply(Q, R, A);

			iterations++;
		} while (!NAMESPACE_FOUNDATION::isCloseEnough(A[M21], ZERO_FLOAT, _epsilon)
			|| !NAMESPACE_FOUNDATION::isCloseEnough(A[M32], ZERO_FLOAT, _epsilon));

		sp_assert(isCloseEnough(Q.transpose() * Q, Mat3Identity), "ApplicationException");
	}
	
	void Mat3::diagonalize(Mat3& output, sp_uint& iterationCounter, const sp_float errorMargin) const
	{
		sp_assert(isSymetric(), "InvalidArgumentException");

		Mat3 matrix;
		clone(&matrix);

		Vec3 result = Vec3Zeros;
		Vec3 previousResult = Vec3Ones;
		iterationCounter = ZERO_UINT;

		while (!isCloseEnough(previousResult, result, errorMargin))
		{
			matrix.primaryDiagonal(&previousResult);

			sp_uint columnIndex;
			sp_uint rowIndex = ZERO_UINT;
			sp_float maxElement;

			if (fabsf(matrix[1]) > fabsf(matrix[2]))
			{
				maxElement = matrix[1];
				columnIndex = ONE_UINT;
			}
			else
			{
				maxElement = matrix[2];
				columnIndex = TWO_UINT;
			}

			if (fabsf(matrix[5]) > fabsf(maxElement))
			{
				maxElement = matrix[5];
				columnIndex = TWO_UINT;
				rowIndex = ONE_UINT;
			}

			const sp_float aqq = matrix.index(columnIndex + ONE_UINT, columnIndex + ONE_UINT);
			const sp_float app = matrix.index(rowIndex + ONE_UINT, rowIndex + ONE_UINT);

			const sp_float theta = (aqq - app) / (TWO_FLOAT * maxElement);
			const sp_float t
				= NAMESPACE_FOUNDATION::isCloseEnough(theta, ZERO_FLOAT, DefaultErrorMargin)
				? ONE_FLOAT
				: ONE_FLOAT / (theta + sign(theta) * sqrtf(theta * theta + ONE_FLOAT));

			const sp_float cosTheta = ONE_FLOAT / (sqrtf(ONE_FLOAT + t * t));
			const sp_float sinTheta = t / (sqrtf(ONE_FLOAT + t * t));

			rotateJacobi(matrix, sinTheta, cosTheta, rowIndex, columnIndex, matrix);
		
			matrix.primaryDiagonal(&result);
			iterationCounter ++;
		}

		std::memcpy(&output, &Mat3Identity, sizeof(Mat3));
		output[M11] = result.x;
		output[M22] = result.y;
		output[M33] = result.z;
	}

	void multiply(const Vec3& v1, const Vec3& v2, Mat3* output)
	{
		output[0][0] = v1.x * v2.x;
		output[0][1] = v1.x * v2.y;
		output[0][2] = v1.x * v2.z;

		output[0][3] = v1.y * v2.x;
		output[0][4] = v1.y * v2.y;
		output[0][5] = v1.y * v2.z;

		output[0][6] = v1.z * v2.x;
		output[0][7] = v1.z * v2.y;
		output[0][8] = v1.z * v2.z;
	}

	void multiply(const Mat3& input, const sp_float value, Mat3* output)
	{
		output[0][0] = input[0] * value;
		output[0][1] = input[1] * value;
		output[0][2] = input[2] * value;

		output[0][3] = input[3] * value;
		output[0][4] = input[4] * value;
		output[0][5] = input[5] * value;

		output[0][6] = input[6] * value;
		output[0][7] = input[7] * value;
		output[0][8] = input[8] * value;
	}

	void multiply(const Mat3& matrix, const Vec3& vector, Vec3& output)
	{
		output.x = matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2];
		output.y = matrix[MAT3_ROW_LENGTH] * vector[0] + matrix[MAT3_ROW_LENGTH + 1] * vector[1] + matrix[MAT3_ROW_LENGTH + 2] * vector[2];
		output.z = matrix[MAT3_TWO_ROW_LENGTH] * vector[0] + matrix[MAT3_TWO_ROW_LENGTH + 1] * vector[1] + matrix[MAT3_TWO_ROW_LENGTH + 2] * vector[2];
	}

	void multiply(const Mat3& matrixA, const Mat3& matrixB, Mat3& output)
	{
#if MAJOR_COLUMN_ORDER
		for (register sp_int line = 0; line < MAT3_ROW_LENGTH; line++)
		{
			const sp_float ai0 = matrixA[line];
			const sp_float ai1 = matrixA[MAT3_ROW_LENGTH + line];
			const sp_float ai2 = matrixA[MAT3_TWO_ROW_LENGTH + line];

			output[line] = ai0 * matrixB[0] + ai1 * matrixB[1] + ai2 * matrixB[2];
			output[MAT3_ROW_LENGTH + line] = ai0 * matrixB[MAT3_ROW_LENGTH] + ai1 * matrixB[MAT3_ROW_LENGTH + 1] + ai2 * matrixB[MAT3_ROW_LENGTH + 2];
			output[MAT3_TWO_ROW_LENGTH + line] = ai0 * matrixB[MAT3_TWO_ROW_LENGTH] + ai1 * matrixB[MAT3_TWO_ROW_LENGTH + 1] + ai2 * matrixB[MAT3_TWO_ROW_LENGTH + 2];
		}
#else
		for (register sp_int column = 0; column < MAT3_ROW_LENGTH; column++)
		{
			const sp_float ai0 = matrixA[(column * MAT3_ROW_LENGTH) + 0];
			const sp_float ai1 = matrixA[(column * MAT3_ROW_LENGTH) + 1];
			const sp_float ai2 = matrixA[(column * MAT3_ROW_LENGTH) + 2];

			output[0][(column * MAT3_ROW_LENGTH) + 0] = ai0 * matrixB[(0 * MAT3_ROW_LENGTH) + 0] + ai1 * matrixB[(1 * MAT3_ROW_LENGTH) + 0] + ai2 * matrixB[(2 * MAT3_ROW_LENGTH) + 0];
			output[0][(column * MAT3_ROW_LENGTH) + 1] = ai0 * matrixB[(0 * MAT3_ROW_LENGTH) + 1] + ai1 * matrixB[(1 * MAT3_ROW_LENGTH) + 1] + ai2 * matrixB[(2 * MAT3_ROW_LENGTH) + 1];
			output[0][(column * MAT3_ROW_LENGTH) + 2] = ai0 * matrixB[(0 * MAT3_ROW_LENGTH) + 2] + ai1 * matrixB[(1 * MAT3_ROW_LENGTH) + 2] + ai2 * matrixB[(2 * MAT3_ROW_LENGTH) + 2];
		}
#endif
	}
	
	//void multiply(const Mat3& A, const Mat3& B, const Mat3& C, Mat3& output)
	//{
	//}

	void inverse(const Mat3& input, Mat3& output)
	{
		const sp_float det = ONE_FLOAT / input.determinant();

		sp_assert(!NAMESPACE_FOUNDATION::isCloseEnough(det, ZERO_FLOAT, SP_EPSILON_FIVE_DIGITS), "InvalidOperationException"); // matrix is singular

		for (sp_int i = 0; i < MAT3_ROW_LENGTH; i++)
			for (sp_int j = 0; j < MAT3_ROW_LENGTH; j++)
			{
				const sp_float detij = input.determinantIJ(j, i);

				output[i * MAT3_ROW_LENGTH + j] = det * 
					((i + j) & 0x1 ? -detij : detij);
			}
	}

	void rotateJacobi(const Mat3& input, const sp_float sinTheta, const sp_float cosTheta, const sp_uint rowIndex, const sp_uint columnIndex, Mat3& output)
	{
		sp_assert((rowIndex == 0u && columnIndex == 1u)
				|| (rowIndex == 0u && columnIndex == 2u)
				|| (rowIndex == 1u && columnIndex == 2u)
				,  "InvalidArgumentException");

		Mat3 jacobiRotation;
		std::memcpy(jacobiRotation, Mat3Identity, sizeof(Mat3));

		jacobiRotation[columnIndex * MAT3_ROW_LENGTH + columnIndex]
			= jacobiRotation[rowIndex * MAT3_ROW_LENGTH + rowIndex]
			= cosTheta;

		jacobiRotation[rowIndex * MAT3_ROW_LENGTH + columnIndex] = -sinTheta;
		jacobiRotation[columnIndex * MAT3_ROW_LENGTH + rowIndex] = sinTheta;

		Mat3 temp;		
		multiply(jacobiRotation.transpose(), input, temp);	
		multiply(temp, jacobiRotation, output);
	}

	void Mat3::convert(Quat& output) const
	{
		output.w = sqrtf(ONE_FLOAT + values[M11] + values[M22] + values[M33]) * HALF_FLOAT;
		const sp_float w4 = div(ONE_FLOAT, (4.0f * output.w));
		
		output.x = (values[M32] - values[M23]) * w4;
		output.y = (values[M13] - values[M31]) * w4;
		output.z = (values[M21] - values[M12]) * w4;
	}

	void sqrtm(const Mat3& input, Mat3& output, const sp_uint maxIterations)
	{
		sp_uint iterations;
		Mat3 eigenVectors;
		Vec3 eigenValues;
		input.eigenValuesAndVectors(eigenValues, eigenVectors, iterations, maxIterations);

		Mat3 eigenVectorsInverse;
		inverse(eigenVectors, eigenVectorsInverse);

		Mat3 temp;
		multiply(eigenVectorsInverse, input, eigenVectors, temp);
		
		Mat3 diagonal;
		std::memcpy(&diagonal, Mat3Zeros, sizeof(Mat3));
		diagonal[M11] = sqrtf(temp[M11]);
		diagonal[M22] = sqrtf(temp[M22]);
		diagonal[M33] = sqrtf(temp[M33]);

		multiply(eigenVectors, diagonal, eigenVectorsInverse, output);
	}

}

#undef M11
#undef M12
#undef M13

#undef M21
#undef M22
#undef M23

#undef M31
#undef M32
#undef M33
