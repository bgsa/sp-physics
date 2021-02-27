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
		m11 = value11;
		m12 = value21;
		m13 = value31;
		m21 = value12;
		m22 = value22;
		m23 = value32;
		m31 = value13;
		m32 = value23;
		m33 = value33;
	}

	sp_float Mat3::get(const sp_uint x, const sp_uint y) const
	{
		return values()[(y-1u) * MAT3_ROW_LENGTH + (x-1u)];
	}
	
	Vec3 Mat3::axis(const sp_uint index) const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec3(
			values()[index],
			values()[index + MAT3_ROW_LENGTH],
			values()[index + MAT3_ROW_LENGTH + MAT3_ROW_LENGTH]
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
		return Vec3(m11, m21, m31);
	#else
		return Vec3(m11, m12, m13);
	#endif
	}
	
	Vec3 Mat3::yAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec3(m12, m22, m32);
	#else
		return Vec3(m21, m22, m23);
	#endif
	}

	Vec3 Mat3::zAxis() const
	{
	#if MAJOR_COLUMN_ORDER
		return Vec3(m13, m23, m33);
	#else
		return Vec3(m31, m32, m33);
	#endif
	}

	void Mat3::primaryDiagonal(Vec3& output) const
	{
		output.x = m11;
		output.y = m22;
		output.z = m33;
	}

	Vec3 Mat3::secondaryDiagonal() const
	{
		return Vec3(m13, m22, m31);
	}

	void Mat3::transpose(Mat3& output) const
	{
		//copy principal diagonal
		output.m11 = m11;
		output.m22 = m22;
		output.m33 = m33;

		//swap others numbers
		output.m12 = m21;
		output.m21 = m12;

		output.m13 = m31;
		output.m31 = m13;

		output.m23 = m32;
		output.m32 = m23;
	}

	void Mat3::multiply(const Vec3& vector, Vec3& output) const
	{
		output.x = m11 * vector.x + m12 * vector.y + m13 * vector.z;
		output.y = m21 * vector.x + m22 * vector.y + m23 * vector.z;
		output.z = m31 * vector.x + m32 * vector.y + m33 * vector.z;
	}
	
	void Mat3::symmetric(Mat3& output) const
	{
		Mat3 matrixT;
		transpose(matrixT);
		NAMESPACE_PHYSICS::multiply(*this, matrixT, output);
	}

	Mat3 Mat3::createScale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
	{
		Mat3 result = Mat3Identity;

		result.scale(xScale, yScale, zScale);

		return result;
	}
	
	void Mat3::scale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
	{
		m11 *= xScale;
		m22 *= yScale;
		m33 *= zScale;
	}
	
	Mat3 Mat3::createRotate(const sp_float angleRadians, const sp_float x, const sp_float y, const sp_float z)
	{	
		const sp_float sine = sinf(angleRadians);
		const sp_float cosine = cosf(angleRadians);

		const sp_float mag = sqrtf(x*x + y * y + z * z);

		if (mag == 0.0f)
			return Mat3Identity;

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
		Mat3 result = Mat3Identity;

	#if MAJOR_COLUMN_ORDER
		result.m31 = x;
		result.m32 = y;
		result.m33 = z;
	#else
		result[2] = x;
		result[5] = y;
		result[8] = z;
	#endif

		return result;
	}
	
	Mat3 Mat3::createTranslate(const Vec3& position)
	{
		Mat3 result = Mat3Identity;

	#if MAJOR_COLUMN_ORDER
		result.m31 = position.x;
		result.m32 = position.y;
		result.m33 = position.z;
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
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			if (values()[i] != Mat3Identity[i])
				return false;

		return true;
	}

	sp_bool Mat3::isOrthogonal() const
	{
		Mat3 matrixT;
		transpose(matrixT);

		Mat3 ident;
		NAMESPACE_PHYSICS::multiply(*this, matrixT, ident);

		return isCloseEnough(ident, Mat3Identity, SP_EPSILON_THREE_DIGITS);
	}

	sp_bool Mat3::isHermitian() const
	{
		sp_assert(false, "NorImplementedException");

		if (!isSymetric())
			return false;

		return false;
	}

	sp_bool Mat3::isPositiveDefinite() const
	{
		Vec3 eigValues;
		sp_uint iterations;
		eigenValues(eigValues, iterations);

		return eigValues.x >= ZERO_FLOAT
			&& eigValues.y >= ZERO_FLOAT
			&& eigValues.z >= ZERO_FLOAT;
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
		return (void*)this;
	}

	Mat3::operator sp_float*() const
	{
		return (sp_float*)this;
	}
	
	Mat3 Mat3::operator-() const
	{
		Mat3 result;
		result.m11 = -m11;
		result.m12 = -m12;
		result.m13 = -m13;
		result.m21 = -m21;
		result.m22 = -m22;
		result.m23 = -m23;
		result.m31 = -m31;
		result.m32 = -m32;
		result.m33 = -m33;
		return result;
	}

	Mat3 Mat3::operator-(const Mat3& matrix) const
	{
		Mat3 result;
		result.m11 = m11 - matrix.m11;
		result.m12 = m12 - matrix.m12;
		result.m13 = m13 - matrix.m13;
		result.m21 = m21 - matrix.m21;
		result.m22 = m22 - matrix.m22;
		result.m23 = m23 - matrix.m23;
		result.m31 = m31 - matrix.m31;
		result.m32 = m32 - matrix.m32;
		result.m33 = m33 - matrix.m33;

		return result;
	}

	Mat3 Mat3::operator+(const Mat3& matrix) const
	{
		Mat3 result;
		result.m11 = m11 + matrix.m11;
		result.m12 = m12 + matrix.m12;
		result.m13 = m13 + matrix.m13;
		result.m21 = m21 + matrix.m21;
		result.m22 = m22 + matrix.m22;
		result.m23 = m23 + matrix.m23;
		result.m31 = m31 + matrix.m31;
		result.m32 = m32 + matrix.m32;
		result.m33 = m33 + matrix.m33;

		return result;
	}

	Mat3 Mat3::operator*(const Mat3& matrix) const
	{
		Mat3 result;
		NAMESPACE_PHYSICS::multiply(*this, matrix, result);
		return result;
	}

	Mat3 Mat3::operator/(const sp_float value) const
	{
		return Mat3 (
			m11 / value, m12 / value, m13 / value,
			m21 / value, m22 / value, m23 / value,
			m31 / value, m32 / value, m33 / value
		);
	}
	
	void Mat3::operator/=(const sp_float value)
	{
		sp_float invValue = ONE_FLOAT / value;

		m11 *= invValue;
		m12 *= invValue;
		m13 *= invValue;
		m21 *= invValue;
		m22 *= invValue;
		m23 *= invValue;
		m31 *= invValue;
		m32 *= invValue;
		m33 *= invValue;
	}

	sp_bool Mat3::operator==(const Mat3& matrix) const
	{
		return isCloseEnough(*this, matrix, SP_EPSILON_FOUR_DIGITS);
	}

	sp_bool Mat3::operator!=(const Mat3& matrix) const
	{
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			if (values()[i] != matrix[i])
				return true;

		return false;
	}
	
	sp_bool Mat3::operator==(const sp_float value) const
	{
		for (sp_int i = 0; i < MAT3_LENGTH; i++)
			if (values()[i] != value)
				return false;

		return true;
	}

	std::string Mat3::toString() const
	{
		return Mat::toString(values(), MAT3_LENGTH);
	}

	void Mat3::decomposeLU(Mat3& lower, Mat3& upper) const
	{
		std::memcpy(lower, Mat3Zeros, sizeof(Mat3));
		std::memcpy(upper, this, sizeof(Mat3));

		lower.m11 = m11;
		lower.m21 = m21;
		lower.m31 = m31;

		upper.m12 = div(m12, upper.m11);
		upper.m13 = div(m13, upper.m11);
		upper.m11 = ONE_FLOAT;

		upper.m22 = -upper.m21 * upper.m12 + upper.m22;
		upper.m23 = -upper.m21 * upper.m13 + upper.m23;
		upper.m21 = ZERO_FLOAT;

		upper.m32 = -upper.m31 * upper.m12 + upper.m32;
		upper.m33 = -upper.m31 * upper.m13 + upper.m33;
		upper.m31 = ZERO_FLOAT;

		lower.m22 = upper.m22;
		upper.m23 = div(upper.m23, upper.m22);
		upper.m22 = ONE_FLOAT;

		lower.m32 = upper.m32;
		upper.m33 = (-upper.m32 * upper.m23) + upper.m33;
		lower.m33 = upper.m33;
		upper.m32 = ZERO_FLOAT;
		upper.m33 = ONE_FLOAT;

		sp_assert(upper * lower == *this, "ApplicationException");

#undef upperMatrix
#undef lowerMatrix
	}
	
	void Mat3::decomposeLDU(Mat3& lower, Mat3& diagonal, Mat3& upper) const
	{
		decomposeLU(lower, upper);

		std::memcpy(diagonal, Mat3Zeros, sizeof(Mat3));

		diagonal.m11 = lower.m11;
		diagonal.m22 = lower.m22;
		diagonal.m33 = lower.m33;

		lower.m21 = div(lower.m21, lower.m11);
		lower.m31 = div(lower.m31, lower.m11);

		lower.m32 = div(lower.m32, lower.m22);

		lower.m11 = lower.m22 = lower.m33 = ONE_FLOAT;

		sp_assert(upper * diagonal * lower == *this, "ApplicationException");
	}

	void Mat3::decomposeLLt(Mat3& lower, Mat3& lowerTransposed) const
	{
		std::memcpy(&lower, Mat3Zeros, sizeof(Mat3));
	
		lower.m11 = sqrtf(m11);
		lower.m21 = div(m21, lower.m11);
		lower.m31 = div(m31, lower.m11);

		lower.m22 = sqrtf(m22 - lower.m21 * lower.m21);

		lower.m32 = (m32 - lower.m21 * lower.m31) / lower.m22;

		lower.m33 = sqrtf(m33 - lower.m31 * lower.m31 - lower.m32 * lower.m32);

		NAMESPACE_PHYSICS::transpose(lower, lowerTransposed);

		sp_assert(lowerTransposed * lower == *this, "ApplicationException");
	}

	void Mat3::polyname(Vec4* output) const
	{
		const sp_float s1 = trace();
		const sp_float a1 = s1;
		
		Mat3 AxA;
		NAMESPACE_PHYSICS::multiply(*this, *this, AxA);
		
		const sp_float s2 = AxA.trace();
		const sp_float a2 = HALF_FLOAT * (s2 - a1 * s1);

		Mat3 AxAxA;
		NAMESPACE_PHYSICS::multiply(AxA, *this, AxAxA);

		const sp_float s3 = AxAxA.trace();
		const sp_float a3 = ONE_OVER_THREE * (s3 - a1 * s2 - a2 * s1);
		
		output[0].x = -ONE_FLOAT;
		output[0].y = a1;
		output[0].z = a2;
		output[0].w = a3;
	}

	void Mat3::eigenValues(Vec3& output, sp_uint& iterations, const sp_float _epsilon) const
	{
#define aqq matrix.get(columnIndex, columnIndex)
#define	app matrix.get(rowIndex, rowIndex)
#define	apq matrix.get(rowIndex, columnIndex)

		if (isSymetric())
		{
			Mat3 matrix;
			clone(&matrix);

			output = Vec3Zeros;
			Vec3 previousResult = Vec3Ones;
			iterations = ZERO_UINT;

			sp_float offDiagonal
				= matrix.m12 * matrix.m12
				+ matrix.m13 * matrix.m13
				+ matrix.m23 * matrix.m23
				+ matrix.m21 * matrix.m21
				+ matrix.m32 * matrix.m32
				+ matrix.m31 * matrix.m31;

			//while (!isCloseEnough(previousResult, eigenValues, _epsilon) && !matrix.isLower(_epsilon))
			while (!NAMESPACE_FOUNDATION::isCloseEnough(offDiagonal, ZERO_FLOAT, _epsilon))
			{
				matrix.primaryDiagonal(previousResult);

				sp_uint columnIndex;
				sp_uint rowIndex;

				if (fabsf(matrix.m12) > fabsf(matrix.m13))
					if (fabsf(matrix.m23) > fabsf(matrix.m12))
					{
						rowIndex = TWO_UINT;
						columnIndex = THREE_UINT;
					}
					else
					{
						rowIndex = ONE_UINT;
						columnIndex = TWO_UINT;
					}
				else
					if (fabsf(matrix.m23) > fabsf(matrix.m13))
					{
						rowIndex = TWO_UINT;
						columnIndex = THREE_UINT;
					}
					else
					{
						rowIndex = ONE_UINT;
						columnIndex = THREE_UINT;
					}

				sp_float theta = (aqq - app) / (TWO_FLOAT * apq);

				//If theta is so large that theta2 would overflow on the computer, 
				// so we set t = 1 / (2 theta)

				if (theta < ZERO_FLOAT)
					theta = -theta;

				const sp_float tangentTheta
					= NAMESPACE_FOUNDATION::isCloseEnough(theta, ZERO_FLOAT, DefaultErrorMargin)
					? ONE_FLOAT
					: ONE_FLOAT / (theta + sign(theta) * sqrtf(ONE_FLOAT + theta * theta));
				//: ONE_FLOAT / (theta + sqrtf(ONE_FLOAT + theta * theta));
				//: sign(theta) / (fabsf(theta) + sqrtf(ONE_FLOAT + theta * theta));

				const sp_float cosTheta = ONE_FLOAT / sqrtf(ONE_FLOAT + tangentTheta * tangentTheta);
				const sp_float sinTheta = cosTheta * tangentTheta;

				Mat3 jacobiRotationMatrix;
				jacobiRotation(matrix, sinTheta, cosTheta, rowIndex - ONE_UINT, columnIndex - ONE_UINT, matrix, jacobiRotationMatrix);

				matrix.primaryDiagonal(output);

				offDiagonal
					= matrix.m12 * matrix.m12
					+ matrix.m13 * matrix.m13
					+ matrix.m23 * matrix.m23
					+ matrix.m21 * matrix.m21
					+ matrix.m32 * matrix.m32
					+ matrix.m31 * matrix.m31;

				iterations++;
			}
		}
		else
		{
			Mat3 Q, R, A;
			decomposeQR(Q, R, iterations, _epsilon);

			NAMESPACE_PHYSICS::multiply(Q, R, A);

			output.x = A[M11];
			output.y = A[M22];
			output.z = A[M33];
		}
#undef apq
#undef app
#undef aqq
	}

	sp_bool Mat3::eigenValuesAndVectors(Vec3& eigenValues, Mat3& eigenVectors, sp_uint& iterations, const sp_uint maxIterations, const sp_float _epsilon) const
	{
#define aqq matrix.get(columnIndex, columnIndex)
#define	app matrix.get(rowIndex, rowIndex)
#define	apq matrix.get(rowIndex, columnIndex)

		if (isSymetric())
		{
			Mat3 matrix;
			clone(&matrix);

			std::memcpy(eigenVectors, Mat3Identity, sizeof(Mat3));

			eigenValues = Vec3Zeros;
			Vec3 previousResult = Vec3Ones;
			iterations = ZERO_UINT;

			sp_float offDiagonal
				= matrix.m12 * matrix.m12
				+ matrix.m13 * matrix.m13
				+ matrix.m23 * matrix.m23
				+ matrix.m21 * matrix.m21
				+ matrix.m32 * matrix.m32
				+ matrix.m31 * matrix.m31;

			//while (!isCloseEnough(previousResult, eigenValues, _epsilon) && !matrix.isLower(_epsilon))
			while (!NAMESPACE_FOUNDATION::isCloseEnough(offDiagonal, ZERO_FLOAT, _epsilon) && iterations < maxIterations)
			{
				matrix.primaryDiagonal(previousResult);

				sp_uint columnIndex;
				sp_uint rowIndex;

				if (fabsf(matrix.m12) > fabsf(matrix.m13))
					if (fabsf(matrix.m23) > fabsf(matrix.m12))
					{
						rowIndex = TWO_UINT;
						columnIndex = THREE_UINT;
					}
					else
					{
						rowIndex = ONE_UINT;
						columnIndex = TWO_UINT;
					}
				else
					if (fabsf(matrix.m23) > fabsf(matrix.m13))
					{
						rowIndex = TWO_UINT;
						columnIndex = THREE_UINT;
					}
					else
					{
						rowIndex = ONE_UINT;
						columnIndex = THREE_UINT;
					}

				sp_float theta = (aqq - app) / (TWO_FLOAT * apq);

				//If theta is so large that theta2 would overflow on the computer, 
				// so we set t = 1 / (2 theta)

				if (theta < ZERO_FLOAT) 
					theta = -theta;

				const sp_float tangentTheta
					= NAMESPACE_FOUNDATION::isCloseEnough(theta, ZERO_FLOAT, DefaultErrorMargin)
					? ONE_FLOAT
					: ONE_FLOAT / (theta + sign(theta) * sqrtf(ONE_FLOAT + theta * theta));
					//: ONE_FLOAT / (theta + sqrtf(ONE_FLOAT + theta * theta));
					//: sign(theta) / (fabsf(theta) + sqrtf(ONE_FLOAT + theta * theta));

				const sp_float cosTheta = ONE_FLOAT / sqrtf(ONE_FLOAT + tangentTheta * tangentTheta);
				const sp_float sinTheta =  cosTheta * tangentTheta;

				Mat3 jacobiRotationMatrix;
				jacobiRotation(matrix, sinTheta, cosTheta, rowIndex - ONE_UINT, columnIndex - ONE_UINT, matrix, jacobiRotationMatrix);
				
				eigenVectors = eigenVectors * jacobiRotationMatrix;

				matrix.primaryDiagonal(eigenValues);

				offDiagonal
					= matrix.m12 * matrix.m12
					+ matrix.m13 * matrix.m13
					+ matrix.m23 * matrix.m23
					+ matrix.m21 * matrix.m21
					+ matrix.m32 * matrix.m32
					+ matrix.m31 * matrix.m31;
				
				iterations++;
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

	void Mat3::eigenValuesAndVectorsMax(sp_float& eigenValue, Vec3& eigenVector, const sp_ushort maxIteration) const
	{
		sp_assert(false, "NotImplementedException");

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

			if (!NAMESPACE_FOUNDATION::isCloseEnough(A.m21, ZERO_FLOAT, _epsilon))
			{
				r = ONE_FLOAT / sqrtf(A.m21 * A.m21 + A.m11 * A.m11);
				cos0 = A.m11 * r;
				sin0 = A.m21 * r;

				P1.m11 = P1[M22] = cos0;
				P1.m12 = sin0;
				P1.m21 = -sin0;

				NAMESPACE_PHYSICS::multiply(A, P1, A);
			}

			if (!NAMESPACE_FOUNDATION::isCloseEnough(A.m32, ZERO_FLOAT, _epsilon))
			{
				r = ONE_FLOAT / sqrtf(A.m32 * A.m32 + A.m22 * A.m22);
				cos0 = A.m22 * r;
				sin0 = A.m32 * r;

				P2.m22 = P2.m33 = cos0;
				P2.m23 = sin0;
				P2.m32 = -sin0;
			}

			NAMESPACE_PHYSICS::multiply(A, P2, R);

			P1.transpose();
			P2.transpose();
			NAMESPACE_PHYSICS::multiply(P2, P1, Q); // Matrix Q = P(i) * P(i+1) * P(i+2) * ... * P(i+N)

			NAMESPACE_PHYSICS::multiply(Q, R, A);

			iterations++;
		} while (!NAMESPACE_FOUNDATION::isCloseEnough(A.m21, ZERO_FLOAT, _epsilon)
			|| !NAMESPACE_FOUNDATION::isCloseEnough(A.m32, ZERO_FLOAT, _epsilon));

#ifdef DEBUG
		NAMESPACE_PHYSICS::transpose(Q, A);
		sp_assert(isCloseEnough(A * Q, Mat3Identity), "ApplicationException");
#endif
	}
	
	void Mat3::diagonalize(Mat3& output, sp_uint& iterations, const sp_float _epsilon) const
	{
#define aqq matrix.get(columnIndex, columnIndex)
#define	app matrix.get(rowIndex, rowIndex)
#define	apq matrix.get(rowIndex, columnIndex)

		sp_assert(isSymetric(), "InvalidArgumentException");

		Mat3 matrix;
		clone(&matrix);

		Vec3 result = Vec3Zeros;
		Vec3 previousResult = Vec3Ones;
		iterations = ZERO_UINT;

		sp_float offDiagonal
			= matrix.m12 * matrix.m12
			+ matrix.m13 * matrix.m13
			+ matrix.m23 * matrix.m23
			+ matrix.m21 * matrix.m21
			+ matrix.m32 * matrix.m32
			+ matrix.m31 * matrix.m31;

		//while (!isCloseEnough(previousResult, eigenValues, _epsilon) && !matrix.isLower(_epsilon))
		while (!NAMESPACE_FOUNDATION::isCloseEnough(offDiagonal, ZERO_FLOAT, _epsilon))
		{
			matrix.primaryDiagonal(previousResult);

			sp_uint columnIndex;
			sp_uint rowIndex;

			if (fabsf(matrix.m12) > fabsf(matrix.m13))
				if (fabsf(matrix.m23) > fabsf(matrix.m12))
				{
					rowIndex = TWO_UINT;
					columnIndex = THREE_UINT;
				}
				else
				{
					rowIndex = ONE_UINT;
					columnIndex = TWO_UINT;
				}
			else
				if (fabsf(matrix.m23) > fabsf(matrix.m13))
				{
					rowIndex = TWO_UINT;
					columnIndex = THREE_UINT;
				}
				else
				{
					rowIndex = ONE_UINT;
					columnIndex = THREE_UINT;
				}

			sp_float theta = (aqq - app) / (TWO_FLOAT * apq);

			//If theta is so large that theta2 would overflow on the computer, 
			// so we set t = 1 / (2 theta)

			if (theta < ZERO_FLOAT)
				theta = -theta;

			const sp_float tangentTheta
				= NAMESPACE_FOUNDATION::isCloseEnough(theta, ZERO_FLOAT, DefaultErrorMargin)
				? ONE_FLOAT
				: ONE_FLOAT / (theta + sign(theta) * sqrtf(ONE_FLOAT + theta * theta));
			//: ONE_FLOAT / (theta + sqrtf(ONE_FLOAT + theta * theta));
			//: sign(theta) / (fabsf(theta) + sqrtf(ONE_FLOAT + theta * theta));

			const sp_float cosTheta = ONE_FLOAT / sqrtf(ONE_FLOAT + tangentTheta * tangentTheta);
			const sp_float sinTheta = cosTheta * tangentTheta;

			Mat3 jacobiRotationMatrix;
			jacobiRotation(matrix, sinTheta, cosTheta, rowIndex - ONE_UINT, columnIndex - ONE_UINT, matrix, jacobiRotationMatrix);

			matrix.primaryDiagonal(result);

			offDiagonal
				= matrix.m12 * matrix.m12
				+ matrix.m13 * matrix.m13
				+ matrix.m23 * matrix.m23
				+ matrix.m21 * matrix.m21
				+ matrix.m32 * matrix.m32
				+ matrix.m31 * matrix.m31;

			iterations++;
		}

		std::memcpy(&output, &Mat3Identity, sizeof(Mat3));
		output.m11 = result.x;
		output.m22 = result.y;
		output.m33 = result.z;
#undef apq
#undef app
#undef aqq
	}

	void multiply(const Vec3& v1, const Vec3& v2, Mat3& output)
	{
		output.m11 = v1.x * v2.x;
		output.m12 = v1.x * v2.y;
		output.m13 = v1.x * v2.z;

		output.m21 = v1.y * v2.x;
		output.m22 = v1.y * v2.y;
		output.m23 = v1.y * v2.z;

		output.m31 = v1.z * v2.x;
		output.m32 = v1.z * v2.y;
		output.m33 = v1.z * v2.z;
	}

	void multiply(const Mat3& input, const sp_float value, Mat3& output)
	{
		output.m11 = input.m11 * value;
		output.m12 = input.m12 * value;
		output.m13 = input.m13 * value;

		output.m21 = input.m21 * value;
		output.m22 = input.m22 * value;
		output.m23 = input.m23 * value;

		output.m31 = input.m31 * value;
		output.m32 = input.m32 * value;
		output.m33 = input.m33 * value;
	}

	void multiply(const Mat3& matrix, const Vec3& vector, Vec3& output)
	{
		output.x = matrix.m11 * vector.x + matrix.m12 * vector.y + matrix.m13 * vector.z;
		output.y = matrix.m21 * vector.x + matrix.m22 * vector.y + matrix.m23 * vector.z;
		output.z = matrix.m31 * vector.x + matrix.m32 * vector.y + matrix.m33 * vector.z;
	}

	void multiply(const Mat3& matrixA, const Mat3& matrixB, Mat3& output)
	{
#if MAJOR_COLUMN_ORDER
		output.m11 = matrixA.m11 * matrixB.m11 + matrixA.m21 * matrixB.m12 + matrixA.m31 * matrixB.m13;
		output.m12 = matrixA.m12 * matrixB.m11 + matrixA.m22 * matrixB.m12 + matrixA.m32 * matrixB.m13;
		output.m13 = matrixA.m13 * matrixB.m11 + matrixA.m23 * matrixB.m12 + matrixA.m33 * matrixB.m13;
	
		output.m21 = matrixA.m11 * matrixB.m21 + matrixA.m21 * matrixB.m22 + matrixA.m31 * matrixB.m23;
		output.m22 = matrixA.m12 * matrixB.m21 + matrixA.m22 * matrixB.m22 + matrixA.m32 * matrixB.m23;
		output.m23 = matrixA.m13 * matrixB.m21 + matrixA.m23 * matrixB.m22 + matrixA.m33 * matrixB.m23;

		output.m31 = matrixA.m11 * matrixB.m31 + matrixA.m21 * matrixB.m32 + matrixA.m31 * matrixB.m33;
		output.m32 = matrixA.m12 * matrixB.m31 + matrixA.m22 * matrixB.m32 + matrixA.m32 * matrixB.m33;
		output.m33 = matrixA.m13 * matrixB.m31 + matrixA.m23 * matrixB.m32 + matrixA.m33 * matrixB.m33;
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

	void givensRotation(Mat3& output, const sp_uint rowIndex, const sp_uint columnIndex, const sp_float sinTheta, const sp_float cosTheta)
	{
		sp_assert((rowIndex == 0u && columnIndex == 1u)
			|| (rowIndex == 0u && columnIndex == 2u)
			|| (rowIndex == 1u && columnIndex == 2u)
			, "InvalidArgumentException");

		std::memcpy(output, Mat3Identity, sizeof(Mat3));

		/*
		output[columnIndex * MAT3_ROW_LENGTH + columnIndex]
			= output[rowIndex * MAT3_ROW_LENGTH + rowIndex]
			= cosTheta;

		output[rowIndex * MAT3_ROW_LENGTH + columnIndex] = sinTheta;
		output[columnIndex * MAT3_ROW_LENGTH + rowIndex] = -sinTheta;
		*/
		
		output[columnIndex * MAT3_ROW_LENGTH + columnIndex]
			= output[rowIndex * MAT3_ROW_LENGTH + rowIndex]
			= sinTheta;

		output[rowIndex * MAT3_ROW_LENGTH + columnIndex] = -cosTheta;
		output[columnIndex * MAT3_ROW_LENGTH + rowIndex] = cosTheta;

/*
		sp_log_debug1s("ROTATION ");
		sp_log_debug1u(rowIndex);
		sp_log_debug1u(columnIndex);
		sp_log_newline();
		SystemOfLinearEquations system;
		sp_log_debug1snl(system.printMatrix(output, 3, 3).c_str());
		*/
	}

	void jacobiRotation(const Mat3& input, const sp_float sinTheta, const sp_float cosTheta, const sp_uint rowIndex, const sp_uint columnIndex, Mat3& output, Mat3& jacobiRotation)
	{
		givensRotation(jacobiRotation, rowIndex, columnIndex, sinTheta, cosTheta);
	
		Mat3 jacobiRotationT;
		jacobiRotation.transpose(jacobiRotationT);

		multiply(jacobiRotationT, input, jacobiRotation, output);
	}

	void Mat3::convert(Quat& output) const
	{
		output.w = sqrtf(ONE_FLOAT + m11 + m22 + m33) * HALF_FLOAT;
		const sp_float w4 = div(ONE_FLOAT, (4.0f * output.w));
		
		output.x = (m32 - m23) * w4;
		output.y = (m13 - m31) * w4;
		output.z = (m21 - m12) * w4;
	}

	sp_bool sqrtm(const Mat3& input, Mat3& output, const sp_uint maxIterations)
	{
		sp_uint iterations;
		Mat3 eigenVectors;
		Vec3 eigenValues;
		if (!input.eigenValuesAndVectors(eigenValues, eigenVectors, iterations, maxIterations))
			return false;

		Mat3 eigenVectorsInverse;
		inverse(eigenVectors, eigenVectorsInverse);
		//transpose(eigenVectors, eigenVectorsInverse);

		Mat3 temp;
		multiply(eigenVectorsInverse, input, eigenVectors, temp);
		
		Mat3 diagonal;
		std::memcpy(&diagonal, Mat3Zeros, sizeof(Mat3));
		diagonal.m11 = sqrtf(temp.m11);
		diagonal.m22 = sqrtf(temp.m22);
		diagonal.m33 = sqrtf(temp.m33);

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
