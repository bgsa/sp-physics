#include "Mat4.h"

namespace NAMESPACE_PHYSICS
{
	
	Mat4::Mat4(const Vec4& vector1, const Vec4& vector2, const Vec4& vector3, const Vec4& vector4)
	{
#if MAJOR_COLUMN_ORDER
		m11 = vector1.x;
		m21 = vector1.y;
		m31 = vector1.z;
		m41 = vector1.w;

		m12 = vector2.x;
		m22 = vector2.y;
		m32 = vector2.z;
		m42 = vector2.w;

		m13 = vector3.x;
		m23 = vector3.y;
		m33 = vector3.z;
		m43 = vector3.w;

		m14 = vector4.x;
		m24 = vector4.y;
		m34 = vector4.z;
		m44 = vector4.w;
#else
		m11 = vector1.x;
		m12 = vector1.y;
		m13 = vector1.z;
		m14 = vector1.w;

		m21 = vector2.x;
		m22 = vector2.y;
		m23 = vector2.z;
		m24 = vector2.w;

		m31 = vector3.x;
		m32 = vector3.y;
		m33 = vector3.z;
		m34 = vector3.w;

		m41 = vector4.x;
		m42 = vector4.y;
		m43 = vector4.z;
		m44 = vector4.w;
#endif
	}

	Vec4 Mat4::xAxis() const
	{
#if MAJOR_COLUMN_ORDER
		return Vec4(m11, m21, m31, m41);
#else
		return Vec4(m11, m12, m13, m14);
#endif
	}

	Vec4 Mat4::yAxis() const
	{
#if MAJOR_COLUMN_ORDER
		return Vec4(m12, m22, m32, m42);
#else
		return Vec4(m21, m22, m23, m24);
#endif
	}

	Vec4 Mat4::zAxis() const
	{
#if MAJOR_COLUMN_ORDER
		return Vec4(m13, m23, m33, m43);
#else
		return Vec4(m31, m32, m33, m34);
#endif
	}

	Vec4 Mat4::wAxis() const
	{
#if MAJOR_COLUMN_ORDER
		return Vec4(m14, m24, m34, m44);
#else
		return Vec4(m41, m42, m43, m44);
#endif
	}

	Vec4 Mat4::primaryDiagonal() const
	{
		return Vec4(m11, m22, m33, m44);
	}

	Vec4 Mat4::secondaryDiagonal() const
	{
		return Vec4(m14, m23, m32, m41);
	}

	void Mat4::multiply(const Vec4 &vector, Vec4& output) const
	{
	#if MAJOR_COLUMN_ORDER
		output.x
			= vector.x * m11
			+ vector.y * m21
			+ vector.z * m31
			+ vector.w * m41;

		output.y
			= vector.x * m12
			+ vector.y * m22
			+ vector.z * m32
			+ vector.w * m42;

		output.z
			= vector.x * m13
			+ vector.y * m23
			+ vector.z * m33
			+ vector.w * m43;

		output.w
			= vector.x * m14
			+ vector.y * m24
			+ vector.z * m34
			+ vector.w * m44;
	#else

		output.x
			= m11 * vector.x
			+ m12 * vector.y
			+ m13 * vector.z
			+ m14 * vector.w;

		output.y
			= m21 * vector.x
			+ m22 * vector.y
			+ m23 * vector.z
			+ m24 * vector.w;

		output.z
			= m31 * vector.x
			+ m32 * vector.y
			+ m33 * vector.z
			+ m34 * vector.w;

		output.w
			= m41 * vector.x
			+ m42 * vector.y
			+ m43 * vector.z
			+ m44 * vector.w;
	#endif
	}
	
	void Mat4::toMat3(Mat3& output) const
	{
		output.m11 = m11;
		output.m12 = m12;
		output.m13 = m13;

		output.m21 = m21;
		output.m22 = m22;
		output.m23 = m23;

		output.m31 = m31;
		output.m32 = m32;
		output.m33 = m33;
	}

	Mat4* Mat4::decomposeLU() const
	{
		Mat4 lowerMatrix = Mat4Identity;
		Mat4 upperMatrix;
		std::memcpy(upperMatrix, this, sizeof(Mat4));

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

			elementarInverseMatrix = Mat4Identity;
			elementarInverseMatrix[pivotRowIndex * rowSize + column] = pivot;
			elementarInverseMatrixes.push_back(elementarInverseMatrix);

			for (sp_int lowerColumns = column + 1; lowerColumns < rowSize; lowerColumns++)
			{
				pivot = upperMatrix[pivotRowIndex * rowSize + lowerColumns];
				pivotOperator = -pivot;

				for (sp_int row = 0; row < rowSize; row++)
					upperMatrix[row * rowSize + lowerColumns] += pivotOperator * upperMatrix[row * rowSize + column];

				elementarInverseMatrix = Mat4Identity;
				elementarInverseMatrix[pivotRowIndex * rowSize + lowerColumns] = pivot;
				elementarInverseMatrixes.push_back(elementarInverseMatrix);
			}

			pivotRowIndex++;
		}

		for (sp_int i = 0; sp_size(i) < elementarInverseMatrixes.size(); i++)
		{
			lowerMatrix.multiply(elementarInverseMatrixes[i], lowerMatrix);
			//lowerMatrix *= elementarInverseMatrixes[i];
		}
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
		Mat4 diagonalMatrix = Mat4Identity;
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
			Vec4 ax;
			matrix.multiply(autovector, ax);

			autovalue = ax.maximum();
			autovector = ax / autovalue;
		}

		return AutoValueAutoVector4{ autovalue, { autovector.x, autovector.y, autovector.z, autovector.w} };
	}

	void Mat4::tridiagonal(Mat4* output) const
	{
		sp_assert(isSymetric(), "InvalidOperationException");

		sp_float alpha = -sign(m21) * sp_sqrt(m21 * m21 + m31 * m31 + m41 * m41);
	
		sp_float r = sp_sqrt((alpha * alpha) * HALF_FLOAT - m21 * alpha * HALF_FLOAT);
		sp_float two_r = ONE_FLOAT / (r * TWO_FLOAT);

		Vec4 w;
		w.x = ZERO_FLOAT;
		w.y = (m21 - alpha) * two_r;
		w.z = m31 * two_r;
		w.w = m41 * two_r;

		Mat4 temp;
		NAMESPACE_PHYSICS::multiply(w, w, &temp);

		Mat4 H;
		NAMESPACE_PHYSICS::multiply(temp, TWO_FLOAT, H);
		diff(Mat4Identity, H, H);

		Mat4 temp2, A;
		multiply(H, temp2);
		H.multiply(temp2, A);

		alpha = -sign(A.m32) * sp_sqrt(A.m32 * A.m32 + A.m42 * A.m42);
		r = sp_sqrt((alpha * alpha) * HALF_FLOAT - A.m32 * alpha * HALF_FLOAT);
		two_r = ONE_FLOAT / (r * TWO_FLOAT);

		w.x = ZERO_FLOAT;
		w.y = ZERO_FLOAT;
		w.z = (A.m32 - alpha) * two_r;
		w.w = A.m42 * two_r;

		NAMESPACE_PHYSICS::multiply(w, w, &temp);

		NAMESPACE_PHYSICS::multiply(temp, TWO_FLOAT, temp);
		diff(Mat4Identity, temp, H);

		A.multiply(H, temp2);
		H.multiply(temp2, A);

		std::memcpy(output, A, sizeof(Mat4));

		sp_assert(output->isTridiagonal(), "InvalidOperationException");
	}

	void Mat4::polyname(sp_float* output) const
	{
		const sp_float s1 = trace();
		const sp_float a1 = s1;

		Mat4 AxA;
		multiply(*this, AxA);

		const sp_float s2 = AxA.trace();
		const sp_float a2 = HALF_FLOAT * (s2 - a1 * s1);

		Mat4 AxAxA;
		multiply(AxA, AxAxA);

		const sp_float s3 = AxAxA.trace();
		const sp_float a3 = ONE_OVER_THREE * (s3 - a1 * s2 - a2 * s1);

		Mat4 AxAxAxA;
		multiply(AxAxA, AxAxAxA);

		const sp_float s4 = AxAxAxA.trace();
		const sp_float a4 = ONE_OVER_FOUR * (s4 - a1 * s3 - a2 * s2 - a3 * s1);

		output[0] = -ONE_FLOAT;
		output[1] = a1;
		output[2] = a2;
		output[3] = a3;
		output[4] = a4;
	}
	
	void createScale(const Vec3& scaleFactor, Mat4& output)
	{
		std::memcpy(output, Mat4Identity, sizeof(Mat4));
		output.m11 = scaleFactor.x;
		output.m22 = scaleFactor.y;
		output.m33 = scaleFactor.z;
	}

	void createTranslate(const Vec3& position, Mat4& output)
	{
		std::memcpy(output, Mat3Identity, sizeof(Mat4));
#if MAJOR_COLUMN_ORDER
		output.m41 = position.x;
		output.m42 = position.y;
		output.m43 = position.z;
#else
		output.m14 = position.x;
		output.m24 = position.y;
		output.m34 = position.z;
#endif
	}

	/*
	Mat3 Mat4::toNormalMatrix()
	{
		return toMat3().transpose().invert();
	}
	*/

}