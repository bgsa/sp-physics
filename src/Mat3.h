#ifndef MAT3_HEADER
#define MAT3_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"
#include "SystemOfLinearEquations.h"

namespace NAMESPACE_PHYSICS
{
#define MAT3_LENGTH         (9)
#define MAT3_SIZE           (MAT3_LENGTH * SIZEOF_FLOAT)
#define MAT3_ROW_LENGTH     (3)
#define MAT3_TWO_ROW_LENGTH (6)

#define M11 (0)
#define M12 (1)
#define M13 (2)

#define M21 (3)
#define M22 (4)
#define M23 (5)

#define M31 (6)
#define M32 (7)
#define M33 (8)

	class Mat3
	{
	public:
		sp_float m11, m12, m13,
				 m21, m22, m23,
				 m31, m32, m33;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE  Mat3();

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE Mat3(
			const sp_float value11, const sp_float value21, const sp_float value31,
			const sp_float value12, const sp_float value22, const sp_float value32,
			const sp_float value13, const sp_float value23, const sp_float value33);

		API_INTERFACE inline sp_float* values() const
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Get the value from current matrix
		/// COLUMN MAJOR ORDER
		/// X and Y: 1 index base
		/// </summary>
		API_INTERFACE sp_float get(const sp_uint x, const sp_uint y) const;

		/// <summary>
		/// Check the matrix is lower
		/// </summary>
		/// <returns>True it the matrix is lower orelse False</returns>
		API_INTERFACE inline sp_bool isLower(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return NAMESPACE_FOUNDATION::isCloseEnough(m12, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m13, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m23, ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Check the matrix is upper
		/// </summary>
		/// <returns>True it the matrix is upper orelse False</returns>
		API_INTERFACE inline sp_bool isUpper(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return NAMESPACE_FOUNDATION::isCloseEnough(m21, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m31, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m32, ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Check the matrix is diagonal
		/// </summary>
		/// <returns>True it the matrix is diagonal orelse False</returns>
		API_INTERFACE inline sp_bool isDiagonal(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return isLower(_epsilon) && isUpper(_epsilon);
		}

		/// <summary>
		/// Check the matrix is symetric
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isSymetric(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return NAMESPACE_FOUNDATION::isCloseEnough(m12, m21, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m13, m31, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m23, m32, _epsilon);
		}

		/// <summary>
		/// Check the matrix is diagonally dominant
		/// </summary>
		/// <returns>True if it is diagonally dominant orelse False</returns>
		API_INTERFACE inline sp_bool isDiagonallyDominant() const
		{
			return
				   fabsf(m11) > fabsf(m12) + fabsf(m13)
				&& fabsf(m22) > fabsf(m21) + fabsf(m23)
				&& fabsf(m33) > fabsf(m31) + fabsf(m32);
		}

		/// <summary>
		/// Check if the the matrix is Positive Definite
		/// </summary>
		/// <returns>True if it is Positive Definite orelse False</returns>
		API_INTERFACE inline sp_bool isPositiveDefinite() const;
		
		/// <summary>
		/// Check the matrix is tridiagonal
		/// </summary>
		/// <param name="_epsilon">Error Margin parameter</param>
		/// <returns>True it the matrix is tridiagonal orelse false</returns>
		API_INTERFACE inline sp_bool isTridiagonal(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return isSymetric()
				&& NAMESPACE_FOUNDATION::isCloseEnough(m13, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m31, ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Get the axis
		/// </summary>
		API_INTERFACE Vec3 axis(const sp_uint index) const;

		/// <summary>
		/// Get the X axis
		/// </summary>
		API_INTERFACE Vec3 xAxis() const;

		/// <summary>
		/// Get the Y axis
		/// </summary>
		API_INTERFACE Vec3 yAxis() const;

		/// <summary>
		/// Get the Z axis
		/// </summary>
		API_INTERFACE Vec3 zAxis() const;

		/// <summary>
		/// Get the main / principal / major / primary diagonal from matrix
		/// </summary>
		/// <param name="output">Result</param>
		/// <returns>output parameter</returns>
		API_INTERFACE inline void primaryDiagonal(Vec3& output) const;

		/// <summary>
		/// Get the antidiagonal / counter / minor / secondary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec3 secondaryDiagonal() const;

		/// <summary>
		/// The trace of a squared matrix is the sum of the primary diagonal elements
		/// </summary>
		/// <param name="output">Result</param>
		/// <returns>output parameter</returns>
		API_INTERFACE inline sp_float trace() const
		{
			return m11 + m22 + m33;
		}

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE inline void transpose()
		{
			std::swap(m12, m21);
			std::swap(m13, m31);
			std::swap(m23, m32);
		}

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE void transpose(Mat3& output) const;

		/// <summary>
		/// Multiply this matrix with the parametrized vector => AxB
		/// </summary>
		API_INTERFACE void multiply(const Vec3& vector, Vec3& output) const;

		/// <summary>
		/// Get the determinant from index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE inline sp_float determinantIJ(const sp_uint i, const sp_uint j) const
		{
			sp_float matrixValues[4];
			sp_uint index = 0u;
			sp_float* v = values();

			for (sp_uint row = 0u; row < MAT3_ROW_LENGTH; row++)
			{
				if (i == row)
					continue;

				for (sp_uint column = 0u; column < MAT3_ROW_LENGTH; column++)
				{
					if (j == column)
						continue;

					matrixValues[index] = v[row * MAT3_ROW_LENGTH + column];
					index++;
				}
			}

			return matrixValues[0] * matrixValues[3] - matrixValues[1] * matrixValues[2];
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m11
		/// </summary>
		/// <returns>Cofactor of element m11</returns>
		API_INTERFACE inline sp_float cofactor11() const
		{
			return m22 * m33 - m23 * m32;
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m12
		/// </summary>
		/// <returns>Cofactor of element m12</returns>
		API_INTERFACE inline sp_float cofactor12() const
		{
			return -(m21 * m33 - m23 * m31);
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m13
		/// </summary>
		/// <returns>Cofactor of element m13</returns>
		API_INTERFACE inline sp_float cofactor13() const
		{
			return m21 * m32 - m22 * m31;
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m21
		/// </summary>
		/// <returns>Cofactor of element m21</returns>
		API_INTERFACE inline sp_float cofactor21() const
		{
			return -(m12 * m33 - m13 * m32);
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m22
		/// </summary>
		/// <returns>Cofactor of element m22</returns>
		API_INTERFACE inline sp_float cofactor22() const
		{
			return m11 * m33 - m13 * m31;
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m23
		/// </summary>
		/// <returns>Cofactor of element m23</returns>
		API_INTERFACE inline sp_float cofactor23() const
		{
			return -(m11 * m32 - m12 * m31);
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m31
		/// </summary>
		/// <returns>Cofactor of element m31</returns>
		API_INTERFACE inline sp_float cofactor31() const
		{
			return m12 * m23 - m13 * m22;
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m32
		/// </summary>
		/// <returns>Cofactor of element m32</returns>
		API_INTERFACE inline sp_float cofactor32() const
		{
			return -(m11 * m23 - m13 * m21);
		}

		/// <summary>
		/// Cofactor/minor of matrix over element m33
		/// </summary>
		/// <returns>Cofactor of element m33</returns>
		API_INTERFACE inline sp_float cofactor33() const
		{
			return m11 * m22 - m12 * m21;
		}

		/// <summary>
		/// Get the cofactor of the index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE inline sp_float cofactorIJ(const sp_uint row, const sp_uint column) const
		{
			return isOdd(row + column)
				? -determinantIJ(row, column)
				: determinantIJ(row, column);
		}

		/// <summary>
		/// Create a scaled matrix
		/// </summary>
		API_INTERFACE static Mat3 createScale(const sp_float xScale, const sp_float yScale, const sp_float zScale);

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE void scale(const sp_float xScale, const sp_float yScale, const sp_float zScale);

		/// <summary>
		/// Create a rotation matrix
		/// Example: x = 1.0 to rotate over X axis; x = 0.0 to not rotate over X axis
		/// </summary>
		API_INTERFACE static Mat3 createRotate(const sp_float angleRadians, const sp_float x, const sp_float y, const sp_float z);

		/// <summary>
		/// Craete a translation matrix
		/// </summary>
		API_INTERFACE static Mat3 createTranslate(const sp_float x, const sp_float y, const sp_float z);

		/// <summary>
		/// Craete a translation matrix
		/// </summary>
		API_INTERFACE static Mat3 createTranslate(const Vec3& position);

		/// <summary>
		/// Get the determinant of the matrix
		/// </summary>
		API_INTERFACE inline sp_float determinant() const
		{
			return
				(m11 * m22 * m33
					+ m12 * m23 * m31
					+ m13 * m21 * m32
				)
				-
				(m13 * m22 * m31
					+ m11 * m23 * m32
					+ m12 * m21 * m33
				);
		}
		
		/// <summary>
		/// Get the inverse matrix from current matrix => A^-1
		/// <summary>
		API_INTERFACE Mat3 invert() const;

		/// <summary>
		/// Get the Classical adjoint/conjugate matrix
		/// Classical Adjoint of a square matrix is the transpose of its cofactor matrix
		/// </summary>
		/// <param name="output">Matrix</param>
		/// <returns>void</returns>
		API_INTERFACE void adjoint(Mat3& output) const
		{
			output.m11 = cofactor11();
			output.m12 = cofactor21();
			output.m13 = cofactor31();

			output.m21 = cofactor12();
			output.m22 = cofactor22();
			output.m23 = cofactor32();

			output.m31 = cofactor13();
			output.m32 = cofactor23();
			output.m33 = cofactor33();
		}

		/// <summary>
		/// Check if the matrix is identity
		/// </summary>
		API_INTERFACE sp_bool isIdentity() const;

		/// <summary>
		/// Check the matrix is orthogonal
		/// </summary>
		/// <returns>True it the matrix is Orthogonal orelse False</returns>
		API_INTERFACE sp_bool isOrthogonal() const;

		/// <summary>
		/// Check if the matrix Hermitian
		/// </summary>
		API_INTERFACE sp_bool isHermitian() const;

		/// <summary>
		/// Get the size in Bytes of Mat3
		/// </summary>
		API_INTERFACE sp_size sizeInBytes() const;

		/// <summary>
		/// Clone this matrix
		/// </summary>
		API_INTERFACE inline void clone(Mat3* output) const
		{
			memcpy(output, this, sizeof(Mat3));
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_int index)
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_uint index)
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_uint index) const
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

#if defined(WINDOWS) && defined(ENV_64BITS)
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](const sp_size index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](const sp_size index) const;
#endif

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE operator sp_float*() const;

		/// <summary>
		/// Get the negative matrix
		/// </summary>
		API_INTERFACE Mat3 operator-() const;

		/// <summary>
		/// Subtract the matrix to another one
		/// </summary>
		API_INTERFACE Mat3 operator-(const Mat3& matrix) const;

		/// <summary>
		/// Sum the matrix to another one
		/// </summary>
		API_INTERFACE Mat3 operator+(const Mat3& matrix) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE Mat3 operator*(const Mat3& matrix) const;

		/// <summary>
		/// Multiply the value by all components of matrix
		/// </summary>
		API_INTERFACE inline Mat3 operator*(const sp_float value) const
		{
			return Mat3(
				m11 * value, m12 * value, m13 * value,
				m21 * value, m22 * value, m23 * value,
				m31 * value, m32 * value, m32 * value
			);
		}

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE void operator*=(const Mat3& matrix);

		/// <summary>
		/// Sum this matrix with another
		/// </summary>
		API_INTERFACE void operator+=(const Mat3& matrix)
		{
			m11 += matrix[0];
			m12 += matrix[1];
			m13 += matrix[2];

			m21 += matrix[3];
			m22 += matrix[4];
			m23 += matrix[5];

			m31 += matrix[6];
			m32 += matrix[7];
			m33 += matrix[8];
		}

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE Mat3 operator/(const sp_float value) const;

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE void operator/=(const sp_float value);

		/// <summary>
		/// Check the matrix is equal the other
		/// </summary>
		API_INTERFACE sp_bool operator==(const Mat3& matrix) const;

		/// <summary>
		/// Check the matrix is equal the other
		/// </summary>
		API_INTERFACE sp_bool operator==(const sp_float value) const;

		/// <summary>
		/// Check the matrix is different the other
		/// </summary>
		API_INTERFACE sp_bool operator!=(const Mat3& matrix) const;

		/// <summary>
		/// Get the matrix content as string
		/// </summary>
		API_INTERFACE std::string toString() const;

		/// <summary>
		/// Decompose the matrix to Q (orthogonal) and R (rotational) matrix
		/// The matrix must be tridiagonal. If it is not, you can use HouseHolder Method first
		/// </summary>
		API_INTERFACE void decomposeQR(Mat3& Q, Mat3& R, sp_uint& iterations, const sp_float _epsilon) const;

		/// <summary>
		/// Decompose the matrix to Lower and Upper matrix
		/// </summary>
		API_INTERFACE void decomposeLU(Mat3& lower, Mat3& upper) const;

		/// <summary>
		/// Decompose the matrix to Lower, Diagonal Matrix and Upper matrix
		/// </summary>
		API_INTERFACE void decomposeLDU(Mat3& lower, Mat3& diagonal, Mat3& upper) const;

		/// <summary>
		/// Decompose the matrix to Lower and Lower Transposed using Cholesky Method
		/// </summary>
		/// <param name="lower">Lower Matrix</param>
		/// <param name="lowerTransposed">Lower Transposed Matrix</param>
		/// <returns>void</returns>
		API_INTERFACE void decomposeLLt(Mat3& lower, Mat3& lowerTransposed) const;
		
		/// <summary>
		/// Find the characteristic polyname of this matrix using Leverrier Method
		/// </summary>
		/// <param name="output"></param>
		/// <returns></returns>
		API_INTERFACE void polyname(Vec4* output) const;

		/// <summary>
		/// Get a symmetric matrix from this matrix
		/// </summary>
		/// <param name="output">Symmetric matrix</param>
		/// <returns></returns>
		API_INTERFACE inline void symmetric(Mat3& output) const;

		/// <summary>
		/// Get dominant (maximum) autovalue and auto vector of this matrix 
		/// This method uses Method of Powers
		/// </summary>
		API_INTERFACE void eigenValuesAndVectorsMax(sp_float& eigenValue, Vec3& eigenVector, const sp_ushort maxIteration = 5) const;

		/// <summary>
		/// Get the EigenValues of this matrix using Jacobi Rotations
		/// </summary>
		/// <param name="output">EigenValues</param>
		/// <param name="iterations">Iteration taken to converge</param>
		/// <param name="_epsilon">Default Margin Error</param>
		/// <returns>output</returns>
		API_INTERFACE void eigenValues(Vec3& output, sp_uint& iterations, const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Get the eigenvalues and eigenvectors of matrix
		/// This method uses Jacobi Rotations
		/// </summary>
		/// <param name="eigenValues">EigenValues output</param>
		/// <param name="eigenVectors">EigenVectors output</param>
		/// <param name="iterations">Iterations length</param>
		/// <param name="_epsilon">Default error margin</param>
		/// <returns>True if the method converge orelse False</returns>
		API_INTERFACE sp_bool eigenValuesAndVectors(Vec3& eigenValues, Mat3& eigenVectors, sp_uint& iterations, const sp_uint maxIterations = SP_UINT_MAX, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;

		/// <summary>
		/// Diagonalize the matrix the symetric matrix using Jacobi method
		/// This method is also called Jacobi rotations
		/// </summary>
		/// <param name="output">Result</param>
		/// <param name="iterations">Provide how many interations was needed</param>
		/// <param name="_epsilon">Epsilon Error</param>
		/// <returns>output parameter</returns>
		API_INTERFACE void diagonalize(Mat3& output, sp_uint& iterations, const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Apply Square Root to all elements of the matrix
		/// </summary>
		/// <param name="output">Matrix</param>
		/// <returns>output</returns>
		API_INTERFACE inline void sqrt(Mat3& output) const
		{
			output.m11 = sqrtf(m11);
			output.m12 = sqrtf(m12);
			output.m13 = sqrtf(m13);

			output.m21 = sqrtf(m21);
			output.m22 = sqrtf(m22);
			output.m23 = sqrtf(m23);
			
			output.m31 = sqrtf(m31);
			output.m32 = sqrtf(m32);
			output.m33 = sqrtf(m33);
		}

		/// <summary>
		/// Convert a Rotational Matrix to Quaternion
		/// </summary>
		/// <param name="output">Quaterion</param>
		/// <returns>output parameter</returns>
		API_INTERFACE void convert(Quat& output) const;

		/// <summary>
		/// Negate all elements of the matrix
		/// A = -A
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void negate()
		{
			m11 = -m11;
			m12 = -m12;
			m13 = -m13;
			m21 = -m21;
			m22 = -m22;
			m23 = -m23;
			m31 = -m31;
			m32 = -m32;
			m33 = -m33;
		}
	};

	const Mat3 Mat3Zeros = {
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT
	};

	const Mat3 Mat3Identity = {
		ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
	};
	
	API_INTERFACE inline sp_bool isCloseEnough(const Mat3& A, const Mat3& B, const sp_float _epsilon = DefaultErrorMargin)
	{
		return NAMESPACE_FOUNDATION::isCloseEnough(A.m11, B.m11, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m12, B.m12, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m13, B.m13, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m21, B.m21, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m22, B.m22, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m23, B.m23, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m31, B.m31, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m32, B.m32, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(A.m33, B.m33, _epsilon);
	}

	API_INTERFACE inline void transpose(const Mat3& input, Mat3& output)
	{
		output[M11] = input[M11];
		output[M12] = input[M21];
		output[M13] = input[M31];
		
		output[M21] = input[M12];
		output[M22] = input[M22];
		output[M23] = input[M32];
		
		output[M31] = input[M13];
		output[M32] = input[M23];
		output[M33] = input[M33];
	}

	/// <summary>
	/// Create Matrix Rotation over X axis
	/// </summary>
	/// <param name="output">Rotation Matrix output</param>
	/// <param name="cosTheta">Cosine angle theta</param>
	/// <param name="sinTheta">Sine angle theta</param>
	/// <returns>Output parameter</returns>
	API_INTERFACE inline void rotationX(Mat3& output, const sp_float cosTheta, const sp_float sinTheta)
	{
		std::memcpy(output, Mat3Identity, sizeof(Mat3));
		output.m22 = cosTheta;
		output.m23 = -sinTheta;
		output.m32 = sinTheta;
		output.m33 = cosTheta;
	}

	/// <summary>
	/// Create Matrix Rotation over Y axis
	/// </summary>
	/// <param name="output">Rotation Matrix output</param>
	/// <param name="cosTheta">Cosine angle theta</param>
	/// <param name="sinTheta">Sine angle theta</param>
	/// <returns>Output parameter</returns>
	API_INTERFACE inline void rotationY(Mat3& output, const sp_float cosTheta, const sp_float sinTheta)
	{
		std::memcpy(output, Mat3Identity, sizeof(Mat3));
		output.m11 = cosTheta;
		output.m13 = sinTheta;
		output.m31 = -sinTheta;
		output.m33 = cosTheta;
	}

	/// <summary>
	/// Create Matrix Rotation over Z axis
	/// </summary>
	/// <param name="output">Rotation Matrix output</param>
	/// <param name="cosTheta">Cosine angle theta</param>
	/// <param name="sinTheta">Sine angle theta</param>
	/// <returns>Output parameter</returns>
	API_INTERFACE inline void rotationZ(Mat3& output, const sp_float cosTheta, const sp_float sinTheta)
	{
		std::memcpy(output, Mat3Identity, sizeof(Mat3));
		output.m11 = cosTheta;
		output.m12 = -sinTheta;
		output.m21 = sinTheta;
		output.m22 = cosTheta;
	}

	API_INTERFACE void multiply(const Vec3& v1, const Vec3& v2, Mat3& output);

	API_INTERFACE void multiply(const Mat3& input, const sp_float value, Mat3& output);

	API_INTERFACE void multiply(const Mat3& matrix, const Vec3& vector, Vec3& output);

	API_INTERFACE void multiply(const Mat3& matrixA, const Mat3& matrixB, Mat3& output);

	API_INTERFACE inline void multiply(const Mat3& A, const Mat3& B, const Mat3& C, Mat3& output)
	{
		Mat3 temp;
		multiply(A, B, temp);
		multiply(temp, C, output);
	}

	API_INTERFACE void inverse(const Mat3& input, Mat3& output);

	API_INTERFACE inline void givensRotation(Mat3& output, const sp_uint rowIndex, const sp_uint columnIndex, const sp_float sinTheta, const sp_float cosTheta);

	/// <summary>
	/// Rotate the given matrix using Jacobi Method
	/// </summary>
	/// <param name="input">Matrix</param>
	/// <param name="sinTheta">Sin</param>
	/// <param name="cosTheta">Cos</param>
	/// <param name="rowIndex">RowIndex</param>
	/// <param name="columnIndex">ColumnIndex</param>
	/// <param name="output">Rotated Matrix</param>
	/// <param name="jacobiRotation">Jacobi Rotation used to rotate the matrix</param>
	/// <returns>void</returns>
	API_INTERFACE void jacobiRotation(const Mat3& input, const sp_float sinTheta, const sp_float cosTheta, const sp_uint rowIndex, const sp_uint columnIndex, Mat3& output, Mat3& jacobiRotation);

	/// <summary>
	/// Find the square root of Matrix
	/// </summary>
	/// <param name="input">Input Matrix</param>
	/// <param name="output">Output Matrix</param>
	/// <returns>True if the method converged orelse False</returns>
	API_INTERFACE sp_bool sqrtm(const Mat3& input, Mat3& output, const sp_uint maxIterations = 20);

#undef M11
#undef M12
#undef M13

#undef M21
#undef M22
#undef M23

#undef M31
#undef M32
#undef M33

}

#endif // ! MAT3_HEADER
