#ifndef MAT3_HEADER
#define MAT3_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"
#include "SystemOfLinearEquations.h"
#include <eigen/Dense>
#include <eigen/Eigenvalues>
#include <unsupported/Eigen/MatrixFunctions>

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
	private:
		sp_float values[MAT3_LENGTH];

	public:

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

		/// <summary>
		/// Get the value from current matrix
		/// COLUMN MAJOR ORDER
		/// X and Y: 1 index base
		/// </summary>
		API_INTERFACE sp_float index(const sp_int x, const sp_int y) const;

		/// <summary>
		/// Check the matrix is symetric
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline sp_bool isSymetric(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return NAMESPACE_FOUNDATION::isCloseEnough(values[1], values[3], _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(values[2], values[6], _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(values[5], values[7], _epsilon);
		}

		/// <summary>
		/// Check the matrix is diagonally dominant
		/// </summary>
		/// <returns>True if it is diagonally dominant orelse False</returns>
		API_INTERFACE inline sp_bool isDiagonallyDominant() const
		{
			return
				   fabsf(values[M11]) > fabsf(values[M12]) + fabsf(values[M13])
				&& fabsf(values[M22]) > fabsf(values[M21]) + fabsf(values[M23])
				&& fabsf(values[M33]) > fabsf(values[M31]) + fabsf(values[M32]);
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
				&& NAMESPACE_FOUNDATION::isCloseEnough(values[2], ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(values[6], ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Get the axis
		/// </summary>
		API_INTERFACE Vec3 axis(const sp_int index) const;

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
		API_INTERFACE Vec3 primaryDiagonal() const;

		/// <summary>
		/// Get the main / principal / major / primary diagonal from matrix
		/// </summary>
		/// <param name="output">Result</param>
		/// <returns>output parameter</returns>
		API_INTERFACE inline void primaryDiagonal(Vec3* output) const;

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
			return values[0] + values[4] + values[8];
		}

		/// <summary>
		/// Load a identity matrix
		/// </summary>
		API_INTERFACE static Mat3 identity();

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE Mat3 transpose() const;

		/// <summary>
		/// Multiply this matrix with the parametrized matrix => AxB
		/// </summary>
		API_INTERFACE Mat3 multiply(const Mat3& matrixB) const;

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

			for (sp_uint row = 0u; row < MAT3_ROW_LENGTH; row++)
			{
				if (i == row)
					continue;

				for (sp_uint column = 0u; column < MAT3_ROW_LENGTH; column++)
				{
					if (j == column)
						continue;

					matrixValues[index] = values[row * MAT3_ROW_LENGTH + column];
					index++;
				}
			}

			return matrixValues[0] * matrixValues[3] - matrixValues[1] * matrixValues[2];
		}

		/// <summary>
		/// Get the cofactor of the index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE sp_float cofactorIJ(const sp_size i, const sp_size j) const;

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
		
		/// <summary>
		/// Get the inverse matrix from current matrix => A^-1
		/// <summary>
		API_INTERFACE Mat3 invert() const;

		/// <summary>
		/// Check if the matrix is identity
		/// </summary>
		API_INTERFACE sp_bool isIdentity() const;

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
			return values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_uint index)
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_uint index) const
		{
			sp_assert(index >= 0 && index < MAT3_LENGTH, "IndexOutOfrangeException");
			return values[index];
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
				values[0] * value, values[1] * value, values[2] * value,
				values[3] * value, values[4] * value, values[5] * value,
				values[6] * value, values[7] * value, values[7] * value
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
			values[0] += matrix[0];
			values[1] += matrix[1];
			values[2] += matrix[2];

			values[3] += matrix[3];
			values[4] += matrix[4];
			values[5] += matrix[5];

			values[6] += matrix[6];
			values[7] += matrix[7];
			values[8] += matrix[8];
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
		API_INTERFACE void decomposeLU(Mat3* lower, Mat3* upper) const;

		/// <summary>
		/// Decompose the matrix to Lower, Diagonal Matrix and Upper matrix
		/// </summary>
		API_INTERFACE void decomposeLDU(Mat3* lower, Mat3* diagonal, Mat3* upper) const;

		/// <summary>
		/// Decompose the matrix to Lower and Lower Transposed using Cholesky Method
		/// </summary>
		/// <param name="lower">Lower Matrix</param>
		/// <param name="lowerTransposed">Lower Transposed Matrix</param>
		/// <returns>void</returns>
		API_INTERFACE void decomposeLLt(Mat3* lower, Mat3* lowerTransposed) const;
		
		/// <summary>
		/// Find the characteristic polyname of this matrix using Leverrier Method
		/// </summary>
		/// <param name="output"></param>
		/// <returns></returns>
		API_INTERFACE void polyname(Vec4* output) const;

		/// <summary>
		/// Get dominant (maximum) autovalue and auto vector of this matrix 
		/// using Method of Powers
		/// </summary>
		API_INTERFACE void eigenValuesAndVectorsMax(sp_float& eigenValue, Vec3& eigenVector, const sp_ushort maxIteration = 5) const;

		/// <summary>
		/// Get the Auto Values of this matrix using Decomposition QR
		/// </summary>
		/// <param name="values">Auto Values</param>
		/// <param name="length">Length</param>
		/// <returns>values</returns>
		API_INTERFACE void eigenValues(Vec3& output, sp_uint& iterations, const sp_float _epsilon = DefaultErrorMargin) const;

		/// <summary>
		/// Get the eigenvalues and eigenvectors of matrix
		/// </summary>
		/// <param name="eigenValues">EigenValues output</param>
		/// <param name="eigenVectors">EigenVectors output</param>
		/// <param name="iterations">Iterations length</param>
		/// <param name="_epsilon">Default error margin</param>
		/// <returns>void</returns>
		API_INTERFACE void eigenValuesAndVectors(Vec3& eigenValues, Mat3& eigenVectors, sp_uint& iterations, const sp_uint maxIterations = SP_UINT_MAX, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;

		/// <summary>
		/// Diagonalize the matrix the symetric matrix using Jacobi method
		/// This method is also called Jacobi rotations
		/// </summary>
		/// <param name="output">Result</param>
		/// <param name="iterationCounter">Provide how many interations was needed</param>
		/// <param name="errorMargin">Epsilon Error</param>
		/// <returns>output parameter</returns>
		API_INTERFACE void diagonalize(Mat3& output, sp_uint& iterationCounter, const sp_float errorMargin = DefaultErrorMargin) const;

		API_INTERFACE void sqrt(Mat3* output) const;

		/// <summary>
		/// Convert a Rotational Matrix to Quaternion
		/// </summary>
		/// <param name="output">Quaterion</param>
		/// <returns>output parameter</returns>
		API_INTERFACE void convert(Quat& output) const;

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
	
	API_INTERFACE inline sp_bool isCloseEnough(const Mat3& m1, const Mat3& m2, const sp_uint _epsilon = DefaultErrorMargin)
	{
		return NAMESPACE_FOUNDATION::isCloseEnough(m1[0], m2[0])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[1], m2[1])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[2], m2[2])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[3], m2[3])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[4], m2[4])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[5], m2[5])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[6], m2[6])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[7], m2[7])
			&& NAMESPACE_FOUNDATION::isCloseEnough(m1[8], m2[8]);
	}

	API_INTERFACE inline void transpose(const Mat3& input, Mat3* output)
	{
		output[0][M11] = input[M11];
		output[0][M12] = input[M21];
		output[0][M13] = input[M31];
		
		output[0][M21] = input[M12];
		output[0][M22] = input[M22];
		output[0][M23] = input[M32];
		
		output[0][M31] = input[M13];
		output[0][M32] = input[M23];
		output[0][M33] = input[M33];
	}

	API_INTERFACE void multiply(const Vec3& v1, const Vec3& v2, Mat3* output);

	API_INTERFACE void multiply(const Mat3& input, const sp_float value, Mat3* output);

	API_INTERFACE void multiply(const Mat3& matrix, const Vec3& vector, Vec3& output);

	API_INTERFACE void multiply(const Mat3& matrixA, const Mat3& matrixB, Mat3& output);

	API_INTERFACE inline void multiply(const Mat3& A, const Mat3& B, const Mat3& C, Mat3& output)
	{
		Mat3 temp;
		multiply(A, B, temp);
		multiply(temp, C, output);
	}

	API_INTERFACE void inverse(const Mat3& input, Mat3& output);

	/// <summary>
	/// Rotate the given matrix using Jacobi Method
	/// </summary>
	API_INTERFACE void rotateJacobi(const Mat3& input, const sp_float sinTheta, const sp_float cosTheta, const sp_uint rowIndex, const sp_uint columnIndex, Mat3& output);

	/// <summary>
	/// Find the square root of Matrix
	/// </summary>
	/// <param name="input">Input Matrix</param>
	/// <param name="output">Output Matrix</param>
	/// <returns>void</returns>
	API_INTERFACE void sqrtm(const Mat3& input, Mat3& output, const sp_uint maxIterations = SP_UINT_MAX);

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
