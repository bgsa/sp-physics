#ifndef MAT3_HEADER
#define MAT3_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"
#include <cstring>
#include <vector>

namespace NAMESPACE_PHYSICS
{
#define MAT3_LENGTH         (9)
#define MAT3_SIZE           (MAT3_LENGTH * SIZEOF_FLOAT)
#define MAT3_ROW_LENGTH     (3)
#define MAT3_TWO_ROW_LENGTH (6)

	class Mat3
	{
	private:
		sp_float values[MAT3_LENGTH];

	public:

		/// <summary>
		/// Default constructor
		/// Load a empty matrix = 0
		/// </summary>
		API_INTERFACE  Mat3(const sp_float defaultValue = ZERO_FLOAT);

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE  Mat3(sp_float* values);

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE Mat3(
			const sp_float value11, const sp_float value21, const sp_float value31,
			const sp_float value12, const sp_float value22, const sp_float value32,
			const sp_float value13, const sp_float value23, const sp_float value33);

		/// <summary>
		/// Get the values from current matrix
		/// </summary>
		API_INTERFACE sp_float* getValues();

		/// <summary>
		/// Get the value from current matrix
		/// COLUMN MAJOR ORDER
		/// X and Y: 1 index base
		/// </summary>
		API_INTERFACE sp_float getValue(const sp_int x, const sp_int y) const;

		/// <summary>
		/// Get the axis
		/// </summary>
		API_INTERFACE Vec3 getAxis(const sp_int index) const;

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
		/// Get the antidiagonal / counter / minor / secondary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec3 secondaryDiagonal() const;

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
		API_INTERFACE Vec3 multiply(const Vec3& vector) const;

		/// <summary>
		/// Get the determinant from index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE sp_float determinantIJ(const sp_size i, const sp_size j) const;

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
		API_INTERFACE sp_float determinant() const;
		
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
		API_INTERFACE Mat3 clone() const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](const sp_int index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](const sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](const sp_uint index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](const sp_uint index) const;
		
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
		/// Multiply the matrix to 3D vector
		/// </summary>
		API_INTERFACE Vec3 operator*(const Vec3& matrix) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE void operator*=(const Mat3& matrix);

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
		/// Decompose the matrix to Lower and Upper matrix
		/// </summary>
		API_INTERFACE Mat3* decomposeLU() const;

		/// <summary>
		/// Decompose the matrix to Lower, Diagonal Matrix and Upper matrix
		/// </summary>
		API_INTERFACE Mat3* decomposeLDU() const;

		/// <summary>
		/// Get the autovalue of the matrix
		/// </summary>
		API_INTERFACE AutoValueAutoVector3 getAutovalueAndAutovector(const sp_ushort maxIteration = 5) const;

	};

}

#endif // ! MAT3_HEADER
