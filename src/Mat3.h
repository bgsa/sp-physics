#ifndef MAT3_HEADER
#define MAT3_HEADER

#include "SpectrumPhysics.h"
#include <cstring>
#include <vector>

namespace NAMESPACE_PHYSICS
{
#define MAT3_SIZE 9
#define MAT3_ROWSIZE 3
#define TWO_MAT3_ROWSIZE 6

	template <typename T>
	struct AutovalueAutovector3 {
		T autovalue;
		Vec3<T> autovector;
	};

	template <typename T>
	class Mat3 : public Mat<T>
	{
	private:
		T values[MAT3_SIZE];

	public:

		/// <summary>
		/// Default constructor
		/// Load a empty matrix = 0
		/// </summary>
		API_INTERFACE  Mat3(const T defaultValue = T(0));

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE  Mat3(T* values);

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE Mat3(
			const T value11, const T value21, const T value31,
			const T value12, const T value22, const T value32,
			const T value13, const T value23, const T value33);

		/// <summary>
		/// Get the values from current matrix
		/// </summary>
		API_INTERFACE T* getValues();

		/// <summary>
		/// Get the value from current matrix
		/// COLUMN MAJOR ORDER
		/// X and Y: 1 index base
		/// </summary>
		API_INTERFACE T getValue(const sp_int x, const sp_int y) const;

		/// <summary>
		/// Get the axis
		/// </summary>
		API_INTERFACE Vec3<T> getAxis(const sp_int index) const;

		/// <summary>
		/// Get the X axis
		/// </summary>
		API_INTERFACE Vec3<T> xAxis() const;

		/// <summary>
		/// Get the Y axis
		/// </summary>
		API_INTERFACE Vec3<T> yAxis() const;

		/// <summary>
		/// Get the Z axis
		/// </summary>
		API_INTERFACE Vec3<T> zAxis() const;

		/// <summary>
		/// Get the main / principal / major / primary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec3<T> primaryDiagonal() const;

		/// <summary>
		/// Get the antidiagonal / counter / minor / secondary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec3<T> secondaryDiagonal() const;

		/// <summary>
		/// Load a identity matrix
		/// </summary>
		API_INTERFACE static Mat3<T> identity();

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE Mat3<T> transpose() const;

		/// <summary>
		/// Multiply this matrix with the parametrized matrix => AxB
		/// </summary>
		API_INTERFACE Mat3<T> multiply(const Mat3<T>& matrixB) const;

		/// <summary>
		/// Multiply this matrix with the parametrized vector => AxB
		/// </summary>
		API_INTERFACE Vec3<T> multiply(const Vec3<T>& vector) const;

		/// <summary>
		/// Get the determinant from index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE T determinantIJ(const sp_size i, const sp_size j) const;

		/// <summary>
		/// Get the cofactor of the index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE T cofactorIJ(const sp_size i, const sp_size j) const;

		/// <summary>
		/// Create a scaled matrix
		/// </summary>
		API_INTERFACE static Mat3<T> createScale(const T xScale, const T yScale, const T zScale);

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE void scale(const T xScale, const T yScale, const T zScale);

		/// <summary>
		/// Create a rotation matrix
		/// Example: x = 1.0 to rotate over X axis; x = 0.0 to not rotate over X axis
		/// </summary>
		API_INTERFACE static Mat3<T> createRotate(const T angleRadians, const T x, const T y, const T z);

		/// <summary>
		/// Craete a translation matrix
		/// </summary>
		API_INTERFACE static Mat3<T> createTranslate(const T x, const T y, const T z);

		/// <summary>
		/// Craete a translation matrix
		/// </summary>
		API_INTERFACE static Mat3<T> createTranslate(const Vec3<T>& position);

		/// <summary>
		/// Get the determinant of the matrix
		/// </summary>
		API_INTERFACE T determinant() const;
		
		/// <summary>
		/// Get the inverse matrix from current matrix => A^-1
		/// <summary>
		API_INTERFACE Mat3<T> invert() const;

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
		API_INTERFACE Mat3<T> clone() const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE T& operator[](const sp_int index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE T operator[](const sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE T& operator[](const sp_uint index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE T operator[](const sp_uint index) const;
		
#if defined(WINDOWS) && defined(ENV_64BITS)
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE T& operator[](const sp_size index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE T operator[](const sp_size index) const;
#endif

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE operator T*() const;

		/// <summary>
		/// Get the negative matrix
		/// </summary>
		API_INTERFACE Mat3<T> operator-() const;

		/// <summary>
		/// Subtract the matrix to another one
		/// </summary>
		API_INTERFACE Mat3<T> operator-(const Mat3<T>& matrix) const;

		/// <summary>
		/// Sum the matrix to another one
		/// </summary>
		API_INTERFACE Mat3<T> operator+(const Mat3<T>& matrix) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE Mat3<T> operator*(const Mat3<T>& matrix) const;

		/// <summary>
		/// Multiply the matrix to 3D vector
		/// </summary>
		API_INTERFACE Vec3<T> operator*(const Vec3<T>& matrix) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE void operator*=(const Mat3<T>& matrix);

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE Mat3<T> operator/(const T value) const;

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE void operator/=(const T value);

		/// <summary>
		/// Check the matrix is equal the other
		/// </summary>
		API_INTERFACE sp_bool operator==(const Mat3<T>& matrix) const;

		/// <summary>
		/// Check the matrix is equal the other
		/// </summary>
		API_INTERFACE sp_bool operator==(const T value) const;

		/// <summary>
		/// Check the matrix is different the other
		/// </summary>
		API_INTERFACE sp_bool operator!=(const Mat3<T>& matrix) const;

		/// <summary>
		/// Get the matrix content as string
		/// </summary>
		API_INTERFACE std::string toString() const;

		/// <summary>
		/// Decompose the matrix to Lower and Upper matrix
		/// </summary>
		API_INTERFACE Mat3<T>* decomposeLU() const;

		/// <summary>
		/// Decompose the matrix to Lower, Diagonal Matrix and Upper matrix
		/// </summary>
		API_INTERFACE Mat3<T>* decomposeLDU() const;

		/// <summary>
		/// Get the autovalue of the matrix
		/// </summary>
		API_INTERFACE AutovalueAutovector3<T> getAutovalueAndAutovector(const sp_ushort maxIteration = 5) const;

	};

	typedef Mat3<sp_int> Mat3i;
	typedef Mat3<sp_float> Mat3f;
	typedef Mat3<sp_double> Mat3d;

}

#endif // ! MAT3_HEADER
