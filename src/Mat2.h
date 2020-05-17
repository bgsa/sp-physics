#ifndef MAT2_HEADER
#define MAT2_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"

namespace NAMESPACE_PHYSICS
{
#define MAT2_SIZE 4
#define MAT2_ROWSIZE 2

	class Mat2
	{
	private:
		sp_float values[MAT2_SIZE];

	public:

		/// <summary>
		/// Default constructor
		/// Load a empty matrix = 0
		/// </summary>
		API_INTERFACE Mat2(const sp_float defaultValue = 0.0f);

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE Mat2(sp_float* values);

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE Mat2(const sp_float value11, const sp_float value12, const sp_float value21, const sp_float value22);

		/// <summary>
		/// Get the values from current matrix
		/// </summary>
		API_INTERFACE sp_float* getValues();

		/// <summary>
		/// Get the value from current matrix
		/// COLUMN MAJOR ORDER
		/// X and Y: 1 index base
		/// </summary>
		API_INTERFACE sp_float getValue(const sp_int x, const sp_int y);

		/// <summary>
		/// Get the X axis
		/// COLUMN MAJOR ORDER
		/// </summary>
		API_INTERFACE Vec2 xAxis() const;

		/// <summary>
		/// Get the Y axis
		/// COLUMN MAJOR ORDER
		/// </summary>
		API_INTERFACE Vec2 yAxis() const;

		/// <summary>
		/// Get the main / principal / major / primary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec2 primaryDiagonal() const;

		/// <summary>
		/// Get the antidiagonal / counter / minor / secondary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec2 secondaryDiagonal() const;

		/// <summary>
		/// Load a identity matrix
		/// </summary>
		API_INTERFACE static Mat2 identity();

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE Mat2 transpose() const;

		/// <summary>
		/// Multiply this matrix with the parametrized matrix => AxB
		/// </summary>
		API_INTERFACE Mat2 multiply(const Mat2& matrixB) const;

		/// <summary>
		/// Multiply this matrix with the parametrized vector => AxB
		/// </summary>
		API_INTERFACE Vec2 multiply(const Vec2& vector) const;

		/// <summary>
		/// Create a scaled matrix
		/// </summary>
		API_INTERFACE static Mat2 createScale(const sp_float xScale, const sp_float yScale);

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE void scale(const sp_float xScale, const sp_float yScale);

		/// <summary>
		/// Craete a translation matrix
		/// </summary>
		API_INTERFACE static Mat2 createTranslate(const sp_float x, const sp_float y);

		/// <summary>
		/// Get the deeterminant of the Matrix
		/// </summary>
		API_INTERFACE sp_float determinant() const;

		/// <summary>
		/// Get the autovalue of the matrix
		/// </summary>
		API_INTERFACE AutoValueAutoVector2 getAutovalueAndAutovector(const sp_ushort maxIteration = 5) const;

		/// <summary>
		/// Get the size in Bytes of Mat3
		/// </summary>
		API_INTERFACE sp_size sizeInBytes() const;

		/// <summary>
		/// Clone this matrix
		/// </summary>
		API_INTERFACE Mat2 clone() const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](sp_int index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](sp_int index) const;

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
		API_INTERFACE Mat2 operator-() const;

		/// <summary>
		/// Subtract a matrix to another one
		/// </summary>
		API_INTERFACE Mat2 operator-(const Mat2& matrix) const;

		/// <summary>
		/// Sum a matrix to another one
		/// </summary>
		API_INTERFACE Mat2 operator+(const Mat2& matrix) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE Mat2 operator*(const Mat2& matrix) const;

		/// <summary>
		/// Multiply the matrix to 2D vector
		/// </summary>
		API_INTERFACE Vec2 operator*(const Vec2& matrix) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE void operator*=(const Mat2& matrix);

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE Mat2 operator/(sp_float value) const;

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE void operator/=(sp_float value);

		/// <summary>
		/// Get the matrix content as string
		/// </summary>
		API_INTERFACE std::string toString() const;

	};
}

#endif // !MAT2_HEADER
