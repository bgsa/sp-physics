#ifndef MAT4_HEADER
#define MAT4_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"

namespace NAMESPACE_PHYSICS
{
#define MAT4_LENGTH     (16)
#define MAT4_SIZE       (MAT4_LENGTH * SIZEOF_FLOAT)
#define MAT4_ROW_LENGTH (4)

	class Mat4
	{
	private:
		sp_float values[MAT4_LENGTH];

	public:

		/// <summary>
		/// Default constructor
		/// Load a empty matrix = 0
		/// </summary>
		API_INTERFACE Mat4(const sp_float defaultValue = ZERO_FLOAT);

		/// <summary>
		/// Construct the matrix from 4 vectors
		/// </summary>
		API_INTERFACE Mat4(const Vec4& vector1, const Vec4& vector2, const Vec4& vector3, const Vec4& vector4);

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE Mat4(sp_float* values);

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE Mat4(
			const sp_float value11, const sp_float value21, const sp_float value31, const sp_float value41,
			const sp_float value12, const sp_float value22, const sp_float value32, const sp_float value42,
			const sp_float value13, const sp_float value23, const sp_float value33, const sp_float value43,
			const sp_float value14, const sp_float value24, const sp_float value34, const sp_float value44);

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
		/// Get the X axis
		/// COLUMN MAJOR ORDER
		/// </summary>
		API_INTERFACE Vec4 xAxis() const;

		/// <summary>
		/// Get the Y axis
		/// COLUMN MAJOR ORDER
		/// <summary>
		API_INTERFACE Vec4 yAxis() const;

		/// <summary>
		/// Get the Z axis
		/// COLUMN MAJOR ORDER
		/// </summary>
		API_INTERFACE Vec4 zAxis() const;

		/// <summary>
		/// Get the Z axis
		/// COLUMN MAJOR ORDER
		/// </summary>
		API_INTERFACE Vec4 wAxis() const;

		/// <summary>
		/// Get the main / principal / major / primary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec4 primaryDiagonal() const;

		/// <summary>
		/// Get the antidiagonal / counter / minor / secondary diagonal from matrix
		/// </summary>
		API_INTERFACE Vec4 secondaryDiagonal() const;

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE Mat4 transpose() const;

		/// <summary>
		/// Get the determinant from index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE sp_float determinantIJ(const sp_int i, const sp_int j) const;

		/// <summary>
		/// Get the cofactor of the index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE sp_float cofactorIJ(const sp_int i, const  sp_int j) const;

		/// <summary>
		/// Get the determinant from matrix
		/// </summary>
		API_INTERFACE sp_float determinant() const;

		/// <summary>
		/// Multiply this matrix with the parametrized matrix => AxB
		/// <summary>
		API_INTERFACE Mat4 multiply(const Mat4 &matrixB) const;

		/// <summary>
		/// Multiply this matrix with the parametrized vector => AxB
		/// </summary>
		API_INTERFACE Vec4 multiply(const Vec4 &vector) const;

		/// <summary>
		/// Get the inverse matrix from current matrix => A^-1
		/// <summary>
		API_INTERFACE Mat4 invert() const;

		/// <summary>
		/// Create a scaled matrix
		/// </summary>
		API_INTERFACE static Mat4 createScale(const sp_float xScale, const sp_float yScale, const sp_float zScale);

		/// <summary>
		/// Create a scaled matrix
		/// </summary>
		API_INTERFACE static Mat4 createScale(const Vec3& scale);

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE void scale(const sp_float xScale, const sp_float yScale, const sp_float zScale);

		/// <summary>
		/// Create a rotation matrix
		/// Example: x = 1.0 to rotate over X axis; x = 0.0 to not rotate over X axis
		/// </summary>
		API_INTERFACE static Mat4 createRotate(const sp_float angleRadians, const sp_float x, const sp_float y, const sp_float z);

		/// <summary>
		/// Create a translation matrix
		/// </summary>
		API_INTERFACE static Mat4 createTranslate(const sp_float x, const sp_float y, const sp_float z);

		/// <summary>
		/// Create a translation matrix
		/// </summary>
		API_INTERFACE static Mat4 createTranslate(const Vec3& position);

		/// <summary>
		/// Craete a orthographic matrix projection
		/// </summary>
		API_INTERFACE static Mat4 createOrthographicMatrix(const sp_float xMin, const sp_float xMax, const sp_float yMin, const sp_float yMax, const sp_float zMin, const sp_float zMax);

		/// <summary>
		/// Clone this matrix
		/// </summary>
		API_INTERFACE Mat4 clone() const;

		/// <summary>
		/// Get the negative matrix
		/// </summary>
		API_INTERFACE Mat4 operator-() const;

		/// <summary>
		/// Subtract a scalar from this matrix
		/// </summary>
		API_INTERFACE Mat4 operator-(const sp_float value) const;

		/// <summary>
		/// Subtract this matrix to another one
		/// </summary>
		API_INTERFACE Mat4 operator-(const Mat4& matrix) const;

		/// <summary>
		/// Sum a scalar to this matrix
		/// </summary>
		API_INTERFACE Mat4 operator+(const sp_float value) const;
		
		/// <summary>
		/// Sum the matrix to another one
		/// </summary>
		API_INTERFACE Mat4 operator+(const Mat4& matrix) const;
		
		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE Mat4 operator*(const sp_float value) const;

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE void operator*=(const Mat4& matrix);

		/// <summary>
		/// Multiply the matrix to another one
		/// </summary>
		API_INTERFACE Mat4 operator*(const Mat4& matrix) const;

		/// <summary>
		/// Multiply the matrix to a vector
		/// </summary>
		API_INTERFACE Vec4 operator*(const Vec4 &vector) const;

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE Mat4 operator/(const sp_float value) const;

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE void operator/=(const sp_float value);

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE sp_bool operator==(const Mat4& matrix) const;

		/// <summary>
		/// Compare this matrix to another one. Compare each component.
		/// </summary>
		API_INTERFACE sp_bool operator==(const sp_float value) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE sp_bool operator!=(const Mat4& matrix) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](const sp_int index);

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](const sp_int index) const;

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to sp_float*
		/// </summary>
		API_INTERFACE operator sp_float*();

		/// <summary>
		/// Auto convertion to const sp_float*
		/// </summary>
		API_INTERFACE operator sp_float*() const;

		/// <summary>
		/// Convert the matrix to Matrix 3x3
		/// Returns the first 3 components of x-Axis, y-Axis, z-Axis
		/// </summary>
		API_INTERFACE Mat3 toMat3() const;

		/// <summary>
		/// Get the matrix content as string
		/// </summary>
		API_INTERFACE std::string toString() const;

		/// <summary>
		/// Decompose the matrix to Lower and Upper matrix
		/// </summary>
		API_INTERFACE Mat4* decomposeLU() const;

		/// <summary>
		/// Decompose the matrix to Lower, Diagonal Matrix and Upper matrix
		/// </summary>
		API_INTERFACE Mat4* decomposeLDU() const;

		/// <summary>
		/// Get the autovalue of the matrix
		/// </summary>
		API_INTERFACE AutoValueAutoVector4 getAutovalueAndAutovector(const sp_short maxIteration = 5) const;

		/*
		/// <summary>
		/// Get the normal matrix
		/// </summary>
		API_INTERFACE Mat3 toNormalMatrix();
		*/
	};

	const Mat4 Mat4Identity = {
		ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
	};

}

#endif // !MAT4_HEADER
