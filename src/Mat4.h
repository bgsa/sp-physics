#ifndef MAT4_HEADER
#define MAT4_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"

namespace NAMESPACE_PHYSICS
{
#define MAT4_LENGTH     (16)
#define MAT4_ROW_LENGTH (4)

	class Mat4
	{
	public:
		sp_float 
			m11, m12, m13, m14,
			m21, m22, m23, m24,
			m31, m32, m33, m34,
			m41, m42, m43, m44;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE inline Mat4() { }

		/// <summary>
		/// Constructur with constant value for all elements
		/// </summary>
		API_INTERFACE inline Mat4(const sp_float defaultValue)
		{
			m11 = m12 = m13 = m14 
				= m21 = m22 = m23 = m24
				= m31 = m32 = m33 = m34
				= m41 = m42 = m43 = m44 
				= defaultValue;
		}

		/// <summary>
		/// Construct the matrix from 4 vectors
		/// </summary>
		API_INTERFACE Mat4(const Vec4& vector1, const Vec4& vector2, const Vec4& vector3, const Vec4& vector4);

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE Mat4(
			const sp_float value11, const sp_float value21, const sp_float value31, const sp_float value41,
			const sp_float value12, const sp_float value22, const sp_float value32, const sp_float value42,
			const sp_float value13, const sp_float value23, const sp_float value33, const sp_float value43,
			const sp_float value14, const sp_float value24, const sp_float value34, const sp_float value44)
		{
			m11 = value11;
			m12 = value21;
			m13 = value31;
			m14 = value41;

			m21 = value12;
			m22 = value22;
			m23 = value32;
			m24 = value42;

			m31 = value13;
			m32 = value23;
			m33 = value33;
			m34 = value43;

			m41 = value14;
			m42 = value24;
			m43 = value34;
			m44 = value44;
		}

		/// <summary>
		/// Check this matrix is symetric
		/// </summary>
		API_INTERFACE inline sp_bool isSymetric(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return NAMESPACE_FOUNDATION::isCloseEnough(m12, m21, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m13, m31, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m14, m41, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m23, m32, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m24, m42, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m34, m43, _epsilon);
		}

		/// <summary>
		/// Check the matrix is tridiagonal
		/// </summary>
		/// <param name="_epsilon">Error Margin</param>
		/// <returns>True if the matrix is tridiagonal orelse False</returns>
		API_INTERFACE inline sp_bool isTridiagonal(const sp_float _epsilon = DefaultErrorMargin) const
		{
			return isSymetric()
				&& NAMESPACE_FOUNDATION::isCloseEnough(m13, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m14, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m24, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m31, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m41, ZERO_FLOAT, _epsilon)
				&& NAMESPACE_FOUNDATION::isCloseEnough(m42, ZERO_FLOAT, _epsilon);
		}

		/// <summary>
		/// Get the values from current matrix
		/// </summary>
		API_INTERFACE inline sp_float* values() const
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Get the value from current matrix
		/// 0 index base
		/// </summary>
		API_INTERFACE inline sp_float get(const sp_int row, const sp_int column) const
		{
			return values()[row * MAT4_ROW_LENGTH + column];
		}

		/// <summary>
		/// Get the X axis
		/// </summary>
		API_INTERFACE inline Vec4 xAxis() const;

		/// <summary>
		/// Get the Y axis
		/// <summary>
		API_INTERFACE inline Vec4 yAxis() const;

		/// <summary>
		/// Get the Z axis
		/// </summary>
		API_INTERFACE inline Vec4 zAxis() const;

		/// <summary>
		/// Get the W axis
		/// </summary>
		API_INTERFACE inline Vec4 wAxis() const;

		/// <summary>
		/// Get the main / principal / major / primary diagonal from matrix
		/// </summary>
		API_INTERFACE inline Vec4 primaryDiagonal() const;

		/// <summary>
		/// Get the antidiagonal / counter / minor / secondary diagonal from matrix
		/// </summary>
		API_INTERFACE inline Vec4 secondaryDiagonal() const;

		/// <summary>
		/// The trace of a squared matrix is the sum of the primary diagonal elements
		/// </summary>
		/// <param name="output">Result</param>
		/// <returns>output parameter</returns>
		API_INTERFACE inline sp_float trace() const
		{
			return m11 + m22 + m33 + m44;
		}

		/// <summary>
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE inline void transpose(Mat4& output) const
		{
			//copy principal diagonal
			output.m11 = m11;
			output.m22 = m22;
			output.m33 = m33;
			output.m44 = m44;

			//swap others numbers
			output.m21 = m12;
			output.m12 = m21;

			output.m31 = m13;
			output.m13 = m31;

			output.m41 = m14;
			output.m14 = m41;

			output.m23 = m32;
			output.m32 = m23;

			output.m24 = m42;
			output.m42 = m24;

			output.m34 = m43;
			output.m43 = m34;
		}

		/// <summary>
		/// Get the determinant from index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE inline sp_float determinantIJ(const sp_uint i, const sp_uint j) const
		{
			sp_uint x, y, ii, jj;
			sp_float ret, mat[3][3];

			sp_float* matrixAsArray = values();

			x = 0u;
			for (ii = 0u; ii < 4u; ii++)
			{
				if (ii == i)
					continue;

				y = 0;

				for (jj = 0u; jj < 4u; jj++)
				{
					if (jj == j)
						continue;

					mat[x][y] = matrixAsArray[(ii * 4u) + jj];
					y++;
				}

				x++;
			}

			ret = mat[0][0] * (mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2]);
			ret -= mat[0][1] * (mat[1][0] * mat[2][2] - mat[2][0] * mat[1][2]);
			ret += mat[0][2] * (mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1]);

			return ret;
		}

		/// <summary>
		/// Get the cofactor of the index i,j
		/// Zero-Index based
		/// </summary>
		API_INTERFACE inline sp_float cofactorIJ(const sp_uint i, const  sp_uint j) const
		{
			sp_float determinantIJValue = determinantIJ(i, j);

			if (isOdd(i + j))
				determinantIJValue *= -1;

			return determinantIJValue;
		}

		/// <summary>
		/// Get the determinant from matrix
		/// </summary>
		API_INTERFACE inline sp_float determinant() const
		{
			sp_float det = ZERO_FLOAT;
			sp_float* matrixAsArray = values();

			for (sp_int i = 0; i < MAT4_ROW_LENGTH; i++)
			{
				det += (i & 0x1) ?
					(-matrixAsArray[i] * determinantIJ(0, i))
					: (matrixAsArray[i] * determinantIJ(0, i));
			}

			return det;
		}

		/// <summary>
		/// Multiply this matrix with the parametrized matrix => AxB
		/// </summary>
		/// <param name="matrixB">B Matrix</param>
		/// <param name="output">Multiplied Matrix</param>
		/// <returns>output parameter</returns>
		API_INTERFACE inline void multiply(const Mat4& matrixB, Mat4& output) const
		{
			sp_float* matrixAsArray = values();

#if MAJOR_COLUMN_ORDER
			for (sp_int line = 0; line < MAT4_ROW_LENGTH; line++)
			{
				const sp_float ai0 = matrixAsArray[line];
				const sp_float ai1 = matrixAsArray[4 + line];
				const sp_float ai2 = matrixAsArray[8 + line];
				const sp_float ai3 = matrixAsArray[12 + line];

				output[line] = ai0 * matrixB.m11 + ai1 * matrixB.m12 + ai2 * matrixB.m13 + ai3 * matrixB.m14;
				output[4 + line] = ai0 * matrixB.m21 + ai1 * matrixB.m22 + ai2 * matrixB.m23 + ai3 * matrixB.m24;
				output[8 + line] = ai0 * matrixB.m31 + ai1 * matrixB.m32 + ai2 * matrixB.m33 + ai3 * matrixB.m34;
				output[12 + line] = ai0 * matrixB.m41 + ai1 * matrixB.m42 + ai2 * matrixB.m43 + ai3 * matrixB.m44;
			}
#else 
			for (sp_int column = 0; column < MAT4_ROW_LENGTH; column++)
			{
				sp_float ai0 = values[(column * MAT4_ROW_LENGTH) + 0];
				sp_float ai1 = values[(column * MAT4_ROW_LENGTH) + 1];
				sp_float ai2 = values[(column * MAT4_ROW_LENGTH) + 2];
				sp_float ai3 = values[(column * MAT4_ROW_LENGTH) + 3];

				output[(column * MAT4_ROW_LENGTH) + 0] = ai0 * matrixB.m11 + ai1 * matrixB.m21 + ai2 * matrixB.m31 + ai3 * matrixB.m14;
				output[(column * MAT4_ROW_LENGTH) + 1] = ai0 * matrixB.m12 + ai1 * matrixB.m22 + ai2 * matrixB.m32 + ai3 * matrixB.m24;
				output[(column * MAT4_ROW_LENGTH) + 2] = ai0 * matrixB.m13 + ai1 * matrixB.m23 + ai2 * matrixB.m33 + ai3 * matrixB.m34;
				output[(column * MAT4_ROW_LENGTH) + 3] = ai0 * matrixB.m14 + ai1 * matrixB.m24 + ai2 * matrixB.m34 + ai3 * matrixB.m44;
			}
#endif
		}

		/// <summary>
		/// Multiply this matrix with the parametrized vector => AxB
		/// </summary>
		API_INTERFACE void multiply(const Vec4 &vector, Vec4& output) const;

		/// <summary>
		/// Get the inverse matrix from current matrix => A^-1
		/// <summary>
		API_INTERFACE inline void invert(Mat4& output) const
		{
			sp_int i, j;
			sp_float det = ZERO_FLOAT;
			sp_float detij;
			sp_float* matrixAsArray = values();

			// calculate 4x4 determinant
			for (i = 0; i < 4; i++)
			{
				det += (i & 0x1) ? (-matrixAsArray[i] * determinantIJ(0, i)) : (matrixAsArray[i] * determinantIJ(0, i));
			}
			det = ONE_FLOAT / det;

			// calculate inverse
			for (i = 0; i < 4; i++)
			{
				for (j = 0; j < 4; j++)
				{
					detij = determinantIJ(j, i);
					output[(i * 4) + j] = ((i + j) & 0x1) ? (-detij * det) : (detij * det);
				}
			}
		}

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE inline void scale(const sp_float xScale, const sp_float yScale, const sp_float zScale)
		{
			m11 *= xScale;
			m22 *= yScale;
			m33 *= zScale;
		}

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE void operator/=(const sp_float value);

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE inline sp_bool operator==(const Mat4& matrix) const
		{
			const sp_float* matrix1AsArray = values();
			const sp_float* matrix2AsArray = matrix.values();

			for (sp_int i = 0; i < MAT4_LENGTH; i++)
				if (matrix1AsArray[i] != matrix2AsArray[i])
					return false;

			return true;
		}

		/// <summary>
		/// Compare this matrix to another one. Compare each component.
		/// </summary>
		API_INTERFACE inline sp_bool operator==(const sp_float value) const
		{
			const sp_float* matrix1AsArray = values();

			for (sp_int i = 0; i < MAT4_LENGTH; i++)
				if (matrix1AsArray[i] != value)
					return false;

			return true;
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE inline sp_bool operator!=(const Mat4& matrix) const
		{
			const sp_float* matrix1AsArray = values();
			const sp_float* matrix2AsArray = matrix.values();

			for (sp_int i = 0; i < MAT4_LENGTH; i++)
				if (matrix1AsArray[i] != matrix2AsArray[i])
					return true;

			return false;
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_int index)
		{
			sp_assert(index >= 0 && index < MAT4_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < MAT4_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_uint index)
		{
			sp_assert(index >= 0u && index < MAT4_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_uint index) const
		{
			sp_assert(index >= 0u && index < MAT4_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE inline operator void*() const
		{
			return (void*)this;
		}

		/// <summary>
		/// Auto convertion to const sp_float*
		/// </summary>
		API_INTERFACE operator sp_float*() const
		{
			return values();
		}

		/// <summary>
		/// Convert the matrix to Matrix 3x3
		/// Returns the first 3 components of x-Axis, y-Axis, z-Axis
		/// </summary>
		API_INTERFACE void toMat3(Mat3& output) const;

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

		/// <summary>
		/// Find the characteristic polyname of this matrix using Leverrier Method
		/// </summary>
		/// <param name="output"></param>
		/// <returns>5 floats</returns>
		API_INTERFACE void polyname(sp_float* output) const;

		/// <summary>
		/// Convert this matrix to Tridiagonal matrix using Householder Method
		/// </summary>
		/// <param name="output">Matrix</param>
		/// <returns>output</returns>
		API_INTERFACE void tridiagonal(Mat4* output) const;

		/*
		/// <summary>
		/// Get the normal matrix
		/// </summary>
		API_INTERFACE Mat3 toNormalMatrix();
		*/
	};

	const Mat4 Mat4Identity(
		ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
	);

	const Mat4 Mat4Zeros(
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT
	);

	const Mat4 Mat4Ones(
		ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
		ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
		ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
		ONE_FLOAT, ONE_FLOAT, ONE_FLOAT, ONE_FLOAT
	);

	/// <summary>
	/// Create a scaled matrix
	/// </summary>
	/// <param name="xScale">X scale factor</param>
	/// <param name="yScale">Y scale factor</param>
	/// <param name="zScale">Z scale factor</param>
	/// <param name="output">Scaled matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void createScale(const sp_float xScale, const sp_float yScale, const sp_float zScale, Mat4& output)
	{
		std::memcpy(output, Mat4Identity, sizeof(Mat4));
		output.m11 = xScale;
		output.m22 = yScale;
		output.m33 = zScale;
	}

	/// <summary>
	/// Create a scaled matrix
	/// </summary>
	/// <param name="scale">Scale factor</param>
	/// <param name="output">Output Matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE void createScale(const Vec3& scale, Mat4& output);

	/// <summary>
	/// Create a translation matrix
	/// </summary>
	/// <param name="x">X translation</param>
	/// <param name="y">Y translation</param>
	/// <param name="z">Z translation</param>
	/// <param name="output">Translation Matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void createTranslate(const sp_float x, const sp_float y, const sp_float z, Mat4& output)
	{
		std::memcpy(output, Mat4Identity, sizeof(Mat4));
#if MAJOR_COLUMN_ORDER
		output.m41 = x;
		output.m42 = y;
		output.m43 = z;
#else
		output.m14 = x;
		output.m24 = y;
		output.m34 = z;
#endif
	}

	/// <summary>
	/// Create a translation matrix
	/// </summary>
	/// <param name="position">Translation Vector</param>
	/// <param name="output">Translation Matrix</param>
	/// <returns>output parameter</returns>	
	API_INTERFACE void createTranslate(const Vec3& position, Mat4& output);

	/// <summary>
	/// Create a rotation matrix
	/// Example: x = 1.0 to rotate over X axis; x = 0.0 to not rotate over X axis
	/// </summary>
	API_INTERFACE inline void createRotate(const sp_float angleRadians, const sp_float x, const sp_float y, const sp_float z, Mat4& output)
	{
		const sp_float sineAngle = sinf(angleRadians);
		const sp_float cosineAngle = cosf(angleRadians);

		const sp_float mag = sqrtf(x * x + y * y + z * z);

		if (NAMESPACE_FOUNDATION::isCloseEnough(mag, ZERO_FLOAT))
		{
			std::memcpy(output, Mat4Identity, sizeof(Mat4));
			return;
		}

		// Rotation matrix is normalized
		const sp_float x1 = x / mag;
		const sp_float y1 = y / mag;
		const sp_float z1 = z / mag;

		const sp_float xx = x * x;
		const sp_float yy = y * y;
		const sp_float zz = z * z;
		const sp_float xy = x * y;
		const sp_float yz = y * z;
		const sp_float zx = z * x;
		const sp_float xs = x * sineAngle;
		const sp_float ys = y * sineAngle;
		const sp_float zs = z * sineAngle;
		const sp_float one_c = 1.0f - cosineAngle;

#define M(row,col)  output[col * MAT4_ROW_LENGTH + row]
		M(0, 0) = (one_c * xx) + cosineAngle;
		M(0, 1) = (one_c * xy) - zs;
		M(0, 2) = (one_c * zx) + ys;
		M(0, 3) = ZERO_FLOAT;

		M(1, 0) = (one_c * xy) + zs;
		M(1, 1) = (one_c * yy) + cosineAngle;
		M(1, 2) = (one_c * yz) - xs;
		M(1, 3) = ZERO_FLOAT;

		M(2, 0) = (one_c * zx) - ys;
		M(2, 1) = (one_c * yz) + xs;
		M(2, 2) = (one_c * zz) + cosineAngle;
		M(2, 3) = ZERO_FLOAT;

		M(3, 0) = ZERO_FLOAT;
		M(3, 1) = ZERO_FLOAT;
		M(3, 2) = ZERO_FLOAT;
		M(3, 3) = ONE_FLOAT;
#undef M
	}

	/// <summary>
	/// Craete a orthographic matrix projection
	/// </summary>
	/// <param name="xMin"></param>
	/// <param name="xMax"></param>
	/// <param name="yMin"></param>
	/// <param name="yMax"></param>
	/// <param name="zMin"></param>
	/// <param name="zMax"></param>
	/// <param name="output">Orthographic Matrix</param>
	/// <returns>output paramter</returns>
	API_INTERFACE inline void createOrthographicMatrix(const sp_float xMin, const sp_float xMax, const sp_float yMin, const sp_float yMax, const sp_float zMin, const sp_float zMax, Mat4& output)
	{
		std::memcpy(output, Mat4Identity, sizeof(Mat4));

		output.m11 = TWO_FLOAT / (xMax - xMin);
		output.m22 = TWO_FLOAT / (yMax - yMin);
		output.m33 = -TWO_FLOAT / (zMax - zMin);
		output.m41 = -((xMax + xMin) / (xMax - xMin));
		output.m42 = -((yMax + yMin) / (yMax - yMin));
		output.m43 = -((zMax + zMin) / (zMax - zMin));
	}

	/// <summary>
	/// Add two matrix
	/// </summary>
	/// <param name="a">Matrix A</param>
	/// <param name="b">Matrix B</param>
	/// <param name="output">Added Matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void add(const Mat4& a, const Mat4& b, Mat4& output)
	{
		output.m11 = a.m11 + b.m11;
		output.m12 = a.m12 + b.m12;
		output.m13 = a.m13 + b.m13;
		output.m14 = a.m14 + b.m14;

		output.m21 = a.m21 + b.m21;
		output.m22 = a.m22 + b.m22;
		output.m23 = a.m23 + b.m23;
		output.m24 = a.m24 + b.m24;

		output.m31 = a.m31 + b.m31;
		output.m32 = a.m32 + b.m32;
		output.m33 = a.m33 + b.m33;
		output.m34 = a.m34 + b.m34;

		output.m41 = a.m41 + b.m41;
		output.m42 = a.m42 + b.m42;
		output.m43 = a.m43 + b.m43;
		output.m44 = a.m44 + b.m44;
	}

	/// <summary>
	/// Subtract two matrix
	/// </summary>
	/// <param name="a">Matrix A</param>
	/// <param name="b">Matrix B</param>
	/// <param name="output">Subtracted Matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void diff(const Mat4& a, const Mat4& b, Mat4& output)
	{
		output.m11 = a.m11 - b.m11;
		output.m12 = a.m12 - b.m12;
		output.m13 = a.m13 - b.m13;
		output.m14 = a.m14 - b.m14;

		output.m21 = a.m21 - b.m21;
		output.m22 = a.m22 - b.m22;
		output.m23 = a.m23 - b.m23;
		output.m24 = a.m24 - b.m24;

		output.m31 = a.m31 - b.m31;
		output.m32 = a.m32 - b.m32;
		output.m33 = a.m33 - b.m33;
		output.m34 = a.m34 - b.m34;

		output.m41 = a.m41 - b.m41;
		output.m42 = a.m42 - b.m42;
		output.m43 = a.m43 - b.m43;
		output.m44 = a.m44 - b.m44;
	}

	/// <summary>
	/// Multiply the matrix to a scalar
	/// </summary>
	/// <param name="input">Input matrix</param>
	/// <param name="value">Scalar value</param>
	/// <param name="output">Multiplied matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void multiply(const Mat4& input, const sp_float value, Mat4& output)
	{
		output.m11 = input.m11 * value;
		output.m12 = input.m12 * value;
		output.m13 = input.m13 * value;
		output.m14 = input.m14 * value;

		output.m21 = input.m21 * value;
		output.m22 = input.m22 * value;
		output.m23 = input.m23 * value;
		output.m24 = input.m24 * value;

		output.m31 = input.m31 * value;
		output.m32 = input.m32 * value;
		output.m33 = input.m33 * value;
		output.m34 = input.m34 * value;

		output.m41 = input.m41 * value;
		output.m42 = input.m42 * value;
		output.m43 = input.m43 * value;
		output.m44 = input.m44 * value;
	}

	/// <summary>
	/// Divide the matrix to a scalar
	/// </summary>
	/// <param name="input">Input matrix</param>
	/// <param name="value">Scalar value</param>
	/// <param name="output">Divided matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void divide(const Mat4& input, const sp_float value, Mat4& output)
	{
		multiply(input, ONE_FLOAT / value, output);
	}

}

#endif // !MAT4_HEADER
