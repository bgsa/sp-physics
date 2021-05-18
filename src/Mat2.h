#ifndef MAT2_HEADER
#define MAT2_HEADER

#include "SpectrumPhysics.h"
#include "AutoValueAutoVector.h"

namespace NAMESPACE_PHYSICS
{
#define MAT2_LENGTH     (4)
#define MAT2_ROW_LENGTH (2)

	class Mat2
	{
	public:
		sp_float m11, m12,
				 m21, m22;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline Mat2() { }

		/// <summary>
		/// Default constructor with constant
		/// </summary>
		/// <param name="constantValue">Default value for all elements</param>
		/// <returns>void</returns>
		API_INTERFACE inline Mat2(const sp_float constantValue)
		{
			m11 = m12 = m21 = m22 = constantValue;
		}

		/// <summary>
		/// Constructor with initialized values - COL ORDER
		/// </summary>
		API_INTERFACE inline Mat2(const sp_float value11, const sp_float value12, const sp_float value21, const sp_float value22)
		{
			m11 = value11;
			m12 = value12;
			m21 = value21;
			m22 = value22;
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
		/// Zero index base
		/// </summary>
		/// <param name="row">Row Index</param>
		/// <param name="column">Column Index</param>
		/// <returns>Value</returns>
		API_INTERFACE inline sp_float get(const sp_int row, const sp_int column)
		{
			return values()[row * MAT2_ROW_LENGTH + column];
		}

		/// <summary>
		/// Get the X axis
		/// </summary>
		API_INTERFACE Vec2 xAxis() const;

		/// <summary>
		/// Get the Y axis
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
		/// Transpose matrix - swap rows by columns
		/// </summary>
		API_INTERFACE inline void transpose(Mat2& output) const
		{
			output.m11 = m11;
			output.m22 = m22;
			output.m12 = m21;
			output.m21 = m12;
		}

		/// <summary>
		/// Multiply this matrix with the parametrized matrix => AxB
		/// </summary>
		API_INTERFACE inline void multiply(const Mat2& matrixB, Mat2& output) const
		{
#ifdef MAJOR_COLUMN_ORDER
			output.m11 = m11 * matrixB.m11 + m21 * matrixB.m12;
			output.m21 = m11 * matrixB.m21 + m21 * matrixB.m22;

			output.m12 = m12 * matrixB.m11 + m22 * matrixB.m12;
			output.m22 = m12 * matrixB.m21 + m22 * matrixB.m22;
#else
			output.m11 = m11 * matrixB.m11 + m12 * matrixB.m21;
			output.m21 = m11 * matrixB.m12 + m12 * matrixB.m22;

			output.m12 = m21 * matrixB.m11 + m22 * matrixB.m21;
			output.m22 = m21 * matrixB.m12 + m22 * matrixB.m22;
#endif
		}

		/// <summary>
		/// Multiply this matrix with the parametrized vector => AxB
		/// </summary>
		API_INTERFACE void multiply(const Vec2& vector, Vec2& output) const;

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE void scale(const sp_float xScale, const sp_float yScale)
		{
			m11 *= xScale;
			m22 *= yScale;
		}

		/// <summary>
		/// Get the deeterminant of the Matrix
		/// </summary>
		API_INTERFACE inline sp_float determinant() const
		{
			return m11 * m22 - m12 * m21;
		}

		/// <summary>
		/// Get the autovalue of the matrix
		/// </summary>
		API_INTERFACE inline AutoValueAutoVector2 getAutovalueAndAutovector(const sp_ushort maxIteration = 5) const;

		/// <summary>
		/// Clone this matrix
		/// </summary>
		API_INTERFACE inline void clone(Mat2& output) const
		{
			std::memcpy(output, this, sizeof(Mat2));
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](sp_int index)
		{
			sp_assert(index >= 0 && index < MAT2_LENGTH, "IndexOutOfrangeException");
			return values()[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_int index) const
		{
			sp_assert(index >= 0 && index < MAT2_LENGTH, "IndexOutOfrangeException");
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
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE inline operator sp_float*() const
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Get the negative matrix
		/// </summary>
		API_INTERFACE inline Mat2 operator-() const
		{
			return Mat2{
				-m11,
				-m12,
				-m21,
				-m22
			};

		}

		/// <summary>
		/// Subtract a matrix to another one
		/// </summary>
		API_INTERFACE inline Mat2 operator-(const Mat2& matrix) const
		{
			return Mat2{
				m11 - matrix.m11,
				m12 - matrix.m12,
				m21 - matrix.m21,
				m22 - matrix.m22
			};
		}

		/// <summary>
		/// Sum a matrix to another one
		/// </summary>
		API_INTERFACE inline Mat2 operator+(const Mat2& matrix) const
		{
			return Mat2{
				m11 + matrix.m11,
				m12 + matrix.m12,
				m21 + matrix.m21,
				m22 + matrix.m22
			};
		}

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE inline Mat2 operator/(sp_float value) const
		{
			const sp_float inverted = ONE_FLOAT / value;
			
			return Mat2{
				m11 * inverted,
				m12 * inverted,
				m21 * inverted,
				m22 * inverted
			};
		}

		/// <summary>
		/// Divide the matrix by a scalar value
		/// </summary>
		API_INTERFACE void operator/=(sp_float value)
		{
			const sp_float inverted = ONE_FLOAT / value;

			m11 *= inverted;
			m12 *= inverted;
			m21 *= inverted;
			m22 *= inverted;
		}

	};

	const Mat2 Mat2Zeros(
		ZERO_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ZERO_FLOAT
	);

	const Mat2 Mat2Ones(
		ONE_FLOAT, ONE_FLOAT,
		ONE_FLOAT, ONE_FLOAT
	);

	const Mat2 Mat2Identity(
		ONE_FLOAT, ZERO_FLOAT,
		ZERO_FLOAT, ONE_FLOAT
	);

	/// <summary>
	/// Craete a translation matrix
	/// </summary>
	API_INTERFACE inline void createTranslate(const sp_float x, const sp_float y, Mat2& output)
	{
		output.m11 = ONE_FLOAT;
		output.m12 = ZERO_FLOAT;
		output.m21 = x;
		output.m22 = y;
	}

	/// <summary>
	/// Create a scaled matrix
	/// </summary>
	API_INTERFACE inline void createScale(const sp_float xScale, const sp_float yScale, Mat2& output)
	{
		output.m12 = output.m21 = ZERO_FLOAT;
		output.m11 = xScale;
		output.m22 = yScale;
	}

}

#endif // !MAT2_HEADER
