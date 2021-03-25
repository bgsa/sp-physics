#ifndef MAT_HEADER
#define MAT_HEADER

#include "SpectrumPhysics.h"
#include <iomanip>
#include <eigen/Dense>

namespace NAMESPACE_PHYSICS
{
	class Mat
	{
	private:
		sp_uint _rows, _columns;
		sp_float* _values;

	public:

		/// <summary>
		/// Default constructor for generic matrix
		/// </summary>
		/// <param name="rows">Row Length</param>
		/// <param name="columns">Column Length</param>
		/// <returns>void</returns>
		API_INTERFACE inline Mat(const sp_uint rows, const sp_uint columns)
		{
			_rows = rows;
			_columns = columns;
			_values = sp_mem_new(sp_float)[rows * columns];
		}

		/// <summary>
		/// Constructor for generic matrix with values
		/// </summary>
		/// <param name="rows">Row Length</param>
		/// <param name="columns">Column Length</param>
		/// <param name="values">This parameter is kept by the matrix</param>
		/// <returns>void</returns>
		API_INTERFACE inline Mat(const sp_uint rows, const sp_uint columns, sp_float* values)
		{
			_rows = rows;
			_columns = columns;
			_values = sp_mem_new_array(sp_float, rows * columns);

			std::memcpy(_values, values, sizeof(sp_float) * rows * columns);
		}

		/// <summary>
		/// Get the Row Length
		/// </summary>
		/// <returns>Row Length</returns>
		API_INTERFACE inline sp_uint rows() const
		{
			return _rows;
		}

		/// <summary>
		/// Get the Column Length
		/// </summary>
		/// <returns>Column Length</returns>
		API_INTERFACE inline sp_uint columns() const
		{
			return _columns;
		}

		/// <summary>
		/// Get the Element Length
		/// </summary>
		/// <returns>Element Length</returns>
		API_INTERFACE inline sp_uint length() const
		{
			return _rows * _columns;
		}

		/// <summary>
		/// Get an element (x,y) from matrix
		/// </summary>
		/// <param name="row">Row Index</param>
		/// <param name="column">Column Index</param>
		/// <returns>Element (x,y)</returns>
		API_INTERFACE inline sp_float get(const sp_uint row, const sp_uint column) const
		{
			return _values[row * _columns + column];
		}

		/// <summary>
		/// Set an element (x,y) from matrix
		/// </summary>
		/// <param name="row">Row Index</param>
		/// <param name="column">Column Index</param>
		/// <param name="value">new Value</param>
		/// <returns>void</returns>
		API_INTERFACE inline void set(const sp_uint row, const sp_uint column, const sp_float value)
		{
			_values[row * _columns + column] = value;
		}

		/// <summary>
		/// Auto convertion to float*
		/// </summary>
		API_INTERFACE operator sp_float*() const
		{
			return _values;
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_int index)
		{
			sp_assert(index >= 0 && index < (sp_int) length(), "IndexOutOfrangeException");
			return _values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < (sp_int) length(), "IndexOutOfrangeException");
			return _values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_uint index)
		{
			sp_assert(index >= 0u && index < length(), "IndexOutOfrangeException");
			return _values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_uint index) const
		{
			sp_assert(index >= 0u && index < length(), "IndexOutOfrangeException");
			return _values[index];
		}

		/// <summary>
		/// Release all allocated resources
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose()
		{
			if (_values != nullptr)
			{
				sp_mem_release(_values);
				_values = nullptr;
			}
		}

		~Mat()
		{
			dispose();
		}

		/// <summary>
		/// Check the matrix is Hessenberg Upper format
		/// All values under subdiagonal are Zero
		/// </summary>
		/// <param name="matrix">Input Square Matrix</param>
		/// <param name="columnLength">Dimension</param>
		/// <returns>True if the format is Hessenberg Upper</returns>
		static inline sp_bool isHessenbergUpper(sp_float* matrix, const sp_uint columnLength, const sp_float _epsilon = DefaultErrorMargin)
		{
			for (sp_uint column = ZERO_UINT; column < columnLength; column++)
				for (sp_uint row = column + TWO_UINT; row < columnLength; row++)
					if (!NAMESPACE_FOUNDATION::isCloseEnough(matrix[row * columnLength + column], ZERO_FLOAT, _epsilon))
						return false;

			return true;
		}

		/// <summary>
		/// Check the square matrix is symetric
		/// </summary>
		/// <param name="matrix">Matrix</param>
		/// <param name="columnLength">Dimension</param>
		/// <returns>True if the matrix is symetric orelse False</returns>
		static inline sp_bool isSymetric(sp_float* matrix, const sp_uint columnLength)
		{
			for (sp_uint row = ZERO_UINT; row < columnLength; row++)
				for (sp_uint column = row + ONE_UINT; column < columnLength; column++)
					if (NAMESPACE_FOUNDATION::isCloseEnough(matrix[row * columnLength + column], matrix[column * columnLength + row], ZERO_FLOAT))
						return false;
			
			return true;
		}

		/// <summary>
		/// Reduce a matrix to a Hessenberg Upper
		/// If the matrix is symetrix, it will reduce to a tridiagonal matrix
		/// </summary>
		/// <param name="matrix">Input Square Matrix</param>
		/// <param name="columnLength">Dimension</param>
		/// <param name="output">Hessenberg Matrix</param>
		static void hessenberg(sp_float* matrix, register const sp_uint columnLength, sp_float* output);

		/// <summary>
		/// Householder numerical method - Reduce a matrix from symetric to tridiagonal
		/// </summary>
		/// <param name="matrix">Input Square Matrix - This matrix has to be square</param>
		/// <param name="columnLength">Column Length</param>
		/// <param name="output">Tridiagonal Matrix</param>
		static void householder(sp_float* matrix, const register sp_uint columnLength, sp_float* output);

		static std::string toString(const sp_float* values, const sp_uint rowLength, const sp_int precision = 4)
		{
			std::string content;
			const sp_uint length = rowLength * rowLength;

			for (sp_uint i = ZERO_UINT; i < length; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << values[i];
				std::string numberAsString = stream.str();

				if (values[i] >= ZERO_FLOAT)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % rowLength == ZERO_UINT)
					content += END_OF_LINE;
			}

			return content;
		}

		static std::string toString(const sp_float* values, const sp_uint rowLength, const sp_uint columnLength, const sp_int precision = 4)
		{
			std::string content;
			const sp_uint length = rowLength * columnLength;

			for (sp_uint i = ZERO_UINT; i < length; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << values[i];
				std::string numberAsString = stream.str();

				if (values[i] >= ZERO_FLOAT)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % columnLength == ZERO_UINT)
					content += END_OF_LINE;
			}

			return content;
		}

		static void gaussianElimination(sp_float *matrix, const sp_int rowSize);

		/// <summary>
		/// Get the Homography Matrix (the transformation perspective matrix from a plane to another one)
		/// <summary>
		static Mat3 getPerspectiveTransform2D(const Vec2 sourcePoints[4], const Vec2 targetPoints[4] );

	};

	/// <summary>
	/// Check the matrix A is closes to matrix B, given an epsilon
	/// </summary>
	/// <param name="a">Input Matrix A</param>
	/// <param name="b">Input Matrix B</param>
	/// <param name="_epsilon">Error Margin</param>
	/// <returns>True if the matrixes are close</returns>
	API_INTERFACE inline sp_bool isCloseEnough(const Mat& a, const Mat& b, const sp_float _epsilon = SP_DEFAULT_ERROR_MARGIN)
	{
		sp_assert(a.length() == b.length(), "InvalidOperationException");

		for (register sp_uint i = 0u; i < a.length(); i++)
			if (!NAMESPACE_FOUNDATION::isCloseEnough(a[i], b[i], _epsilon))
				return false;

		return true;
	}

	/// <summary>
	/// Transpose A matrix
	/// </summary>
	/// <param name="a">Input Matrix</param>
	/// <param name="output">Output Matrix</param>
	/// <return>output parameter</return>
	API_INTERFACE inline void transpose(const Mat& a, Mat& output)
	{
		for (register sp_uint row = 0u; row < a.rows(); row++)
		{
			const register sp_uint rowIndex = row * a.rows();

			for (register sp_uint column = row; column < a.columns(); column++)
			{
				output[rowIndex + column] = a[column * a.rows() + row];
				output[column * a.rows() + row] = a[rowIndex + column];
			}
		}
	}

	/// <summary>
	/// Multiply A matrix by B matrix and put the result in output matrix parameter
	/// </summary>
	/// <param name="a">Input Matrix A</param>
	/// <param name="b">Input Matrix B</param>
	/// <param name="output">Output Matrix</param>
	/// <return>output parameter</return>
	API_INTERFACE inline void multiply(const Mat& a, const Mat& b, Mat& output);
	
}

#endif // !MAT_HEADER