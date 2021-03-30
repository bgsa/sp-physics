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
			sp_assert(rows != ZERO_UINT, "InvalidArgumentException");
			sp_assert(columns != ZERO_UINT, "InvalidArgumentException");

			_rows = rows;
			_columns = columns;
			_values = sp_mem_new_array(sp_float, rows * columns);
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
			sp_assert(rows != ZERO_UINT, "InvalidArgumentException");
			sp_assert(columns != ZERO_UINT, "InvalidArgumentException");

			_rows = rows;
			_columns = columns;
			_values = sp_mem_new_array(sp_float, rows * columns);

			std::memcpy(_values, values, sizeof(sp_float) * rows * columns);
		}

		/// <summary>
		/// Constructor for generic matrix
		/// </summary>
		/// <param name="source">Matrix to be cloned</param>
		/// <param name="columns">Column Length</param>
		/// <returns>void</returns>
		API_INTERFACE inline Mat(const Mat* source)
		{
			sp_assert(source != nullptr, "InvalidArgumentException");

			_rows = source->_rows;
			_columns = source->_columns;

			_values = sp_mem_new_array(sp_float, _rows * _columns);

			std::memcpy(_values, source->_values, sizeof(sp_float) * _rows * _columns);
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
		/// Fill all matrix with the constant value
		/// </summary>
		/// <param name="value">Constant Value</param>
		/// <returns>void</returns>
		API_INTERFACE inline void fill(const sp_int value)
		{
			std::memset(_values, value, sizeof(sp_float) * length());
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
		/// Set this matrix to identity
		/// </summary>
		/// <returns>void</returns>
		API_INTERFACE inline void setIdentity()
		{
			std::memset(_values, 0, sizeof(sp_float) * length());

			const sp_uint minOrder = std::min(_rows, _columns);

			for (sp_uint i = 0; i < minOrder; i++)
				set(i, i, ONE_FLOAT);
		}

		/// <summary>
		/// Check if the matrix is square
		/// </summary>
		/// <returns>True if the matrix is square orelse False</returns>
		API_INTERFACE inline sp_bool isSquare() const
		{
			return _rows == _columns;
		}

		/// <summary>
		/// Get the primary diagonal of the matrix
		/// </summary>
		/// <returns>output parameter</returns>
		API_INTERFACE inline void primaryDiagonal(sp_float* output) const
		{
			const sp_uint m = std::min(_rows, _columns);

			for (sp_uint i = 0; i < m; i++)
				output[i] = get(i, i);
		}

		/// <summary>
		/// Transpose A matrix
		/// </summary>
		/// <param name="output">Output Matrix</param>
		/// <return>output parameter</return>
		API_INTERFACE inline void transpose(Mat& output) const;

		/// <summary>
		/// Get the maximum element off diagonal of Symmetric matrix
		/// </summary>
		/// <param name="rowIndex">Row Index of the maximum element</param>
		/// <param name="columnIndex">Column Index of the maximum element</param>
		/// <param name="value">Maximum element value</param>
		/// <returns>output parameters</returns>
		API_INTERFACE inline void maxOffDiagonalSymmetric(sp_uint& rowIndex, sp_uint& columnIndex, sp_float& value) const
		{
			sp_assert(isSquare(), "InvalidOperationException");
			sp_assert(isSymmetric(), "InvalidOperationException");

			value = SP_FLOAT_MIN;

			for (register sp_uint row = ONE_UINT; row < _rows; row++)
			{
				const register sp_uint r = row * _rows;

				for (register sp_uint column = row; column < _columns; column++)
					if (_values[r + column] > value)
					{
						value = _values[r + column];
						rowIndex = row;
						columnIndex = column;
					}
			}
		}

		/// <summary>
		/// Get the maximum element off diagonal
		/// </summary>
		/// <param name="rowIndex">Row Index of the maximum element</param>
		/// <param name="columnIndex">Column Index of the maximum element</param>
		/// <param name="value">Maximum element value</param>
		/// <returns>output parameters</returns>
		API_INTERFACE inline void maxOffDiagonal(sp_uint& rowIndex, sp_uint& columnIndex, sp_float& value) const
		{
			sp_assert(isSquare(), "InvalidOperationException");

			value = SP_FLOAT_MIN;

			for (register sp_uint row = ZERO_UINT; row < _rows; row++)
			{
				const register sp_uint r = row * _rows;

				for (register sp_uint column = ZERO_UINT; column < _columns; column++)
				{
					if (row == column) // ignore principal diagonal
						continue;

					if (_values[r + column] > value)
					{
						value = _values[r + column];
						rowIndex = row;
						columnIndex = column;
					}
				}
			}
		}

		/// <summary>
		/// Check if the matrix is symmetric
		/// </summary>
		/// <returns>True if the matrix is symmetric orelse False</returns>
		API_INTERFACE inline sp_bool isSymmetric() const
		{
			if (!isSquare())
				return false;

			for (register sp_uint row = 0u; row < _rows; row++)
			{
				const register sp_uint rowIndex = row * _rows;

				for (register sp_uint column = row; column < _columns; column++)
					if (_values[column * _rows + row] != _values[rowIndex + column])
						return false;
			}

			return true;
		}

		/// <summary>
		/// Swap/exchange two lines in the matrix
		/// </summary>
		/// <param name="line1Index">Line 1 Index</param>
		/// <param name="line2Index">Line 2 Index</param>
		/// <returns>void</returns>
		API_INTERFACE inline void swapLines(const sp_uint line1Index, const sp_uint line2Index) const
		{
			const sp_size rowSize = sizeof(sp_float) * _columns;

			sp_float* tempStorage = ALLOC_NEW_ARRAY(sp_float, _columns);

			std::memcpy(tempStorage, &_values[line1Index * _columns], rowSize);
			std::memcpy(&_values[line1Index * _columns], &_values[line2Index * _columns], rowSize);
			std::memcpy(&_values[line2Index * _columns], tempStorage, rowSize);

			ALLOC_RELEASE(tempStorage);
		}

		/// <summary>
		/// Swap/exchange two columns in the matrix
		/// </summary>
		/// <param name="column1Index">Column 1 Index</param>
		/// <param name="column2Index">Column 2 Index</param>
		/// <returns>void</returns>
		API_INTERFACE inline void swapColumns(const sp_uint column1Index, const sp_uint column2Index) const
		{
			for (sp_uint row = 0u; row < _rows; row++)
			{
				const sp_uint rowIndex = row * _columns;

				const sp_float temp = _values[rowIndex + column1Index];
				_values[rowIndex + column1Index] = _values[rowIndex + column2Index];
				_values[rowIndex + column2Index] = temp;
			}
		}

		/// <summary>
		/// Get the eigen values and vectors from this matrix
		/// </summary>
		/// <param name="eigenValues">Eigen values output parameter</param>
		/// <param name="eigenVectors">Eigen vectors output parameter</param>
		/// <param name="iterations">Iterations count to converge</param>
		/// <param name="maxIterations">Limit iterations, just in case not converge</param>
		/// <param name="_epsilon">Error margin for convergence</param>
		/// <returns>eigenValues and eigenVectors parameters</returns>
		API_INTERFACE sp_bool eigenValuesAndVectors(sp_float* eigenValues, Mat& eigenVectors, 
			sp_uint& iterations, const sp_uint maxIterations = SP_UINT_MAX,
			const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;

		/// <summary>
		/// Decompose this matrix in single values (SVD) using Jacobi method
		/// </summary>
		/// <param name="u">U Matrix (row x row)</param>
		/// <param name="s">S Matrix (column x column)</param>
		/// <param name="v">V Matrix (column x column)</param>
		/// <param name="iterations">Number of iterations to converge</param>
		/// <param name="maxIterations">Maximum of iterations to converge</param>
		/// <param name="_epsilon">Error margin</param>
		/// <returns>True if the method converged orelse False</returns>
		API_INTERFACE sp_bool svd(Mat& u, Mat& s, Mat& v, sp_uint& iterations, 
			const sp_uint maxIterations = SP_UINT_MAX, const sp_float _epsilon = SP_EPSILON_TWO_DIGITS) const;

		/// <summary>
		/// Get the gram-schmidt decomposition.
		/// It os also known as Gram-Schmidt
		/// </summary>
		/// <param name="output">OrthoNormal Matrix</param>
		/// <returns>output parameter</returns>
		API_INTERFACE void gramSchmidt(Mat& output) const;

		/// <summary>
		/// Decompose the matrix to QR
		/// This QR algorithm uses Gram-Schmidt method
		/// </summary>
		/// <param name="q">Q Matrix</param>
		/// <param name="r">R Matrix</param>
		/// <returns>output parameters</returns>
		API_INTERFACE void qr(Mat& q, Mat& r) const;

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
		/// Get formated matrix as string
		/// </summary>
		/// <param name="precision">Number precision of prints</param>
		/// <returns>Content</returns>
		API_INTERFACE inline std::string toString(const sp_int precision = 4)
		{
			std::string content;
			const sp_uint _length = _rows * _columns;

			for (sp_uint i = ZERO_UINT; i < _length; i++)
			{
				std::stringstream stream;
				stream << std::fixed << std::setprecision(precision) << _values[i];
				std::string numberAsString = stream.str();

				if (_values[i] >= ZERO_FLOAT)
					numberAsString = " " + numberAsString;

				content += " " + numberAsString + " ";

				if ((i + 1) % _columns == ZERO_UINT)
					content += END_OF_LINE;
			}

			return content;
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
	/// Check the square matrix is symmetric
	/// </summary>
	/// <param name="matrix">Matrix</param>
	/// <param name="columnLength">Dimension</param>
	/// <returns>True if the matrix is symmetric orelse False</returns>
	API_INTERFACE inline sp_bool isSymmetric(sp_float* matrix, const sp_uint columnLength)
	{
		for (sp_uint row = ZERO_UINT; row < columnLength; row++)
			for (sp_uint column = row + ONE_UINT; column < columnLength; column++)
				if (NAMESPACE_FOUNDATION::isCloseEnough(matrix[row * columnLength + column], matrix[column * columnLength + row], ZERO_FLOAT))
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
	
	/// <summary>
	/// Multiply A matrix by B and by C matrix and put the result in output matrix parameter
	/// output = A x B x C
	/// </summary>
	/// <param name="a">Input Matrix A</param>
	/// <param name="b">Input Matrix B</param>
	/// <param name="c">Input Matrix C</param>
	/// <param name="output">Output Matrix</param>
	/// <return>output parameter</return>
	API_INTERFACE inline void multiply(const Mat& a, const Mat& b, const Mat& c, Mat& output);

	API_INTERFACE void givensRotation(Mat& output, const sp_uint rowIndex, const sp_uint columnIndex, const sp_float sinTheta, const sp_float cosTheta);

	API_INTERFACE void jacobiRotation(const Mat& input, const sp_float sinTheta, const sp_float cosTheta, const sp_uint rowIndex, const sp_uint columnIndex, Mat& output, Mat& jacobiRotation);

	/// <summary>
	/// Reverse Sort the eigenvalues and eigenvectors column
	/// </summary>
	/// <param name="eigenValues"></param>
	/// <param name="eigenVectors"></param>
	/// <param name="eigenValuesLength">EigenValues Length</param>
	/// <returns>void</returns>
	API_INTERFACE inline void sortEigens(sp_float* eigenValues, Mat& eigenVectors, const sp_uint eigenValuesLength)
	{
		for (sp_uint i = 0; i < eigenValuesLength - 1u; i++)
			for (sp_uint j = i + 1u; j < eigenValuesLength; j++)
				if (eigenValues[j] > eigenValues[i])
				{
					// swap eigenvalues
					const sp_float temp = eigenValues[j];
					eigenValues[j] = eigenValues[i];
					eigenValues[i] = temp;

					// swap eigenvectors
					eigenVectors.swapColumns(i, j);
				}
	}

}

#endif // !MAT_HEADER