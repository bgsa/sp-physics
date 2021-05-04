#ifndef VEC_HEADER
#define VEC_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
	class Vec
	{
	private:
		sp_uint _length;
		sp_float* _values;

	public:

		/// <summary>
		/// Default Construct
		/// </summary>
		API_INTERFACE inline Vec()
		{
			_length = ZERO_UINT;
			_values = nullptr;
		}

		/// <summary>
		/// Construct with length of the elements
		/// </summary>
		API_INTERFACE inline Vec(const sp_uint length)
		{
			_length = length;
			_values = sp_mem_new_array(sp_float, length);
		}

		API_INTERFACE inline void resize(const sp_uint length)
		{
			dispose();

			_length = length;
			_values = sp_mem_new_array(sp_float, length);
		}

		API_INTERFACE inline sp_uint length() const
		{
			return _length;
		}

		/// <summary>
		/// Fill all array with the constant value
		/// </summary>
		/// <param name="value">Constant Value</param>
		/// <returns>void</returns>
		API_INTERFACE inline void fill(const sp_int value)
		{
			std::memset(_values, value, sizeof(sp_float)* _length);
		}

		/// <summary>
		/// Get the square length of the vector
		/// </summary>
		/// <returns>Square Length</returns>
		API_INTERFACE inline sp_float squareLength() const
		{
			sp_float value = ZERO_FLOAT;

			for (sp_uint i = 0; i < _length; i++)
				value += _values[i] * _values[i];

			return value;
		}

		/// <summary>
		/// Get the dot product of the vectors
		/// </summary>
		/// <returns>Dot Product</returns>
		API_INTERFACE inline sp_float dot(const Vec& vector) const
		{
			sp_assert(length() == vector.length(), "InvalidArgumentException");

			sp_float value = ZERO_FLOAT;

			for (sp_uint i = 0; i < _length; i++)
				value += _values[i] * vector._values[i];

			return value;
		}

		/// <summary>
		/// Get the Length/Norm of the vector
		/// </summary>
		/// <returns>Length</returns>
		API_INTERFACE inline sp_float norm() const
		{
			return sp_sqrt(squareLength());
		}

		/// <summary>
		/// Auto convertion to float*
		/// </summary>
		API_INTERFACE operator sp_float* () const
		{
			return _values;
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float& operator[](const sp_int index)
		{
			sp_assert(index >= 0 && index < (sp_int)length(), "IndexOutOfrangeException");
			return _values[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < (sp_int)length(), "IndexOutOfrangeException");
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
		/// Dispose all allocated resources from this vector
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline void dispose()
		{
			if (_values != nullptr)
			{
				sp_mem_release(_values);
				_values = nullptr;
			}

			_length = ZERO_UINT;
		}
	};

	/// <summary>
	/// Subtract vector A from vector B
	/// </summary>
	/// <param name="a">Vector A</param>
	/// <param name="b">Vector B</param>
	/// <param name="output">Result</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void add(const Vec& a, const Vec& b, Vec& output)
	{
		sp_assert(a.length() == b.length(), "InvalidArgumentException");

		for (sp_uint i = 0; i < a.length(); i++)
			output[i] = a[i] + b[i];
	}

	/// <summary>
	/// Subtract vector A from vector B
	/// </summary>
	/// <param name="a">Vector A</param>
	/// <param name="b">Vector B</param>
	/// <param name="output">Result</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void diff(const Vec& a, const Vec& b, Vec& output)
	{
		sp_assert(a.length() == b.length(), "InvalidArgumentException");

		for (sp_uint i = 0; i < a.length(); i++)
			output[i] = a[i] - b[i];
	}

	/// <summary>
	/// Multiply vector A by scalar value
	/// </summary>
	/// <param name="a">Vector A</param>
	/// <param name="value">Scalar Value</param>
	/// <param name="output">Result</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void multiply(const Vec& a, const sp_float value, Vec& output)
	{
		for (sp_uint i = 0; i < a.length(); i++)
			output[i] = a[i] * value;
	}

	/// <summary>
	/// Divide vector A by scalar value
	/// </summary>
	/// <param name="a">Vector A</param>
	/// <param name="value">Scalar Value</param>
	/// <param name="output">Result</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void div(const Vec& a, const sp_float value, Vec& output)
	{
		multiply(a, NAMESPACE_FOUNDATION::div(ONE_FLOAT, value), output);
	}

}

#endif // VEC_HEADER