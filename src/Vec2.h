#ifndef VEC2_HEADER
#define VEC2_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define VEC2_LENGTH (2)

	class Vec2
	{
	public:
		sp_float x, y;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE inline Vec2() { }

		/// <summary>
		/// Constructor with constant value
		/// </summary>
		API_INTERFACE inline Vec2(const sp_float value)
		{
			x = value;
			y = value;
		}

		/// <summary>
		/// Constructor with parameters
		/// </summary>
		API_INTERFACE Vec2(const sp_float x, const sp_float y)
		{
			this->x = x;
			this->y = y;
		}

		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE sp_float* values()
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Get the maximum value
		/// </summary>
		API_INTERFACE inline sp_float maximum() const
		{
			return (x > y) ? x : y;
		}

		/// <summary>
		/// Get the minimum value
		/// </summary>
		API_INTERFACE sp_float minimum() const
		{
			return (x < y) ? x : y;
		}

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2
		/// </summary>
		API_INTERFACE inline sp_float squared() const
		{
			return (x * x) + (y * y);
		}

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE inline sp_float length() const
		{
			return sp_sqrt(squared());
		}

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE inline void add(const Vec2& vector)
		{
			x += vector.x;
			y += vector.y;
		}

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE inline void subtract(const Vec2& vector)
		{
			x -= vector.x;
			y -= vector.y;
		}

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE inline void scale(const sp_float scale)
		{
			x *= scale;
			y *= scale;
		}

		/// <summary>
		/// Dot Product / Scalar Product between two vectors
		/// return u dot v
		/// <summary>
		API_INTERFACE inline sp_float dot(const Vec2& vector) const
		{
			return x * vector.x + y * vector.y;
		}

		/// <summary>
		/// Get the angle in radians between two vectors: A . B
		/// <summary>
		API_INTERFACE inline sp_float angle(const Vec2& vectorB) const
		{
			sp_float vec1Len = length();
			sp_float vec2Len = vectorB.length();

			if (vec1Len == ZERO_FLOAT) // vec-Len == 0 means division by zero and return "nan" (not a number)
				vec1Len = 0.000001f;

			if (vec2Len == 0.0f) // vec-Len == 0 means division by zero and return "nan" (not a number)
				vec2Len = 0.000001f;

			return dot(vectorB) / (vec1Len * vec2Len);
		}

		/// <summary>
		/// Get a normalized vector
		/// <summary>
		API_INTERFACE Vec2 normalize() const;

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// <summary>
		API_INTERFACE inline sp_float distance(const Vec2& vector) const
		{
			return sp_sqrt((x - vector.x) * (x - vector.x) + (y - vector.y) * (y - vector.y));
		}

		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE Vec2 fractional() const;

		/// <summary>
		/// Find a orthogonal projection between two vectors
		/// Return Two vectors
		/// <summary>
		API_INTERFACE Vec2* orthogonalProjection(const Vec2& vector) const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE inline Vec2 operator*(const sp_float value) const
		{
			return Vec2(x * value, y * value);
		}

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE friend Vec2 operator*(const sp_float value, const Vec2& vector)
		{
			return vector * value;
		}

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE inline Vec2 operator/(const sp_float value) const
		{
			return Vec2(x / value, y / value);
		}

		/// <summary>
		/// Sum this vector to another one
		/// <summary>
		API_INTERFACE inline Vec2 operator+(const Vec2& vector) const
		{
			return Vec2(x + vector.x, y + vector.y);
		}

		/// <summary>
		/// Sum a scalar to this vector
		/// <summary>
		API_INTERFACE inline Vec2 operator+(const sp_float value) const
		{
			return Vec2(x + value, y + value);
		}

		/// <summary>
		/// Subtract this vector to another one
		/// <summary>
		API_INTERFACE inline Vec2 operator-(const Vec2& vector) const
		{
			return Vec2(x - vector.x, y - vector.y);
		}

		/// <summary>
		/// Subtract a scalar from this vector
		/// <summary>
		API_INTERFACE inline Vec2 operator-(const sp_float value) const
		{
			return Vec2(x - value, y - value);
		}

		/// <summary>
		/// Get negative vector
		/// </summary>
		API_INTERFACE inline Vec2 operator-() const
		{
			return Vec2(-x, -y);
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE inline sp_bool operator==(const Vec2& vector) const
		{
			return x == vector.x && y == vector.y;
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE inline sp_bool operator==(const sp_float value) const
		{
			return x == value && y == value;
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE inline sp_bool operator!=(const Vec2& vector) const
		{
			return x != vector.x || y != vector.y;
		}

		/// <summary>
		/// Compare this vector to scalar (for each component)
		/// </summary>
		API_INTERFACE sp_bool operator!=(const sp_float value) const
		{
			return x != value || y != value;
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE inline sp_float& operator[](const sp_int index)
		{
			sp_assert(index >= ZERO_INT && index < VEC2_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE inline sp_float& operator[](const sp_uint index)
		{
			sp_assert(index >= ZERO_UINT && index < VEC2_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_uint index) const;

#ifdef ENV_64BTIS
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float& operator[](const sp_size index)
		{
			sp_assert(index >= ZERO_SIZE && index < VEC2_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_size index) const;
#endif

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator sp_float* () const
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Serialize current object
		/// </summary>
		API_INTERFACE std::ostream& serialize(std::ostream& outputStream) const;

		/// <summary>
		/// Deserialize current object
		/// </summary>
		API_INTERFACE std::istream& deserialize(std::istream& inputStream);
	};

	const Vec2 Vec2Zeros(ZERO_FLOAT, ZERO_FLOAT);
	const Vec2 Vec2Ones(ONE_FLOAT, ONE_FLOAT);

}

#endif // !VEC2_HEADER
