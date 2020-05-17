#ifndef VEC2_HEADER
#define VEC2_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define VEC2_SIZE 2

	class Vec2
	{		
	public:
		sp_float x, y;

		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE Vec2(const sp_float value = ZERO_FLOAT);

		/// <summary>
		/// Constructor with parameters
		/// </summary>
		API_INTERFACE Vec2(const sp_float x, const sp_float y);
		
		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE sp_float* getValues();

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE sp_float maximum() const;

		/// <summary>
		/// Scale the current matrix
		/// </summary>
		API_INTERFACE sp_float minimum() const;

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE sp_float length() const;

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2
		/// </summary>
		API_INTERFACE sp_float squared() const;

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE void add(const Vec2& vector);

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE void subtract(const Vec2& vector);

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE void scale(sp_float scale);

		/// <summary>
		/// Dot Product / Scalar Product between two vectors
		/// return u dot v
		/// <summary>
		API_INTERFACE sp_float dot(const Vec2& vector) const;

		/// <summary>
		/// Get the angle in radians between two vectors: A . B
		/// <summary>
		API_INTERFACE sp_float angle(const Vec2& vectorB) const;
		
		/// <summary>
		/// Get a normalized vector
		/// <summary>
		API_INTERFACE Vec2 normalize() const;

		/// <summary>
		/// Normalize the current vector - change to unit vector
		/// <summary>
		API_INTERFACE void transformToUnit();

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// <summary>
		API_INTERFACE sp_float distance(const Vec2& vector) const;
		
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
		/// Clone the vector to a new instance
		/// <summary>
		API_INTERFACE Vec2 clone() const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE Vec2 operator*(const sp_float value) const;

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
		API_INTERFACE Vec2 operator/(const sp_float value) const;

		/// <summary>
		/// Sum this vector to another one
		/// <summary>
		API_INTERFACE Vec2 operator+(const Vec2& vector) const;

		/// <summary>
		/// Sum a scalar to this vector
		/// <summary>
		API_INTERFACE Vec2 operator+(const sp_float value) const;

		/// <summary>
		/// Subtract this vector to another one
		/// <summary>
		API_INTERFACE Vec2 operator-(const Vec2& vector) const;

		/// <summary>
		/// Subtract a scalar from this vector
		/// <summary>
		API_INTERFACE Vec2 operator-(const sp_float value) const;

		/// <summary>
		/// Get negative vector
		/// </summary>
		API_INTERFACE Vec2 operator-() const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE sp_bool operator==(const Vec2& vector) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE sp_bool operator==(const sp_float value) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE sp_bool operator!=(const Vec2& vector) const;

		/// <summary>
		/// Compare this vector to scalar (for each component)
		/// </summary>
		API_INTERFACE sp_bool operator!=(const sp_float value) const;

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float& operator[](const sp_int index);
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float& operator[](const sp_uint index);
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_uint index) const;

#ifdef ENV_64BTIS
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float& operator[](const sp_size index);
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_size index) const;
#endif

		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator sp_float*() const;

		/// <summary>
		/// Serialize current object
		/// </summary>
		API_INTERFACE std::ostream& serialize(std::ostream& outputStream) const;

		/// <summary>
		/// Deserialize current object
		/// </summary>
		API_INTERFACE std::istream& deserialize(std::istream& inputStream);
	};

}

#endif // !VEC2_HEADER
