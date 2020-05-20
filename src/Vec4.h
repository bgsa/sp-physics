#ifndef VEC4_HEADER
#define VEC4_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define VEC4_LENGTH (4)
#define VEC4_SIZE   (VEC4_LENGTH * SIZEOF_FLOAT)

	class Vec4
	{
	public:
		sp_float x, y, z, w;

		/// <summary>
		/// Default constructor
		/// Optional default value 0.0
		/// </summary>
		API_INTERFACE Vec4(const sp_float defaultValue = ZERO_FLOAT);
		
		/// <summary>
		/// Set all components X and Y with the primer vector and Z and W components with latter vector
		/// </summary>
		API_INTERFACE Vec4(const Vec2& xyComponents, const Vec2& zwComponents);
		
		/// <summary>
		/// Constructor with a vector and a W homogeneous coordinate
		/// </summary>
		API_INTERFACE Vec4(const Vec3& vector, sp_float w);
		/// <summary>
		/// Constructor with scalar values
		/// </summary>
		API_INTERFACE Vec4(const sp_float x, const sp_float y, const sp_float z, const sp_float w);
		
		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE sp_float* getValues();

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE sp_float length() const;

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2 + z^2 + w^2
		/// </summary>
		API_INTERFACE sp_float squared() const;

		/// <summary>
		/// Get the maximun value in the vector
		/// </summary>
		API_INTERFACE sp_float maximum() const;

		/// <summary>
		/// Get the min value in the vector
		/// </summary>
		API_INTERFACE sp_float minimum() const;

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE void add(const Vec4& vector);

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE void subtract(const Vec4& vector);

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE void scale(const sp_float scale);

		/// <summary>
		/// Dot Product / Scalar Product - return the angle between two vectors: A . B
		/// return u dot v
		/// <summary>
		API_INTERFACE sp_float dot(const Vec4& vector) const;

		/// <summary>
		/// Get the andle in radians between the vectors
		/// <summary>
		API_INTERFACE sp_float angle(const Vec4& vectorB) const;
		
		/// <summary>
		/// Get a normalized vector
		/// <summary>
		API_INTERFACE Vec4 normalize() const;

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// <summary>
		API_INTERFACE sp_float distance(const Vec4& vector) const;
		
		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE Vec4 fractional();

		/// <summary>
		/// Clone the vector to a new instance
		/// <summary>
		API_INTERFACE Vec4 clone() const;
		
		/// <summary>
		/// Clip w component
		/// <summary>
		API_INTERFACE Vec3 toVec3() const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE Vec4 operator*(const sp_float value);
		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE Vec4 operator*(const sp_float value) const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE friend Vec4 operator*(sp_float value, const Vec4& vector)
		{
			return vector * value;
		}

		/// <summary>
		/// Multiply this vector to a matrix
		/// <summary>
		API_INTERFACE Vec4 operator*(const Mat4& matrix4x4) const;

		/// <summary>
		/// Divide the vector to a scalar
		/// <summary>
		API_INTERFACE Vec4 operator/(const sp_float value) const;

		/// <summary>
		/// Divide the vector to a scalar
		/// <summary>
		API_INTERFACE void operator/=(const sp_float value);

		/// <summary>
		/// Sum this vector to another one
		/// <summary>
		API_INTERFACE Vec4 operator+(const Vec4& vector) const;

		/// <summary>
		/// Sum a scalar to this vector
		/// <summary>
		API_INTERFACE Vec4 operator+(const sp_float value) const;

		/// <summary>
		/// Subtract this vector to another one
		/// <summary>
		API_INTERFACE Vec4 operator-(const Vec4& vector) const;

		/// <summary>
		/// Subtract a scalar from this vector
		/// <summary>
		API_INTERFACE Vec4 operator-(const sp_float value) const;

		/// <summary>
		/// Get negative vector
		/// </summary>
		API_INTERFACE Vec4 operator-() const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE bool operator==(const Vec4& vector) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE bool operator==(const sp_float value) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE bool operator!=(const Vec4& vector) const;

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

#if defined(WINDOWS) && defined(ENV_64BITS)
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE T& operator[](const sp_size index);
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE T operator[](const sp_size index) const;
#endif
		
		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;
		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*();

		/// <summary>
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE operator sp_float*() const;

	};

}

#endif // VEC4_HEADER
