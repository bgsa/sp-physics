#ifndef VEC4_HEADER
#define VEC4_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define VEC4_SIZE 4

	template <typename T>
	class Vec4
	{
	public:
		T x, y, z, w;

		/// <summary>
		/// Default constructor
		/// Optional default value 0.0
		/// </summary>
		API_INTERFACE Vec4(const T defaultValue = T(0));
		
		/// <summary>
		/// Set all components X and Y with the primer vector and Z and W components with latter vector
		/// </summary>
		API_INTERFACE Vec4(const Vec2<T>& xyComponents, const Vec2<T>& zwComponents);
		
		/// <summary>
		/// Constructor with a vector and a W homogeneous coordinate
		/// </summary>
		API_INTERFACE Vec4(const Vec3<T>& vector, T w);
		/// <summary>
		/// Constructor with scalar values
		/// </summary>
		API_INTERFACE Vec4(const T x, const T y, const T z, const T w);
		
		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE T* getValues();

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE T length() const;

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2 + z^2 + w^2
		/// </summary>
		API_INTERFACE T squared() const;

		/// <summary>
		/// Get the maximun value in the vector
		/// </summary>
		API_INTERFACE T maximum() const;

		/// <summary>
		/// Get the min value in the vector
		/// </summary>
		API_INTERFACE T minimum() const;

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE void add(const Vec4<T>& vector);

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE void subtract(const Vec4<T>& vector);

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE void scale(const T scale);

		/// <summary>
		/// Dot Product / Scalar Product - return the angle between two vectors: A . B
		/// return u dot v
		/// <summary>
		API_INTERFACE T dot(const Vec4<T>& vector) const;

		/// <summary>
		/// Get the andle in radians between the vectors
		/// <summary>
		API_INTERFACE T angle(const Vec4<T>& vectorB) const;
		
		/// <summary>
		/// Get a normalized vector
		/// <summary>
		API_INTERFACE Vec4<T> normalize() const;

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// <summary>
		API_INTERFACE T distance(const Vec4<T>& vector) const;
		
		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE Vec4<T> fractional();

		/// <summary>
		/// Clone the vector to a new instance
		/// <summary>
		API_INTERFACE Vec4<T> clone() const;
		
		/// <summary>
		/// Clip w component
		/// <summary>
		API_INTERFACE Vec3<T> toVec3() const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE Vec4<T> operator*(const T value);
		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE Vec4<T> operator*(const T value) const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE friend Vec4<T> operator*(T value, const Vec4<T>& vector)
		{
			return vector * value;
		}

		/// <summary>
		/// Multiply this vector to a matrix
		/// <summary>
		API_INTERFACE Vec4<T> operator*(const Mat4<T>& matrix4x4) const;

		/// <summary>
		/// Divide the vector to a scalar
		/// <summary>
		API_INTERFACE Vec4<T> operator/(const T value) const;

		/// <summary>
		/// Divide the vector to a scalar
		/// <summary>
		API_INTERFACE void operator/=(const T value);

		/// <summary>
		/// Sum this vector to another one
		/// <summary>
		API_INTERFACE Vec4<T> operator+(const Vec4<T>& vector) const;

		/// <summary>
		/// Sum a scalar to this vector
		/// <summary>
		API_INTERFACE Vec4<T> operator+(const T value) const;

		/// <summary>
		/// Subtract this vector to another one
		/// <summary>
		API_INTERFACE Vec4<T> operator-(const Vec4<T>& vector) const;

		/// <summary>
		/// Subtract a scalar from this vector
		/// <summary>
		API_INTERFACE Vec4<T> operator-(const T value) const;

		/// <summary>
		/// Get negative vector
		/// </summary>
		API_INTERFACE Vec4<T> operator-() const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE bool operator==(const Vec4<T>& vector) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE bool operator==(const T value) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE bool operator!=(const Vec4<T>& vector) const;

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE T& operator[](const sp_int index);
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE T operator[](const sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE T& operator[](const sp_uint index);
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE T operator[](const sp_uint index) const;

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
		API_INTERFACE operator T*() const;

	};

	typedef Vec4<sp_float> Vec4f;
	typedef Vec4<sp_double> Vec4d;

}

#endif // VEC4_HEADER
