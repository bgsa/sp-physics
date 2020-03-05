#ifndef VEC3_HEADER
#define VEC3_HEADER

#include "OpenML.h"

namespace OpenML
{
#define VEC3_SIZE 3

	template <typename T>
	class Vec3
	{
	public:
		T x, y, z;

		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE inline Vec3(const T defaultValue = T(0));
		/// <summary>
		/// Construct with args
		/// </summary>
		API_INTERFACE inline Vec3(const T x, const T y, const T z);
		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE inline Vec3(const Vec3<T>& value);
		
		/// <summary>
		/// Construct with args
		/// </summary>
		API_INTERFACE Vec3(Vec2<T> vector2D, T z);
		
		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE inline T* getValues();

		/// <summary>
		/// Get the absolute value from each component
		/// </summary>
		API_INTERFACE inline Vec3<T> abs() const;

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2 + z^2
		/// </summary>
		API_INTERFACE inline T squaredLength() const;

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE inline T length() const;

		/// <summary>
		/// Get the maximun value in the vector
		/// </summary>
		API_INTERFACE inline T maximum() const;

		/// <summary>
		/// Get the min value in the vector
		/// </summary>
		API_INTERFACE inline T minimum() const;

		/// <summary>
		/// Get the scalar triple product: (u x v) . w
		/// It also give the volume of parallelepiped
		/// </summary>
		API_INTERFACE inline T tripleProduct(const Vec3<T> &v, const Vec3<T> &u) const;

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE inline void add(const Vec3<T>& vector);

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE inline Vec3<T> subtract(const Vec3<T>& vector);

		/// <summary>
		/// Multiply each components from current vector to another one
		/// </summary>
		API_INTERFACE inline Vec3<T> multiply(const Vec3<T>& vector) const;

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE inline void scale(T scale);

		/// <summary>
		/// Rotate the vector on X axis, given an angle
		/// </summary>
		API_INTERFACE inline Vec3<T> rotateX(sp_float angle);

		/// <summary>
		/// Rotate the vector on X axis, given an angle and the reference point (rotate around this point)
		/// </summary>
		API_INTERFACE inline Vec3<T> rotateX(sp_float angle, Vec3<T> referencePoint);

		/// <summary>
		/// Rotate the vector on Y axis, given an angle
		/// </summary>
		API_INTERFACE inline Vec3<T> rotateY(sp_float angle);

		/// <summary>
		/// Rotate the vector on Y axis, given an angle and the reference point (rotate around this point)
		/// </summary>
		API_INTERFACE inline Vec3<T> rotateY(sp_float angle, Vec3<T> referencePoint);

		/// <summary>
		/// Rotate the vector on Z axis, given an angle
		/// </summary>
		API_INTERFACE inline Vec3<T> rotateZ(sp_float angle);

		/// <summary>
		/// Rotate the vector on Z axis, given an angle and the reference point (rotate around this point)
		/// </summary>
		API_INTERFACE inline Vec3<T> rotateZ(sp_float angle, Vec3<T> referencePoint);

		/// <summary>
		/// Cross Product - return a perpendicular vector, regards two vectors => u x v
		/// </summary>
		API_INTERFACE inline Vec3<T> cross(const Vec3<T>& vector) const;

		/// <summary>
		/// Dot Product / Scalar Product - between two vectors: A . B
		/// return u dot v
		/// </summary>
		API_INTERFACE inline T dot(const Vec3<T>& vector) const;

		/// <summary>
		/// Get the andle in radians between the vectors
		/// Obs.: if the vectors are normalized, can use: acosf(dot(vectorB));
		/// </summary>
		API_INTERFACE inline T angle(const Vec3<T>& vectorB) const;

		/// <summary>
		/// Get a normalized vector
		/// </summary>
		API_INTERFACE inline Vec3<T> normalize() const;

		/// <summary>
		/// Normalize the current vector - change to unit vector
		/// </summary>
		API_INTERFACE inline void transformToUnit();

		/// <summary>
		/// Compute the SQUARED distance from this vector/point to another one
		/// The difference is the squared root is not applied on the result
		/// </summary>
		API_INTERFACE inline T squaredDistance(const Vec3<T>& vector) const;

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// </summary>
		API_INTERFACE inline T distance(const Vec3<T>& vector) const;

		/// <summary>
		/// Calculate the SIGNED distance (Euclidean) from this vector to another one
		/// </summary>
		API_INTERFACE inline T signedDistance(const Vec3<T>& vector) const;
		
		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE Vec3<T> fractional();

		/// <summary>
		/// Clone the vector to a new instance
		/// </summary>
		API_INTERFACE inline Vec3<T> clone();

		/// <summary>
		/// Sum the value to vector
		/// </summary>
		API_INTERFACE inline Vec3<T> operator+=(const T value);
		
		/// <summary>
		/// Subtract the value to vector
		/// </summary>
		API_INTERFACE inline Vec3<T> operator-=(const T value);

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE inline Vec3<T> operator/(const T value) const;
		
		/// <summary>
		/// Divide the each component by other component's vector
		/// </summary>
		API_INTERFACE inline Vec3<T> operator/(const Vec3<T> vector);

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE inline Vec3<T> operator*(const T value) const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE void operator*=(const T value);
		
		/// <summary>
		/// Multiply the vector to another one using Cross Product
		/// </summary>
		API_INTERFACE inline Vec3<T> operator*(const Vec3<T>& vector) const;

		/// <summary>
		/// Sum this vector to another one
		/// </summary>
		API_INTERFACE inline Vec3<T> operator+(const Vec3<T>& vector) const;

		/// <summary>
		/// Sum this vector to another one
		/// </summary>
		API_INTERFACE inline Vec3<T> operator+=(const Vec3<T>& vector);

		/// <summary>
		/// Sum a scalar to this vector
		/// </summary>
		API_INTERFACE inline Vec3<T> operator+(const T value) const;

		/// <summary>
		/// Subtract this vector to another one
		/// </summary>
		API_INTERFACE inline Vec3<T> operator-(const Vec3<T>& vector) const;

		/// <summary>
		/// Get the negative vector
		/// </summary>
		API_INTERFACE inline Vec3<T> operator-() const;

		/// <summary>
		/// Subtract this vector to another one
		/// </summary>
		API_INTERFACE inline Vec3<T> operator-=(const Vec3<T>& vector) const;

		/// <summary>
		/// Subtract a scalar from this vector
		/// </summary>
		API_INTERFACE inline Vec3<T> operator-(const T value) const;

		/// <summary>
		/// Assign operator
		/// </summary>
		API_INTERFACE inline Vec3<T>& operator=(const Vec3<T>& vector);

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE inline sp_bool operator==(const Vec3<T>& vector) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE inline sp_bool operator==(const T value) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE inline sp_bool operator!=(const Vec3<T>& vector) const;

		/// <summary>
		/// Compare this vector to scalar (for each component)
		/// </summary>
		API_INTERFACE inline sp_bool operator!=(const T value) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline T& operator[](sp_int index);
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline T operator[](sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline T& operator[](sp_uint index);
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE inline T operator[](sp_uint index) const;

		/// <summary>
		/// Auto convertion to void *
		/// </summary> 
		API_INTERFACE inline operator void*() const;
		/// <summary>
		/// Auto convertion to void *
		/// </summary> 
		API_INTERFACE inline operator void*();

		/// <summary>
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE inline operator T*();

	};

	typedef Vec3<sp_int> Vec3i;
	typedef Vec3<sp_float> Vec3f;
	typedef Vec3<sp_double> Vec3d;

}

#endif // !VEC3_HEADER