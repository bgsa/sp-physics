#ifndef VEC3_HEADER
#define VEC3_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define VEC3_LENGTH (3)
#define VEC3_SIZE   (VEC3_LENGTH * SIZEOF_FLOAT)

	class Vec3
	{
	public:
		sp_float x, y, z;

		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE Vec3(const sp_float defaultValue = ZERO_FLOAT);
		/// <summary>
		/// Construct with args
		/// </summary>
		API_INTERFACE Vec3(const sp_float x, const sp_float y, const sp_float z);
		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE Vec3(const Vec3& value);
		
		/// <summary>
		/// Construct with args
		/// </summary>
		API_INTERFACE Vec3(Vec2 vector2D, sp_float z);
		
		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE sp_float* getValues();

		/// <summary>
		/// Get the absolute value from each component
		/// </summary>
		API_INTERFACE Vec3 abs() const;

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2 + z^2
		/// </summary>
		API_INTERFACE sp_float squaredLength() const;

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE sp_float length() const;

		/// <summary>
		/// Get the maximun value in the vector
		/// </summary>
		API_INTERFACE sp_float maximum() const;

		/// <summary>
		/// Get the min value in the vector
		/// </summary>
		API_INTERFACE sp_float minimum() const;

		/// <summary>
		/// Get the scalar triple product: (u x v) . w
		/// It also give the volume of parallelepiped
		/// </summary>
		API_INTERFACE sp_float tripleProduct(const Vec3 &v, const Vec3 &u) const;

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE void add(const Vec3& vector);

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE Vec3 subtract(const Vec3& vector);

		/// <summary>
		/// Multiply each components from current vector to another one
		/// </summary>
		API_INTERFACE Vec3 multiply(const Vec3& vector) const;

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE void scale(sp_float scale);

		/// <summary>
		/// Rotate the vector over a given axis and angle
		/// </summary>
		API_INTERFACE Vec3 rotate(sp_float angle, const Vec3& axis);

		/// <summary>
		/// Rotate the vector on X axis, given an angle
		/// </summary>
		API_INTERFACE Vec3 rotateX(sp_float angle);

		/// <summary>
		/// Rotate the vector on Y axis, given an angle
		/// </summary>
		API_INTERFACE Vec3 rotateY(sp_float angle);

		/// <summary>
		/// Rotate the vector on Y axis, given an angle and the reference point (rotate around this point)
		/// </summary>
		API_INTERFACE Vec3 rotateY(sp_float angle, const Vec3& referencePoint);

		/// <summary>
		/// Rotate the vector on Z axis, given an angle
		/// </summary>
		API_INTERFACE Vec3 rotateZ(sp_float angle);

		/// <summary>
		/// Rotate the vector on Z axis, given an angle and the reference point (rotate around this point)
		/// </summary>
		API_INTERFACE Vec3 rotateZ(sp_float angle, const Vec3& referencePoint);

		/// <summary>
		/// Cross Product - return a perpendicular vector, regards two vectors => u x v
		/// </summary>
		API_INTERFACE Vec3 cross(const Vec3& vector) const;

		/// <summary>
		/// Dot Product / Scalar Product - between two vectors: A . B
		/// return u dot v
		/// </summary>
		API_INTERFACE sp_float dot(const Vec3& vector) const;

		/// <summary>
		/// Get the andle in radians between the vectors
		/// Obs.: if the vectors are normalized, can use: acosf(dot(vectorB));
		/// </summary>
		API_INTERFACE sp_float angle(const Vec3& vectorB) const;

		/// <summary>
		/// Get a normalized vector
		/// </summary>
		API_INTERFACE Vec3 normalize() const;

		/// <summary>
		/// Normalize the current vector - change to unit vector
		/// </summary>
		API_INTERFACE void transformToUnit();

		/// <summary>
		/// Compute the SQUARED distance from this vector/point to another one
		/// The difference is the squared root is not applied on the result
		/// </summary>
		API_INTERFACE sp_float squaredDistance(const Vec3& vector) const;

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// </summary>
		API_INTERFACE sp_float distance(const Vec3& vector) const;

		/// <summary>
		/// Calculate the SIGNED distance (Euclidean) from this vector to another one
		/// </summary>
		API_INTERFACE sp_float signedDistance(const Vec3& vector) const;
		
		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE Vec3 fractional();

		/// <summary>
		/// Clone the vector to a new instance
		/// </summary>
		API_INTERFACE Vec3 clone();

		/// <summary>
		/// Sum the value to vector
		/// </summary>
		API_INTERFACE Vec3 operator+=(const sp_float value);
		
		/// <summary>
		/// Subtract the value to vector
		/// </summary>
		API_INTERFACE Vec3 operator-=(const sp_float value);

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE Vec3 operator/(const sp_float value) const;
		
		/// <summary>
		/// Divide the each component by other component's vector
		/// </summary>
		API_INTERFACE Vec3 operator/(const Vec3& vector) const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// </summary>
		API_INTERFACE Vec3 operator*(const sp_float value) const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE void operator*=(const sp_float value);
		
		/// <summary>
		/// Multiply the vector to another one using Cross Product
		/// </summary>
		API_INTERFACE Vec3 operator*(const Vec3& vector) const;

		/// <summary>
		/// Sum this vector to another one
		/// </summary>
		API_INTERFACE Vec3 operator+(const Vec3& vector) const;

		/// <summary>
		/// Sum this vector to another one
		/// </summary>
		API_INTERFACE void operator+=(const Vec3& vector);

		/// <summary>
		/// Sum a scalar to this vector
		/// </summary>
		API_INTERFACE Vec3 operator+(const sp_float value) const;

		/// <summary>
		/// Subtract this vector to another one
		/// </summary>
		API_INTERFACE Vec3 operator-(const Vec3& vector) const;

		/// <summary>
		/// Get the negative vector
		/// </summary>
		API_INTERFACE Vec3 operator-() const;

		/// <summary>
		/// Subtract this vector to another one
		/// </summary>
		API_INTERFACE void operator-=(const Vec3& vector);

		/// <summary>
		/// Subtract a scalar from this vector
		/// </summary>
		API_INTERFACE Vec3 operator-(const sp_float value) const;

		/// <summary>
		/// Assign operator
		/// </summary>
		API_INTERFACE Vec3& operator=(const Vec3& vector);

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE sp_bool operator==(const Vec3& vector) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE sp_bool operator==(const sp_float value) const;

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// </summary>
		API_INTERFACE sp_bool operator!=(const Vec3& vector) const;

		/// <summary>
		/// Compare this vector to scalar (for each component)
		/// </summary>
		API_INTERFACE sp_bool operator!=(const sp_float value) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](sp_int index);
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](sp_int index) const;

		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](sp_uint index);
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](sp_uint index) const;

#ifdef ENV_64BITS
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float& operator[](sp_size index);
		/// <summary>
		/// Get a index from the vector
		/// </summary>
		API_INTERFACE sp_float operator[](sp_size index) const;
#endif

		/// <summary>
		/// Auto convertion to void *
		/// </summary> 
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE operator sp_float*() const;

	};

}

#endif // !VEC3_HEADER
