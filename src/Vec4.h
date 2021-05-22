#ifndef VEC4_HEADER
#define VEC4_HEADER

#include "SpectrumPhysics.h"

namespace NAMESPACE_PHYSICS
{
#define VEC4_LENGTH (4)

	class Vec4
	{
	public:
		sp_float x, y, z, w;
	
		/// <summary>
		/// Default constructor
		/// </summary>
		API_INTERFACE inline Vec4() { }

		/// <summary>
		/// Constructor with constant value
		/// </summary>
		/// <param name="constant">Default value for all elements</param>
		API_INTERFACE inline Vec4(const sp_float constant)
		{
			x = constant;
			y = constant;
			z = constant;
			w = constant;
		}
		
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
		API_INTERFACE inline Vec4(const sp_float x, const sp_float y, const sp_float z, const sp_float w)
		{
			this->x = x;
			this->y = y;
			this->z = z;
			this->w = w;
		}

		/// <summary>
		/// Get the component values in the vector
		/// </summary>
		API_INTERFACE inline sp_float* values()
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE inline sp_float length() const
		{
			return sp_sqrt(squared());
		}

		/// <summary>
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2 + z^2 + w^2
		/// </summary>
		API_INTERFACE inline sp_float squared() const
		{
			return (x * x) + (y * y) + (z * z) + (w * w);
		}

		/// <summary>
		/// Get the maximun value in the vector
		/// </summary>
		API_INTERFACE inline sp_float maximum() const
		{
			sp_float value = x;

			if (y > value)
				value = y;

			if (z > value)
				value = z;

			if (w > value)
				value = w;

			return value;
		}

		/// <summary>
		/// Get the min value in the vector
		/// </summary>
		API_INTERFACE inline sp_float minimum() const
		{
			sp_float value = x;

			if (y < value)
				value = y;

			if (z < value)
				value = z;

			if (w < value)
				value = w;

			return value;
		}

		/// <summary>
		/// Add a vector from current vector
		/// </summary>
		API_INTERFACE inline void add(const Vec4& vector)
		{
			x += vector.x;
			y += vector.y;
			z += vector.z;
			w += vector.w;
		}

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE inline void subtract(const Vec4& vector)
		{
			x -= vector.x;
			y -= vector.y;
			z -= vector.z;
			w -= vector.w;
		}

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE inline void scale(const sp_float scale)
		{
			x *= scale;
			y *= scale;
			z *= scale;
			w *= scale;
		}

		/// <summary>
		/// Dot Product / Scalar Product - return the angle between two vectors: A . B
		/// return u dot v
		/// <summary>
		API_INTERFACE inline sp_float dot(const Vec4& vector) const
		{
			return x * vector.x + y * vector.y + z * vector.z + w * vector.w;
		}

		/// <summary>
		/// Get the andle in radians between the vectors
		/// <summary>
		API_INTERFACE inline sp_float angle(const Vec4& vectorB) const
		{
			return dot(vectorB) / (length() * vectorB.length());
		}
		
		/// <summary>
		/// Get a normalized vector
		/// <summary>
		API_INTERFACE Vec4 normalize() const;

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// <summary>
		API_INTERFACE inline sp_float distance(const Vec4& vector) const
		{
#define xDiff (x - vector.x)
#define yDiff (y - vector.y)
#define zDiff (z - vector.z)
#define wDiff (w - vector.w)
			return sp_sqrt(xDiff * xDiff + yDiff * yDiff + zDiff * zDiff + wDiff * wDiff);
#undef wDiff
#undef zDiff
#undef yDiff
#undef xDiff
		}
		
		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE inline Vec4 fractional() const
		{
			return Vec4{
				x - floorf(x),
				y - floorf(y),
				z - floorf(z),
				w - floorf(w)
			};
		}

		/// <summary>
		/// Clip w component
		/// <summary>
		API_INTERFACE Vec3 toVec3() const;

		/// <summary>
		/// Multiply the vector to a scalar
		/// <summary>
		API_INTERFACE inline Vec4 operator*(const sp_float value) const
		{
			return Vec4(
				x * value,
				y * value,
				z * value,
				w * value
			);
		}

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
		API_INTERFACE inline Vec4 operator/(const sp_float value) const
		{
			return Vec4(
				x / value,
				y / value,
				z / value,
				w / value
			);
		}

		/// <summary>
		/// Divide the vector to a scalar
		/// <summary>
		API_INTERFACE inline void operator/=(const sp_float value)
		{
			x /= value;
			y /= value;
			z /= value;
			w /= value;
		}

		/// <summary>
		/// Sum this vector to another one
		/// <summary>
		API_INTERFACE inline Vec4 operator+(const Vec4& vector) const
		{
			return Vec4(
				x + vector.x,
				y + vector.y,
				z + vector.z,
				w + vector.w
			);
		}

		/// <summary>
		/// Sum a scalar to this vector
		/// <summary>
		API_INTERFACE inline Vec4 operator+(const sp_float value) const
		{
			return Vec4(
				x + value,
				y + value,
				z + value,
				w + value
			);
		}

		/// <summary>
		/// Subtract this vector to another one
		/// <summary>
		API_INTERFACE inline Vec4 operator-(const Vec4& vector) const
		{
			return Vec4(
				x - vector.x,
				y - vector.y,
				z - vector.z,
				w - vector.w
			);
		}

		/// <summary>
		/// Subtract a scalar from this vector
		/// <summary>
		API_INTERFACE inline Vec4 operator-(const sp_float value) const
		{
			return Vec4(
				x - value,
				y - value,
				z - value,
				w - value
			);
		}

		/// <summary>
		/// Get negative vector
		/// </summary>
		API_INTERFACE inline Vec4 operator-() const
		{
			return Vec4(-x, -y, -z, -w);
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE inline sp_bool operator==(const Vec4& vector) const
		{
			return x == vector.x
				&& y == vector.y
				&& z == vector.z
				&& w == vector.w;
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE inline sp_bool operator==(const sp_float value) const
		{
			return x == value
				&& y == value
				&& z == value
				&& w == value;
		}

		/// <summary>
		/// Compare this vector to another one. Compare each component.
		/// <summary>
		API_INTERFACE inline sp_bool operator!=(const Vec4& vector) const;

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE inline sp_float& operator[](const sp_int index)
		{
			sp_assert(index >= 0 && index < VEC4_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < VEC4_LENGTH, "IndexOutOfrangeException");
			return reinterpret_cast<const sp_float*>(this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE inline sp_float& operator[](const sp_uint index)
		{
			sp_assert(index >= 0 && index < VEC4_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE inline sp_float operator[](const sp_uint index) const
		{
			sp_assert(index >= 0 && index < VEC4_LENGTH, "IndexOutOfrangeException");
			return reinterpret_cast<const sp_float*>(this)[index];
		}

#if defined(WINDOWS) && defined(ENV_64BITS)
		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float& operator[](const sp_size index)
		{
			sp_assert(index >= ZERO_SIZE && index < VEC4_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the vector
		/// <summary>
		API_INTERFACE sp_float operator[](const sp_size index) const
		{
			sp_assert(index >= ZERO_SIZE && index < VEC4_LENGTH, "IndexOutOfrangeException");
			return reinterpret_cast<const sp_float*>(this)[index];
		}
#endif
		
		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const
		{
			return (void*)this;
		}
		
		/// <summary>
		/// Auto convertion to T *
		/// It is the same of convertion to float* or int* or double*, whatever T is.
		/// </summary>
		API_INTERFACE operator sp_float*() const
		{
			return (sp_float*)this;
		}

	};

	API_INTERFACE void multiply(const Vec4& vectorA, const Vec4& vectorB, Mat4* output);

}



#endif // VEC4_HEADER
