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
		/// Get the squared of the vector. It means the Vector Pow2 -> x^2 + y^2 + z^2
		/// </summary>
		API_INTERFACE inline sp_float squaredLength() const
		{
			return (x * x) + (y * y) + (z * z);
		}

		/// <summary>
		/// Get the length / norma from the vector -> ||v||
		/// </summary>
		API_INTERFACE inline sp_float length() const
		{
			return sqrtf(squaredLength());
		}

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
		API_INTERFACE void add(const Vec3& vector);

		/// <summary>
		/// Subtract a vector from current vector
		/// </summary>
		API_INTERFACE Vec3 subtract(const Vec3& vector);

		/// <summary>
		/// Scale the vector from a scalar => v * scalar
		/// </summary>
		API_INTERFACE void scale(sp_float scale);

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
		/// Dot Product / Scalar Product - between two vectors: A . B
		/// return u dot v
		/// </summary>
		API_INTERFACE inline sp_float dot(const Vec3& vector) const
		{
			return x * vector.x + y * vector.y + z * vector.z;
		}

		/// <summary>
		/// Get the andle in radians between the vectors
		/// Obs.: if the vectors are normalized, can use: acosf(dot(vectorB));
		/// </summary>
		API_INTERFACE sp_float angle(const Vec3& vectorB) const;

		/// <summary>
		/// Get a normalized vector
		/// </summary>
		API_INTERFACE inline Vec3 normalize() const
		{
			const sp_float len = length();

			sp_assert(len != ZERO_FLOAT, "InvalidArgumentException");   // avoid division by zero
			
			const sp_float vectorLengthInverted = ONE_FLOAT / len;

			return Vec3{
				x * vectorLengthInverted,
				y * vectorLengthInverted,
				z * vectorLengthInverted
			};
		}

		/// <summary>
		/// Check the orientation of 3 ordered vertexes (left, right or inline)
		/// Z-axis is inverted due to right-hand rule
		/// Returns Zero if the vertexes are (close) inline. 
		/// Lesser than Zero if the third vertex is on the right, 
		/// else the third vertex is on the left.
		/// </summary>
		API_INTERFACE sp_float orientation(const Vec3& vertex1, const Vec3& vertex2) const;

		/// <summary>
		/// Compute the SQUARED distance from this vector/point to another one
		/// The difference is the squared root is not applied on the result
		/// </summary>
		API_INTERFACE inline sp_float squaredDistance(const Vec3& vector) const
		{
			return
				((x - vector.x) * (x - vector.x)) +
				((y - vector.y) * (y - vector.y)) +
				((z - vector.z) * (z - vector.z));
		}

		/// <summary>
		/// Calculate the distance (Euclidean) from this vector to another one
		/// </summary>
		API_INTERFACE inline sp_float distance(const Vec3& vector) const
		{
			return std::sqrtf(
				((x - vector.x) * (x - vector.x)) +
				((y - vector.y) * (y - vector.y)) +
				((z - vector.z) * (z - vector.z))
			);
		}

		/// <summary>
		/// Calculate the SIGNED distance (Euclidean) from this vector to another one
		/// </summary>
		API_INTERFACE sp_float signedDistance(const Vec3& vector) const;
		
		/// <summary>
		/// Get the fractionals values from the vector (component-wise)
		/// <summary>
		API_INTERFACE inline Vec3 fractional()
		{
			return Vec3{
				x - floorf(x),
				y - floorf(y),
				z - floorf(z)
			};
		}

		/// <summary>
		/// Copy this vector to target
		/// </summary>
		/// <param name="target">Vector to e copied</param>
		/// <returns>void</returns>
		API_INTERFACE inline void copy(Vec3* target)
		{
			target->x = x;
			target->y = y;
			target->z = z;
		}

		/// <summary>
		/// Sum the value to vector
		/// </summary>
		API_INTERFACE Vec3 operator+=(const sp_float value);
		
		/// <summary>
		/// Subtract the value to vector
		/// </summary>
		API_INTERFACE Vec3 operator-=(const sp_float value);

		/// <summary>
		/// Divide the vector to a scalar
		/// </summary>
		API_INTERFACE Vec3 operator/(const sp_float value) const;
		
		/// <summary>
		/// Divide the each component by other component's vector
		/// </summary>
		API_INTERFACE Vec3 operator/(const Vec3& vector) const;

		/// <summary>
		/// Divide the vector to a scalar
		/// <summary>
		API_INTERFACE inline void operator/=(const sp_float value)
		{
			const sp_float _inverse = ONE_FLOAT / value;
			x *= _inverse;
			y *= _inverse;
			z *= _inverse;
		}

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
		/// Compare each component of vector is greater than scalar
		/// </summary>
		API_INTERFACE sp_bool operator>(const sp_float value) const
		{
			return (x > value && y > value && z > value);
		}

		/// <summary>
		/// Compare each component of vector is greater  or equal than scalar
		/// </summary>
		API_INTERFACE sp_bool operator>=(const sp_float value) const
		{
			return (x >= value && y >= value && z >= value);
		}

		/// <summary>
		/// Compare each component of vector is lesser than scalar
		/// </summary>
		API_INTERFACE sp_bool operator<(const sp_float value) const
		{
			return (x < value && y < value && z < value);
		}

		/// <summary>
		/// Compare each component of vector is lesser or equal than scalar
		/// </summary>
		API_INTERFACE sp_bool operator<=(const sp_float value) const
		{
			return (x <= value && y <= value && z <= value);
		}

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

		API_INTERFACE friend std::ostream& operator<<(std::ostream& out, class Vec3& vector)
		{
			out << vector.x << vector.y << vector.z;
			return out;
		}
		API_INTERFACE inline void toString(std::ostream& out) const
		{
			out << "(" << x << ", " << y << ", " << z << ")";
		}

	};

	API_INTERFACE inline void abs(Vec3* vec)
	{
		vec->x = std::fabsf(vec->x);
		vec->y = std::fabsf(vec->y);
		vec->z = std::fabsf(vec->z);
	}

	API_INTERFACE inline void abs(const Vec3& input, Vec3* output)
	{
		output->x = std::fabsf(input.x);
		output->y = std::fabsf(input.y);
		output->z = std::fabsf(input.z);
	}

	API_INTERFACE inline void add(const Vec3& vec1, const Vec3& vec2, Vec3* output)
	{
		output->x = vec1.x + vec2.x;
		output->y = vec1.y + vec2.y;
		output->z = vec1.z + vec2.z;
	}
	API_INTERFACE inline void diff(const Vec3& vec1, const Vec3& vec2, Vec3* output)
	{
		output->x = vec1.x - vec2.x;
		output->y = vec1.y - vec2.y;
		output->z = vec1.z - vec2.z;
	}
	API_INTERFACE inline void multiply(const Vec3& vec1, const Vec3& vec2, Vec3* output)
	{
		output->x = vec1.x * vec2.x;
		output->y = vec1.y * vec2.y;
		output->z = vec1.z * vec2.z;
	}

	/// <summary>
	/// Check if this vector is close to "compare" parameter, given _epsilon
	/// </summary>
	/// <param name="compare">Value to compare</param>
	/// <param name="_epsilon">Error margin</param>
	/// <returns></returns>
	API_INTERFACE inline sp_bool isCloseEnough(const Vec3& vector, const Vec3& compare, const sp_float _epsilon = DefaultErrorMargin)
	{
		Vec3 difff;
		diff(vector, compare, &difff);
		abs(&difff);

		return difff.x <= _epsilon
			&& difff.y <= _epsilon
			&& difff.z <= _epsilon;
	}

	/// <summary>
	/// Normalize the vector
	/// </summary>
	/// <param name="vector">Vector to be normalized</param>
	/// <returns>void</returns>
	API_INTERFACE inline void normalize(Vec3* vector)
	{
		const sp_float len = vector->length();

		sp_assert(len != ZERO_FLOAT, "InvalidArgumentException");   // avoid division by zero

		const sp_float vectorLengthInverted = ONE_FLOAT / len;

		vector->x *= vectorLengthInverted;
		vector->y *= vectorLengthInverted;
		vector->z *= vectorLengthInverted;
	}

	/// <summary>
	/// Normalize the vector
	/// </summary>
	/// <param name="input">Vector to be normalized</param>
	/// <param name="output">Normalized vector</param>
	/// <returns>void</returns>
	API_INTERFACE inline void normalize(const Vec3& input, Vec3* output)
	{
		const sp_float len = input.length();

		sp_assert(len != ZERO_FLOAT, "InvalidArgumentException");   // avoid division by zero

		const sp_float vectorLengthInverted = ONE_FLOAT / len;

		output->x = input.x * vectorLengthInverted;
		output->y = input.y * vectorLengthInverted;
		output->z = input.z * vectorLengthInverted;
	}

	/// <summary>
	/// Cross product vector1 x vector2
	/// </summary>
	/// <param name="vector1">First vector</param>
	/// <param name="vector2">Second vector</param>
	/// <param name="output">Crossed vector (not normalized)</param>
	/// <returns>void</returns>
	API_INTERFACE inline void cross(const Vec3& vector1, const Vec3& vector2, Vec3* output)
	{
		output[0].x = vector2.y * vector1.z - vector1.y * vector2.z;
		output[0].y = -vector2.x * vector1.z + vector1.x * vector2.z;
		output[0].z = vector2.x * vector1.y - vector1.x * vector2.y;
	}

	/// <summary>
	/// Rotate the vector over a given axis and angle
	/// </summary>
	/// <param name="point">Point to be rotated</param>
	/// <param name="angle">Angle in radians</param>
	/// <param name="axis">Axis to rotate</param>
	/// <param name="output">Point rotated</param>
	/// <returns>void</returns>
	API_INTERFACE inline void rotate(const Vec3& point, sp_float angle, const Vec3& axis, Vec3* output)
	{
		sp_float cosAngle = cosf(angle);
		sp_float sinAngle = sinf(angle);

		Vec3 temp;
		cross(axis, point, &temp);

		output[0] = (point * cosAngle) + (temp * sinAngle) + (axis * point.dot(axis)) * (ONE_FLOAT - cosAngle);
	}

	/// <summary>
	/// Scalar triple product = (AxB).C
	/// </summary>
	/// <param name="a">First vector</param>
	/// <param name="b">Second vector</param>
	/// <param name="c">Third vector</param>
	/// <returns>Value</returns>
	API_INTERFACE inline sp_float scalarTriple(const Vec3& a, const Vec3& b, const Vec3& c)
	{
		Vec3 temp;
		cross(a, b, &temp);
		return temp.dot(c);
	}

	/// <summary>
	/// Define if the rays are perpendicular
	/// </summary>
	/// <param name="ray1">Normalized Ray 1</param>
	/// <param name="ray2">Normalized Ray 2</param>
	/// <param name="_epsilon">Error Margin</param>
	/// <returns>True if they are perpendicular, or else False</returns>
	API_INTERFACE inline sp_bool isPerpendicular(const Vec3& ray1, const Vec3& ray2, const sp_float _epsilon = DefaultErrorMargin)
	{
		return isCloseEnough(ray1.dot(ray2), ZERO_FLOAT, _epsilon);
	}

	/// <summary>
	/// Build normal vector from three ordered vertexes (right-hand rule)
	/// </summary>
	/// <param name="p1">Point 1</param>
	/// <param name="p2">Point 2</param>
	/// <param name="p3">Point 3</param>
	/// <param name="output">Result</param>
	/// <returns>void</returns>
	API_INTERFACE inline void normal(const Vec3& p1, const Vec3& p2, const Vec3& p3, Vec3* output)
	{
		// right-hand rule (B-A)x(C-A)
		cross(p3 - p2, p2 - p1, output);
		normalize(output);
	}

	/// <summary>
	/// Check the value parameter contains in list
	/// </summary>
	/// <param name="list">List Of Vec3</param>
	/// <param name="listLength">Length of List</param>
	/// <param name="value">Value to be looked up</param>
	/// <param name="_epsilon">Error margin</param>
	/// <returns>True if found or else False</returns>
	API_INTERFACE inline sp_bool contains(const Vec3* list, const sp_uint listLength, const Vec3& value, const sp_float _epsilon = DefaultErrorMargin)
	{
		for (sp_uint i = 0; i < listLength; i++)
			if (isCloseEnough(list[i], value, _epsilon))
				return true;
	
		return false;
	}

}

#endif // !VEC3_HEADER
