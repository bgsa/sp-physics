#ifndef VEC3_HEADER
#define VEC3_HEADER

#include "SpectrumPhysics.h"
#include "SpSIMD.h"

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
		API_INTERFACE inline Vec3()
		{
			x = ZERO_FLOAT;
			y = ZERO_FLOAT;
			z = ZERO_FLOAT;
		}
		
		/// <summary>
		/// Construct with args
		/// </summary>
		API_INTERFACE inline Vec3(const sp_float x, const sp_float y, const sp_float z)
		{
			this->x = x;
			this->y = y;
			this->z = z;
		}

		/// <summary>
		/// Default construct
		/// </summary>
		API_INTERFACE inline Vec3(const Vec3& value)
		{
			x = value.x;
			y = value.y;
			z = value.z;
		}
		
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
		/// Get the norma/length of the vector
		/// </summary>
		/// <param name="vector">Vector</param>
		/// <returns>Length of the vector</returns>
		API_INTERFACE inline sp_float length() const
		{
#ifdef AVX_ENABLED
			return sp_vec3_length_simd(sp_vec3_convert_simd((*this))).m128_f32[0];
#else
			return sp_sqrt(vector.squaredLength());
#endif
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

			return value;
		}

		API_INTERFACE inline void abs()
		{
			x = sp_abs(x);
			y = sp_abs(y);
			z = sp_abs(z);
		}

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
		/// Check the orientation of 3 ordered vertexes (left, right or inline)
		/// Z-axis is inverted due to right-hand rule
		/// Returns Zero if the vertexes are (close) inline. 
		/// Lesser than Zero if the third vertex is on the right, 
		/// else the third vertex is on the left.
		/// </summary>
		API_INTERFACE sp_float orientation(const Vec3& vertex1, const Vec3& vertex2) const;

		/// <summary>
		/// Get the triple product A x B x C
		/// </summary>
		/// <param name="B"></param>
		/// <param name="C"></param>
		/// <param name="output">Result</param>
		/// <returns>output</returns>
		API_INTERFACE void tripleProduct(const Vec3& B, const Vec3& C, Vec3* output) const;

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

	const Vec3 Vec3Zeros = Vec3(ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT);
	const Vec3 Vec3Ones = Vec3(ONE_FLOAT, ONE_FLOAT, ONE_FLOAT);
	
	const Vec3 Vec3Up = Vec3(ZERO_FLOAT, ONE_FLOAT, ZERO_FLOAT);
	const Vec3 Vec3Down = Vec3(ZERO_FLOAT, -ONE_FLOAT, ZERO_FLOAT);
	const Vec3 Vec3Left = Vec3(-ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT);
	const Vec3 Vec3Right = Vec3(ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT);
	const Vec3 Vec3Front = Vec3(ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT);
	const Vec3 Vec3Depth = Vec3(ZERO_FLOAT, ZERO_FLOAT, -ONE_FLOAT);

	/// <summary>
	/// Fast Swap the vectors A and B
	/// </summary>
	/// <param name="a">Vector A</param>
	/// <param name="b">Vector B</param>
	/// <returns>void</returns>
	API_INTERFACE inline void swap(Vec3& a, Vec3& b)
	{
		Vec3 temp = a;
		a = b;
		b = temp;
	}

	API_INTERFACE inline void abs(Vec3& output)
	{
		output.x = sp_abs(output.x);
		output.y = sp_abs(output.y);
		output.z = sp_abs(output.z);
	}

	API_INTERFACE inline void abs(const Vec3& input, Vec3& output)
	{
		output.x = sp_abs(input.x);
		output.y = sp_abs(input.y);
		output.z = sp_abs(input.z);
	}

	/// <summary>
	/// Get the norma/length of the vector
	/// </summary>
	/// <param name="vector">Vector</param>
	/// <returns>Length of the vector</returns>
	API_INTERFACE inline sp_float length(const Vec3& vector)
	{
#ifdef AVX_ENABLED
		return sp_vec3_length_simd(sp_vec3_convert_simd(vector)).m128_f32[0];
#else
		return sp_sqrt(vector.squaredLength());
#endif
	}

	API_INTERFACE inline void add(const Vec3& vec1, const Vec3& vec2, Vec3& output)
	{
		output.x = vec1.x + vec2.x;
		output.y = vec1.y + vec2.y;
		output.z = vec1.z + vec2.z;
	}
	
	API_INTERFACE inline void diff(const Vec3& vec1, const Vec3& vec2, Vec3& output)
	{
		output.x = vec1.x - vec2.x;
		output.y = vec1.y - vec2.y;
		output.z = vec1.z - vec2.z;
	}
	
	API_INTERFACE inline void multiply(const Vec3& vec1, const Vec3& vec2, Vec3* output)
	{
		output->x = vec1.x * vec2.x;
		output->y = vec1.y * vec2.y;
		output->z = vec1.z * vec2.z;
	}

	API_INTERFACE inline void multiply(const Vec3& vector, const sp_float value, Vec3& output)
	{
		output.x = vector.x * value;
		output.y = vector.y * value;
		output.z = vector.z * value;
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
		diff(vector, compare, difff);
		difff.abs();

		return difff.x <= _epsilon
			&& difff.y <= _epsilon
			&& difff.z <= _epsilon;
	}

	/// <summary>
	/// Get the normalized direction
	/// output = noramlize(input2 - input1)
	/// </summary>
	/// <param name="input1">Vector 1</param>
	/// <param name="input2">Vector 2</param>
	/// <param name="output">Normalized vector direction</param>
	/// <returns></returns>
	API_INTERFACE inline void direction(const Vec3& input1, const Vec3& input2, Vec3* output)
	{
#ifdef AVX_ENABLED
		__m128 temp = sp_vec3_sub_simd(sp_vec3_convert_simd(input2), sp_vec3_convert_simd(input1));
		__m128 output_simd = sp_vec3_normalize_simd(temp);
		std::memcpy(output, output_simd.m128_f32, SIZEOF_FLOAT * 3u);
#else
		diff(input2, input1, output);
		normalize(output);
#endif
	}

	/// <summary>
	/// Normalize the vector
	/// </summary>
	/// <param name="vector">Vector to be normalized</param>
	/// <returns>void</returns>
	API_INTERFACE inline void normalize(Vec3& vector)
	{
#ifdef AVX_ENABLED
		const __m128 v = sp_vec3_convert_ref_simd(vector); // convert
		const __m128 vDot = sp_vec3_dot_simd(v, v); // dot
		const __m128 vDotSquaredRoot = sp_vec3_rsqrt_simd(vDot); // root reverse
		const sp_float* result = sp_vec3_mult_simd(v, vDotSquaredRoot).m128_f32;
		
		vector.x = result[0];
		vector.y = result[1];
		vector.z = result[2];
#else
		const sp_float len = length(*vector);

		sp_assert(len != ZERO_FLOAT, "InvalidArgumentException");   // avoid division by zero

		const sp_float vectorLengthInverted = ONE_FLOAT / len;

		vector.x *= vectorLengthInverted;
		vector.y *= vectorLengthInverted;
		vector.z *= vectorLengthInverted;
#endif
	}

	/// <summary>
	/// Normalize the vector
	/// </summary>
	/// <param name="input">Vector to be normalized</param>
	/// <param name="output">Normalized vector</param>
	/// <returns>void</returns>
	API_INTERFACE inline void normalize(const Vec3& input, Vec3& output)
	{
#ifdef AVX_ENABLED
		const __m128 v = sp_vec3_convert_simd(input);
		const __m128 vDot = sp_vec3_dot_simd(v, v); // dot
		const __m128 vDotSquaredRoot = sp_vec3_rsqrt_simd(vDot); // root
		const __m128 result = sp_vec3_mult_simd(v, vDotSquaredRoot);

		std::memcpy(output, result.m128_f32, sizeof(sp_float) * VEC3_LENGTH);
#else
		const sp_float len = NAMESPACE_PHYSICS::length(input);

		//sp_assert(len != ZERO_FLOAT, "InvalidArgumentException");   // avoid division by zero

		const sp_float vectorLengthInverted = ONE_FLOAT / len;

		output.x = input.x * vectorLengthInverted;
		output.y = input.y * vectorLengthInverted;
		output.z = input.z * vectorLengthInverted;
#endif
	}

	/// <summary>
	/// Compute the distance (Euclidean) from this vector to another one
	/// </summary>
	API_INTERFACE inline sp_float distance(const Vec3& vector1, const Vec3& vector2)
	{
#ifdef AVX_ENABLED
		const __m128 v1 = sp_vec3_convert_simd(vector1);
		const __m128 v2 = sp_vec3_convert_simd(vector2);
		
		sp_vec3_distance_simd(v2, v1, const __m128 result);
		
		return result.m128_f32[0];
#else
		return sp_sqrt(
			((vector2.x - vector1.x) * (vector2.x - vector1.x)) +
			((vector2.y - vector1.y) * (vector2.y - vector1.y)) +
			((vector2.z - vector1.z) * (vector2.z - vector1.z))
		);
#endif
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
		output->x = vector2.y * vector1.z - vector1.y * vector2.z;
		output->y = -vector2.x * vector1.z + vector1.x * vector2.z;
		output->z = vector2.x * vector1.y - vector1.x * vector2.y;
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
		return NAMESPACE_FOUNDATION::isCloseEnough(ray1.dot(ray2), ZERO_FLOAT, _epsilon);
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
#ifdef AVX_ENABLED
		const __m128 v1_simd = sp_vec3_convert_simd(p1);
		const __m128 v2_simd = sp_vec3_convert_simd(p2);
		const __m128 v3_simd = sp_vec3_convert_simd(p3);

		const __m128 result = sp_vec3_normal_simd(v1_simd, v2_simd, v3_simd);

		std::memcpy(output, result.m128_f32, SIZEOF_FLOAT * 3u);
#else
		// right-hand rule (B-A)x(C-A)
		cross(p3 - p2, p2 - p1, output);
		normalize(output);
#endif
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
