#ifndef QUAT_HEADER
#define QUAT_HEADER

#include "SpectrumPhysics.h"
#include "SpSIMD.h"

namespace NAMESPACE_PHYSICS
{

	class Quat
	{
	public:
		sp_float w, x, y, z;

		/// <summary>
		/// Default constructor
		/// </summary>
		/// <returns></returns>
		API_INTERFACE inline Quat()
		{
			/*
			static sp_float identityQuaternion[QUAT_LENGTH] = {
				ONE_FLOAT,
				ZERO_FLOAT,
				ZERO_FLOAT,
				ZERO_FLOAT
			};

			std::memcpy(this, identityQuaternion, QUAT_SIZE);
			*/
		}

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE inline Quat(const sp_float* values)
		{
			std::memcpy(this, values, sizeof(Quat));
		}

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE inline Quat(const sp_float w, const sp_float x, const sp_float y, const sp_float z)
		{
			this->w = w;
			this->x = x;
			this->y = y;
			this->z = z;
		}

		/// <summary>
		/// Constructor with a 3D vector
		/// </summary>
		API_INTERFACE Quat(const Vec3& vector);

		/// <summary>
		/// Constructor with a 3D vector
		/// </summary>
		API_INTERFACE Quat(const sp_float w, const Vec3& vector);

		/// <summary>
		/// Scale a quaternion
		/// </summary>
		API_INTERFACE inline Quat scale(const sp_float value) const
		{
			return Quat(
				w * value,
				x * value,
				y * value,
				z * value
			);
		}

		/// <summary>
		/// Length/Magnitude of quaternion
		/// </summary>
		API_INTERFACE inline sp_float length() const
		{
			return sp_sqrt(x * x + y * y + z * z + w * w);
		}

		/// <summary>
		/// Get the rotation axis from this quaternion
		/// </summary>
		API_INTERFACE Vec3 axis() const;

		/// <summary>
		/// Product Scalar of two quaternion
		/// </summary>
		API_INTERFACE inline sp_float dot(const Quat& q) const
		{
			return (w * q.w) + (x * q.x) + (y * q.y) + (z * q.z);
		}

		/// <summary>
		/// Create a rotation unit quaternion bases on angle (in radians) and directional vector provided
		/// </summary>
		API_INTERFACE static Quat createRotate(const sp_float angle, const Vec3& axis);

		/// <summary>
		/// Create a quaternion rotation around X axis
		/// </summary>
		API_INTERFACE static Quat createRotationAxisX(const sp_float angle)
		{
			return Quat(
				sp_cos(angle * HALF_FLOAT),
				sp_sin(angle * HALF_FLOAT),
				ZERO_FLOAT,
				ZERO_FLOAT
			);
		}

		/// <summary>
		/// Create a quaternion rotation around Y axis
		/// </summary>
		API_INTERFACE static Quat createRotationAxisY(const sp_float angle)
		{
			return Quat(
				sp_cos(angle * HALF_FLOAT),
				ZERO_FLOAT,
				sp_sin(angle * HALF_FLOAT),
				ZERO_FLOAT
			);
		}

		/// <summary>
		/// Create a quaternion rotation around Z axis
		/// </summary>
		API_INTERFACE static Quat createRotationAxisZ(const sp_float angle)
		{
			return Quat(
				sp_cos(angle * HALF_FLOAT),
				ZERO_FLOAT,
				ZERO_FLOAT,
				sp_sin(angle * HALF_FLOAT)
			);
		}

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_int index) const
		{
			sp_assert(index >= 0 && index < 4, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_uint index) const
		{
			sp_assert(index >= 0 && index < 4, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

#ifdef ENV_64BITS
		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](const sp_size index) const
		{
			sp_assert(index >= 0 && index < 4, "InvalidArgumentException");
			return ((sp_float*)this)[index];
		}
#endif

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE inline Quat operator+(const Quat& q) const
		{
			return Quat(
				w + q.w,
				x + q.x,
				y + q.y,
				z + q.z
			);
		}

		/// <summary>
		/// Add/Sum this quaternion to a scalar value
		/// </summary>
		API_INTERFACE inline Quat operator+(const sp_float value) const
		{
			return Quat(w + value, x, y, z);
		}

		/// <summary>
		/// Sum this quaternion to another one
		/// </summary>
		API_INTERFACE inline void operator+=(const Quat& q)
		{
			w += q.w;
			x += q.x;
			y += q.y;
			z += q.z;
		}

		/// <summary>
		/// Subtract two quaternions
		/// </summary>
		API_INTERFACE inline Quat operator-(const Quat& q) const
		{
			return Quat(
				w - q.w,
				x - q.x,
				y - q.y,
				z - q.z
			);
		}

		/// <summary>
		/// Subtract this quaternion to a scalar value
		/// </summary>
		API_INTERFACE inline Quat operator-(const sp_float value) const
		{
			return Quat(w - value, x, y, z);
		}

		/// <summary>
		/// Subtract this quaternion to another one
		/// </summary>
		API_INTERFACE inline void operator-=(const Quat& q)
		{
			w -= q.w;
			x -= q.x;
			y -= q.y;
			z -= q.z;
		}

		/// <summary>
		/// Negate the current quaternion
		/// </summary>
		API_INTERFACE inline Quat operator-() const
		{
			return Quat(-w, -x, -y, -z);
		}

		/// <summary>
		/// Multiply the quaternion to another one
		/// </summary>
		API_INTERFACE inline Quat operator*(const Quat& q) const
		{
			return Quat(
				(q.w * w) - (q.x * x) - (q.y * y) - (q.z * z),
				(q.w * x) + (q.x * w) - (q.y * z) + (q.z * y),
				(q.w * y) + (q.x * z) + (q.y * w) - (q.z * x),
				(q.w * z) - (q.x * y) + (q.y * x) + (q.z * w)
			);
		}

		/// <summary>
		/// Multiply the quaternion to a scalar
		/// Return a new quaternion scaled
		/// </summary>
		API_INTERFACE inline Quat operator*(const sp_float value) const
		{
			return scale(value);
		}

		/// <summary>
		/// Multiply the quaternion to a scalar
		/// </summary>
		API_INTERFACE inline void operator*=(const sp_float value)
		{
			w *= value;
			x *= value;
			y *= value;
			z *= value;
		}

		/// <summary>
		/// Rotate the current quaternion
		/// </summary>
		/// <param name="rotation">Rotation quaternion</param>
		/// <returns>void</returns>
		API_INTERFACE inline void operator*=(const Quat& rotation)
		{
			w = (w * rotation.w) - (x * rotation.x) - (y * rotation.y) - (z * rotation.z);
			x = (w * rotation.x) + (x * rotation.w) - (y * rotation.z) + (z * rotation.y);
			y = (w * rotation.y) + (x * rotation.z) + (y * rotation.w) - (z * rotation.x);
			z = (w * rotation.z) - (x * rotation.y) + (y * rotation.x) + (z * rotation.w);
		}

		/// <summary>
		/// Divide the quaternion by scalar
		/// </summary>
		API_INTERFACE Quat operator/(const sp_float value) const
		{
			const sp_float temp = NAMESPACE_FOUNDATION::div(ONE_FLOAT, value);

			return Quat(
				w * temp, 
				x * temp, 
				y * temp,
				z * temp
			);
		}
		
		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE inline operator void*() const
		{
			return (void*)this;
		}

		/// <summary>
		/// Auto convertion to sp_float*
		/// </summary>
		API_INTERFACE inline operator sp_float* () const
		{
			return (sp_float*)this;
		}

		/// <summary>
		/// Check quaterions are equal
		/// </summary>
		/// <param name="q">Other quaterion</param>
		/// <returns>True if they are equals orelse False</returns>
		API_INTERFACE sp_bool operator==(const Quat& q) const;
		
	};

	const Quat QuatUnit(ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT);
	const Quat QuatZeros(ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT);

	/// <summary>
	/// Check the quaternion "q1" is close enough quaternion "q2"
	/// </summary>
	/// <param name="q1">Quaternion 1</param>
	/// <param name="q2">Quaternion 2</param>
	/// <param name="_epsilon">Error Margin</param>
	/// <returns>True if they are close enough orelse False</returns>
	API_INTERFACE inline sp_bool isCloseEnough(const Quat& q1, const Quat& q2, const sp_float _epsilon = DefaultErrorMargin)
	{
		return NAMESPACE_FOUNDATION::isCloseEnough(q1.w, q2.w, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(q1.x, q2.x, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(q1.y, q2.y, _epsilon)
			&& NAMESPACE_FOUNDATION::isCloseEnough(q1.z, q2.z, _epsilon);
	}

	/// <summary>
	/// Check the quaternion is normalized
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <returns>True if the quaternion os normalized orelse False</returns>
	API_INTERFACE inline sp_bool isNormalized(const Quat& q)
	{
		return NAMESPACE_FOUNDATION::isCloseEnough(q.length(), ONE_FLOAT, SP_EPSILON_THREE_DIGITS);
	}

	/// <summary>
	/// Convert the quaternion "q" to Vec3
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <param name="output">Vector</param>
	/// <returns>output parameter</returns>
	API_INTERFACE void vec3(const Quat& q, Vec3& output);

	/// <summary>
	/// Build an unit quaterion
	/// </summary>
	/// <param name="output">Quaterion</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void identity(Quat& output)
	{
		output.w = ONE_FLOAT;
		output.x = ZERO_FLOAT;
		output.y = ZERO_FLOAT;
		output.z = ZERO_FLOAT;
	}

	/// <summary>
	/// Sum a quaternion by another one
	/// </summary>
	/// <param name="quat1">Quaternion 1</param>
	/// <param name="quat2">Quaternion 2</param>
	/// <param name="output">Result</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void add(const Quat& q1, const Quat& q2, Quat& output)
	{
		output.w = q1.w + q2.w;
		output.x = q1.x + q2.x;
		output.y = q1.y + q2.y;
		output.z = q1.z + q2.z;
	}

	/// <summary>
	/// Multiply a quaternion by a scalar value
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <param name="scalarValue">Scalar value</param>
	/// <param name="output">Inverted quaternion</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void multiply(const Quat& q, const sp_float scalarValue, Quat& output)
	{
		output.w = q.w * scalarValue;
		output.x = q.x * scalarValue;
		output.y = q.y * scalarValue;
		output.z = q.z * scalarValue;
	}

	/// <summary>
	/// Multiply a quaternion by another one
	/// </summary>
	/// <param name="quat1">Quaternion 1</param>
	/// <param name="quat2">Quaternion 2</param>
	/// <param name="output">Result</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void multiply(const Quat& q1, const Quat& q2, Quat& output)
	{
		output.w = (q2.w * q1.w) - (q2.x * q1.x) - (q2.y * q1.y) - (q2.z * q1.z);
		output.x = (q2.w * q1.x) + (q2.x * q1.w) - (q2.y * q1.z) + (q2.z * q1.y);
		output.y = (q2.w * q1.y) + (q2.x * q1.z) + (q2.y * q1.w) - (q2.z * q1.x);
		output.z = (q2.w * q1.z) - (q2.x * q1.y) + (q2.y * q1.x) + (q2.z * q1.w);
	}

	/// <summary>
	/// Divide a quaternion to scalar value
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <param name="value">Scalar value</param>
	/// <param name="output">Quaternion divided</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void div(const Quat& q, const sp_float value, Quat& output)
	{
		const sp_float temp = NAMESPACE_FOUNDATION::div(ONE_FLOAT, value);
		output.w = q.w * temp;
		output.x = q.x * temp;
		output.y = q.y * temp;
		output.z = q.z * temp;
	}

	API_INTERFACE inline void normalize(const Quat& q, Quat& output)
	{
		sp_float magnitude = q.length();

		sp_assert(magnitude != ZERO_FLOAT, "InvalidArgumentException");

		magnitude = NAMESPACE_FOUNDATION::div(ONE_FLOAT, magnitude);

		output.w = q.w * magnitude;
		output.x = q.x * magnitude;
		output.y = q.y * magnitude;
		output.z = q.z * magnitude;
	}

	API_INTERFACE inline void normalize(Quat& q)
	{
		normalize(q, q);
	}
	
	/// <summary>
	/// Create a conjugated quaternion
	/// </summary>
	/// <param name="input">Quaterion</param>
	/// <param name="output">Result</param>
	/// <returns>void</returns>
	API_INTERFACE inline void conjugate(const Quat& input, Quat& output)
	{
		output.w = input.w;
		output.x = -input.x;
		output.y = -input.y;
		output.z = -input.z;
	}

	/// <summary>
	/// Cross Product of two quaternion
	/// </summary>
	API_INTERFACE inline void cross(const Quat& q1, const Quat& q2, Quat& output)
	{
		multiply(q1, q2, output);
	}

	API_INTERFACE void rotate(const Quat& axis, const Vec3& point, Vec3& output);

	/// <summary>
	/// Create a new quanternion from euler angles provided
	/// </summary>
	API_INTERFACE inline void fromEulerAngles(const sp_float roll, const sp_float pitch, const sp_float yaw, Quat& output)
	{
		const sp_float cosRoll = sp_cos(roll * HALF_FLOAT);
		const sp_float sinRoll = sp_sin(roll * HALF_FLOAT);
		const sp_float cosPitch = sp_cos(pitch * HALF_FLOAT);
		const sp_float sinPitch = sp_sin(pitch * HALF_FLOAT);
		const sp_float cosYaw = sp_cos(yaw * HALF_FLOAT);
		const sp_float sinYaw = sp_sin(yaw * HALF_FLOAT);

		output.w = cosYaw * cosPitch * cosRoll - sinYaw * sinPitch * sinRoll;
		output.x = cosYaw * cosPitch * sinRoll + sinYaw * sinPitch * cosRoll;
		output.y = cosYaw * sinPitch * cosRoll - sinYaw * cosPitch * sinRoll;
		output.z = sinYaw * cosPitch * cosRoll + cosYaw * sinPitch * sinRoll;
	}

	/// <summary>
	/// Convert the quaternions to angles (x, y, z)
	/// The quaternion has to be normalized !
	/// </summary>
	API_INTERFACE void eulerAnglesXYZ(const Quat& q, Vec3& output);

	/// <summary>
	/// Convert the quaternions to angles (z, y, x)
	/// The quaternion has to be normalized !
	/// </summary>
	API_INTERFACE void eulerAnglesZYX(const Quat& q, Vec3& output);

	/// <summary>
	/// Get the angle (radians) between two quaternions
	/// </summary>
	/// <param name="q1">Quaternion 1</param>
	/// <param name="q2">Quaternion 2</param>
	/// <returns>Angle (radians)</returns>
	API_INTERFACE inline sp_float angle(const Quat& q1, const Quat& q2)
	{
		Quat q3;
		conjugate(q2, q3);

		return sp_arccos((q1 * q3).w) * TWO_FLOAT;
	}

	/// <summary>
	/// Craete a Inversed Quaternion
	/// Return a quaternion that if multiplied by current quaternion results in value 1 of the current quternion ifs length/norm is zero
	/// </summary>
	API_INTERFACE inline void inverse(const Quat& q, Quat& output)
	{
		sp_assert(q.length() != ZERO_FLOAT, "InvalidOperationException");

		conjugate(q, output);

		const sp_float magnitude = NAMESPACE_FOUNDATION::div(ONE_FLOAT, (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w));

		multiply(output, magnitude, output);

		sp_assert(q * output == QuatUnit, "ApplicationException");
	}

	/// <summary>
	/// Craete a Inversed Quaternion
	/// Return a quaternion that if multiplied by current quaternion results in value 1 of the current quternion ifs length/norm is zero
	/// </summary>
	API_INTERFACE inline void inverse(Quat& q)
	{
		sp_assert(q.length() != ZERO_FLOAT, "InvalidOperationException");

		conjugate(q, q);

		const sp_float magnitude = NAMESPACE_FOUNDATION::div(ONE_FLOAT, (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w));

		multiply(q, magnitude, q);

		sp_assert(q.length() == ONE_FLOAT, "ApplicationException");
	}

	/// <summary>
	/// Rotate the point provided by this quaternion axis
	/// This quaternion is the rotation axis
	/// Returns the point rotated
	/// </summary>
	API_INTERFACE void rotate(const Quat& rotation, const Vec3& point, Vec3& output);

	/// <summary>
	/// Quaternion Linear Interpolation. 
	/// This method is the quickest, but is also least accurate. 
	/// The method does not always generate normalized output.
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <param name="t">Interval parameter should be in [0,1]</param>
	/// <param name="output">Output quaternion</param>
	/// <returns>output parameter</returns>
	API_INTERFACE inline void lerp(const Quat& q1, const Quat& q2, const sp_float t, Quat& output)
	{
		sp_assert(isNormalized(q1), "InvalidArgumentException");
		sp_assert(isNormalized(q2), "InvalidArgumentException");
		sp_assert(t >= ZERO_FLOAT && t <= ONE_FLOAT, "InvalidArgumentException");
		
		Quat temp;
		multiply(q1, ONE_FLOAT - t, temp);

		multiply(q2, t, output);

		add(temp, output, output);
	}

	/// <summary>
	/// Spherical quaternion interpolation method
	/// Both of quaternions should be normalized
	/// t parameter is [0,1]
	/// </summary>
	API_INTERFACE inline void slerp(const Quat& q1, const Quat& q2, const sp_float t, Quat& output)
	{
		sp_assert(isNormalized(q1), "InvalidArgumentException");
		sp_assert(isNormalized(q2), "InvalidArgumentException");

		Quat copyQuat2;
		sp_float cosTheta = q1.dot(q2);

		// If cosTheta < 0, the interpolation will take the long way around the sphere.
		// To fix this, one quat must be negated.
		if (cosTheta < ZERO_FLOAT)
		{
			copyQuat2 = -q2;
			cosTheta = -cosTheta;
		}
		else
			copyQuat2 = q2;

		// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
		if (NAMESPACE_FOUNDATION::isCloseEnough(cosTheta, ONE_FLOAT))
		{
			output.w = NAMESPACE_FOUNDATION::lerp(q1.w, copyQuat2.w, t);
			output.x = NAMESPACE_FOUNDATION::lerp(q1.x, copyQuat2.x, t);
			output.y = NAMESPACE_FOUNDATION::lerp(q1.y, copyQuat2.y, t);
			output.z = NAMESPACE_FOUNDATION::lerp(q1.z, copyQuat2.z, t);
			return;
		}
		else
		{
			const sp_float angle = sp_arccos(cosTheta);

			Quat temp;
			multiply(q1, sp_sin((ONE_FLOAT - t) * angle), temp);

			multiply(copyQuat2, sp_sin(t * angle), output);

			add(temp, output, output);

			div(output, sp_sin(angle), output);
		}
	}

	/// <summary>
	/// Spherical quaternion interpolation method
	/// Both of quaternions should be normalized
	/// t parameter is [0,1]
	/// </summary>
	API_INTERFACE inline void slerp(const Quat& q1, const Quat& q2, sp_float t, sp_int spinCount, Quat& output)
	{
		sp_assert(isNormalized(q1), "InvalidArgumentException");
		sp_assert(isNormalized(q2), "InvalidArgumentException");

		Quat copyQuat2;
		sp_float cosTheta = q1.dot(q2);

		// If cosTheta < 0, the interpolation will take the long way around the sphere.
		// To fix this, one quat must be negated.
		if (cosTheta < ZERO_FLOAT)
		{
			copyQuat2 = -q2;
			cosTheta = -cosTheta;
		}
		else
			copyQuat2 = q2;

		// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
		if (NAMESPACE_FOUNDATION::isCloseEnough(cosTheta, ONE_FLOAT))
		{
			output.w = NAMESPACE_FOUNDATION::lerp(q1.w, copyQuat2.w, t);
			output.x = NAMESPACE_FOUNDATION::lerp(q1.x, copyQuat2.x, t);
			output.y = NAMESPACE_FOUNDATION::lerp(q1.y, copyQuat2.y, t);
			output.z = NAMESPACE_FOUNDATION::lerp(q1.z, copyQuat2.z, t);
			return;
		}
		else
		{
			const sp_float angle = sp_arccos(cosTheta);
			const sp_float phiT = (angle + spinCount * PI) * t;

			Quat temp;
			multiply(copyQuat2, sp_sin(phiT), temp);

			multiply(q1, sp_sin(angle - phiT), output);

			add(temp, output, output);

			div(output, sp_sin(angle), output);
		}
	}

	/// <summary>
	/// Convertion to rotational matrix 3x3
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <param name="output">Output matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE void mat3(const Quat& q, Mat3& output);

	/// <summary>
	/// Convertion to rotational matrix 4x4
	/// </summary>
	/// <param name="q">Quaternion</param>
	/// <param name="output">Output matrix</param>
	/// <returns>output parameter</returns>
	API_INTERFACE void mat4(const Quat& q, Mat4& output);

}

#endif // QUAT_HEADER