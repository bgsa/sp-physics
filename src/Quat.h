#ifndef QUAT_HEADER
#define QUAT_HEADER

#include "SpectrumPhysics.h"
#include "SpSIMD.h"

namespace NAMESPACE_PHYSICS
{
#define QUAT_LENGTH 4
#define QUAT_SIZE (QUAT_LENGTH * SIZEOF_FLOAT)

	class Quat
	{
	public:
		sp_float w, x, y, z;

		/// <summary>
		/// Default constructor
		/// Load a empty quaternion = 0
		/// </summary>
#pragma warning(push, 0)
#pragma warning(disable : 26495)
		API_INTERFACE Quat()
		{
			static sp_float identityQuaternion[QUAT_LENGTH] = {
				ONE_FLOAT,
				ZERO_FLOAT,
				ZERO_FLOAT,
				ZERO_FLOAT
			};

			std::memcpy(this, identityQuaternion, QUAT_SIZE);
		}
#pragma warning(pop)

		/// <summary>
		/// Default constructor
		/// Load a empty quaternion = 0
		/// </summary>
		API_INTERFACE Quat(const sp_float value)
		{
			w = value;
			x = value;
			y = value;
			z = value;
		}

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
#pragma warning(push, 0)
#pragma warning(disable : 26495)
		API_INTERFACE Quat(sp_float* values)
		{
			std::memcpy(this, values, QUAT_SIZE);
		}
#pragma warning(pop)

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE Quat(const sp_float w, const sp_float x, const sp_float y, const sp_float z)
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
		/// Create a new quanternion from euler angles provided
		/// </summary>
		API_INTERFACE static Quat fromEulerAngles(const sp_float roll, const sp_float pitch, const sp_float yaw)
		{
			const sp_float cosRoll = sp_cos(roll * HALF_FLOAT);
			const sp_float sinRoll = sp_sin(roll * HALF_FLOAT);
			const sp_float cosPitch = sp_cos(pitch * HALF_FLOAT);
			const sp_float sinPitch = sp_sin(pitch * HALF_FLOAT);
			const sp_float cosYaw = sp_cos(yaw * HALF_FLOAT);
			const sp_float sinYaw = sp_sin(yaw * HALF_FLOAT);

			return Quat(
				cosYaw * cosPitch * cosRoll - sinYaw * sinPitch * sinRoll,
				cosYaw * cosPitch * sinRoll + sinYaw * sinPitch * cosRoll,
				cosYaw * sinPitch * cosRoll - sinYaw * cosPitch * sinRoll,
				sinYaw * cosPitch * cosRoll + cosYaw * sinPitch * sinRoll
			);
		}

		/// <summary>
		/// Convert the quaternions to angles (x, y, z)
		/// The quaternion has to be normalized !
		/// </summary>
		API_INTERFACE Vec3 toEulerAngles() const;

		/// <summary>
		/// Constructor with a 3D vector
		/// </summary>
		API_INTERFACE static Quat identity()
		{
			return Quat();
		}

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE inline Quat add(const Quat& quatB) const
		{
			return Quat(
				w + quatB.w,
				x + quatB.x,
				y + quatB.y,
				z + quatB.z
			);
		}

		/// <summary>
		/// Subtract two quaternions
		/// </summary>
		API_INTERFACE inline Quat subtract(const Quat& quatB) const
		{
			return Quat(
				w - quatB.w,
				x - quatB.x,
				y - quatB.y,
				z - quatB.z
			);
		}

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
		/// Get the angle from this quaternion
		/// </summary>
		API_INTERFACE inline sp_float angle() const
		{
			// TODO: TESTS !!!!
			if (sp_abs(w) > sp_cos(0.5f))
				return sp_arcsin(sp_sqrt(x * x + y * y + z * z)) * TWO_FLOAT;

			return acos(w) * TWO_FLOAT;
		}

		/// <summary>
		/// Get the rotation axis from this quaternion
		/// </summary>
		API_INTERFACE Vec3 axis() const;

		/// <summary>
		/// Product Scalar of two quaternion
		/// </summary>
		API_INTERFACE inline sp_float dot(const Quat& quatB) const
		{
			return (w * quatB.w) + (x * quatB.x) + (y * quatB.y) + (z * quatB.z);
		}

		/// <summary>
		/// Cross Product of two quaternion
		/// </summary>
		API_INTERFACE inline Quat cross(const Quat& quatB) const
		{
			return *this * quatB;
		}

		/// <summary>
		/// Craete a Inversed Quaternion
		/// Return a quaternion that if multiplied by current quaternion results in value 1 ot the current quternion ifs length/norm is zero
		/// </summary>
		API_INTERFACE inline Quat inverse() const;

		/// <summary>
		/// Rotate the point provided by this quaternion axis
		/// This quaternion is the rotation axis
		/// Returns the point rotated
		/// </summary>
		API_INTERFACE Vec3 rotate(const Vec3& point) const;

		/// <summary>
		/// Create a rotation unit quaternion bases on angle (in radians) and directional vector provided
		/// </summary>
		API_INTERFACE static Quat createRotate(sp_float angle, const Vec3& axis);

		/// <summary>
		/// Create a quaternion rotation around X axis
		/// </summary>
		API_INTERFACE static Quat createRotationAxisX(sp_float angle)
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
		API_INTERFACE static Quat createRotationAxisY(sp_float angle)
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
		API_INTERFACE static Quat createRotationAxisZ(sp_float angle)
		{
			return Quat(
				sp_cos(angle * HALF_FLOAT),
				ZERO_FLOAT,
				ZERO_FLOAT,
				sp_sin(angle * HALF_FLOAT)
			);
		}
		
		/// <summary>
		/// Quaternion Linear Interpolation. This method is the quickest, but is also least accurate. The method does not always generate normalized output.
		/// t parameter is [0,1]
		/// </summary>
		API_INTERFACE inline Quat lerp(const Quat& quatB, sp_float t) const
		{
			return scale(ONE_FLOAT - t) + quatB.scale(t);
		}

		/// <summary>
		/// Spherical quaternion interpolation method
		/// Both of quaternions should be normalized
		/// t parameter is [0,1]
		/// </summary>
		API_INTERFACE Quat slerp(const Quat& quatB, sp_float t) const;

		/// <summary>
		/// Spherical quaternion interpolation method
		/// Both of quaternions should be normalized
		/// t parameter is [0,1]
		/// </summary>
		API_INTERFACE Quat slerp(const Quat& quatB, sp_float t, sp_int spinCount) const;

		/// <summary>
		/// Convertion to Vec3
		/// </summary>
		API_INTERFACE Vec3 toVec3() const;

		/// <summary>
		/// Convertion to rotational matrix 3x3
		/// </summary>
		API_INTERFACE Mat3 toMat3() const;

		/// <summary>
		/// Convertion to rotational and translation matrix 4x4
		/// </summary>
		API_INTERFACE Mat4 toMat4(const Vec3& position) const;

		/// <summary>
		/// Convertion to rotational matrix 4x4
		/// </summary>
		API_INTERFACE Mat4 toMat4() const;

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_int index) const
		{
			sp_assert(index >= ZERO_INT && index < QUAT_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_uint index) const
		{
			sp_assert(index >= ZERO_UINT && index < QUAT_LENGTH, "IndexOutOfrangeException");
			return ((sp_float*)this)[index];
		}

#ifdef ENV_64BITS
		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_size index) const;
#endif

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE inline Quat operator+(const Quat& quatB) const
		{
			return add(quatB);
		}

		/// <summary>
		/// Sum this quaternion to another one
		/// </summary>
		API_INTERFACE inline void operator+=(const Quat& quaternion)
		{
			w += quaternion.w;
			x += quaternion.x;
			y += quaternion.y;
			z += quaternion.z;
		}

		/// <summary>
		/// Sibtract two quaternions
		/// </summary>
		API_INTERFACE inline Quat operator-(const Quat& quatB) const
		{
			return subtract(quatB);
		}

		/// <summary>
		/// Subtract this quaternion to another one
		/// </summary>
		API_INTERFACE inline void operator-=(const Quat& quaternion)
		{
			w -= quaternion.w;
			x -= quaternion.x;
			y -= quaternion.y;
			z -= quaternion.z;
		}

		/// <summary>
		/// Subtract the current quaternion
		/// </summary>
		API_INTERFACE inline Quat operator-() const
		{
			return Quat(-w, -x, -y, -z);
		}

		/// <summary>
		/// Multiply the quaternion to another one
		/// </summary>
		API_INTERFACE inline Quat operator*(const Quat& quat) const
		{
			return Quat(
				(w * quat.w) - (x * quat.x) - (y * quat.y) - (z * quat.z),
				(w * quat.x) + (x * quat.w) - (y * quat.z) + (z * quat.y),
				(w * quat.y) + (x * quat.z) + (y * quat.w) - (z * quat.x),
				(w * quat.z) - (x * quat.y) + (y * quat.x) + (z * quat.w)
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
			const sp_float temp = ONE_FLOAT / value;
			return Quat(w*temp, x  * temp, y*temp, z*temp);
		}
		
		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE inline operator void*() const
		{
			return (void*)this;
		}

		/// <summary>
		/// Auto convertion to Vec3
		/// </summary>
		API_INTERFACE operator Vec3() const;

		API_INTERFACE inline sp_bool isCloseEnough(const Quat& compare, const sp_float _epsilon = DefaultErrorMargin) const
		{
			return sp_abs(w - compare.w) <= _epsilon
				&& sp_abs(x - compare.x) <= _epsilon
				&& sp_abs(y - compare.y) <= _epsilon
				&& sp_abs(z - compare.z) <= _epsilon;
		}
	};

	const Quat QUAT_UNIT(ONE_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT);
	const Quat QUAT_ZERO(ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT);

	API_INTERFACE inline void multiply(const Quat& quat1, const Quat& quat2, Quat* output)
	{
		output->w = (quat1.w * quat2.w) - (quat1.x * quat2.x) - (quat1.y * quat2.y) - (quat1.z * quat2.z);
		output->x = (quat1.w * quat2.x) + (quat1.x * quat2.w) - (quat1.y * quat2.z) + (quat1.z * quat2.y);
		output->y = (quat1.w * quat2.y) + (quat1.x * quat2.z) + (quat1.y * quat2.w) - (quat1.z * quat2.x);
		output->z = (quat1.w * quat2.z) - (quat1.x * quat2.y) + (quat1.y * quat2.x) + (quat1.z * quat2.w);
	}
	API_INTERFACE void multiply(const Quat& quat1, const Quat& quat2, Vec3* output);
	API_INTERFACE void multiply(const Vec3& vector, const Quat& quat, Quat* output);
	API_INTERFACE void multiplyAndSum(const Quat& quat1, const Quat& quat2, const Vec3& sumVector, Vec3* output);

	API_INTERFACE inline void normalize(Quat* quat)
	{
		sp_float magnitude = quat->length();

		sp_assert(magnitude != ZERO_FLOAT, "InvalidArgumentException");

		magnitude = ONE_FLOAT / magnitude;

		quat->w *= magnitude;
		quat->x *= magnitude;
		quat->y *= magnitude;
		quat->z *= magnitude;
	}
	
	/// <summary>
	/// Create a conjugated quaternion
	/// </summary>
	/// <param name="input">Quaterion</param>
	/// <param name="output">Result</param>
	/// <returns>void</returns>
	API_INTERFACE inline void conjugate(const Quat& input, Quat* output)
	{
		output->w = input.w;
		output->x = -input.x;
		output->y = -input.y;
		output->z = -input.z;
	}

	API_INTERFACE void rotate(const Quat& rotation, const Vec3& point, Vec3* output);
	API_INTERFACE void rotateAndTranslate(const Quat& rotation, const Vec3& point, const Vec3& translation, Vec3* output);

}

#endif // QUAT_HEADER