#ifndef QUAT_HEADER
#define QUAT_HEADER

#include "SpectrumPhysics.h"

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
		API_INTERFACE Quat();

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE Quat(sp_float* values);

		/// <summary>
		/// Constructor with initialized values
		/// </summary>
		API_INTERFACE Quat(sp_float w, sp_float x, sp_float y, sp_float z);

		/// <summary>
		/// Constructor with a 3D vector
		/// </summary>
		API_INTERFACE Quat(const Vec3& vector);

		/// <summary>
		/// Create a new quanternion from euler angles provided
		/// </summary>
		API_INTERFACE static Quat fromEulerAngles(sp_float roll, sp_float pitch, sp_float yaw);

		/// <summary>
		/// Convert the quaternions to angles (x, y, z)
		/// The quaternion has to be normalized !
		/// </summary>
		API_INTERFACE Vec3 toEulerAngles() const;

		/// <summary>
		/// Constructor with a 3D vector
		/// </summary>
		API_INTERFACE static Quat identity();

		/// <summary>
		/// Get the values from current quaternion
		/// </summary>
		API_INTERFACE sp_float* values();

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE Quat add(const Quat& quatB) const;

		/// <summary>
		/// Subtract two quaternions
		/// </summary>
		API_INTERFACE Quat subtract(const Quat& quatB) const;

		/// <summary>
		/// Scale a quaternion
		/// </summary>
		API_INTERFACE Quat scale(sp_float value) const;

		/// <summary>
		/// Multiply / Cross Product of quaternions
		/// </summary>
		API_INTERFACE Quat multiply(const Quat& quat) const;

		/// <summary>
		/// Length/Magnitude of quaternion
		/// </summary>
		API_INTERFACE sp_float length() const;

		/// <summary>
		/// Craete a new Quaternion Normalized
		/// </summary>
		API_INTERFACE Quat normalize() const;

		/// <summary>
		/// Craete a new Quaternion Conjugated
		/// </summary>
		API_INTERFACE Quat conjugate() const;

		/// <summary>
		/// Get the angle from this quaternion
		/// </summary>
		API_INTERFACE sp_float angle() const;

		/// <summary>
		/// Get the rotation axis from this quaternion
		/// </summary>
		API_INTERFACE Vec3 axis() const;

		/// <summary>
		/// Product Scalar of two quaternion
		/// </summary>
		API_INTERFACE sp_float dot(const Quat& quatB) const;

		/// <summary>
		/// Cross Product of two quaternion
		/// </summary>
		API_INTERFACE Quat cross(const Quat& quatB) const;

		/// <summary>
		/// Craete a Inversed Quaternion
		/// Return a quaternion that if multiplied by current quaternion results in value 1 ot the current quternion ifs length/norm is zero
		/// </summary>
		API_INTERFACE Quat inverse() const;

		/// <summary>
		/// Rotate the point provided by this quaternion axis
		/// This quaternion is the rotation axis
		/// Returns the point rotated
		/// </summary>
		API_INTERFACE Vec3 rotate(const Vec3& point) const;

		/// <summary>
		/// Craete a rotation unit quaternion bases on angle (in radians) and directional vector provided
		/// </summary>
		API_INTERFACE static Quat createRotate(sp_float angle, const Vec3& axis);

		/// <summary>
		/// Create a quaternion rotation around X axis
		/// </summary>
		API_INTERFACE static Quat createRotationAxisX(sp_float angle)
		{
			return Quat(
				std::cosf(angle / TWO_FLOAT),
				std::sinf(angle / TWO_FLOAT),
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
				std::cosf(angle / TWO_FLOAT),
				ZERO_FLOAT,
				std::sinf(angle / TWO_FLOAT),
				ZERO_FLOAT
			);
		}

		/// <summary>
		/// Create a quaternion rotation around Z axis
		/// </summary>
		API_INTERFACE static Quat createRotationAxisZ(sp_float angle)
		{
			return Quat(
				std::cosf(angle * HALF_FLOAT),
				ZERO_FLOAT,
				ZERO_FLOAT,
				std::sinf(angle * HALF_FLOAT)
			);
		}
		
		/// <summary>
		/// Quaternion Linear Interpolation. This method is the quickest, but is also least accurate. The method does not always generate normalized output.
		/// t parameter is [0,1]
		/// </summary>
		API_INTERFACE Quat lerp(const Quat& quatB, sp_float t) const;

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
		API_INTERFACE inline Vec3 toVec3() const;

		/// <summary>
		/// Convertion to rotational matrix 3x3
		/// </summary>
		API_INTERFACE inline Mat3 toMat3() const;

		/// <summary>
		/// Convertion to rotational and translation matrix 4x4
		/// </summary>
		API_INTERFACE inline Mat4 toMat4(const Vec3& position) const;

		/// <summary>
		/// Convertion to rotational matrix 4x4
		/// </summary>
		API_INTERFACE inline Mat4 toMat4() const;

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_int index) const;

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_uint index) const;

#ifdef ENV_64BITS
		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE inline sp_float operator[](sp_size index) const;
#endif

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE Quat operator+(const Quat& quatB) const;

		/// <summary>
		/// Sibtract two quaternions
		/// </summary>
		API_INTERFACE Quat operator-(const Quat& quatB) const;

		/// <summary>
		/// Subtract the current quaternion
		/// </summary>
		API_INTERFACE Quat operator-() const;

		/// <summary>
		/// Multiply the quaternion to another one
		/// </summary>
		API_INTERFACE Quat operator*(const Quat& quat) const;

		/// <summary>
		/// Multiply the quaternion to a scalar
		/// Return a new quaternion scaled
		/// </summary>
		API_INTERFACE Quat operator*(sp_float value) const;

		/// <summary>
		/// Divide the quaternion by scalar
		/// </summary>
		API_INTERFACE Quat operator/(sp_float value) const;
		
		/// <summary>
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to Vec3
		/// </summary>
		API_INTERFACE operator Vec3() const;
	};
}

#endif // QUAT_HEADER