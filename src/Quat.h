#ifndef QUAT_HEADER
#define QUAT_HEADER

#include "SpectrumPhysics.h"
#include "Vec3.h"

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
		API_INTERFACE Quat(const Vec3<sp_float>& vector);

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
		API_INTERFACE void scale(sp_float value);

		/// <summary>
		/// Create a new quaternion scaled
		/// </summary>
		API_INTERFACE Quat createScale(sp_float value) const;

		/// <summary>
		/// Scale a quaternion
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
		/// Product Scalar of two quaternion
		/// </summary>
		API_INTERFACE sp_float dot(Quat quatB) const;

		/// <summary>
		/// Craete a Inversed Quaternion
		/// Return a quaternion that if multiplied by current quaternion results in value 1 ot the current quternion ifs length/norm is zero
		/// </summary>
		API_INTERFACE Quat inverse() const;

		/// <summary>
		/// Craete a rotation unit quaternion bases on angle (in radians) and directional vector provided
		/// </summary>
		API_INTERFACE static Quat createRotate(sp_float angleInRadians, const Vec3<sp_float>& position);

		/// <summary>
		/// Return a quaternion rotated bases on rotation quaternion provided in parameter
		/// The parameter can/must be used with createRotation static method
		/// </summary>
		API_INTERFACE Quat rotate(const Quat& r) const;

		/// <summary>
		/// Return a quaternion rotated bases on angle and a directional vector
		/// </summary>
		API_INTERFACE Quat rotate(sp_float angle, const Vec3<sp_float>& vector) const;

		/// <summary>
		/// Quaternion lerp. Linear quaternion interpolation method. This method is the quickest, but is also least accurate. The method does not always generate normalized output.
		/// t parameter is [0,1]
		/// </summary>
		API_INTERFACE Quat linearInterpolate(const Quat& quatB, sp_float t) const;

		/// <summary>
		/// Quaternion nlerp. Linear quaternion interpolation method. This method is the quickest, but is also least accurate.
		/// This method normalize the result
		/// t parameter is [0,1]
		/// </summary>
		API_INTERFACE Quat linearInterpolateNormalized(const Quat& quatB, sp_float t) const;

		/// <summary>
		/// Get the size in Bytes of Quaternion
		/// </summary>
		API_INTERFACE sp_size sizeInBytes() const;

		/// <summary>
		/// Convertion to Vec3
		/// </summary>
		API_INTERFACE Vec3<sp_float> toVec3() const;

		/// <summary>
		/// Get a index from the quaternion
		/// </summary>
		API_INTERFACE sp_float operator[](sp_int index);

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE Quat operator+(const Quat& quatB) const;

		/// <summary>
		/// Add/Sum two quaternions
		/// </summary>
		API_INTERFACE Quat operator-(const Quat& quatB) const;

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
		/// Auto convertion to void *
		/// </summary>
		API_INTERFACE operator void*() const;

		/// <summary>
		/// Auto convertion to Vec3
		/// </summary>
		API_INTERFACE operator Vec3<sp_float>() const;
	};
}

#endif // QUAT_HEADER