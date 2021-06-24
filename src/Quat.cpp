#include "Quat.h"

namespace NAMESPACE_PHYSICS
{

	Quat::Quat(const Vec3& vector)
	{
		w = ONE_FLOAT;
		x = vector.x;
		y = vector.y;
		z = vector.z;
	}

	Quat::Quat(const sp_float w, const Vec3& vector)
	{
		this->w = w;
		x = vector.x;
		y = vector.y;
		z = vector.z;
	}

	sp_bool Quat::operator==(const Quat& q) const
	{
		return isCloseEnough(*this, q);
	}

	Vec3 Quat::axis() const
	{
		// TODO: TESTS !!!!
		const sp_float tmp1 = ONE_FLOAT - w * w;

		if (tmp1 <= ZERO_FLOAT)
			return Vec3Zeros;

		const sp_float tmp2 = NAMESPACE_FOUNDATION::div(ONE_FLOAT, sp_sqrt(tmp1));

		return Vec3(x * tmp2, y * tmp2, z * tmp2);
	}

	Quat Quat::createRotate(const sp_float angle, const Vec3& axis)
	{
		sp_assert((axis.x <= ONE_FLOAT + SP_EPSILON_THREE_DIGITS
			&& axis.y <= ONE_FLOAT + SP_EPSILON_THREE_DIGITS
			&& axis.z <= ONE_FLOAT + SP_EPSILON_THREE_DIGITS), "InvalidArgumentException");

		const sp_float halfAngle = angle * HALF_FLOAT;
		const sp_float sinHalfAngle = sp_sin(halfAngle);

		return Quat(
			sp_cos(halfAngle),
			sinHalfAngle * axis.x,
			sinHalfAngle * axis.y,
			sinHalfAngle * axis.z
		);
	}

	void mat3(const Quat& q, Mat3& output)
	{
		const sp_float ww = q.w * q.w;
		const sp_float xx = q.x * q.x;
		const sp_float yy = q.y * q.y;
		const sp_float zz = q.z * q.z;

		// invs (inverse square length) is only required if quaternion is not already normalised
		const sp_float invs = NAMESPACE_FOUNDATION::div(ONE_FLOAT, (xx + yy + zz + ww));

		// row,col
		output.m11 = (xx - yy - zz + ww) * invs; // since sqw + sqx + sqy + sqz = 1 / invs*invs
		output.m22 = (-xx + yy - zz + ww) * invs;
		output.m33 = (-xx - yy + zz + ww) * invs;

		sp_float tmp1 = q.x * q.y;
		sp_float tmp2 = q.z * q.w;
		output.m21 = TWO_FLOAT * (tmp1 + tmp2) * invs;
		output.m12 = TWO_FLOAT * (tmp1 - tmp2) * invs;

		tmp1 = q.x * q.z;
		tmp2 = q.y * q.w;
		output.m31 = TWO_FLOAT * (tmp1 - tmp2) * invs;
		output.m13 = TWO_FLOAT * (tmp1 + tmp2) * invs;

		tmp1 = q.y * q.z;
		tmp2 = q.x * q.w;
		output.m32 = TWO_FLOAT * (tmp1 + tmp2)*invs;
		output.m23 = TWO_FLOAT * (tmp1 - tmp2)*invs;
	}

	void mat4(const Quat& q, Mat4& output)
	{
		const sp_float ww = q.w * q.w;
		const sp_float xx = q.x * q.x;
		const sp_float yy = q.y * q.y;
		const sp_float zz = q.z * q.z;

		// invs (inverse square length) is only required if quaternion is not already normalised
		const sp_float invs = NAMESPACE_FOUNDATION::div(ONE_FLOAT, (xx + yy + zz + ww));

		std::memcpy(output, Mat4Identity, sizeof(Mat4));

		// row,col
		output.m11 = (xx - yy - zz + ww) * invs; // since sqw + sqx + sqy + sqz = 1 / invs*invs
		output.m22 = (-xx + yy - zz + ww) * invs;
		output.m33 = (-xx - yy + zz + ww) * invs;

		sp_float tmp1 = q.x * q.y;
		sp_float tmp2 = q.z * q.w;
		output.m21 = TWO_FLOAT * (tmp1 + tmp2) * invs;
		output.m12 = TWO_FLOAT * (tmp1 - tmp2) * invs;

		tmp1 = q.x * q.z;
		tmp2 = q.y * q.w;
		output.m31 = TWO_FLOAT * (tmp1 - tmp2) * invs;
		output.m13 = TWO_FLOAT * (tmp1 + tmp2) * invs;

		tmp1 = q.y * q.z;
		tmp2 = q.x * q.w;
		output.m32 = TWO_FLOAT * (tmp1 + tmp2) * invs;
		output.m23 = TWO_FLOAT * (tmp1 - tmp2) * invs;
	}

	void eulerAnglesXYZ(const Quat& q, Vec3& output)
	{
		const sp_float ww = q.w * q.w;
		const sp_float xx = q.x * q.x;
		const sp_float yy = q.y * q.y;
		const sp_float zz = q.z * q.z;

		/*
		const sp_float unit = xx + yy + zz + ww; // if normalised is one, otherwise is correction factor
		const sp_float test = q.x * q.y + q.z * q.w;

		if (test > 0.499f * unit) 
		{ // singularity at north pole
			output.x = ZERO_FLOAT;
			output.y = HALF_PI;
			output.z = TWO_FLOAT * sp_arctan2(q.x, q.w); 
			return;
		}

		if (test < -0.499f * unit) 
		{ // singularity at south pole
			TEST !!!
			output.x = -TWO_FLOAT * sp_arctan2(q.x, q.w);
			output.y = -HALF_PI;
			output.z = ZERO_FLOAT;
			return;
		}
		*/

		output.x = sp_arctan2(TWO_FLOAT * (q.x * q.w - q.y * q.z), ww - xx - yy + zz);
		output.y = sp_arcsin(TWO_FLOAT * (q.x * q.z + q.w * q.y));
		output.z = sp_arctan2(-TWO_FLOAT * (q.x * q.y - q.w * q.z), ww + xx - yy - zz);
	}
	
	void eulerAnglesZYX(const Quat& q, Vec3& output)
	{
		const sp_float xx = q.x * q.x;
		const sp_float yy = q.y * q.y;
		const sp_float zz = q.z * q.z;
		const sp_float unit = xx + yy + zz + (q.w * q.w); // if normalised is one, otherwise is correction factor
		const sp_float test = q.x * q.y + q.z * q.w;

		// roll (x-axis rotation)
		const sp_float sinr_cosp = TWO_FLOAT * (q.w * q.z + q.x * q.y);
		const sp_float cosr_cosp = ONE_FLOAT - TWO_FLOAT * (yy + zz);
		output.x = sp_arctan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		const sp_float sinp = TWO_FLOAT * q.w * q.y - TWO_FLOAT * q.z * q.x;
		if (sp_abs(sinp) >= ONE_FLOAT)
			output.y = std::copysign(HALF_PI, sinp); // use 90 degrees if out of range
		else
			output.y = std::asin(sinp);

		// yaw (z-axis rotation)
		const sp_float siny_cosp = TWO_FLOAT * (q.w * q.x + q.y * q.z);
		const sp_float cosy_cosp = ONE_FLOAT - TWO_FLOAT * (xx + yy);
		output.z = sp_arctan2(siny_cosp, cosy_cosp);
	}

	void vec3(const Quat& q, Vec3& output)
	{
		output.x = q.x;
		output.y = q.y;
		output.z = q.z;
	}

	void rotate(const Quat& rotation, const Vec3& point, Vec3& output)
	{
		Quat conjugated;
		conjugate(rotation, conjugated);

		Quat q1;
		q1.w = rotation.w - (point.x * rotation.x) - (point.y * rotation.y) - (point.z * rotation.z);
		q1.x = rotation.x + (point.x * rotation.w) - (point.y * rotation.z) + (point.z * rotation.y);
		q1.y = rotation.y + (point.x * rotation.z) + (point.y * rotation.w) - (point.z * rotation.x);
		q1.z = rotation.z - (point.x * rotation.y) + (point.y * rotation.x) + (point.z * rotation.w);

		output.x = (conjugated.w * q1.x) + (conjugated.x * q1.w) - (conjugated.y * q1.z) + (conjugated.z * q1.y);
		output.y = (conjugated.w * q1.y) + (conjugated.x * q1.z) + (conjugated.y * q1.w) - (conjugated.z * q1.x);
		output.z = (conjugated.w * q1.z) - (conjugated.x * q1.y) + (conjugated.y * q1.x) + (conjugated.z * q1.w);
	}

}