#include "Quat.h"

namespace NAMESPACE_PHYSICS
{

	Quat Quat::inverse() const
	{
		const sp_float magnitude = (x * x + y * y + z * z + w * w);

		sp_assert(magnitude != ZERO_FLOAT, "InvalidOperationException");

		Quat temp;
		conjugate(*this, &temp);

		return temp.scale(ONE_FLOAT / magnitude);
	}

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

	Vec3 Quat::axis() const
	{
		// TODO: TESTS !!!!
		const sp_float tmp1 = ONE_FLOAT - w * w;

		if (tmp1 <= ZERO_FLOAT)
			return Vec3(ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT);

		const sp_float tmp2 = ONE_FLOAT / sp_sqrt(tmp1);

		return Vec3(x * tmp2, y * tmp2, z * tmp2);
	}

	Quat Quat::createRotate(sp_float angle, const Vec3& axis)
	{
		sp_assert(axis <= ONE_FLOAT + SP_EPSILON_THREE_DIGITS, "InvalidArgumentException");

		sp_float halfAngle = angle * HALF_FLOAT;
		sp_float sinHalfAngle = sinf(halfAngle);
		sp_float cosineHalfAngle = cosf(halfAngle);

		return Quat(
			cosineHalfAngle,
			sinHalfAngle * axis.x,
			sinHalfAngle * axis.y,
			sinHalfAngle * axis.z
		);
	}

	Quat Quat::slerp(const Quat& quatB, sp_float t) const
	{
		Quat copyQuatB = quatB;
		sp_float cosTheta = dot(quatB);

		// If cosTheta < 0, the interpolation will take the long way around the sphere.
		// To fix this, one quat must be negated.
		if (cosTheta < ZERO_FLOAT)
		{
			copyQuatB = -quatB;
			cosTheta = -cosTheta;
		}

		// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
		if (cosTheta > ONE_FLOAT - epsilon<sp_float>())
		{
			return Quat(
				NAMESPACE_FOUNDATION::lerp(w, copyQuatB.w, t),
				NAMESPACE_FOUNDATION::lerp(x, copyQuatB.x, t),
				NAMESPACE_FOUNDATION::lerp(y, copyQuatB.y, t),
				NAMESPACE_FOUNDATION::lerp(z, copyQuatB.z, t)
			);
		}
		else
		{
			sp_float angle = std::acos(cosTheta);
			return (scale(std::sinf((ONE_FLOAT - t) * angle)) + (copyQuatB * std::sinf(t * angle))) 
						/ std::sinf(angle);
		}
	}

	Quat Quat::slerp(const Quat& quatB, sp_float t, sp_int spinCount) const
	{
		Quat copyQuatB = quatB;
		sp_float cosTheta = dot(quatB);

		// If cosTheta < 0, the interpolation will take the long way around the sphere.
		// To fix this, one quat must be negated.
		if (cosTheta < ZERO_FLOAT)
		{
			copyQuatB = -quatB;
			cosTheta = -cosTheta;
		}

		// Perform a linear interpolation when cosTheta is close to 1 to avoid side effect of sin(angle) becoming a zero denominator
		if (cosTheta > ONE_FLOAT - epsilon<sp_float>())
		{
			return Quat(
				NAMESPACE_FOUNDATION::lerp(w, copyQuatB.w, t),
				NAMESPACE_FOUNDATION::lerp(x, copyQuatB.x, t),
				NAMESPACE_FOUNDATION::lerp(y, copyQuatB.y, t),
				NAMESPACE_FOUNDATION::lerp(z, copyQuatB.z, t)
			);
		}
		else
		{
			sp_float angle = std::acos(cosTheta);
			sp_float phi = angle + spinCount * PI;

			return (scale(std::sinf(angle - t * phi)) + (copyQuatB * std::sinf(t * phi)))
						/ std::sinf(angle);
		}
	}

	Vec3 Quat::toVec3() const
	{
		return Vec3(x, y, z);
	}

	Mat3 Quat::toMat3() const
	{
		Mat3 matrix;

		sp_float sqw = w * w;
		sp_float sqx = x * x;
		sp_float sqy = y * y;
		sp_float sqz = z * z;
		sp_float tmp1 = x * y;
		sp_float tmp2 = z * w;

		// invs (inverse square length) is only required if quaternion is not already normalised
		sp_float invs = ONE_FLOAT / (sqx + sqy + sqz + sqw);

		// row,col
		matrix[0] = (sqx - sqy - sqz + sqw) *invs; // since sqw + sqx + sqy + sqz =1/invs*invs
		matrix[4] = (-sqx + sqy - sqz + sqw)*invs;
		matrix[8] = (-sqx - sqy + sqz + sqw)*invs;

		matrix[3] = TWO_FLOAT * (tmp1 + tmp2)*invs;
		matrix[1] = TWO_FLOAT * (tmp1 - tmp2)*invs;

		tmp1 = x * z;
		tmp2 = y * w;
		matrix[6] = TWO_FLOAT * (tmp1 - tmp2)*invs;
		matrix[2] = TWO_FLOAT * (tmp1 + tmp2)*invs;
		tmp1 = y * z;
		tmp2 = x * w;
		matrix[7] = TWO_FLOAT * (tmp1 + tmp2)*invs;
		matrix[5] = TWO_FLOAT * (tmp1 - tmp2)*invs;

		return matrix;
	}

	Mat4 Quat::toMat4(const Vec3& position) const
	{
		sp_float sqw = w*w;
		sp_float sqx = x*x;
		sp_float sqy = y*y;
		sp_float sqz = z*z;

		sp_float m00 = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
		sp_float m11 = -sqx + sqy - sqz + sqw;
		sp_float m22 = -sqx - sqy + sqz + sqw;

		sp_float tmp1 = x*y;
		sp_float tmp2 = z*w;
		sp_float m01 = TWO_FLOAT * (tmp1 + tmp2);
		sp_float m10 = TWO_FLOAT * (tmp1 - tmp2);

		tmp1 = x*z;
		tmp2 = y*w;
		sp_float m02 = TWO_FLOAT * (tmp1 - tmp2);
		sp_float m20 = TWO_FLOAT * (tmp1 + tmp2);

		tmp1 = y*z;
		tmp2 = x*w;
		sp_float m12 = TWO_FLOAT * (tmp1 + tmp2);
		sp_float m21 = TWO_FLOAT * (tmp1 - tmp2);

		sp_float m03 = position.x - position.x * m00 - position.y * m01 - position.z * m02;
		sp_float m13 = position.y - position.x * m10 - position.y * m11 - position.z * m12;
		sp_float m23 = position.z - position.x * m20 - position.y * m21 - position.z * m22;

		return Mat4(
			m00, m10, m20, ZERO_FLOAT,
			m01, m11, m21, ZERO_FLOAT,
			m02, m12, m22, ZERO_FLOAT,
			m03, m13, m23, ONE_FLOAT
		);
	}

	Mat4 Quat::toMat4() const
	{
		sp_float sqw = w * w;
		sp_float sqx = x * x;
		sp_float sqy = y * y;
		sp_float sqz = z * z;

		sp_float m00 = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
		sp_float m11 = -sqx + sqy - sqz + sqw;
		sp_float m22 = -sqx - sqy + sqz + sqw;

		sp_float tmp1 = x * y;
		sp_float tmp2 = z * w;
		sp_float m01 = TWO_FLOAT * (tmp1 + tmp2);
		sp_float m10 = TWO_FLOAT * (tmp1 - tmp2);

		tmp1 = x * z;
		tmp2 = y * w;
		sp_float m02 = TWO_FLOAT * (tmp1 - tmp2);
		sp_float m20 = TWO_FLOAT * (tmp1 + tmp2);

		tmp1 = y * z;
		tmp2 = x * w;
		sp_float m12 = TWO_FLOAT * (tmp1 + tmp2);
		sp_float m21 = TWO_FLOAT * (tmp1 - tmp2);

		return Mat4(
			m00, m10, m20, ZERO_FLOAT,
			m01, m11, m21, ZERO_FLOAT,
			m02, m12, m22, ZERO_FLOAT,
			ZERO_FLOAT, ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT
		);
	}

	Vec3 Quat::toEulerAngles() const
	{
		sp_float sqw = w*w;
		sp_float sqx = x*x;
		sp_float sqy = y*y;
		sp_float sqz = z*z;
		sp_float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
		sp_float test = x*y + z*w;

		if (test > 0.499f * unit) 
		{ // singularity at north pole
			return Vec3(
				TWO_FLOAT * std::atan2(x, w),
				HALF_PI,
				ZERO_FLOAT
			);
		}

		if (test < -0.499f * unit) 
		{ // singularity at south pole
			return Vec3(
				-TWO_FLOAT * std::atan2(x, w),
				-HALF_PI,
				ZERO_FLOAT
			);
		}

		Vec3 angles;
		angles.x = std::atan2f(TWO_FLOAT * (x*w - y*z), sqw - sqx - sqy + sqz);
		angles.y = std::asinf(TWO_FLOAT * (x*z + w * y));
		angles.z = std::atan2f(-TWO_FLOAT * (x * y - w * z), sqw + sqx - sqy - sqz);
		
		/*
		sp_float test = x*y + z*w;
		if (test > 0.499) { // singularity at north pole
			angles.x = 2 * atan2(x, w);
			angles.y = HALF_PI;
			angles.z = 0;
			return angles;
		}
		if (test < -0.499) { // singularity at south pole
			angles.x = -2 * atan2(x, w);
			angles.y = -HALF_PI;
			angles.z = 0;
			return angles;
		}

		sp_float sqx = x*x;
		sp_float sqy = y*y;
		sp_float sqz = z*z;

		angles.x = atan2(2 * y*w - 2 * x*z, 1 - 2 * sqy - 2 * sqz);
		angles.y = asin(2 * test);
		angles.z = atan2(2 * x*w - 2 * y*z, 1 - 2 * sqx - 2 * sqz);
		*/

		/*
		// roll (x-axis rotation)
		//sp_float sinr_cosp = TWO_FLOAT * (w * x + y * z);
		//sp_float cosr_cosp = ONE_FLOAT - TWO_FLOAT * (x * x + y * y);
		//angles.x = std::atan2(sinr_cosp, cosr_cosp);

		sp_float sinr_cosp = TWO_FLOAT * y * w - 2.0f * x * z;
		sp_float cosr_cosp = ONE_FLOAT - TWO_FLOAT * y * y - TWO_FLOAT * z * z;
		angles.x = std::atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		//sp_float sinp = (TWO_FLOAT * w * y + TWO_FLOAT * z * x);
		sp_float sinp = (TWO_FLOAT * w * y - TWO_FLOAT * z * x);
		if (sp_abs(sinp) >= ONE_FLOAT)
			angles.y = std::copysign(HALF_PI, sinp); // use 90 degrees if out of range
		else
			angles.y = std::asin(sinp);

		// yaw (z-axis rotation)
		sp_float siny_cosp = TWO_FLOAT * (w * z + x * y);
		sp_float cosy_cosp = ONE_FLOAT - TWO_FLOAT * (y * y + z * z);
		angles.z = std::atan2(siny_cosp, cosy_cosp);
		*/

		return angles;
	}

#ifdef ENV_64BITS
	sp_float Quat::operator[](sp_size index) const
	{
		sp_assert(index >= ZERO_SIZE && index < QUAT_LENGTH);
		return ((sp_float*)this)[index];
	}
#endif

	Quat::operator Vec3() const
	{
		return Vec3(x, y, z);
	}

	Vec3 Quat::rotate(const Vec3& point) const
	{
		Quat conjugated;
		conjugate(*this, &conjugated);

		return (conjugated * (Quat(point) * (*this))).toVec3();
	}

	void multiply(const Vec3& vector, const Quat& quat, Quat* output)
	{
		output->w = quat.w - (vector.x * quat.x) - (vector.y * quat.y) - (vector.z * quat.z);
		output->x = quat.x + (vector.x * quat.w) - (vector.y * quat.z) + (vector.z * quat.y);
		output->y = quat.y + (vector.x * quat.z) + (vector.y * quat.w) - (vector.z * quat.x);
		output->z = quat.z - (vector.x * quat.y) + (vector.y * quat.x) + (vector.z * quat.w);
	}
	void multiply(const Quat& quat1, const Quat& quat2, Vec3* output)
	{
		output->x = (quat1.w * quat2.x) + (quat1.x * quat2.w) - (quat1.y * quat2.z) + (quat1.z * quat2.y);
		output->y = (quat1.w * quat2.y) + (quat1.x * quat2.z) + (quat1.y * quat2.w) - (quat1.z * quat2.x);
		output->z = (quat1.w * quat2.z) - (quat1.x * quat2.y) + (quat1.y * quat2.x) + (quat1.z * quat2.w);
	}
	void multiplyAndSum(const Quat& quat1, const Quat& quat2, const Vec3& sumVector, Vec3* output)
	{
		output->x = (quat1.w * quat2.x) + (quat1.x * quat2.w) - (quat1.y * quat2.z) + (quat1.z * quat2.y) + sumVector.x;
		output->y = (quat1.w * quat2.y) + (quat1.x * quat2.z) + (quat1.y * quat2.w) - (quat1.z * quat2.x) + sumVector.y;
		output->z = (quat1.w * quat2.z) - (quat1.x * quat2.y) + (quat1.y * quat2.x) + (quat1.z * quat2.w) + sumVector.z;
	}

	void rotate(const Quat& rotation, const Vec3& point, Vec3* output)
	{
		Quat conjugated;
		conjugate(rotation, &conjugated);

		Quat q1;
		multiply(point, rotation, &q1);

		multiply(conjugated, q1, output);
	}

	void rotateAndTranslate(const Quat& rotation, const Vec3& point, const Vec3& translation, Vec3* output)
	{
		Quat conjugated;
		conjugate(rotation, &conjugated);

		Quat q1;
		multiply(point, rotation, &q1);

		multiplyAndSum(conjugated, q1, translation, output);
	}

}