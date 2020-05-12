#include "Quat.h"

namespace NAMESPACE_PHYSICS
{
	Quat::Quat()
	{
		static sp_float identityQuaternion[QUAT_LENGTH] = { 
			ONE_FLOAT,
			ZERO_FLOAT,
			ZERO_FLOAT, 
			ZERO_FLOAT
		};

		std::memcpy(this, identityQuaternion, QUAT_SIZE);
	}

	Quat::Quat(sp_float* values)
	{
		std::memcpy(this, values, QUAT_SIZE);
	}

	Quat::Quat(sp_float w, sp_float x, sp_float y, sp_float z)
	{
		this->w = w;
		this->x = x;
		this->y = y;
		this->z = z;
	}

	Quat::Quat(const Vec3f& vector)
	{
		w = ZERO_FLOAT;
		x = vector.x;
		y = vector.y;
		z = vector.z;
	}

	Quat Quat::identity()
	{
		return Quat();
	}

	sp_float* Quat::values()
	{
		return (sp_float*)this;
	}

	Quat Quat::add(const Quat& quatB) const
	{
		return Quat(
			w + quatB.w,
			x + quatB.x,
			y + quatB.y,
			z + quatB.z
		);
	}

	Quat Quat::subtract(const Quat& quatB) const
	{
		return Quat(
			w - quatB.w,
			x - quatB.x,
			y - quatB.y,
			z - quatB.z
		);
	}

	Quat Quat::scale(sp_float value) const
	{
		return Quat(
			w * value,
			x * value,
			y * value,
			z * value
		);
	}

	Quat Quat::multiply(const Quat& quat) const
	{
		return Quat(
			(w * quat.w) - (x * quat.x) - (y * quat.y) - (z * quat.z),
			(w * quat.x) + (x * quat.w) - (y * quat.z) + (z * quat.y),
			(w * quat.y) + (x * quat.z) + (y * quat.w) - (z * quat.x),
			(w * quat.z) - (x * quat.y) + (y * quat.x) + (z * quat.w)
		);
	}

	sp_float Quat::length() const
	{
		return std::sqrtf(x * x + y * y + z * z + w * w);
	}

	Quat Quat::normalize() const
	{
		sp_float magnitude = ONE_FLOAT / length();

		assert(magnitude != ZERO_FLOAT);

		return Quat(
			w * magnitude,
			x * magnitude,
			y * magnitude,
			z * magnitude
		);
	}

	Quat Quat::conjugate() const
	{
		return Quat(w, -x, -y, -z);
	}

	sp_float Quat::angle() const
	{
		// TODO: TESTS !!!!
		if (std::fabsf(w) > std::cosf(0.5f))
			return std::asinf(sqrtf(x * x + y * y + z * z)) * TWO_FLOAT;

		return acos(w) * TWO_FLOAT;
	}

	Vec3<sp_float> Quat::axis() const
	{
		// TODO: TESTS !!!!
		const sp_float tmp1 = ONE_FLOAT - w * w;

		if (tmp1 <= ZERO_FLOAT)
			return Vec3f(ZERO_FLOAT, ZERO_FLOAT, ONE_FLOAT);

		const sp_float tmp2 = ONE_FLOAT / std::sqrtf(tmp1);

		return Vec3f(x * tmp2, y * tmp2, z * tmp2);
	}

	sp_float Quat::dot(const Quat& quatB) const
	{
		return (w * quatB.w) + (x * quatB.x) + (y * quatB.y) + (z * quatB.z);
	}

	Quat Quat::cross(const Quat& quatB) const
	{
		return multiply(quatB);
	}

	Quat Quat::inverse() const
	{
		sp_float magnitude = (x * x + y * y + z * z + w * w);

		assert(magnitude != ZERO_FLOAT);

		return conjugate().scale(ONE_FLOAT / magnitude);
	}

	Quat Quat::createRotate(sp_float angle, const Vec3f& axis)
	{
		sp_float halfAngle = (angle / TWO_FLOAT);
		sp_float sinHalfAngle = sinf(halfAngle);
		sp_float cosineHalfAngle = cosf(halfAngle);

		Vec3f positionNomralized = axis.normalize();

		return Quat(
			cosineHalfAngle,
			sinHalfAngle * positionNomralized.x,
			sinHalfAngle * positionNomralized.y,
			sinHalfAngle * positionNomralized.z
		);
	}

	Vec3f Quat::rotate(const Vec3<sp_float>& point) const
	{
		return (conjugate() * (Quat(point) * (*this))).toVec3();
	}

	Quat Quat::lerp(const Quat& quatB, sp_float t) const
	{
		return scale(ONE_FLOAT - t) + quatB.scale(t);
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

	Quat Quat::operator+(const Quat& quatB) const
	{
		return add(quatB);
	}

	Quat Quat::operator-(const Quat& quatB) const
	{
		return subtract(quatB);
	}

	Quat Quat::operator-() const
	{
		return Quat(-w, -x, -y, -z);
	}

	Quat Quat::operator*(const Quat& quat) const
	{
		return multiply(quat);
	}

	Quat Quat::operator*(sp_float value) const
	{
		return scale(value);
	}

	Quat Quat::operator/(sp_float value) const
	{
		sp_float temp = ONE_FLOAT / value;
		return Quat(w*temp, x  * temp, y*temp, z*temp);
	}

	Vec3<sp_float> Quat::toVec3() const
	{
		return Vec3<sp_float>(x, y, z);
	}

	Mat3<sp_float> Quat::toMat3() const
	{
		Mat3f result;

		sp_float sqw = w * w;
		sp_float sqx = x * x;
		sp_float sqy = y * y;
		sp_float sqz = z * z;
		sp_float tmp1 = x * y;
		sp_float tmp2 = z * w;

		// invs (inverse square length) is only required if quaternion is not already normalised
		sp_float invs = ONE_FLOAT / (sqx + sqy + sqz + sqw);

		// row,col
		result[0] = (sqx - sqy - sqz + sqw) *invs; // since sqw + sqx + sqy + sqz =1/invs*invs
		result[4] = (-sqx + sqy - sqz + sqw)*invs;
		result[8] = (-sqx - sqy + sqz + sqw)*invs;

		result[3] = TWO_FLOAT * (tmp1 + tmp2)*invs;
		result[1] = TWO_FLOAT * (tmp1 - tmp2)*invs;

		tmp1 = x * z;
		tmp2 = y * w;
		result[6] = TWO_FLOAT * (tmp1 - tmp2)*invs;
		result[2] = TWO_FLOAT * (tmp1 + tmp2)*invs;
		tmp1 = y * z;
		tmp2 = x * w;
		result[7] = TWO_FLOAT * (tmp1 + tmp2)*invs;
		result[5] = TWO_FLOAT * (tmp1 - tmp2)*invs;

		return result;
	}

	/*
	toMat4(const Vec3f& postion = Vec3f(ZERO_FLOAT))
	{
	// HANDLE EQUAL MAT3 ...

	   double sqw = q.w*q.w;
	   double sqx = q.x*q.x;
	   double sqy = q.y*q.y;
	   double sqz = q.z*q.z;
	   m00 = sqx - sqy - sqz + sqw; // since sqw + sqx + sqy + sqz =1
	   m11 = -sqx + sqy - sqz + sqw;
	   m22 = -sqx - sqy + sqz + sqw;

	   double tmp1 = q.x*q.y;
	   double tmp2 = q.z*q.w;
	   m01 = 2.0 * (tmp1 + tmp2);
	   m10 = 2.0 * (tmp1 - tmp2);

	   tmp1 = q.x*q.z;
	   tmp2 = q.y*q.w;
	   m02 = 2.0 * (tmp1 - tmp2);
	   m20 = 2.0 * (tmp1 + tmp2);

	   tmp1 = q1.y*q.z;
	   tmp2 = q1.x*q.w;
	   m12 = 2.0 * (tmp1 + tmp2);
	   m21 = 2.0 * (tmp1 - tmp2);


	   // SET POSITION TO MAT4  or ZERO
		double a1,a2,a3;
		 if (centre == null) {
			a1=a2=a3=0;
		 } else {
			a1 = centre.x;
			a2 = centre.y;
			a3 = centre.z;
		}
		m03 = a1 - a1 * m00 - a2 * m01 - a3 * m02;
		m13 = a2 - a1 * m10 - a2 * m11 - a3 * m12;
		m23 = a3 - a1 * m20 - a2 * m21 - a3 * m22;
		m30 = m31 = m32 = ZERO_FLOAT;
		m33 = ONE_FLOAT;
	}
	*/

	Quat Quat::fromEulerAngles(sp_float roll, sp_float pitch, sp_float yaw)
	{
		sp_float cosRoll = std::cosf(roll * HALF_FLOAT);
		sp_float sinRoll = std::sinf(roll * HALF_FLOAT);
		sp_float cosPitch = std::cosf(pitch * HALF_FLOAT);
		sp_float sinPitch = std::sinf(pitch * HALF_FLOAT);
		sp_float cosYaw = std::cosf(yaw * HALF_FLOAT);
		sp_float sinYaw = std::sinf(yaw * HALF_FLOAT);

		return Quat(
			cosYaw * cosPitch * cosRoll - sinYaw * sinPitch * sinRoll,
			cosYaw * cosPitch * sinRoll + sinYaw * sinPitch * cosRoll,
			cosYaw * sinPitch * cosRoll - sinYaw * cosPitch * sinRoll,
			sinYaw * cosPitch * cosRoll + cosYaw * sinPitch * sinRoll
		);
	}

	Vec3<sp_float> Quat::toEulerAngles() const
	{
		sp_float sqw = w*w;
		sp_float sqx = x*x;
		sp_float sqy = y*y;
		sp_float sqz = z*z;
		sp_float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
		sp_float test = x*y + z*w;

		if (test > 0.499f * unit) 
		{ // singularity at north pole
			return Vec3f(
				TWO_FLOAT * std::atan2(x, w),
				HALF_PI,
				ZERO_FLOAT
			);
		}

		if (test < -0.499f * unit) 
		{ // singularity at south pole
			return Vec3f(
				-TWO_FLOAT * std::atan2(x, w),
				-HALF_PI,
				ZERO_FLOAT
			);
		}

		Vec3<sp_float> angles;
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
		if (std::fabsf(sinp) >= ONE_FLOAT)
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

	sp_float Quat::operator[](sp_int index) const
	{
		sp_assert(index >= ZERO_INT && index < QUAT_LENGTH);
		return ((sp_float*)this)[index];
	}

	sp_float Quat::operator[](sp_uint index) const
	{
		sp_assert(index >= ZERO_UINT && index < QUAT_LENGTH);
		return ((sp_float*)this)[index];
	}

#ifdef ENV_64BITS
	sp_float Quat::operator[](sp_size index) const
	{
		sp_assert(index >= ZERO_SIZE && index < QUAT_LENGTH);
		return ((sp_float*)this)[index];
	}
#endif

	Quat::operator Vec3<sp_float>() const
	{
		return Vec3<sp_float>(x, y, z);
	}

	Quat::operator void*() const
	{
		return (void*)this;
	}

}