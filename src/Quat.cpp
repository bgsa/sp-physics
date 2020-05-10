#include "Quat.h"

namespace NAMESPACE_PHYSICS
{
	Quat::Quat()
	{
		static sp_float emptyQuaternion[QUAT_LENGTH] = { 
			ZERO_FLOAT, 
			ZERO_FLOAT, 
			ZERO_FLOAT, 
			ZERO_FLOAT 
		};

		std::memcpy(this, emptyQuaternion, QUAT_SIZE);
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

	void Quat::scale(sp_float value)
	{
		w *= value;
		x *= value;
		y *= value;
		z *= value;
	}

	Quat Quat::createScale(sp_float value) const
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
			-x * quat.x - y * quat.y - z * quat.z + w * quat.w,
			x * quat.w + y * quat.z - z * quat.y + w * quat.x,
			-x * quat.z + y * quat.w + z * quat.x + w * quat.y,
			x * quat.y - y * quat.x + z * quat.w + w * quat.z
		);
	}

	sp_float Quat::length() const
	{
		return sqrtf(x * x + y * y + z * z + w * w);
	}

	Quat Quat::normalize() const
	{
		sp_float magnitude = ONE_FLOAT / length();

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

	sp_float Quat::dot(Quat quatB) const
	{
		// ERROR !!!!
		sp_float result = sqrtf(w * quatB.w + x * quatB.x + y * quatB.y + z * quatB.z);

		return result;
	}

	Quat Quat::inverse() const
	{
		Quat result(w, x, y, z);

		sp_float magnitude = length();

		if (magnitude == ZERO_FLOAT)
			return result;

		result.scale(ONE_FLOAT / (magnitude * magnitude));

		return result;
	}

	Quat Quat::createRotate(sp_float angle, const Vec3f& position)
	{
		sp_float halfAngle = (angle / TWO_FLOAT);
		sp_float sinHalfAngle = sinf(halfAngle);
		sp_float cosineHalfAngle = cosf(halfAngle);

		Vec3f positionNomralized = position.normalize();

		return Quat(
			cosineHalfAngle,
			sinHalfAngle * positionNomralized.x,
			sinHalfAngle * positionNomralized.y,
			sinHalfAngle * positionNomralized.z
		);
	}

	Quat Quat::rotate(const Quat& r) const
	{
		return r * (*this * r.conjugate());
	}

	Quat Quat::rotate(sp_float angle, const Vec3f& vector) const
	{
		Quat rotationalQuaternion = Quat::createRotate(angle, vector);

		return rotate(rotationalQuaternion);
	}

	Quat Quat::linearInterpolate(const Quat& quatB, sp_float t) const
	{
		return createScale(ONE_FLOAT - t) + quatB.createScale(t);
	}

	Quat Quat::linearInterpolateNormalized(const Quat& quatB, sp_float t) const
	{
		return linearInterpolate(quatB, t).normalize();
	}

	sp_size Quat::sizeInBytes() const
	{
		return QUAT_SIZE;
	}

	Vec3f Quat::toVec3() const
	{
		return Vec3f(x, y, z);
	}

	sp_float Quat::operator[](sp_int index)
	{
		sp_assert(index >= 0 && index < QUAT_LENGTH);

		return ((sp_float*)this)[index];
	}

	Quat Quat::operator+(const Quat& quatB) const
	{
		return add(quatB);
	}

	Quat Quat::operator-(const Quat& quatB) const
	{
		return subtract(quatB);
	}

	Quat Quat::operator*(const Quat& quat) const
	{
		return multiply(quat);
	}

	Quat Quat::operator*(sp_float value) const
	{
		return createScale(value);
	}

	Quat::operator void*() const
	{
		return (void*)this;
	}

	Quat::operator Vec3f() const
	{
		return Vec3f(x, y, z);
	}

}