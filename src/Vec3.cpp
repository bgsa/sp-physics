#include "Vec3.h"

namespace NAMESPACE_PHYSICS
{
	
	Vec3::Vec3(const sp_float defaultValue) {
		x = defaultValue;
		y = defaultValue;
		z = defaultValue;
	}

	
	Vec3::Vec3(const sp_float x, const sp_float y, const sp_float z) {
		this->x = x;
		this->y = y;
		this->z = z;
	}

	
	Vec3::Vec3(const Vec3& value)
	{
		x = value.x;
		y = value.y;
		z = value.z;
	}

	
	Vec3::Vec3(Vec2 vector2D, sp_float z) {
		x = vector2D[0];
		y = vector2D[1];
		z = z;
	}

	
	sp_float* Vec3::getValues()
	{
		return (sp_float*)(this);
	}

	
	Vec3 Vec3::abs() const
	{
		return Vec3(
			std::fabsf(x),
			std::fabsf(y),
			std::fabsf(z)
		);
	}
	
	sp_float Vec3::squaredLength() const
	{
		return (x * x) + (y * y) + (z * z);
	}

	
	sp_float Vec3::length() const
	{
		return sqrtf(squaredLength());
	}

	
	sp_float Vec3::maximum() const
	{
		sp_float value = x;

		if (y > value)
			value = y;

		if (z > value)
			value = z;

		return value;
	}

	
	sp_float Vec3::minimum() const
	{
		sp_float value = x;

		if (y < value)
			value = y;

		if (z < value)
			value = z;

		return value;
	}

	
	sp_float Vec3::tripleProduct(const Vec3 &v, const Vec3 &u) const
	{
		return this->cross(v).dot(u);
	}

	
	void Vec3::add(const Vec3& vector)
	{
		x += vector.x;
		y += vector.y;
		z += vector.z;
	}

	
	Vec3 Vec3::subtract(const Vec3& vector)
	{
		return Vec3(
			x -= vector.x,
			y -= vector.y,
			z -= vector.z
			);
	}

	
	Vec3 Vec3::multiply(const Vec3& vector) const
	{
		return Vec3(
			x * vector.x,
			y * vector.y,
			z * vector.z
			);
	}

	
	void Vec3::scale(sp_float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;
	}

	
	Vec3 Vec3::rotate(sp_float angle, const Vec3& axis)
	{
		sp_float cosAngle = cosf(angle);
		sp_float sinAngle = sinf(angle);

		Vec3 result = (operator*(cosAngle)) + (axis.cross(*this) * sinAngle) + (axis * dot(axis)) * (ONE_FLOAT - cosAngle);

		return result;
	}
	
	Vec3 Vec3::rotateX(sp_float angle)
	{
		return Vec3(
				x,
				y * cosf(angle) - z * sinf(angle),
				y * sinf(angle) + z * cosf(angle)
			);
	}

	
	Vec3 Vec3::rotateY(sp_float angle)
	{
		return Vec3(
			x * cosf(angle) + z * sinf(angle),
			y,
			z * cosf(angle) - x * sinf(angle)
		);
	}

	
	Vec3 Vec3::rotateY(sp_float angle, const Vec3& referencePoint)
	{
		Vec3 direction = this->subtract(referencePoint);
		angle *= -1.0f;

		return Vec3(
			x,
			referencePoint[1] + direction[1] * cosf(angle) + (referencePoint[2] - z) * sinf(angle),
			referencePoint[2] + direction[1] * sinf(angle) + direction[2] * cosf(angle)
		);
	}

	
	Vec3 Vec3::rotateZ(sp_float angle)
	{
		return Vec3(
				x * cosf(angle) - y * sinf(angle),
				x * sinf(angle) + y * cosf(angle),
				z
			);
	}

	
	Vec3 Vec3::rotateZ(sp_float angle, const Vec3& referencePoint)
	{
		Vec3 direction = this->subtract(referencePoint);

		return Vec3(
				referencePoint.x + direction.x * cosf(angle) + (referencePoint[1] - y) * sinf(angle),
				referencePoint[1] + direction.x * sinf(angle) + direction[1] * cosf(angle),
				z
			);
	}

	
	Vec3 Vec3::cross(const Vec3& vector) const
	{
		Vec3 result = Vec3(
			y * vector[2] - vector[1] * z,
			-x * vector[2] + vector[0] * z,
			x * vector[1] - vector[0] * y
		);

		return result;
	}
	
	sp_float Vec3::dot(const Vec3& vector) const
	{
		return x * vector.x + y * vector.y + z * vector.z;
	}

	
	sp_float Vec3::angle(const Vec3& vectorB) const
	{
		return dot(vectorB) / (length() * vectorB.length());
	}

	
	Vec3 Vec3::normalize() const
	{
		//sp_assert(length() != T(0));   // avoid division by zero
		sp_float len = length();

		if (len == 0.0f)
			return Vec3(0.0f);

		sp_float vectorLengthInverted = 1.0f / len;

		return Vec3 {
			x * vectorLengthInverted,
			y * vectorLengthInverted,
			z * vectorLengthInverted
		};
	}

	
	void Vec3::transformToUnit()
	{
		scale(1.0f / length());
	}

	
	sp_float Vec3::squaredDistance(const Vec3& vector) const
	{
		sp_float xTemp = x - vector.x;
		xTemp = xTemp * xTemp;

		sp_float yTemp = y - vector.y;
		yTemp = yTemp * yTemp;

		sp_float zTemp = z - vector.z;
		zTemp = zTemp * zTemp;

		return xTemp + yTemp + zTemp;
	}

	
	sp_float Vec3::distance(const Vec3& vector) const
	{
		sp_float xTemp = x - vector.x;
		sp_float yTemp = y - vector.y;
		sp_float zTemp = z - vector.z;

		return std::sqrtf(xTemp *xTemp + yTemp * yTemp + zTemp * zTemp);
	}

	
	sp_float Vec3::signedDistance(const Vec3& point) const
	{
		return (*this - point).dot(point);
	}

	
	Vec3 Vec3::fractional()
	{
		return Vec3 {
			x - floorf(x),
			y - floorf(y),
			z - floorf(z)
		};
	}

	
	Vec3 Vec3::clone()
	{
		return Vec3(x, y, z);
	}

	
	Vec3 Vec3::operator/(const sp_float value) const
	{
		return Vec3(
			x / value,
			y / value,
			z / value
			);
	}

	
	Vec3 Vec3::operator/(const Vec3& vector) const
	{
		return Vec3(
			x / vector.x,
			y / vector.y,
			z / vector.z
		);
	}

	
	Vec3 Vec3::operator*(const sp_float value) const
	{
		return Vec3(
			x * value,
			y * value,
			z * value
			);
	}

	
	void Vec3::operator*=(const sp_float value)
	{
		x *= value;
		y *= value;
		z *= value;
	}

	
	Vec3 Vec3::operator*(const Vec3& vector) const
	{
		return multiply(vector);
	}

	
	Vec3 Vec3::operator+(const Vec3& vector) const
	{
		return Vec3(
			x + vector.x,
			y + vector.y,
			z + vector.z
			);
	}

	
	void Vec3::operator+=(const Vec3& vector)
	{
		x += vector.x;
		y += vector.y;
		z += vector.z;
	}

	
	Vec3 Vec3::operator+(const sp_float value) const
	{
		return Vec3(
			x + value,
			y + value,
			z + value
			);
	}

	
	Vec3 Vec3::operator-(const Vec3& vector) const
	{
		return Vec3(
			x - vector.x,
			y - vector.y,
			z - vector.z
		);
	}

	
	void Vec3::operator-=(const Vec3& vector)
	{
		x -= vector.x;
		y -= vector.y;
		z -= vector.z;
	}

	
	Vec3 Vec3::operator-(const sp_float value) const
	{
		return Vec3(
			x - value,
			y - value,
			z - value
			);
	}

	
	Vec3 Vec3::operator-() const
	{
		return Vec3(
			-x,
			-y,
			-z
			);
	}

	sp_bool Vec3::operator==(const Vec3& vector) const
	{
		return x == vector.x
			&& y == vector.y
			&& z == vector.z;
	}

	
	sp_bool Vec3::operator==(const sp_float value) const
	{
		return x == value
			&& y == value
			&& z == value;
	}

	
	sp_bool Vec3::operator!=(const Vec3& vector) const
	{
		return x != vector.x
			|| y != vector.y
			|| z != vector.z;
	}

	
	sp_bool Vec3::operator!=(const sp_float value) const
	{
		return x != value
			|| y != value
			|| z != value;
	}

	
	sp_float& Vec3::operator[](sp_int index)
	{
		sp_assert(index >= 0 && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return ((sp_float*)this)[index];
	}
	
	sp_float Vec3::operator[](sp_int index) const
	{
		sp_assert(index >= 0 && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

	
	sp_float& Vec3::operator[](sp_uint index)
	{
		sp_assert(index >= 0 && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return ((sp_float*)this)[index];
	}
	
	sp_float Vec3::operator[](sp_uint index) const
	{
		sp_assert(index >= 0 && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

#ifdef ENV_64BITS
	
	sp_float& Vec3::operator[](sp_size index)
	{
		sp_assert(index >= 0 && index < VEC3_LENGTH);

		return ((sp_float*)this)[index];
	}
	
	sp_float Vec3::operator[](sp_size index) const
	{
		sp_assert(index >= 0 && index < VEC3_LENGTH);

		return reinterpret_cast<const sp_float*>(this)[index];
	}
#endif

	
	Vec3::operator void*() const
	{
		return (void*) this;
	}

	
	Vec3::operator sp_float*() const
	{
		return (sp_float*) this;
	}

	
	Vec3 Vec3::operator+=(const sp_float value)
	{
		return Vec3(
			x + value,
			y + value,
			z + value
			);
	}

	
	Vec3 Vec3::operator-=(const sp_float value)
	{
		return Vec3(
			x - value,
			y - value,
			z - value
			);
	}

	
	Vec3& Vec3::operator=(const Vec3& vector)
	{
		x = vector.x;
		y = vector.y;
		z = vector.z;

		return *this;
	}

}