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
	
	sp_float Vec3::orientation(const Vec3& vertex1, const Vec3& vertex2) const
	{
		const Vec3 ray1 = (vertex1 - *this);
		const Vec3 ray2 = (vertex2 - *this);

		const Mat3 m(
			ONE_FLOAT, ONE_FLOAT, ONE_FLOAT,
			ray1.x, ray1.y, ray1.z,
			ray2.x, ray2.y, ray2.z
		);

		return m.determinant();
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
	
	void Vec3::scale(sp_float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;
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

	sp_float Vec3::angle(const Vec3& vectorB) const
	{
		return dot(vectorB) / (length(*this) * length(vectorB));
	}

	sp_float Vec3::signedDistance(const Vec3& point) const
	{
		return (*this - point).dot(point);
	}

	Vec3 Vec3::operator/(const sp_float value) const
	{
		sp_float intertedValue = 1.0f / value;

		return Vec3(
			x * intertedValue,
			y * intertedValue,
			z * intertedValue
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
		return Vec3(
			x * vector.x,
			y * vector.y,
			z * vector.z
		);
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
		return isCloseEnough(x, vector.x)
			&& isCloseEnough(y, vector.y)
			&& isCloseEnough(z, vector.z);
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
		sp_assert(index >= ZERO_INT && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return ((sp_float*)this)[index];
	}
	
	sp_float Vec3::operator[](sp_int index) const
	{
		sp_assert(index >= ZERO_INT && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

	
	sp_float& Vec3::operator[](sp_uint index)
	{
		sp_assert(index >= ZERO_UINT && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return ((sp_float*)this)[index];
	}
	
	sp_float Vec3::operator[](sp_uint index) const
	{
		sp_assert(index >= ZERO_UINT && index < VEC3_LENGTH, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

#ifdef ENV_64BITS
	
	sp_float& Vec3::operator[](sp_size index)
	{
		sp_assert(index >= ZERO_SIZE && index < VEC3_LENGTH);

		return ((sp_float*)this)[index];
	}
	
	sp_float Vec3::operator[](sp_size index) const
	{
		sp_assert(index >= ZERO_SIZE && index < VEC3_LENGTH);

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