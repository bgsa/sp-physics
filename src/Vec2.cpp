#include "Vec2.h"

namespace NAMESPACE_PHYSICS
{

	Vec2 Vec2::normalize() const
	{
		//sp_assert(length() != T(0));  // avoid division by zero

		sp_float len = length();

		if (len == 0.0f)
			return Vec2(0.0f);

		return Vec2{
			x / len,
			y / len
		};
	}

	Vec2 Vec2::fractional() const
	{
		return Vec2{
			x - floorf(x),
			y - floorf(y)
		};
	}

	Vec2* Vec2::orthogonalProjection(const Vec2& vector) const
	{
		sp_float value = dot(vector) / vector.squared();

		Vec2 v1 = vector * value;
		Vec2 v2 = {
			vector.x - v1.x,
			vector.y - v1.y
		};

		Vec2* result = ALLOC_ARRAY(Vec2, 2);
		result[0] = v1;
		result[1] = v2;

		return result;
	}

	sp_float Vec2::operator[](const sp_int index) const
	{
		sp_assert(index >= ZERO_INT && index < VEC2_LENGTH, "IndexOutOfrangeException");
		return reinterpret_cast<const sp_float*>(this)[index];
	}

	sp_float Vec2::operator[](const sp_uint index) const
	{
		sp_assert(index >= ZERO_UINT && index < VEC2_LENGTH, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

#ifdef ENV_64BITS

	sp_float Vec2::operator[](const sp_size index) const
	{
		sp_assert(index >= ZERO_SIZE && index < VEC2_LENGTH, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}
#endif


	std::ostream& operator<<(std::ostream& outputStream, const Vec2& vector)
	{
		return outputStream << vector.x << "," << vector.y;
	}

	std::istream& operator>>(std::istream& inputStream, Vec2& vector)
	{
		char separator;
		return inputStream >> vector.x >> separator >> vector.y;
	}

	std::ostream& Vec2::serialize(std::ostream& outputStream) const
	{
		return outputStream << *this;
	}

	std::istream& Vec2::deserialize(std::istream& inputStream)
	{
		inputStream >> *this;
		return inputStream;
	}

}