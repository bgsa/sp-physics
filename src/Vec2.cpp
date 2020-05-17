#include "Vec2.h"

namespace NAMESPACE_PHYSICS
{
	
	Vec2::Vec2(const sp_float value)
	{
		x = value;
		y = value;
	}

	
	Vec2::Vec2(const sp_float x, const sp_float y)
	{
		this->x = x;
		this->y = y;
	}

	
	sp_float* Vec2::getValues()
	{
		return (sp_float*)(this);
	}

	
	sp_float Vec2::maximum() const
	{
		if (x > y)
			return x;
		else
			return y;
	}

	
	sp_float Vec2::minimum() const
	{
		if (x < y)
			return x;
		else
			return y;
	}

	
	sp_float Vec2::length() const
	{
		return sqrtf(squared());
	}

	
	sp_float Vec2::squared() const
	{
		return (x * x) + (y * y);
	}

	
	void Vec2::add(const Vec2& vector)
	{
		x += vector.x;
		y += vector.y;
	}

	
	void Vec2::subtract(const Vec2& vector)
	{
		x -= vector.x;
		y -= vector.y;
	}

	
	void Vec2::scale(sp_float scale)
	{
		x *= scale;
		y *= scale;
	}

	
	sp_float Vec2::dot(const Vec2& vector) const
	{
		return x * vector.x + y * vector.y;
	}

	
	sp_float Vec2::angle(const Vec2& vectorB) const
	{
		sp_float vec1Len = length();
		sp_float vec2Len = vectorB.length();

		if (vec1Len == ZERO_FLOAT) // vec-Len == 0 means division by zero and return "nan" (not a number)
			vec1Len = 0.000001f;

		if (vec2Len == 0.0f) // vec-Len == 0 means division by zero and return "nan" (not a number)
			vec2Len = 0.000001f;

		return dot(vectorB) / (vec1Len * vec2Len);
	}

	
	Vec2 Vec2::normalize() const
	{
		//sp_assert(length() != T(0));  // avoid division by zero

		sp_float len = length();

		if (len == 0.0f)
			return Vec2(0.0f);

		return Vec2 {
			x / len,
			y / len
		};
	}

	
	void Vec2::transformToUnit()
	{
		scale(1.0f / length());
	}

	
	sp_float Vec2::distance(const Vec2& vector) const
	{
		sp_float xTemp = x - vector.x;
		sp_float yTemp = y - vector.y;

		return sqrtf(xTemp * xTemp + yTemp * yTemp);
	}

	
	Vec2 Vec2::fractional() const
	{
		return Vec2 {
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

		Vec2* result = ALLOC_ARRAY(Vec2,2);
		result[0] = v1;
		result[1] = v2;

		return result;
	}

	
	Vec2 Vec2::clone() const
	{
		return Vec2(x, y);
	}

	
	Vec2 Vec2::operator*(const sp_float value) const
	{
		return Vec2(
			x * value,
			y * value
			);
	}

	
	Vec2 Vec2::operator/(const sp_float value) const
	{
		return Vec2 (
			x / value,
			y / value
			);
	}

	
	Vec2 Vec2::operator+(const Vec2& vector) const
	{
		return Vec2 (
			x + vector.x,
			y + vector.y
			);
	}

	
	Vec2 Vec2::operator+(const sp_float value) const
	{
		return Vec2(
			x + value,
			y + value
			);
	}

	
	Vec2 Vec2::operator-(const Vec2& vector) const
	{
		return Vec2(
			x - vector.x,
			y - vector.y
			);
	}

	
	Vec2 Vec2::operator-(const sp_float value) const
	{
		return Vec2 (
			x - value,
			y - value
			);
	}

	
	Vec2 Vec2::operator-() const
	{
		return Vec2(
			-x,
			-y
			);
	}
	
	sp_bool Vec2::operator==(const Vec2& vector) const
	{
		return x == vector.x
			&& y == vector.y;
	}

	
	sp_bool Vec2::operator==(const sp_float value) const
	{
		return x == value 
			&& y == value;
	}

	
	sp_bool Vec2::operator!=(const Vec2& vector) const
	{
		return x != vector.x
			|| y != vector.y;
	}

	
	sp_bool Vec2::operator!=(const sp_float value) const
	{
		return x != value
			|| y != value;
	}

	
	sp_float& Vec2::operator[](const sp_int index)
	{
		sp_assert(index >= ZERO_INT && index < VEC2_SIZE);

		return ((sp_float*)this)[index];
	}

	
	sp_float Vec2::operator[](const sp_int index) const
	{
		sp_assert(index >= ZERO_INT && index < VEC2_SIZE);

		return reinterpret_cast<const sp_float*>(this)[index];
	}

	
	sp_float& Vec2::operator[](const sp_uint index)
	{
		sp_assert(index >= ZERO_UINT && index < VEC2_SIZE);

		return ((sp_float*)this)[index];
	}

	
	sp_float Vec2::operator[](const sp_uint index) const
	{
		sp_assert(index >= ZERO_UINT && index < VEC2_SIZE);

		return reinterpret_cast<const sp_float*>(this)[index];
	}

#ifdef ENV_64BITS
	
	sp_float& Vec2::operator[](const sp_size index)
	{
		sp_assert(index >= ZERO_SIZE && index < VEC2_SIZE);

		return ((sp_float*)this)[index];
	}

	
	sp_float Vec2::operator[](const sp_size index) const
	{
		sp_assert(index >= ZERO_SIZE && index < VEC2_SIZE);

		return reinterpret_cast<const sp_float*>(this)[index];
	}
#endif

	
	Vec2::operator sp_float*() const
	{
		return (sp_float*) this;
	}

	
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