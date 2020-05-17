#include "Vec4.h"

namespace NAMESPACE_PHYSICS
{
	
	Vec4::Vec4(const sp_float defaultValue)
	{
		x = defaultValue;
		y = defaultValue;
		z = defaultValue;
		w = defaultValue;
	}

	
	Vec4::Vec4(const Vec2& xyComponents, const Vec2& zwComponents)
	{
		x = xyComponents.x;
		y = xyComponents.y;
		z = zwComponents.x;
		w = zwComponents.y;
	}

	
	Vec4::Vec4(const Vec3& vector, sp_float w)
	{
		x = vector.x;
		y = vector.y;
		z = vector.z;
		this->w = w;
	}

	
	Vec4::Vec4(const sp_float x,const  sp_float y, const sp_float z, const sp_float w)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->w = w;
	}

	
	sp_float* Vec4::getValues()
	{
		return (sp_float*)(this);
	}

	
	sp_float Vec4::length() const
	{
		return std::sqrtf(squared());
	}

	
	sp_float Vec4::squared() const
	{
		return (x * x) + (y * y) + (z * z) + (w * w);
	}

	
	sp_float Vec4::maximum() const
	{
		sp_float value = x;

		if (y > value)
			value = y;

		if (z > value)
			value = z;

		if (w > value)
			value = w;

		return value;
	}

	
	sp_float Vec4::minimum() const
	{
		sp_float value = x;

		if (y < value)
			value = y;

		if (z < value)
			value = z;

		if (w < value)
			value = w;

		return value;
	}

	
	void Vec4::add(const Vec4& vector)
	{
		x += vector.x;
		y += vector.y;
		z += vector.z;
		w += vector.w;
	}

	
	void Vec4::subtract(const Vec4& vector)
	{
		x -= vector.x;
		y -= vector.y;
		z -= vector.z;
		w -= vector.w;
	}

	
	void Vec4::scale(const sp_float scale)
	{
		x *= scale;
		y *= scale;
		z *= scale;
		w *= scale;
	}

	
	sp_float Vec4::dot(const Vec4& vector) const
	{
		return x * vector.x + y * vector.y + z * vector.z + w * vector.w;
	}

	
	sp_float Vec4::angle(const Vec4& vectorB) const
	{
		return dot(vectorB) / (length() * vectorB.length());
	}

	
	Vec4 Vec4::normalize() const
	{
		//sp_assert(length() != T(0));  // avoid division by zero

		sp_float len = length();

		if (len == 0.0f)
			return Vec4(0.0f);

		sp_float vectorLengthInverted = 1.0f / len;

		return Vec4 {
			x * vectorLengthInverted,
			y * vectorLengthInverted,
			z * vectorLengthInverted,
			w * vectorLengthInverted
		};
	}

	
	sp_float Vec4::distance(const Vec4& vector) const
	{
		sp_float xTemp = x - vector.x;
		sp_float yTemp = y - vector.y;
		sp_float zTemp = z - vector.z;
		sp_float wTemp = w - vector.w;

		return std::sqrtf(xTemp * xTemp + yTemp * yTemp + zTemp * zTemp + wTemp * wTemp);
	}

	
	Vec4 Vec4::fractional()
	{
		return Vec4 {
			x - floorf(x),
			y - floorf(y),
			z - floorf(z),
			w - floorf(w)
		};
	}

	
	Vec4 Vec4::clone() const
	{
		return Vec4(x, y, z, w);
	}

	
	Vec3 Vec4::toVec3() const
	{
		return Vec3(x, y, z);
	}

	
	Vec4 Vec4::operator*(const sp_float value)
	{
		return Vec4(
			x * value,
			y * value,
			z * value,
			w * value
			);
	}

	
	Vec4 Vec4::operator*(const sp_float value) const
	{
		return Vec4(
			x * value,
			y * value,
			z * value,
			w * value
			);
	}

	
	Vec4 Vec4::operator/(const sp_float value) const
	{
		return Vec4 (
			x / value,
			y / value,
			z / value,
			w / value
			);
	}

	
	void Vec4::operator/=(const sp_float value)
	{
		x /= value;
		y /= value;
		z /= value;
		w /= value;
	}

	
	Vec4 Vec4::operator+(const Vec4& vector) const
	{
		return Vec4(
			x + vector.x,
			y + vector.y,
			z + vector.z,
			w + vector.w
			);
	}

	
	Vec4 Vec4::operator+(const sp_float value) const
	{
		return Vec4(
			x + value,
			y + value,
			z + value,
			w + value
			);
	}

	
	Vec4 Vec4::operator-(const Vec4& vector) const
	{
		return Vec4(
			x - vector.x,
			y - vector.y,
			z - vector.z,
			w - vector.w
			);
	}

	
	Vec4 Vec4::operator-(const sp_float value) const
	{
		return Vec4(
			x - value,
			y - value,
			z - value,
			w - value
			);
	}

	
	Vec4 Vec4::operator-() const
	{
		return Vec4(
			-x,
			-y,
			-z,
			-w
			);
	}

	
	Vec4 Vec4::operator*(const Mat4& matrix4x4) const
	{
		Vec4 result;

	#if MAJOR_COLUMN_ORDER

		result[0]
			= matrix4x4[0 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[0 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[0 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[0 * MAT4_ROW_LENGTH + 3] * w;

		result[1]
			= matrix4x4[1 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[1 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[1 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[1 * MAT4_ROW_LENGTH + 3] * w;

		result[2]
			= matrix4x4[2 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[2 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[2 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[2 * MAT4_ROW_LENGTH + 3] * w;

		result[3]
			= matrix4x4[3 * MAT4_ROW_LENGTH + 0] * x
			+ matrix4x4[3 * MAT4_ROW_LENGTH + 1] * y
			+ matrix4x4[3 * MAT4_ROW_LENGTH + 2] * z
			+ matrix4x4[3 * MAT4_ROW_LENGTH + 3] * w;

	#else

		result[0]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];

		result[1]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];

		result[2]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];

		result[3]
			= x * matrix4x4[0 * MAT4_ROW_LENGTH + 0]
			+ y * matrix4x4[1 * MAT4_ROW_LENGTH + 0]
			+ z * matrix4x4[2 * MAT4_ROW_LENGTH + 0]
			+ w * matrix4x4[3 * MAT4_ROW_LENGTH + 0];
	#endif

		return result;
	}

	
	sp_bool Vec4::operator==(const Vec4& vector) const
	{
		return x == vector.x
			&& y == vector.y
			&& z == vector.z
			&& w == vector.w;
	}

	
	sp_bool Vec4::operator==(const sp_float value) const
	{
		return x == value
			&& y == value
			&& z == value
			&& w == value;
	}

	
	sp_bool Vec4::operator!=(const Vec4& vector) const
	{
		return x != vector.x
			|| y != vector.y
			|| z != vector.z
			|| w != vector.w;
	}

	
	sp_float& Vec4::operator[](const sp_int index)
	{
		sp_assert(index >= 0 && index < VEC4_SIZE, "IndexOutOfrangeException");

		return ((sp_float*)this)[index];
	}

	
	sp_float Vec4::operator[](const sp_int index) const
	{
		sp_assert(index >= 0 && index < VEC4_SIZE, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

	
	sp_float& Vec4::operator[](const sp_uint index)
	{
		sp_assert(index >= 0 && index < VEC4_SIZE, "IndexOutOfrangeException");

		return ((sp_float*)this)[index];
	}

	
	sp_float Vec4::operator[](const sp_uint index) const
	{
		sp_assert(index >= 0 && index < VEC4_SIZE, "IndexOutOfrangeException");

		return reinterpret_cast<const sp_float*>(this)[index];
	}

#ifdef ENV_64BITS
	
	sp_float& Vec4::operator[](const sp_size index)
	{
		sp_assert(index >= 0 && index < VEC4_SIZE);

		return ((sp_float*)this)[index];
	}

	
	sp_float Vec4::operator[](const sp_size index) const
	{
		sp_assert(index >= 0 && index < VEC4_SIZE);

		return reinterpret_cast<const sp_float*>(this)[index];
	}
#endif

	
	Vec4::operator void*() const
	{
		return (void*) this;
	}

	
	Vec4::operator void*()
	{
		return (void*) this;
	}

	
	Vec4::operator sp_float*() const
	{
		return (sp_float*) this;
	}

}