#include "Vec2.h"

namespace NAMESPACE_PHYSICS
{
	template <typename T>
	Vec2<T>::Vec2(const T value)
	{
		x = value;
		y = value;
	}

	template <typename T>
	Vec2<T>::Vec2(const T x, const T y)
	{
		this->x = x;
		this->y = y;
	}

	template <typename T>
	T* Vec2<T>::getValues()
	{
		return reinterpret_cast<T*>(this);
	}

	template <typename T>
	T Vec2<T>::maximum() const
	{
		if (x > y)
			return x;
		else
			return y;
	}

	template <typename T>
	T Vec2<T>::minimum() const
	{
		if (x < y)
			return x;
		else
			return y;
	}

	template <typename T>
	T Vec2<T>::length() const
	{
		return T(sqrt(squared()));
	}

	template <typename T>
	T Vec2<T>::squared() const
	{
		return (x * x) + (y * y);
	}

	template <typename T>
	void Vec2<T>::add(const Vec2<T>& vector)
	{
		x += vector.x;
		y += vector.y;
	}

	template <typename T>
	void Vec2<T>::subtract(const Vec2<T>& vector)
	{
		x -= vector.x;
		y -= vector.y;
	}

	template <typename T>
	void Vec2<T>::scale(T scale)
	{
		x *= scale;
		y *= scale;
	}

	template <typename T>
	T Vec2<T>::dot(const Vec2<T>& vector) const
	{
		return x * vector.x + y * vector.y;
	}

	template <typename T>
	T Vec2<T>::angle(const Vec2<T>& vectorB) const
	{
		T vec1Len = length();
		T vec2Len = vectorB.length();

		if (vec1Len == T(0)) // vec-Len == 0 means division by zero and return "nan" (not a number)
			vec1Len = T(0.000001);

		if (vec2Len == T(0)) // vec-Len == 0 means division by zero and return "nan" (not a number)
			vec2Len = T(0.000001);

		return dot(vectorB) / (vec1Len * vec2Len);
	}

	template <typename T>
	Vec2<T> Vec2<T>::normalize() const
	{
		//sp_assert(length() != T(0));  // avoid division by zero

		T len = length();

		if (len == T(0))
			return Vec2<T>(T(0));

		return Vec2<T> {
			x / len,
			y / len
		};
	}

	template <typename T>
	void Vec2<T>::transformToUnit()
	{
		scale(T(1) / length());
	}

	template <typename T>
	T Vec2<T>::distance(const Vec2<T>& vector) const
	{
		T xTemp = x - vector.x;
		T yTemp = y - vector.y;

		return T(sqrt(xTemp * xTemp + yTemp * yTemp));
	}

	template <typename T>
	Vec2<T> Vec2<T>::fractional() const
	{
		return Vec2<T> {
			T(x - floor(x)),
			T(y - floor(y))
		};
	}

	template <typename T>
	Vec2<T>* Vec2<T>::orthogonalProjection(const Vec2<T>& vector) const
	{
		T value = dot(vector) / vector.squared();

		Vec2<T> v1 = vector * value;
		Vec2<T> v2 = {
			vector.x - v1.x,
			vector.y - v1.y
		};

		Vec2<T>* result = ALLOC_ARRAY(Vec2<T>,2);
		result[0] = v1;
		result[1] = v2;

		return result;
	}

	template <typename T>
	Vec2<T> Vec2<T>::clone() const
	{
		return Vec2<T>(x, y);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator*(const T value) const
	{
		return Vec2<T>(
			x * value,
			y * value
			);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator/(const T value) const
	{
		return Vec2<T> (
			x / value,
			y / value
			);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator+(const Vec2<T>& vector) const
	{
		return Vec2<T> (
			x + vector.x,
			y + vector.y
			);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator+(const T value) const
	{
		return Vec2<T>(
			x + value,
			y + value
			);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator-(const Vec2<T>& vector) const
	{
		return Vec2<T>(
			x - vector.x,
			y - vector.y
			);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator-(const T value) const
	{
		return Vec2<T> (
			x - value,
			y - value
			);
	}

	template <typename T>
	Vec2<T> Vec2<T>::operator-() const
	{
		return Vec2<T>(
			-x,
			-y
			);
	}
	template <>
	Vec2<sp_uint> Vec2<sp_uint>::operator-() const
	{
		return Vec2<sp_uint>(x,y);
	}
#ifdef ENV_64BTIS
	template <>
	Vec2<sp_size> Vec2<sp_size>::operator-() const
	{
		return Vec2<sp_size>(x, y);
	}
#endif

	template <typename T>
	bool Vec2<T>::operator==(const Vec2<T>& vector) const
	{
		return x == vector.x
			&& y == vector.y;
	}

	template <typename T>
	bool Vec2<T>::operator==(const T value) const
	{
		return x == value 
			&& y == value;
	}

	template <typename T>
	bool Vec2<T>::operator!=(const Vec2<T>& vector) const
	{
		return x != vector.x
			|| y != vector.y;
	}

	template <typename T>
	bool Vec2<T>::operator!=(const T value) const
	{
		return x != value
			|| y != value;
	}

	template <typename T>
	T& Vec2<T>::operator[](const sp_int index)
	{
		sp_assert(index >= ZERO_INT && index < VEC2_SIZE);

		return reinterpret_cast<T*>(this)[index];
	}

	template <typename T>
	T Vec2<T>::operator[](const sp_int index) const
	{
		sp_assert(index >= ZERO_INT && index < VEC2_SIZE);

		return reinterpret_cast<const T*>(this)[index];
	}

	template <typename T>
	T& Vec2<T>::operator[](const sp_uint index)
	{
		sp_assert(index >= ZERO_UINT && index < VEC2_SIZE);

		return reinterpret_cast<T*>(this)[index];
	}

	template <typename T>
	T Vec2<T>::operator[](const sp_uint index) const
	{
		sp_assert(index >= ZERO_UINT && index < VEC2_SIZE);

		return reinterpret_cast<const T*>(this)[index];
	}

#ifdef ENV_64BITS
	template <typename T>
	T& Vec2<T>::operator[](const sp_size index)
	{
		sp_assert(index >= ZERO_SIZE && index < VEC2_SIZE);

		return reinterpret_cast<T*>(this)[index];
	}

	template <typename T>
	T Vec2<T>::operator[](const sp_size index) const
	{
		sp_assert(index >= ZERO_SIZE && index < VEC2_SIZE);

		return reinterpret_cast<const T*>(this)[index];
	}
#endif

	template <typename T>
	Vec2<T>::operator T*() const
	{
		return (T*) this;
	}

	template <typename T>
	std::ostream& operator<<(std::ostream& outputStream, const Vec2<T>& vector)
	{
		return outputStream << vector.x << "," << vector.y;
	}

	template <typename T>
	std::istream& operator>>(std::istream& inputStream, Vec2<T>& vector)
	{
		char separator;
		return inputStream >> vector.x >> separator >> vector.y;
	}

	template <typename T>
	std::ostream& Vec2<T>::serialize(std::ostream& outputStream) const
	{
		return outputStream << *this;
	}

	template <typename T>
	std::istream& Vec2<T>::deserialize(std::istream& inputStream)
	{
		inputStream >> *this;
		return inputStream;
	}

	template class Vec2<sp_int>;
	template class Vec2<sp_uint>;
	template class Vec2<sp_float>;
	template class Vec2<sp_double>;
#ifdef ENV_64BTIS
	template class Vec2<sp_size>;
#endif

}