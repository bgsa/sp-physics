#ifndef SPECTRUM_PHYSICS_HEADER
#define SPECTRUM_PHYSICS_HEADER

#include <SpectrumFoundation.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <sstream>
#include <stdarg.h>
#include <limits.h>

#ifndef NAMESPACE_PHYSICS
	#define NAMESPACE_PHYSICS SpPhyiscs
#endif

using namespace NAMESPACE_FOUNDATION;

#define EULER_NUMBER (2.71828f)   // e^1 = 2.71828

#define MAX_DIGITS_EXPOENT  (5)
#define MAX_DIGITS_MANTISSA (4)

namespace NAMESPACE_PHYSICS
{
	const float DefaultErrorMargin = 0.0001f;

	template <typename T>
	class Vec2;
	template <typename T>
	class Vec3;
	template <typename T>
	class Vec4;

	template <typename T>
	class Vec2List;
	template <typename T>
	class Vec3List;

	template <typename T>
	class Mat;
	template <typename T>
	class Mat2;
	template <typename T>
	class Mat3;
	template <typename T>
	class Mat4;

	template <typename T>
	class Quat;

	template <class T>
	class BoundingVolume;

	template <typename T>
	class BinaryTree;
	template <typename T>
	class BinaryTreeNode;

	template <typename T>
	class Line2D;
	class Line3D;
	class Plane3D;
	template <typename T>
	class Triangle2D;
	template <typename T>
	class Circle2D;

	class AABB;
	class OBB;
	class Sphere;
	class DOP18;

	template <typename T>
	class Rectangle2D;

	///<summary>
	///Check the number is even or not
	///</summary>
	template <typename T>
	inline sp_bool API_INTERFACE isEven(T value)
	{
		return (value % 2) == 0;
	}
	template <>
	inline sp_bool API_INTERFACE isEven<sp_int>(sp_int value)
	{
		return !(value & 1);
	}
	template <>
	inline sp_bool API_INTERFACE isEven<size_t>(size_t value)
	{
		return !(value & 1);
	}
		
	///<summary>
	///Check the number is odd or not
	///</summary>
	template <typename T>
	inline sp_bool API_INTERFACE isOdd(T value)
	{
		return ! isEven(value);
	}
	template <>
	inline sp_bool API_INTERFACE isOdd<sp_int>(sp_int value)
	{
		return value & 1;
	}
	template <>
	inline sp_bool API_INTERFACE isOdd<size_t>(size_t value)
	{
		return value & 1;
	}

	///<summary>
	///Modify a bit of a integer value, given a index bit
	///</summary>
	inline sp_int API_INTERFACE modifyBit(sp_int value, sp_int index, sp_int bit)
	{
		return (value & ~(SHIFT_BIT_ONE << index)) | ((bit << index) & (SHIFT_BIT_ONE << index));
	}

	///<summary>
	///Set a bit to ZERO of a integer value, given a index bit
	///</summary>
	inline sp_int API_INTERFACE clearBit(sp_int value, sp_int index)
	{
		return modifyBit(value, index, 0);
	}

	///<summary>
	///Set a bit to ONE of a integer value, given a index bit
	///</summary>
	inline sp_int API_INTERFACE setBit(sp_int value, sp_int index)
	{
		return modifyBit(value, index, 1);
	}

	///<summary>
	///GFet a bit of a integer value, given a index bit
	///</summary>
	inline sp_int API_INTERFACE getBit(sp_int value, sp_int index)
	{
		return (value & (SHIFT_BIT_ONE << index)) >> index;
	}
	///<summary>
	///GFet a bit of a integer value, given a index bit
	///</summary>
	inline sp_int API_INTERFACE getBit(size_t value, sp_int index)
	{
		return (value & (SHIFT_BIT_ONE << index)) >> index;
	}

	///<summary>
	///Get the parts of float (expoent and mantissa)
	///The mantissa just returns the 4th numbers
	///</summary>
	inline size_t API_INTERFACE floatParts(sp_float value, size_t* expoent)
	{
		*expoent = (size_t)value;
		return size_t(fabsf(*expoent - value) * 10000.0f);
	}

	///<summary>
	///Get a digit of the number given by value parameter and the index
	///</summary>
	inline sp_short API_INTERFACE digit(sp_int value, sp_int index)
	{
		sp_short result = (sp_short)((sp_uint)(value / std::pow(DECIMAL_BASE, index)) % DECIMAL_BASE);

		assert(result >= 0 && result < 10);
		return result;
	}
	///<summary>
	///Get a digit of the number given by value parameter and the index
	///</summary>
	inline sp_short API_INTERFACE digit(sp_uint value, sp_uint index)
	{
		sp_short result = (sp_short)((sp_uint)(value / std::pow(DECIMAL_BASE, index)) % DECIMAL_BASE);

		assert(result >= 0 && result < 10);
		return result;
	}
	///<summary>
	///Get a digit of the number given by value parameter and the index
	///</summary>
	inline sp_short API_INTERFACE digit(sp_float value, sp_int index)
	{
		sp_short result = (sp_short)( (sp_uint)(value / std::pow(DECIMAL_BASE, index)) % DECIMAL_BASE);

		assert(result >= 0 && result < 10);
		return result;
	}
	///<summary>
	///Get a digit of the number given by value parameter and the index
	///</summary>
	inline sp_short API_INTERFACE digit(sp_size value, sp_int index)
	{
		sp_short result = (sp_short)((sp_uint)(value / std::pow(DECIMAL_BASE, index)) % DECIMAL_BASE);

		assert(result >= 0 && result < 10);
		return result;
	}

	///<summary>
	///Check the numbers have the same sign
	///</summary>
	inline sp_bool API_INTERFACE sameSign(sp_int value1, sp_int value2)
	{
		return (value1 ^ value2) >= 0;
	}

	///<summary>
	///Return -1 if value <  0
	///Return  0 if value == 0
	///Return  1 if value >  0
	///</summary>
	template <typename T> 
	inline int API_INTERFACE sign(T value) 
	{
		return (T(0) < value) - (value < T(0));
	}

	///<summary>
	///Check the number is power of 2
	///</summary>
	API_INTERFACE inline sp_bool isPowerOf2(size_t value)
	{
		return value && !(value & (value - 1));
	}

	///<summary>
	///Get the next number power of 2
	///</summary>
	API_INTERFACE inline sp_uint nextPowOf2(sp_uint value)
	{
		sp_uint rval = 1;

		while (rval < value) 
			rval = rval << 1; // multiply by 2

		return rval;
	}

	///<summary>
	///Get the next number power of 2
	///</summary>
	API_INTERFACE inline sp_int nextPowOf2(sp_int value)
	{
		sp_int rval = 1;

		while (rval < value)
			rval = rval << 1; // multiply by 2

		return rval;
	}

	API_INTERFACE inline sp_uint nextDivisorOf(sp_uint n, sp_uint startFrom)
	{
		for (sp_uint i = startFrom; i <= n; i++)
			if (n % i == 0)
				return i;

		return n;
	}


#if defined(WINDOWS) && defined(ENV_64BITS)
	///<summary>
	///Get the next number power of 2
	///</summary>
	inline sp_size API_INTERFACE nextPowOf2(sp_size value)
	{
		sp_size rval = 1;

		while (rval < value)
			rval = rval << 1; // multiply by 2

		return rval;
	}
#endif
		
	///<summary>
	///Round the number given a amount of decimals
	///</summary>
	template<typename T>		
	inline T API_INTERFACE round(T number, sp_int decimals)
	{
		sp_double m = (number < 0.0) ? -1.0 : 1.0;   // check if input is negative
		sp_double power = pow(10, decimals);

		return T((floor(m * number * power + 0.5) / power) * m);
	}

	///<summary>
	///Round the number (in float) given a amount of decimals
	///</summary>
	inline sp_float API_INTERFACE roundf(sp_float number, sp_int decimals)
	{
		return round<sp_float>(number, decimals);
	}
	
	///<summary>
	///Round the number (in double) given a amount of decimals
	///</summary>
	inline sp_double API_INTERFACE roundd(sp_double number, sp_int decimals)
	{
		return round<sp_double>(number, decimals);
	}

	///<summary>
	///
	///</summary>
	template <typename T>
	inline T API_INTERFACE clamp(T value, T minValue, T maxValue)
	{
		if (value < minValue)
			value = minValue;
		else
			if (value > maxValue)
				value = maxValue;

		return value;
	}

	///<summary>
	///Check the number is close enough given a other number. It is used to check aproximation value and calculate the error measure.
	///Epsilon is the tolerated value
	///</summary>
	template<typename T>
	inline sp_bool API_INTERFACE isCloseEnough(T value, T compare, T epsilon)
	{
		return abs(value - compare) < epsilon;
	}

	///<summary>
	///Check the number is close enough given a other number. It is used to check aproximation value and calculate the error measure.
	///</summary>
	template<typename T>
	inline sp_bool API_INTERFACE isCloseEnough(T value, T compare)
	{
		return isCloseEnough(value, compare, T(DefaultErrorMargin));
	}

	///<summary>
	///Get the count of digits of the number given by value parameter
	///</summary>
	inline sp_short API_INTERFACE digitCount(sp_int value)
	{
		sp_short len = 1;

		if (value < 0)
			value *= -1;

		for (len = 0; value > 0; len++)
			value = value / 10;

		return len;
	}
	///<summary>
	///Get the count of digits of the number given by value parameter
	///</summary>
	inline sp_size API_INTERFACE digitCount(sp_size value)
	{
		sp_size len = 1;

		for (len = 0; value > 0; len++)
			value = value / 10;

		return len;
	}
	///<summary>
	///Get the count of digits of the number given by value parameter
	///</summary>
	inline size_t API_INTERFACE digitMantissaCount(sp_float value)
	{
		sp_float temp;
		sp_float d = modff(value, &temp);
		size_t counter = 0;

		while (d > 0 && !isCloseEnough(d, 0.0f)) {
			d *= 10;
			d = d - ((size_t)d);
			counter++;
		}

		return counter;
	}

}

#include "CollisionStatus.h"

#include "Vec2.h"
#include "Vec3.h"
#include "Vec4.h"

#include "Vec2List.h"
#include "Vec3List.h"

#include "Mat.h"
#include "Mat2.h"
#include "Mat3.h"
#include "Mat4.h"

#include "Quat.h"

#include "Line2D.h"
#include "Circle2D.h"

#include "Rectangle2D.h"

#endif // !OPENML_HEADER