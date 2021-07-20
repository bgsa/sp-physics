#include "AlgorithmSorting.h"

namespace NAMESPACE_PHYSICS
{
	/*
	// ---- utils for accessing 11-bit quantities
	#define _0(x)	(x & 0x7FF)
	#define _1(x)	(x >> 11 & 0x7FF)
	#define _2(x)	(x >> 22 )

	#if PREFETCH
	#include <xmmintrin.h>	// for prefetch
	#define pfval	64
	#define pfval2	128
	#define pf(x)	_mm_prefetch(cpointer(x + i + pfval), 0)
	#define pf2(x)	_mm_prefetch(cpointer(x + i + pfval2), 0)
	#else
	#define pf(x)
	#define pf2(x)
	#endif

	// ================================================================================================
	// flip a float for sorting
	//  finds SIGN of fp number.
	//  if it's 1 (negative float), it flips all bits
	//  if it's 0 (positive float), it flips the sign only
	// ================================================================================================
	inline size_t FloatFlip(size_t f)
	{
		size_t mask = -int(f >> 31) | 0x80000000;
		return f ^ mask;
	}

	inline void FloatFlipX(size_t &f)
	{
		size_t mask = -int(f >> 31) | 0x80000000;
		f ^= mask;
	}

	// ================================================================================================
	// flip a float back (invert FloatFlip)
	//  signed was flipped from above, so:
	//  if sign is 1 (negative), it flips the sign bit back
	//  if sign is 0 (positive), it flips all bits back
	// ================================================================================================
	inline size_t IFloatFlip(size_t f)
	{
		size_t mask = ((f >> 31) - 1) | 0x80000000;
		return f ^ mask;
	}

	float* AlgorithmSorting::radix(float* vector, size_t count)
	{
		float* sorted = new float[count];

		size_t i;
		size_t *sort = (size_t*)sorted;
		size_t *array = (size_t*)vector;

		// 3 histograms on the stack:
		const size_t kHist = 2048;
		size_t b0[kHist * 3];

		size_t *b1 = b0 + kHist;
		size_t *b2 = b1 + kHist;

		for (i = 0; i < kHist * 3; i++)
			b0[i] = 0;
		//memset(b0, 0, kHist * 12);
		
		// 1.  parallel histogramming pass
		for (i = 0; i < count; i++) 
		{
			pf(array);

			size_t fi = FloatFlip((size_t&)array[i]);

			b0[_0(fi)] ++;
			b1[_1(fi)] ++;
			b2[_2(fi)] ++;
		}

		// 2.  Sum the histograms -- each histogram entry records the number of values preceding itself.
		{
			size_t sum0 = 0, sum1 = 0, sum2 = 0;
			size_t tsum;
			for (i = 0; i < kHist; i++) 
			{
				tsum = b0[i] + sum0;
				b0[i] = sum0 - 1;
				sum0 = tsum;

				tsum = b1[i] + sum1;
				b1[i] = sum1 - 1;
				sum1 = tsum;

				tsum = b2[i] + sum2;
				b2[i] = sum2 - 1;
				sum2 = tsum;
			}
		}

		// byte 0: floatflip entire value, read/write histogram, write out flipped
		for (i = 0; i < count; i++) 
		{
			size_t fi = array[i];
			FloatFlipX(fi);
			size_t pos = _0(fi);

			pf2(array);
			sort[++b0[pos]] = fi;
		}

		// byte 1: read/write histogram, copy sorted -> array
		for (i = 0; i < count; i++) 
		{
			size_t si = sort[i];
			size_t pos = _1(si);
			pf2(sort);
			array[++b1[pos]] = si;
		}

		// byte 2: read/write histogram, copy & flip out array -> sorted
		for (i = 0; i < count; i++) 
		{
			size_t ai = array[i];
			size_t pos = _2(ai);

			pf2(array);

			sort[++b2[pos]] = IFloatFlip(ai);
		}

		return sorted;
	}
	*/

	void AlgorithmSorting::radix(sp_float* vector, sp_size n)
	{
		const sp_size maxDigitMantissa = 4;
		sp_int tempExp;
		sp_int minElement = INT_MAX;
		sp_int maxElement = 0;
		sp_size exp;
		
		for (sp_size i = 0; i < n; i++)
		{
			tempExp = (sp_int)vector[i];

			if (tempExp > maxElement)
				maxElement = tempExp;

			if (tempExp < minElement)
				minElement = tempExp;
		}
		minElement = sp_abs(minElement);
		const sp_uint maxDigitExpoent = digitCount(sp_max(minElement, maxElement));

		sp_float* output = ALLOC_ARRAY(sp_float, n);
		sp_uint* digitsCache = ALLOC_ARRAY(sp_uint, n);
		const sp_uint bucketCount = 10;
		sp_uint bucket[bucketCount];
		sp_uint bucketIndex;

		for (sp_size digitIndex = 0; digitIndex < maxDigitMantissa; digitIndex++)
		{
			std::memset(bucket, 0, sizeof(sp_uint) * bucketCount);

			for (sp_size j = 0; j < n; j++)    //make histogram
			{
				bucketIndex = digitsCache[j] = digit(floatParts(vector[j], &exp), (sp_int)digitIndex);
				bucket[bucketIndex]++;
			}

			for (sp_uint j = 1; j < bucketCount; j++)
				bucket[j] += bucket[j - 1];

			for (sp_size j = n - 1; j != SP_SIZE_MAX; j--)
			{
				bucketIndex = digitsCache[j];

				output[bucket[bucketIndex] - 1] = vector[j];
				bucket[bucketIndex]--;
			}

			std::memcpy(vector, output, sizeof(sp_float) * n);
		}

		for (sp_uint digitIndex = 0; digitIndex < maxDigitExpoent; digitIndex++)
		{
			std::memset(bucket, 0, sizeof(sp_uint) * bucketCount);

			for (sp_uint j = 0; j < n; j++)    //make histogram
			{
				bucketIndex = digitsCache[j] = digit((sp_int)vector[j] + minElement, (sp_int) digitIndex);
				bucket[bucketIndex]++;
			}

			for (sp_size j = 1; j < bucketCount; j++)
				bucket[j] += bucket[j - 1];

			for (sp_size j = n - 1; j != SP_SIZE_MAX; j--)
			{
				bucketIndex = digitsCache[j];

				output[bucket[bucketIndex] - 1] = vector[j];
				bucket[bucketIndex]--;
			}

			std::memcpy(vector, output, sizeof(sp_float) * n);
		}

		ALLOC_RELEASE(output);
	}

	void AlgorithmSorting::radix(sp_size *vector, sp_size n)
	{
		sp_size maxElement = 0;

		for (sp_size i = 0; i < n; i++)
			if (vector[i] > maxElement)
				maxElement = vector[i];

		sp_uint maxDigit = digitCount(maxElement);

		sp_size* output = ALLOC_ARRAY(sp_size, n);
		sp_uint* digitsCache = ALLOC_ARRAY(sp_uint, n);
		const sp_uint bucketCount = 10;
		sp_uint bucket[bucketCount];
		sp_uint bucketIndex;

		for (sp_uint digitIndex = 0; digitIndex < maxDigit; digitIndex++)
		{
			std::memset(bucket, 0, sizeof(sp_uint) * bucketCount);

			for (sp_size j = 0; j < n; j++)    //make histogram
			{
				bucketIndex = digitsCache[j] = digit((sp_uint)vector[j], digitIndex);
				bucket[bucketIndex]++;
			}

			for (sp_uint j = 1; j < bucketCount; j++)
				bucket[j] += bucket[j - 1];

			for (sp_size j = n - 1; j != SP_SIZE_MAX; j--)
			{
				bucketIndex = digitsCache[j];

				output[bucket[bucketIndex] - 1] = vector[j];
				bucket[bucketIndex]--;
			}

			std::memcpy(vector, output, sizeof(sp_size) * n);
		}

		ALLOC_RELEASE(output);
	}

	void AlgorithmSorting::radix(sp_int* vector, sp_size n)
	{
		sp_int maxElement = INT_MIN;
		sp_int minElement = INT_MAX;

		for (sp_uint i = 0; i < n; i++)
		{
			if (vector[i] > maxElement)
				maxElement = vector[i];

			if (vector[i] < minElement)
				minElement = vector[i];
		}

		sp_uint maxDigit = digitCount(maxElement);
		maxDigit = sp_max(maxDigit, (sp_uint) digitCount(minElement));

		if (minElement > 0)
			minElement = 0;
		else
			minElement *= -1;

		sp_int* output = ALLOC_ARRAY(sp_int, n);
		sp_int* digitsCache = ALLOC_ARRAY(sp_int, n);
		const sp_uint bucketCount = 10;
		sp_uint bucket[bucketCount];
		sp_uint bucketIndex;

		for (sp_uint digitIndex = 0; digitIndex < maxDigit; digitIndex++)
		{
			std::memset(bucket, 0, sizeof(sp_uint) * bucketCount);

			for (sp_uint j = 0; j < n; j++)    //make histogram
			{
				bucketIndex = digitsCache[j] = digit( (sp_uint)(vector[j] + minElement), digitIndex);
				bucket[bucketIndex]++;
			}

			for (sp_uint j = 1; j < bucketCount; j++)
				bucket[j] += bucket[j - 1];

			for (sp_size j = n - 1; j != SP_SIZE_MAX; j--)
			{
				bucketIndex = digitsCache[j];

				output[bucket[bucketIndex] - 1] = vector[j];
				bucket[bucketIndex]--;
			}

			std::memcpy(vector, output, sizeof(sp_int) * n);
		}

		ALLOC_RELEASE(output);
	}

	void AlgorithmSorting::native(float* vector, size_t count)
	{
		std::sort(vector, vector + count, std::less<float>());
	}

	bool compare(int a, int b, float* data)
	{
		return data[a] < data[b];
	}

	size_t* AlgorithmSorting::nativeIndex(float* vector, size_t count)
	{
		size_t* index = ALLOC_ARRAY(size_t, count);

		std::iota(index, index + count, 0); // fill index with {0,1,2,...} This only needs to happen once

		std::sort(index, index + count, std::bind(compare, std::placeholders::_1, std::placeholders::_2, vector));

		return index;
	}

	template <typename T>
	size_t* AlgorithmSorting::nativeIndex(T* vector, size_t count, bool(*comparator)(int, int, T*))
	{
		size_t* index = ALLOC_ARRAY(size_t, count);

		std::iota(index, index + count, 0); // fill index with {0,1,2,...} This only needs to happen once

		std::sort(index, index + count, std::bind(comparator, std::placeholders::_1, std::placeholders::_2, vector));
			
		return index;
	}
	template size_t* AlgorithmSorting::nativeIndex(AABB* vector, size_t count, bool(*comparator)(int, int, AABB*));


	void AlgorithmSorting::quickSortNative(void* vector, size_t count, size_t sizeOfOneElement, int(*comparator)(const void*, const void*))
	{
		std::qsort(vector, count, sizeOfOneElement, comparator);
	}
}