#pragma once

#include "OpenML.h"
#include <algorithm>
#include <functional>
#include <numeric>

#include "AABB.h"

#ifdef OPENCL_ENABLED
	#include "GpuCommands.h"
	#include "IFileManager.h"
	#include "Factory.h"
	#undef max
	#undef min
#endif

namespace OpenML
{

	class AlgorithmSorting
	{
	public:
		
		///<summary>
		///Fast sorting of numbers array using Counting method
		///Complexity O(n)
		///</summary>
		API_INTERFACE static void radix(sp_int* vector, sp_size count);

		///<summary>
		///Fast sorting of numbers array using Counting method
		///Complexity O(n)
		///</summary>
		API_INTERFACE static void radix(sp_size* vector, sp_size count);

		///<summary>
		///Fast sorting of numbers array using Radix method
		///Complexity O(n)
		///</summary>
		API_INTERFACE static void radix(sp_float* vector, sp_size count);

		///<summary>
		///Fast sorting of numbers array using "C" native method
		///Faster than radix sorting when large dataset >= 500k
		///</summary>
		API_INTERFACE static void native(sp_float* vetor, sp_size count);

		///<summary>
		///Fast sorting of numbers array using "C" native method
		///Faster than radix sorting when large dataset >= 500k
		///Returns the index of sorted vector
		///</summary>
		API_INTERFACE static sp_size* nativeIndex(sp_float* vector, sp_size count);

		///<summary>
		///Fast native sorting of custom type array using "C" native method
		///Faster than radix sorting when large dataset >= 500k
		///Returns the index of sorted vector
		///</summary>
		template <typename T>
		API_INTERFACE static sp_size* nativeIndex(T* vector, sp_size count, sp_bool(*comparator)(sp_int, sp_int, T*));

		///<summary>
		///Fast native Quick Sorting of custom type array using "C" native method
		///</summary>
		API_INTERFACE static void quickSortNative(void* vector, sp_size count, sp_size sizeOfOneElement, sp_int(*comparator)(const void*, const  void*));

	};

}