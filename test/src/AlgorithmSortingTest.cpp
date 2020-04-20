#include "SpectrumPhysicsTest.h"
#include <AlgorithmSorting.h>
#include "Randomizer.h"

#define CLASS_NAME AlgorithmSortingTest

namespace NAMESPACE_PHYSICS_TEST
{

	static int comparatorFloatTest(const void* param1, const void* param2)
	{
		const float obj1 = *(float*)param1;
		const float obj2 = *(float*)param2;

		if (obj1 < obj2)
			return -1;
		else
			if (obj1 > obj2)
				return 1;

		return 0;
	}

	static int comparatorAABBirstAxisTest(const void* param1, const void* param2)
	{
		const AABB obj1 = *(AABB*)param1;
		const AABB obj2 = *(AABB*)param2;

		if (obj1.minPoint.x == obj2.minPoint.x)
			return 0;

		if (obj1.minPoint.x < obj2.minPoint.x)
			return -1;

		return 1;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:

		float* getRandom(size_t count, size_t spaceSize = 10000)
		{
			Randomizer<int> randomizer(0, spaceSize);

			float* result = ALLOC_ARRAY(float, count);

			for (size_t i = 0; i < count; i++)
				result[i] = randomizer.rand() / 100.0f;

			return result;
		}

		AABB* getRandomAABBs(size_t count, size_t spaceSize = 1000)
		{
			Randomizer<int> randomizerSize(0, 30);
			Randomizer<int> randomizerLocation(0, spaceSize);

			AABB* aabbs = ALLOC_ARRAY(AABB, count);

			for (size_t i = 0; i < count; i++)
			{
				int xMin = randomizerSize.rand();
				int yMin = randomizerSize.rand();
				int zMin = randomizerSize.rand();

				int xMax = randomizerSize.rand();
				int yMax = randomizerSize.rand();
				int zMax = randomizerSize.rand();

				int locationX = randomizerLocation.rand();
				int locationY = randomizerLocation.rand();
				int locationZ = randomizerLocation.rand();

				if (xMin == xMax)
					xMax++;

				if (yMin == yMax)
					yMax++;

				if (zMin == zMax)
					zMax++;

				if (xMin > xMax)
					std::swap(xMin, xMax);

				if (yMin > yMax)
					std::swap(yMin, yMax);

				if (zMin > zMax)
					std::swap(zMin, zMax);

				aabbs[i] = AABB({ float(xMin + locationX), float(yMin + locationY), float(zMin + locationZ) }
				, { float(xMax + locationX), float(yMax + locationY), float(zMax + locationZ) });
			}

			return aabbs;
		}

	public:

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test1);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test2);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test3);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test4);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test5);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test6);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radix_Test7);

		SP_TEST_METHOD_DEF(AlgorithmSorting_native_Test1);

		SP_TEST_METHOD_DEF(AlgorithmSorting_nativeIndex_Test1);

		SP_TEST_METHOD_DEF(AlgorithmSorting_quickSortNnative_Test1);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test1)
	{
		const sp_size count = 7;

		sp_float vec[count] = { 3.4f, 2.0f, 7.0f, 3.1f, -2.0f, 3.12f, 3.01f };
		sp_float expected[count] = { -2.0f, 2.0f, 3.01f, 3.1f, 3.12f, 3.4f, 7.0f };
		
		AlgorithmSorting::radix(vec, count);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], vec[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test2)
	{
		sp_size result[8] = { 2, 5,3,0,2,3,0,3 };
		sp_size expected[8] = { 0,0,2,2,3,3,3,5 };

		AlgorithmSorting::radix(result, 8);

		for (sp_uint i = 0; i < 8; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test3)
	{
		sp_size result[8] = { 200, 54, 35, 10, 4, 3, 50, 33 };
		sp_size expected[8] = { 3, 4, 10, 33, 35, 50, 54, 200 };

		AlgorithmSorting::radix(result, 8);

		for (sp_uint i = 0; i < 8; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test4)
	{
		sp_int result[8] = { 200, 54, 35, -10, 4, 3, -50, 33 };
		sp_int expected[8] = { -50, -10, 3, 4, 33, 35, 54, 200 };
		
		AlgorithmSorting::radix(result, 8);

		for (sp_uint i = 0; i < 8; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test5)
	{
		const sp_size count = 6;
		sp_float result[count] = { 0.1f,0.111f,0.23f,0.03f,0.53f,0.08f };
		sp_float expected[count] = { 0.03f, 0.08f, 0.1f, 0.111f, 0.23f, 0.53f };

		AlgorithmSorting::radix(result, count);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test6)
	{
		const sp_size count = 6;
		sp_float result[count] = { 57.1f, 0.111f, 1.23f, 0.03f, 0.53f, 2.08f };
		sp_float expected[count] = { 0.03f, 0.111f, 0.53f, 1.23f, 2.08f, 57.1f };

		AlgorithmSorting::radix(result, count);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radix_Test7)
	{
		const sp_size count = 7;
		sp_float result[count] = { 57.1f, 0.111f, -1.23f, 0.03f, 0.53f, 2.08f, -34567.56f };
		sp_float expected[count] = { -34567.56f, -1.23f, 0.03f, 0.111f, 0.53f, 2.08f, 57.1f };

		AlgorithmSorting::radix(result, count);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_native_Test1)
	{
		const sp_size count = 7;

		sp_float vec[count] = { 3.4f, 2.0f, 7.0f, 3.1f, -2.0f, 3.12f, 3.01f };
		sp_float expected[count] = { -2.0f, 2.0f, 3.01f, 3.1f, 3.12f, 3.4f, 7.0f };

		AlgorithmSorting::native(vec, count);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], vec[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_nativeIndex_Test1)
	{
		const sp_size count = 7;

		sp_float vec[count] = { 3.4f, 2.0f, 7.0f, 3.1f, -2.0f, 3.12f, 3.01f };
		sp_size expected[count] = { 4, 1, 6, 3, 5, 0, 2 };

		sp_size* result = AlgorithmSorting::nativeIndex(vec, count);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_quickSortNnative_Test1)
	{
		const sp_size count = 7;

		sp_float vec[count] = { 3.4f, 2.0f, 7.0f, 3.1f, -2.0f, 3.12f, 3.01f };
		sp_float expected[count] = { -2.0f, 2.0f, 3.01f, 3.1f, 3.12f, 3.4f, 7.0f };

		AlgorithmSorting::quickSortNative(vec, count, SIZEOF_FLOAT, comparatorFloatTest);

		for (sp_uint i = 0; i < count; i++)
			Assert::AreEqual(expected[i], vec[i], L"Wrong value.", LINE_INFO());
	}

}

#undef CLASS_NAME
