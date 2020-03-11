#if OPENCL_ENABLED

#include "TestHeader.h"
#include "Randomizer.h"
#include <GpuRadixSorting.h>
#include <AABB.h>
#include <AlgorithmSorting.h>

#define CLASS_NAME GpuRadixSortingTest

namespace NAMESPACE_PHYSICS_TEST
{

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

	public:

	};

}

#undef CLASS_NAME

#endif
