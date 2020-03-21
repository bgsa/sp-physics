#ifdef OPENCL_ENABLED

#ifndef GPU_FIND_MIN_MAX_TEST_HEADER
#define GPU_FIND_MIN_MAX_TEST_HEADER

#include "TestHeader.h"
#include "Randomizer.h"
#include <GpuFindMinMax.h>
#include "GpuContext.h"
#include <limits>
#include <AABB.h>

#define CLASS_NAME GpuFindMinMaxTest

#undef max
#undef min

namespace NAMESPACE_PHYSICS_TEST
{

	AABB* getRandomAABBs(size_t count, size_t spaceSize = 1000)
	{
		Randomizer<int> randomizerSize(0, 30);
		Randomizer<int> randomizerLocation(0, spaceSize);

		AABB* aabbs = ALLOC_NEW_ARRAY(AABB, count);

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

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andSomeData);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andFewData);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint offsetCpu = AABB_OFFSET;

		const sp_size count = (sp_size)std::pow(2.0, 17.0);

		std::ostringstream buildOptions;
		buildOptions
			<< " -DLOCAL_MEM_LENGTH=" << gpu->localMemoryLength
			<< " -DINPUT_LENGTH=" << count
			<< " -DINPUT_STRIDE=" << AABB_STRIDER
			<< " -DINPUT_OFFSET=" << AABB_OFFSET;

		const sp_uint maxIterations = 50;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		for (sp_size i = 0; i < maxIterations; i++)
		{
			AABB* aabbs = getRandomAABBs(count);

			float min = FLT_MAX;
			float max = -FLT_MAX;

			sp_size expectedIndexesMinMax[2];
			for (sp_size i = 0; i < count; i++)
			{
				if (aabbs[i].minPoint.x > max) {
					max = aabbs[i].minPoint.x;
					expectedIndexesMinMax[0] = i;
				}

				if (aabbs[i].minPoint.x < min) {
					min = aabbs[i].minPoint.x;
					expectedIndexesMinMax[1] = i;
				}
			}

			GpuFindMinMax* findMinMax = ALLOC_NEW(GpuFindMinMax)();
			findMinMax->init(gpu, buildOptions.str().c_str());
			findMinMax->setParameters((sp_float*)aabbs, count, AABB_STRIDER, AABB_OFFSET);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			findMinMax->execute();

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			sp_float* result = ALLOC_ARRAY(sp_float, 2);
			gpu->commandManager->executeReadBuffer(findMinMax->output, 2 * SIZEOF_FLOAT, result, true);

			Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(findMinMax, GpuFindMinMax);
			ALLOC_RELEASE(aabbs);
		}
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets_andSomeData)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint offsetMultiplier = 8;
		const sp_uint offsetSum = 2;

		const sp_uint count = 100u;

		for (sp_uint i = 0u; i < 100u; i++)
		{
			AABB* aabbs = getRandomAABBs(count);

			sp_float min = FLT_MAX;
			sp_float max = -FLT_MAX;

			sp_uint expectedIndexesMinMax[2];
			for (sp_uint i = 0; i < count; i++)
			{
				if (aabbs[i].minPoint.x > max) {
					max = aabbs[i].minPoint.x;
					expectedIndexesMinMax[0] = i;
				}

				if (aabbs[i].minPoint.x < min) {
					min = aabbs[i].minPoint.x;
					expectedIndexesMinMax[1] = i;
				}
			}

			std::ostringstream buildOptions;
			buildOptions
				<< " -DLOCAL_MEM_LENGTH=" << gpu->localMemoryLength
				<< " -DINPUT_LENGTH=" << count
				<< " -DINPUT_STRIDE=" << AABB_STRIDER
				<< " -DINPUT_OFFSET=" << AABB_OFFSET;

			GpuFindMinMax* findMinMax = ALLOC_NEW(GpuFindMinMax)();
			findMinMax->init(gpu, buildOptions.str().c_str());
			findMinMax->setParameters((sp_float*)aabbs, count, AABB_STRIDER, AABB_OFFSET);

			findMinMax->execute();

			sp_float* result = ALLOC_ARRAY(sp_float, 2);
			gpu->commandManager->executeReadBuffer(findMinMax->output, 2 * SIZEOF_FLOAT, result, true);

			Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(findMinMax, GpuFindMinMax);
			ALLOC_RELEASE(aabbs);
		}
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets_andFewData)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint offsetMultiplier = 8;
		const sp_uint offsetSum = 2;

		const sp_uint count = 5;

		for (sp_uint i = 0; i < 100; i++)
		{
			AABB* aabbs = getRandomAABBs(count);

			sp_float min = FLT_MAX;
			sp_float max = -FLT_MAX;

			sp_uint expectedIndexesMinMax[2];
			for (sp_uint i = 0; i < count; i++)
			{
				if (aabbs[i].minPoint.x > max) {
					max = aabbs[i].minPoint.x;
					expectedIndexesMinMax[0] = i;
				}

				if (aabbs[i].minPoint.x < min) {
					min = aabbs[i].minPoint.x;
					expectedIndexesMinMax[1] = i;
				}
			}

			sp_uint threadCount = gpu->getGridConfigForOneDimension(count)[0];

			std::ostringstream buildOptions;
			buildOptions
				<< " -DLOCAL_MEM_LENGTH=" << gpu->localMemoryLength
				<< " -DINPUT_LENGTH=" << count
				<< " -DINPUT_STRIDE=" << AABB_STRIDER
				<< " -DINPUT_OFFSET=" << AABB_OFFSET;

			GpuFindMinMax* findMinMax = ALLOC_NEW(GpuFindMinMax)();
			findMinMax->init(gpu, buildOptions.str().c_str());
			findMinMax->setParameters((sp_float*)aabbs, count, AABB_STRIDER, AABB_OFFSET);

			findMinMax->execute();

			sp_float* result = ALLOC_ARRAY(sp_float, 2);
			gpu->commandManager->executeReadBuffer(findMinMax->output, 2 * SIZEOF_FLOAT, result, true);

			Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(findMinMax, GpuFindMinMax);
			ALLOC_RELEASE(aabbs);
		}
	}

}

#endif // GPU_FIND_MIN_MAX_TEST_HEADER

#endif // OPENCL