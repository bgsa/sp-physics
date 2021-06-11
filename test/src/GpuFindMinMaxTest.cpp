#ifdef OPENCL_ENABLED

#ifndef GPU_FIND_MIN_MAX_TEST_HEADER
#define GPU_FIND_MIN_MAX_TEST_HEADER

#include "SpectrumPhysicsTest.h"
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

	AABB* getRandomAABBs(sp_size count, sp_uint spaceSize = 1000)
	{
		Randomizer randomizerSize(0, 30);
		Randomizer randomizerLocation(0, spaceSize);

		AABB* aabbs = ALLOC_NEW_ARRAY(AABB, count);

		for (sp_size i = 0; i < count; i++)
		{
			sp_int xMin = randomizerSize.randInt();
			sp_int yMin = randomizerSize.randInt();
			sp_int zMin = randomizerSize.randInt();

			sp_int xMax = randomizerSize.randInt();
			sp_int yMax = randomizerSize.randInt();
			sp_int zMax = randomizerSize.randInt();

			sp_int locationX = randomizerLocation.randInt();
			sp_int locationY = randomizerLocation.randInt();
			sp_int locationZ = randomizerLocation.randInt();

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

			aabbs[i] = AABB({ sp_float(xMin + locationX), sp_float(yMin + locationY), sp_float(zMin + locationZ) }
			, { sp_float(xMax + locationX), sp_float(yMax + locationY), sp_float(zMax + locationZ) });
		}

		return aabbs;
	}

	SP_TEST_CLASS(CLASS_NAME)
	{
	public:

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andSomeData);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andFewData);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andOdd);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint offsetCpu = AABB_OFFSET;

		const sp_size count = (sp_size)std::pow(2.0, 17.0);

		std::ostringstream buildOptions;
		buildOptions
			<< " -DLOCAL_MEM_LENGTH=" << gpu->localMemoryLength
			<< " -DINPUT_LENGTH=" << count
			<< " -DINPUT_STRIDE=" << AABB_STRIDER
			<< " -DINPUT_OFFSET=" << AABB_OFFSET;
#ifdef ENV_64BITS
		buildOptions << " -DENV_64BITS";
#endif

		const sp_uint maxIterations = 20;
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
			findMinMax->setParameters((sp_float*)aabbs, (sp_uint) count, AABB_STRIDER, AABB_OFFSET);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			findMinMax->execute();

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			sp_float* result = ALLOC_ARRAY(sp_float, 2);
			gpu->commandManager->readBuffer(findMinMax->output, 2 * sizeof(sp_float), result, ZERO_UINT, NULL);

			Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(findMinMax, GpuFindMinMax);
			ALLOC_RELEASE(aabbs);
		}
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets_andSomeData)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint offsetMultiplier = 8;
		const sp_uint offsetSum = 2;

		const sp_uint count = 20u;
		const sp_uint maxIteration = 20u;

		for (sp_uint i = 0u; i < maxIteration; i++)
		{
			AABB* aabbs = getRandomAABBs(count);

			sp_float min = FLT_MAX;
			sp_float max = -FLT_MAX;

			sp_uint expectedIndexesMinMax[2];
			for (sp_uint i = 0; i < count; i++)
			{
				if (aabbs[i].minPoint.x < min) {
					min = aabbs[i].minPoint.x;
					expectedIndexesMinMax[0] = i;
				}

				if (aabbs[i].minPoint.x > max) {
					max = aabbs[i].minPoint.x;
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
			gpu->commandManager->readBuffer(findMinMax->output, 2 * sizeof(sp_float), result, ZERO_UINT, NULL);

			Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(findMinMax, GpuFindMinMax);
			ALLOC_RELEASE(aabbs);
		}
	}
	
	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets_andOdd)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint offsetMultiplier = 8;
		const sp_uint offsetSum = 2;

		const sp_uint count = 3u;

		AABB* aabbs = getRandomAABBs(count);

		sp_float min = FLT_MAX;
		sp_float max = -FLT_MAX;

		aabbs[count - 1].minPoint.x = 999.0f; //

		sp_uint expectedIndexesMinMax[2];
		for (sp_uint i = 0; i < count; i++)
		{
			if (aabbs[i].minPoint.x < min) {
				min = aabbs[i].minPoint.x;
				expectedIndexesMinMax[0] = i;
			}

			if (aabbs[i].minPoint.x > max) {
				max = aabbs[i].minPoint.x;
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
		gpu->commandManager->readBuffer(findMinMax->output, 2 * sizeof(sp_float), result, ZERO_UINT, NULL);

		Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

		ALLOC_DELETE(findMinMax, GpuFindMinMax);
		ALLOC_RELEASE(aabbs);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets_andFewData)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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
			gpu->commandManager->readBuffer(findMinMax->output, 2 * sizeof(sp_float), result, ZERO_UINT, NULL);

			Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
			Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(findMinMax, GpuFindMinMax);
			ALLOC_RELEASE(aabbs);
		}
	}

}

#endif // GPU_FIND_MIN_MAX_TEST_HEADER

#endif // OPENCL