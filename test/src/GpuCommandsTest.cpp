#ifdef OPENCL_ENABLED

#include "TestHeader.h"
#include "Randomizer.h"
#include <GpuCommands.h>
#include <GpuFindMinMax.h>
#include <limits>
#include <AABB.h>

#define CLASS_NAME GpuCommandsTest

#undef max
#undef min

namespace SP_PHYSICS_TEST_NAMESPACE
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

		SP_TEST_METHOD_DEF(GpuCommands_createIndexes);

		SP_TEST_METHOD_DEF(GpuCommands_createIndexes_Many);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andFewData);

	};

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_createIndexes)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const size_t count = 10;

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";
		GpuCommands::init(gpu, buildOptions.str().c_str());

		cl_mem indexes = GpuCommands::creteIndexes(gpu, count);

		sp_size* values = ALLOC_ARRAY(sp_size, count);
		gpu->commandManager->executeReadBuffer(indexes, count * SIZEOF_SIZE, values, true);

		for (size_t i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(indexes);
		ALLOC_RELEASE(values);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_createIndexes_Many)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const size_t count = (size_t)powf(2.0f, 17.0f);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";
		GpuCommands::init(gpu, buildOptions.str().c_str());

		cl_mem indexes = GpuCommands::creteIndexes(gpu, count);

		size_t* values = ALLOC_ARRAY(size_t, count);
		gpu->commandManager->executeReadBuffer(indexes, count * SIZEOF_UINT, (void*)values, true);

		for (size_t i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(indexes);
		ALLOC_RELEASE(values);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxIndexesGPU_withOffsets)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint offsetCpu = AABB_OFFSET;

		const sp_uint maxIterations = 100;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		const size_t count = (size_t)std::pow(2.0, 17.0);
		const size_t threadCount = gpu->getThreadLength(count);

		std::ostringstream buildOptions;
		buildOptions
			<< " -DLOCAL_MEM_LENGTH=" << gpu->localMemoryLength
			<< " -DINPUT_LENGTH=" << count
			<< " -DINPUT_STRIDE=" << AABB_STRIDER
			<< " -DINPUT_OFFSET=" << AABB_OFFSET
			<< " -DINPUT_OFFSET=" << AABB_OFFSET;

		for (size_t i = 0; i < maxIterations; i++)
		{
			AABB* aabbs = getRandomAABBs(count);

			float min = FLT_MAX;
			float max = -FLT_MAX;

			size_t expectedIndexesMinMax[2];
			for (size_t i = 0; i < count; i++)
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
			findMinMax->setParameters((float*)aabbs, count, AABB_STRIDER, AABB_OFFSET);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			findMinMax->execute();

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			float* result = ALLOC_ARRAY(float, threadCount * 2);
			gpu->commandManager->executeReadBuffer(findMinMax->output, threadCount * 2 * SIZEOF_FLOAT, result, true);

			float temp = 0;
			size_t tt = 0;
			for (size_t i = 0; i < threadCount * 2; i++)
				if (result[i] > temp)
				{
					temp = result[i];
					tt = i;
				}

			//Assert::AreEqual(temp, result[256 * (tt / 256) + 1], L"Wrong value.", LINE_INFO());

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
		GpuCommands::init(gpu, NULL);

		const size_t offsetMultiplier = 8;
		const size_t offsetSum = 2;

		const size_t count = 5;
		AABB* aabbs = getRandomAABBs(count);

		float min = FLT_MAX;
		float max = -FLT_MAX;

		size_t expectedIndexesMinMax[2];
		for (size_t i = 0; i < count; i++)
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

		size_t threadCount = gpu->getThreadLength(count);

		cl_mem indexes = GpuCommands::creteIndexes(gpu, count);
		cl_mem elements = gpu->createBuffer((void*)aabbs, sizeof(AABB) * count * SIZEOF_FLOAT, CL_MEM_READ_WRITE);
		cl_mem indexesLength = gpu->createBuffer((void*)&count, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem offset = gpu->createBuffer((void*)&offsetSum, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem output = gpu->createBuffer(threadCount * 2 * SIZEOF_FLOAT, CL_MEM_READ_WRITE);

		GpuCommands::findMinMaxIndexesGPU(gpu, elements, indexes, indexesLength, offset, count, offsetMultiplier, output);

		float* result = ALLOC_ARRAY(float, threadCount * 2);
		gpu->commandManager->executeReadBuffer(output, threadCount * 2 * SIZEOF_FLOAT, result, true);

		Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(elements);
		gpu->releaseBuffer(indexes);
		gpu->releaseBuffer(indexesLength);
		gpu->releaseBuffer(offset);
		gpu->releaseBuffer(output);
		ALLOC_RELEASE(aabbs);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED