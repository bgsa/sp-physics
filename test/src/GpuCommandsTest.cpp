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

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxGPU);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxGPU_withOffsets);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets);

		SP_TEST_METHOD_DEF(GpuCommands_findMinMaxIndexesGPU_withOffsets_andFewData);

		SP_TEST_METHOD_DEF(GpuCommands_findMaxGPU);

		SP_TEST_METHOD_DEF(GpuCommands_findMaxGPUBuffer);
		
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

		size_t* values = ALLOC_ARRAY(size_t, count);
		gpu->commandManager->executeReadBuffer(indexes, count * SIZEOF_UINT, (void*)values, true);

		for (size_t i = 0; i < count; i++)
			Assert::AreEqual(i, values[i], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(indexes);
		ALLOC_RELEASE(values);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_createIndexes_Many)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const size_t count = (size_t) powf(2.0f, 17.0f);

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

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxGPU)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const size_t count = (size_t)std::pow(2.0, 17.0);
		float* input = getRandom(count);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";
		GpuCommands::init(gpu, buildOptions.str().c_str());

		float min = std::numeric_limits<float>().max();
		float max = std::numeric_limits<float>().min();

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		for (size_t i = 0; i < count; i++)
		{
			if (input[i] > max)
				max = input[i];

			if (input[i] < min)
				min = input[i];
		}

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		currentTime = std::chrono::high_resolution_clock::now();

		float* result = GpuCommands::findMinMaxGPU(gpu, input, count, 1, 0);

		currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMinMaxGPU_withOffsets)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const size_t offsetMultiplier = 8;
		const size_t offsetSum = 2;

		const size_t count = (size_t)std::pow(2.0, 17.0);
		AABB* aabbs = getRandomAABBs(count);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=" << AABB_STRIDER;
		buildOptions << " -DINPUT_OFFSET=" << AABB_OFFSET;
		GpuCommands::init(gpu, buildOptions.str().c_str());

		float min = FLT_MAX;
		float max = -FLT_MAX;

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		for (size_t i = 0; i < count; i++)
		{
			if (aabbs[i].minPoint.x > max)
				max = aabbs[i].minPoint.x;

			if (aabbs[i].minPoint.x < min)
				min = aabbs[i].minPoint.x;
		}

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		currentTime = std::chrono::high_resolution_clock::now();

		float* result = GpuCommands::findMinMaxGPU(gpu, (float*)aabbs, count, offsetMultiplier, offsetSum);
		
		currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(min, result[0], L"Wrong value.", LINE_INFO());
		Assert::AreEqual(max, result[1], L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(aabbs);
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

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMaxGPU)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;
		GpuCommands::init(gpu, NULL);

		const size_t count = (size_t)std::pow(2.0, 17.0);
		float* input = getRandom(count);

		float max = -FLT_MAX;

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		for (size_t i = 0; i < count; i++)
			if (input[i] > max)
				max = input[i];

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		currentTime = std::chrono::high_resolution_clock::now();

		float result = GpuCommands::findMaxGPU(gpu, input, count, 1, 0);

		currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(max, result, L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, GpuCommands_findMaxGPUBuffer)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;
		GpuCommands::init(gpu, NULL);

		const size_t count = (size_t)std::pow(2.0, 17.0);
		float* input = getRandom(count);

		float max = -FLT_MAX;

		std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

		for (size_t i = 0; i < count; i++)
			if (input[i] > max)
				max = input[i];

		std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		currentTime = std::chrono::high_resolution_clock::now();

		cl_mem buffer = GpuCommands::findMaxGPUBuffer(gpu, input, count, 1, 0);
		float* output = ALLOC_ARRAY(float, 8);
		gpu->commandManager->executeReadBuffer(buffer, SIZEOF_FLOAT*8, output, true);
		float result = -FLT_MAX;

		for (size_t i = 0; i < 8; i++)
			if (result < output[i])
				result = output[i];

		currentTime2 = std::chrono::high_resolution_clock::now();
		std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

		Assert::AreEqual(max, result, L"Wrong value.", LINE_INFO());

		ALLOC_RELEASE(input);
	}

}

#undef CLASS_NAME

#endif // OPENCL_ENABLED