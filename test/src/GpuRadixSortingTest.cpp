#if OPENCL_ENABLED

#ifndef GPU_RADIX_SORTING_TEST
#define GPU_RADIX_SORTING_TEST

#include "TestHeader.h"
#include "Randomizer.h"
#include <GpuRadixSorting.h>
#include <AABB.h>
#include <AlgorithmSorting.h>
#include "GpuContext.h"
#include "GpuIndexes.h"

#define CLASS_NAME GpuRadixSortingTest

namespace NAMESPACE_PHYSICS_TEST
{

	SP_TEST_CLASS(CLASS_NAME)
	{
	private:

		float* getRandom(sp_uint count, sp_uint spaceSize = 10000)
		{
			Randomizer<int> randomizer(0, spaceSize);

			sp_float* result = ALLOC_ARRAY(sp_float, count);

			for (sp_uint i = 0; i < count; i++)
				result[i] = randomizer.rand() / 100.0f;

			return result;
		}

		AABB* getRandomAABBs(sp_uint count, sp_uint spaceSize = 1000)
		{
			Randomizer<int> randomizerSize(0, 30);
			Randomizer<int> randomizerLocation(0, spaceSize);

			AABB* aabbs = ALLOC_ARRAY(AABB, count);

			for (sp_uint i = 0; i < count; i++)
			{
				sp_int xMin = randomizerSize.rand();
				sp_int yMin = randomizerSize.rand();
				sp_int zMin = randomizerSize.rand();

				sp_int xMax = randomizerSize.rand();
				sp_int yMax = randomizerSize.rand();
				sp_int zMax = randomizerSize.rand();

				sp_int locationX = randomizerLocation.rand();
				sp_int locationY = randomizerLocation.rand();
				sp_int locationZ = randomizerLocation.rand();

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

		static sp_int comparatorFloatTest(const void* param1, const void* param2)
		{
			const sp_float obj1 = *(sp_float*)param1;
			const sp_float obj2 = *(sp_float*)param2;

			if (obj1 < obj2)
				return -1;
			else
				if (obj1 > obj2)
					return 1;

			return 0;
		}

		static sp_int comparatorAABBirstAxisTest(const void* param1, const void* param2)
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

		SP_TEST_METHOD_DEF(AlgorithmSorting_radixGPU_Test1);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radixGPU_Test2);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radixGPU_Test_Count);

		SP_TEST_METHOD_DEF(AlgorithmSorting_radixGPU_Test_PrefixScan);

	};

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test_Count)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;
		
		sp_uint count = (sp_uint)std::pow(2.0, 17.0);
		sp_float* input = getRandom(count);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);
				
		IFileManager* fileManager = Factory::getFileManagerInstance();
		std::string sourceRadixSort = fileManager->readTextFile("RadixSorting.cl");
		sp_uint radixSortProgramIndex = gpu->commandManager->cacheProgram(sourceRadixSort.c_str(), SIZEOF_CHAR * sourceRadixSort.length(), buildOptions.str().c_str());
		cl_program program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];
		delete fileManager;

		sp_uint offsetPrefixScanCpu = 10;
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;
		sp_uint stride = 1u;
		sp_uint offset = 0u;
		const sp_uint inputSize = count * stride * SIZEOF_FLOAT;

		cl_mem inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		cl_mem indexesGpu = commandIndexes->execute();
		cl_mem indexesLengthGpu = gpu->createBuffer(&count, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem offsetPrefixScanGpu = gpu->createBuffer(&offsetPrefixScanCpu, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem digitIndexGpu = gpu->createBuffer(&digitIndex, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem useExpoentGpu = gpu->createBuffer(&useExpoent, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem output = gpu->createBuffer(count * SIZEOF_UINT, CL_MEM_READ_WRITE);

		const sp_uint offsetTableSize = SIZEOF_UINT * 10 * count;
		cl_mem offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		cl_mem offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		cl_mem offsetTableResult = offsetTable2;

		sp_size* gridConfig = gpu->getGridConfigForOneDimension(count);

		const sp_uint threadsLength = gridConfig[0];
		const sp_uint groupLength = gridConfig[0];
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		ALLOC_RELEASE(gridConfig);

		GpuFindMinMax* findMinMax = ALLOC_NEW(GpuFindMinMax)();
		findMinMax->init(gpu, buildOptions.str().c_str());
		findMinMax->setParameters(input, count, 1, offset);
		findMinMax->execute();

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * count)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(useExpoentGpu, SIZEOF_BOOL)
			->setInputParameter(findMinMax->output, SIZEOF_FLOAT * 2)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize);

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->executeReadBuffer(offsetTable1, offsetTableSize, orderedIndexes, true);

		for (sp_uint shift = 0; shift < threadsLength * 10; shift+=10)
		{
			sp_uint value = 0u;

			for (sp_uint i = 0; i < 10; i++)
				value += orderedIndexes[i + shift];

			if (value != groupLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}

		ALLOC_DELETE(findMinMax, GpuFindMinMax);
		ALLOC_DELETE(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test_PrefixScan)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		sp_uint count = (sp_uint)std::pow(2.0, 17.0);
		sp_float* input = getRandom(count);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);
		
		IFileManager* fileManager = Factory::getFileManagerInstance();
		std::string sourceRadixSort = fileManager->readTextFile("RadixSorting.cl");
		sp_uint radixSortProgramIndex = gpu->commandManager->cacheProgram(sourceRadixSort.c_str(), SIZEOF_CHAR * sourceRadixSort.length(), buildOptions.str().c_str());
		cl_program program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];
		delete fileManager;

		sp_uint offsetPrefixScanCpu = 10;
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;
		sp_uint stride = 1u;
		sp_uint offset = 0u;
		const sp_uint inputSize = count * stride * SIZEOF_FLOAT;

		sp_size* gridConfig = gpu->getGridConfigForOneDimension(count);

		const sp_uint threadsLength = gridConfig[0];
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { gridConfig[1], 0, 0 };

		ALLOC_RELEASE(gridConfig);

		cl_mem inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		cl_mem indexesGpu = commandIndexes->execute();
		cl_mem indexesLengthGpu = gpu->createBuffer(&count, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem offsetPrefixScanGpu = gpu->createBuffer(&offsetPrefixScanCpu, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem digitIndexGpu = gpu->createBuffer(&digitIndex, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem useExpoentGpu = gpu->createBuffer(&useExpoent, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem output = gpu->createBuffer(count * SIZEOF_UINT, CL_MEM_READ_WRITE);

		const sp_uint offsetTableSize = SIZEOF_UINT * multiplyBy10(threadsLength);
		cl_mem offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		cl_mem offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		cl_mem offsetTableResult = offsetTable2;

		GpuFindMinMax* findMinMax = ALLOC_NEW(GpuFindMinMax)();
		findMinMax->init(gpu, buildOptions.str().c_str());
		findMinMax->setParameters(input, count, 1, offset);
		findMinMax->execute();

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * count)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(useExpoentGpu, SIZEOF_BOOL)
			->setInputParameter(findMinMax->output, SIZEOF_FLOAT * 2)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize);

		GpuCommand* commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable2, offsetTableSize)
			->setInputParameter(offsetPrefixScanGpu, SIZEOF_UINT)
			->buildFromProgram(program, "prefixScan");

		GpuCommand* commandPrefixScanSwaped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable1, offsetTableSize)
			->setInputParameter(offsetPrefixScanGpu, SIZEOF_UINT)
			->buildFromProgram(program, "prefixScan");

		sp_bool prefixScanSwaped = true;
		sp_bool offsetChanged = false;

		commandPrefixScan->execute(1, globalWorkSize, localWorkSize);

		const sp_uint maxIteration = (sp_uint) std::ceil(std::log(multiplyBy10(threadsLength)));

		for (sp_uint i = 0; i < maxIteration; i++)
		{
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);

			if (prefixScanSwaped)
				commandPrefixScanSwaped->execute(1, globalWorkSize, localWorkSize);
			else
				commandPrefixScan->execute(1, globalWorkSize, localWorkSize);

			offsetChanged = !offsetChanged;
			prefixScanSwaped = !prefixScanSwaped;
		}

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, multiplyBy10(threadsLength));
		gpu->commandManager->executeReadBuffer(prefixScanSwaped ? offsetTable2 : offsetTable1, offsetTableSize, orderedIndexes, true);

		Assert::AreEqual(count, orderedIndexes[multiplyBy10(threadsLength) - 1] + orderedIndexes[multiplyBy10(threadsLength) - 10], L"Wrong value.", LINE_INFO());

		ALLOC_DELETE(findMinMax, GpuFindMinMax);
		ALLOC_DELETE(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test1)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint maxIterations = 30;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		const sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);

		for (sp_size i = 0; i < maxIterations; i++)
		{	
			sp_float* input1 = getRandom(inputLength);
			sp_float* input2 = ALLOC_COPY(input1, sp_float, inputLength);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::native(input1, inputLength);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << inputLength;
			buildOptions << " -DINPUT_STRIDE=1";
			buildOptions << " -DINPUT_OFFSET=0";

			sp_uint strider = 1u;
			sp_uint offset = 0u;
			GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
			radixGpu->init(gpu, buildOptions.str().c_str())->setParameters(input2, inputLength, strider, offset);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_mem output = radixGpu->execute();

			currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			gpu->commandManager->executeReadBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, true);

			for (sp_uint i = 0; i < inputLength; i++)
				Assert::AreEqual(input1[i], input2[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

			gpu->releaseBuffer(output);
			ALLOC_DELETE(radixGpu, GpuRadixSorting);
			ALLOC_RELEASE(input1);
		}
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test2)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		std::chrono::milliseconds times[10];

		for (sp_size i = 0; i < 10; i++)
		{

			const sp_size count = (sp_size)std::pow(2.0, 17.0);
			AABB* input1 = getRandomAABBs(count);
			AABB* input2 = ALLOC_COPY(input1, AABB, count);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << count;
			buildOptions << " -DINPUT_STRIDE=" << AABB_STRIDER;
			buildOptions << " -DINPUT_OFFSET=" << AABB_OFFSET;
			//buildOptions << " –cl-fast-relaxed-math";
			//buildOptions << " -cl-unsafe-math-optimizations";

			GpuRadixSorting* radixSorting = ALLOC_NEW(GpuRadixSorting)();
			radixSorting->init(gpu, buildOptions.str().c_str())
				->setParameters((sp_float*)input2, count, AABB_STRIDER, AABB_OFFSET);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::quickSortNative(input1, count, sizeof(AABB), comparatorAABBirstAxisTest);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_mem output = radixSorting->execute();

			currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);
			times[i] = ms2;

			sp_size* result = ALLOC_ARRAY(sp_size, count * SIZEOF_UINT);
			gpu->commandManager->executeReadBuffer(output, count * SIZEOF_UINT, result, true);

			for (sp_uint i = 0; i < count; i++)
				Assert::AreEqual(input1[i].minPoint.x, input2[result[i]].minPoint.x, L"Wrong value.", LINE_INFO());

			ALLOC_RELEASE(input1);
			ALLOC_DELETE(radixSorting, GpuRadixSorting);

		}
	}

}

#undef CLASS_NAME

#endif // GPU_RADIX_SORTING_TEST

#endif // OPENCL_ENABLED
