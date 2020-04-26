#if OPENCL_ENABLED

#ifndef GPU_RADIX_SORTING_TEST
#define GPU_RADIX_SORTING_TEST

#include "SpectrumPhysicsTest.h"
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

		sp_float* getRandom(sp_uint count, sp_uint spaceSize = 10000)
		{
			Randomizer<sp_int> randomizer(0, spaceSize);

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

		SP_TEST_METHOD_DEF(AlgorithmSorting_radixGPU_Test3_WithNegatives);

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
				
		SP_FILE file;
		file.open("RadixSorting.cl", std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.close();

		sp_uint radixSortProgramIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions.str().c_str());

		ALLOC_RELEASE(source);

		cl_program program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];
		

		sp_uint offsetPrefixScanCpu = 10;
		sp_bool useExpoent = false;
		sp_uint digitIndex = 3;
		sp_uint stride = 1u;
		sp_uint offset = 0u;
		const sp_uint inputSize = count * stride * SIZEOF_FLOAT;

		cl_mem inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		cl_mem indexesGpu = commandIndexes->execute();
		cl_mem indexesLengthGpu = gpu->createBuffer(&count, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem digitIndexGpu = gpu->createBuffer(&digitIndex, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem useExpoentGpu = gpu->createBuffer(&useExpoent, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem output = gpu->createBuffer(count * SIZEOF_UINT, CL_MEM_READ_WRITE);

		const sp_uint offsetTableSize = SIZEOF_UINT * 10 * count;
		cl_mem offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		cl_mem offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		cl_mem offsetTableResult = offsetTable2;

		const sp_uint threadsLength = gpu->getThreadLength(count) / 2;
		const sp_uint groupLength = gpu->getGroupLength(threadsLength, count);
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * count)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
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

			if (value != count / threadsLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}

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
		
		SP_FILE file;
		file.open("RadixSorting.cl", std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.close();

		sp_uint radixSortProgramIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions.str().c_str());

		ALLOC_RELEASE(source);

		cl_program program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];

		sp_uint offsetPrefixScanCpu = 10;
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;
		sp_uint stride = 1u;
		sp_uint offset = 0u;
		const sp_uint inputSize = count * stride * SIZEOF_FLOAT;

		sp_uint threadsLength = gpu->getThreadLength(count);
		sp_uint iterations = (sp_uint) std::ceil(std::log(threadsLength));
		threadsLength = divideBy2(threadsLength);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, count);

		sp_uint maxLength = 2;
		for (; maxLength < threadsLength; maxLength *= 2)
		{
			sp_uint v = nextDivisorOf(maxLength - 1, defaultLocalWorkSize);
			if (v > gpu->maxWorkGroupSize)
				break;
		}
		while (threadsLength >= maxLength)
			threadsLength = divideBy2(threadsLength);
		const sp_uint elementsPerThread = count / threadsLength;

		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		cl_mem inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		cl_mem indexesGpu = commandIndexes->execute();
		cl_mem indexesLengthGpu = gpu->createBuffer(&count, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem offsetGpu = gpu->createBuffer(&offset, SIZEOF_UINT, CL_MEM_READ_ONLY);
		cl_mem digitIndexGpu = gpu->createBuffer(&digitIndex, SIZEOF_UINT, CL_MEM_READ_WRITE);
		cl_mem output = gpu->createBuffer(count * SIZEOF_UINT, CL_MEM_READ_WRITE);

		const sp_uint offsetTableSize = SIZEOF_UINT * multiplyBy10(threadsLength);
		cl_mem offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * count)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize);

		GpuCommand* commandPrefixScanUp = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "prefixScanUp");

		GpuCommand* commandPrefixScanDown = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "prefixScanDown");

		sp_uint* temp = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->executeReadBuffer(offsetTable1, offsetTableSize, temp, true);

		for (sp_uint shift = 0; shift < threadsLength * 10; shift += 10)
		{
			sp_uint value = 0u;

			for (sp_uint i = 0; i < 10; i++)
				value += temp[i + shift];

			if (value != count / threadsLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}
		sp_float r = 0u;
		for (sp_uint i = 0; i < threadsLength * 10; i++)
			r += temp[i];

		sp_uint* result = ALLOC_ARRAY(sp_uint, multiplyBy10(threadsLength));
		sp_uint threadLength = (sp_uint)std::ceil(globalWorkSize[0] / 2.0);
		offset = 1u;
		while(globalWorkSize[0] != 1u)
		{
			offset = multiplyBy2(offset);
			globalWorkSize[0] = (sp_uint)std::ceil(globalWorkSize[0] / 2.0);

			if (globalWorkSize[0] < localWorkSize[0])
				localWorkSize[0] = globalWorkSize[0];

			commandPrefixScanUp->execute(1, globalWorkSize, localWorkSize, &offset);
			
			// test scan up
			gpu->commandManager->executeReadBuffer(offsetTable1, offsetTableSize, result, true);
			for (sp_uint w = 0; w < globalWorkSize[0]; w++)
			{
				sp_uint sum = 0;
				sp_uint index = (w + 1) * offset * 10u - 10u;

				for (sp_uint j = index; j < index + 10u; j++)
					sum += result[j];

				Assert::AreEqual(elementsPerThread * offset, sum, L"Wrong value.", LINE_INFO());
			}
		}

		threadLength = globalWorkSize[0];
		while (offset != 2u)
		{
			offset = divideBy2(offset);
			threadLength = multiplyBy2(threadLength);
			globalWorkSize[0] = threadLength - 1;

			if (globalWorkSize[0] <= defaultLocalWorkSize)
				localWorkSize[0] = globalWorkSize[0];
			else
				localWorkSize[0] = nextDivisorOf(globalWorkSize[0], defaultLocalWorkSize);

			commandPrefixScanDown->execute(1, globalWorkSize, localWorkSize, &offset);

			// test scan down
			gpu->commandManager->executeReadBuffer(offsetTable1, offsetTableSize, result, true);
			for (sp_uint w = 0; w < globalWorkSize[0]; w++)
			{
				sp_uint sum = 0;
				sp_uint index = ((w + 1) * offset + divideBy2(offset)) * 10u - 10u;

				for (sp_uint j = index; j < index + 10u; j++)
					sum += result[j];

				Assert::AreEqual( (index + 10u) / 10u * elementsPerThread, sum, L"Wrong value.", LINE_INFO());
			}
		}

		sp_uint expected = 0u;
		for (sp_uint i = 0; i < threadLength; i++)
		{
			sp_uint value = 0;
			expected += elementsPerThread;

			for (sp_uint j = 0; j < 10; j++)
				value += result[i * 10 + j];

			Assert::AreEqual(expected, value, L"Wrong value.", LINE_INFO());
		}

		ALLOC_DELETE(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test1)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint maxIterations = 20;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		const sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);

		for (sp_size i = 0; i < maxIterations; i++)
		{	
			sp_float* input1 = getRandom(inputLength);
			input1[2] = -0.01f;
			sp_float* input2 = ALLOC_COPY(input1, sp_float, inputLength);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::native(input1, inputLength);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << inputLength
						 << " -DINPUT_STRIDE=1"
						 << " -DINPUT_OFFSET=0";
			
			sp_uint strider = 1u;
			sp_uint offset = 0u;
			GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
			radixGpu->init(gpu, buildOptions.str().c_str())
				->setParameters(input2, inputLength, strider, offset);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_mem output = radixGpu->execute();

			currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			gpu->commandManager->executeReadBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, true);

			for (sp_uint i = 0; i < inputLength; i++)
				Assert::AreEqual(input1[i], input2[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

			ALLOC_DELETE(radixGpu, GpuRadixSorting);
			ALLOC_RELEASE(input1);
		}
	}

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test2)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint iterations = 10u;
		std::chrono::milliseconds times[iterations];

		for (sp_size i = 0; i < iterations; i++)
		{
			const sp_size count = (sp_size)std::pow(2.0, 17.0);
			AABB* input1 = getRandomAABBs(count);
			AABB* input2 = ALLOC_COPY(input1, AABB, count);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << count;
			buildOptions << " -DINPUT_STRIDE=" << AABB_STRIDER;
			buildOptions << " -DINPUT_OFFSET=" << AABB_OFFSET;

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

	SP_TEST_METHOD(CLASS_NAME, AlgorithmSorting_radixGPU_Test3_WithNegatives)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice;

		const sp_uint inputLength = 8u;
		sp_uint strider = 1u;
		sp_uint offset = 0u;

		sp_float input1[8] = { 50.0f, 2.0f, -5.0f, 4.0f, 12.0f, -10.0f, -1.0f, 30.0f };
		sp_float input2[8] = { 50.0f, 2.0f, -5.0f, 4.0f, 12.0f, -10.0f, -1.0f, 30.0f };

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=1"
			<< " -DINPUT_OFFSET=0";

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu->init(gpu, buildOptions.str().c_str())
			->setParameters(input2, inputLength, strider, offset);

		AlgorithmSorting::native(input1, inputLength);

		cl_mem output = radixGpu->execute();

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->executeReadBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, true);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(input1[i], input2[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

		ALLOC_DELETE(radixGpu, GpuRadixSorting);
		ALLOC_RELEASE(radixGpu);
	}

}

#undef CLASS_NAME

#endif // GPU_RADIX_SORTING_TEST

#endif // OPENCL_ENABLED
