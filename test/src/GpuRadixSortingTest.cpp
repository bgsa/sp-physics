#if OPENCL_ENABLED

#ifndef GPU_RADIX_SORTING_TEST
#define GPU_RADIX_SORTING_TEST

#include "SpectrumPhysicsTest.h"
#include "Randomizer.h"
#include <GpuRadixSorting.h>
#include <AABB.h>
#include <DOP18.h>
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
			Randomizer randomizer(0, spaceSize);

			sp_float* result = ALLOC_ARRAY(sp_float, count);

			for (sp_uint i = 0; i < count; i++)
				result[i] = randomizer.randInt() / 100.0f;

			return result;
		}

		AABB* getRandomAABBs(sp_uint count, sp_uint spaceSize = 1000)
		{
			Randomizer randomizerSize(0, 30);
			Randomizer randomizerLocation(0, spaceSize);

			AABB* aabbs = ALLOC_ARRAY(AABB, count);

			for (sp_uint i = 0; i < count; i++)
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

		DOP18* getRandomKDOPs(sp_uint length, sp_uint spaceSize = 10000u)
		{
			Randomizer randomizer(0, spaceSize);

			DOP18* kdops = ALLOC_NEW_ARRAY(DOP18, length);

			for (sp_uint i = 0; i < length; i++)
			{
				sp_float x = randomizer.randInt() / 100.0f;
				sp_float y = randomizer.randInt() / 100.0f;
				sp_float z = randomizer.randInt() / 100.0f;

				kdops[i].scale(Vec3(3.0f));
				kdops[i].translate({ x, 0.5f, z });
			}

			return kdops;
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

		static sp_int comparatorXAxisForQuickSortKDOP(const void* a, const void* b)
		{
			DOP18* obj1 = (DOP18*)a;
			DOP18* obj2 = (DOP18*)b;

			if (obj1->min[0] < obj2->min[0])
				return -1;
			else
				if (obj1->min[0] > obj2->min[0])
					return 1;

			return 0;
		}

	public:
		SP_TEST_METHOD_DEF(radixGPU_Count);
		SP_TEST_METHOD_DEF(radixGPU_PrefixScan);
		SP_TEST_METHOD_DEF(radixGPU_1);
		SP_TEST_METHOD_DEF(radixGPU_2);
		SP_TEST_METHOD_DEF(radixGPU_WithNegatives);
		SP_TEST_METHOD_DEF(radixGPU_WithKDOPs);
		SP_TEST_METHOD_DEF(radixGPU_WithKDOPs_2);
		SP_TEST_METHOD_DEF(radixGPU_manyTimes);
	};

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Count)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();
		
		sp_uint count = (sp_uint)std::pow(2.0, 17.0);
		sp_float* input = getRandom(count);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("RadixSorting.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_mem_delete(filename, SpDirectory);

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
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize, &digitIndex, NULL, ZERO_UINT);

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->readBuffer(offsetTable1, offsetTableSize, orderedIndexes, ONE_UINT, &commandCount->lastEvent);

		for (sp_uint shift = 0; shift < threadsLength * 10; shift+=10)
		{
			sp_uint value = 0u;

			for (sp_uint i = 0; i < 10; i++)
				value += orderedIndexes[i + shift];

			if (value != count / threadsLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}

		sp_mem_delete(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_PrefixScan)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		sp_uint count = (sp_uint)std::pow(2.0, 17.0);
		sp_float* input = getRandom(count);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << count;
		buildOptions << " -DINPUT_STRIDE=1";
		buildOptions << " -DINPUT_OFFSET=0";

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, buildOptions.str().c_str());
		commandIndexes->setParametersCreateIndexes(count);
		
		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("RadixSorting.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_mem_delete(filename, SpDirectory);

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
		cl_mem output = gpu->createBuffer(count * SIZEOF_UINT, CL_MEM_READ_WRITE);

		const sp_uint offsetTableSize = SIZEOF_UINT * multiplyBy10(threadsLength);
		cl_mem offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * count)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize, &digitIndex);

		GpuCommand* commandPrefixScanUp = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "prefixScanUp");

		GpuCommand* commandPrefixScanDown = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "prefixScanDown");

		sp_uint* temp = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->readBuffer(offsetTable1, offsetTableSize, temp, ONE_UINT, &commandCount->lastEvent);

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
			gpu->commandManager->readBuffer(offsetTable1, offsetTableSize, result, ONE_UINT, &commandPrefixScanUp->lastEvent);
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
			gpu->commandManager->readBuffer(offsetTable1, offsetTableSize, result, ONE_UINT, &commandPrefixScanDown->lastEvent);
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

		sp_mem_delete(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
		ALLOC_RELEASE(input);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_1)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint maxIterations = 20;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		for (sp_size i = 0; i < maxIterations; i++)
		{	
			sp_float* input1 = getRandom(inputLength);
			input1[2] = -0.01f;
			sp_float* input2 = ALLOC_COPY(input1, sp_float, inputLength);
			cl_mem inputGpu = gpu->createBuffer(input2, SIZEOF_FLOAT * inputLength, CL_MEM_READ_ONLY, true);

			cl_mem newIndexesLength = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_WRITE);
			createIndexes->setParametersCreateIndexes(inputLength);
			cl_mem newIndexes = createIndexes->execute();

			AlgorithmSorting::native(input1, inputLength);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << inputLength
						 << " -DINPUT_STRIDE=1"
						 << " -DINPUT_OFFSET=0";
			
			sp_uint strider = 1u;
			sp_uint offset = 0u;
			GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
			radixGpu->init(gpu, buildOptions.str().c_str())
				->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, strider);

			cl_mem output = radixGpu->execute();

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			gpu->commandManager->readBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, ONE_UINT, &radixGpu->lastEvent);

			for (sp_uint i = 0; i < inputLength; i++)
				Assert::AreEqual(input1[i], input2[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

			gpu->releaseBuffer(inputGpu);
			ALLOC_DELETE(radixGpu, GpuRadixSorting);
			ALLOC_RELEASE(input1);
		}

		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_2)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint iterations = 10u;
		std::chrono::milliseconds times[iterations];

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		for (sp_size i = 0; i < iterations; i++)
		{
			sp_size length = (sp_size)std::pow(2.0, 17.0);
			AABB* input1 = getRandomAABBs(length);
			AABB* input2 = ALLOC_COPY(input1, AABB, length);
			cl_mem inputGpu = gpu->createBuffer(input2, sizeof(AABB) * length, CL_MEM_READ_ONLY, true);

			cl_mem newIndexesLength = gpu->createBuffer(&length, SIZEOF_UINT, CL_MEM_READ_WRITE);
			createIndexes->setParametersCreateIndexes(length);
			cl_mem newIndexes = createIndexes->execute();

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << length
						<< " -DINPUT_STRIDE=" << AABB_STRIDER
						<< " -DINPUT_OFFSET=" << AABB_OFFSET;

			GpuRadixSorting* radixSorting = ALLOC_NEW(GpuRadixSorting)();
			radixSorting->init(gpu, buildOptions.str().c_str())
				->setParameters(inputGpu, length, newIndexes, newIndexesLength, AABB_STRIDER);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::quickSortNative(input1, length, sizeof(AABB), comparatorAABBirstAxisTest);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_mem output = radixSorting->execute();

			currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);
			times[i] = ms2;

			sp_size* result = ALLOC_ARRAY(sp_size, length * SIZEOF_UINT);
			gpu->commandManager->readBuffer(output, length * SIZEOF_UINT, result, ONE_UINT, &radixSorting->lastEvent);

			for (sp_uint i = 0; i < length; i++)
				Assert::AreEqual(input1[i].minPoint.x, input2[result[i]].minPoint.x, L"Wrong value.", LINE_INFO());

			gpu->releaseBuffer(inputGpu);
			ALLOC_RELEASE(input1);
			ALLOC_DELETE(radixSorting, GpuRadixSorting);
		}

		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_WithNegatives)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 8u;
		sp_uint strider = 1u;
		sp_uint offset = 0u;

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_WRITE);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_mem newIndexes = createIndexes->execute();

		sp_float input1[8] = { 50.0f, 2.0f, -5.0f, 4.0f, 12.0f, -10.0f, -1.0f, 30.0f };
		sp_float input2[8] = { 50.0f, 2.0f, -5.0f, 4.0f, 12.0f, -10.0f, -1.0f, 30.0f };
		cl_mem inputGpu = gpu->createBuffer(input2, SIZEOF_FLOAT * inputLength, CL_MEM_READ_ONLY, true);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=1"
			<< " -DINPUT_OFFSET=0";

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu->init(gpu, buildOptions.str().c_str())
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, strider);

		AlgorithmSorting::native(input1, inputLength);

		cl_mem output = radixGpu->execute();

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, ONE_UINT, &radixGpu->lastEvent);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(input1[i], input2[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(inputGpu);
		ALLOC_DELETE(radixGpu, GpuRadixSorting);
		ALLOC_RELEASE(radixGpu);
		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_WithKDOPs)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint maxIterations = 20;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		for (sp_uint i = 0; i < maxIterations; i++)
		{
			DOP18* input1 = getRandomKDOPs(inputLength);
			input1[2].min[0] = -0.01f;
			DOP18* input2 = ALLOC_COPY(input1, DOP18, inputLength);
			cl_mem inputGpu = gpu->createBuffer(input2, DOP18_SIZE * inputLength, CL_MEM_READ_ONLY, true);

			cl_mem newIndexesLength = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_WRITE);
			createIndexes->setParametersCreateIndexes(inputLength);
			cl_mem newIndexes = createIndexes->execute();

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::quickSortNative(input1, inputLength, DOP18_SIZE, comparatorXAxisForQuickSortKDOP);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << inputLength
				<< " -DINPUT_STRIDE=" << DOP18_STRIDER
				<< " -DINPUT_OFFSET=" << 0
				<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

			GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
			radixGpu
				->init(gpu, buildOptions.str().c_str())
				->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, DOP18_STRIDER);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_mem output = radixGpu->execute();

			currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			gpu->commandManager->readBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, ONE_UINT, &radixGpu->lastEvent);

			for (sp_uint i = 0; i < inputLength; i++)
				for (sp_uint j = 0; i < DOP18_ORIENTATIONS; i++)
				{
					Assert::AreEqual(input1[i].min[j], input2[orderedIndexes[i]].min[j], L"Wrong value.", LINE_INFO());
					Assert::AreEqual(input1[i].max[j], input2[orderedIndexes[i]].max[j], L"Wrong value.", LINE_INFO());
				}

			gpu->releaseBuffer(inputGpu);
			ALLOC_DELETE(radixGpu, GpuRadixSorting);
			ALLOC_RELEASE(input1);
		}

		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_WithKDOPs_2)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();
		sp_uint inputLength = 4u;

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		sp_float values[18*4] = {
			-50.0000f, -0.10000f, -50.0000f, -49.8499f, -49.8499f, -49.8499f, -49.8499f, -99.7500f, -99.7500f, 50.00000f, 0.100000f, 50.00000f, 49.84999f, 49.84999f, 49.84999f, 49.84999f, 99.75000f, 99.75000f, 
			0.600000f, 8.500000f, 1.500000f, -10.6500f, 9.350000f, 10.25000f, 4.250000f, 2.350000f, -3.65000f, 3.400000f, 11.50000f, 4.500000f, -5.35000f, 14.65000f, 15.75000f, 9.750000f, 7.650000f, 1.650000f, 
			-1.40000f, -1.50000f, -1.50000f, -2.65000f, -2.65000f, -2.75000f, -2.75000f, -2.65000f, -2.65000f, 1.400000f, 1.500000f, 1.500000f, 2.650000f, 2.650000f, 2.750000f, 2.750000f, 2.650000f, 2.650000f,
			-17.7899f, 43.73000f, 2.750000f, -64.2699f, 26.19000f, 46.73000f, 38.23000f, -14.7899f, -23.2899f, -14.9900f, 46.73000f, 5.750000f, -58.9699f, 31.49000f, 52.23000f, 43.73000f, -9.49000f, -17.9900f
		};
		DOP18* input1 = (DOP18*)values;
		cl_mem inputGpu = gpu->createBuffer(input1, DOP18_SIZE * inputLength, CL_MEM_READ_ONLY, true);

		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_WRITE);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_mem newIndexes = createIndexes->execute();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0
			<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu
			->init(gpu, buildOptions.str().c_str())
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, DOP18_STRIDER);

		cl_mem output = radixGpu->execute();

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, ONE_UINT, &radixGpu->lastEvent);

		sp_uint exepectedIndexes[] = { 0, 3, 2, 1 };
		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(exepectedIndexes[i], orderedIndexes[i], L"Wrong value.", LINE_INFO());
		
		gpu->releaseBuffer(inputGpu);
		ALLOC_DELETE(radixGpu, GpuRadixSorting);
		ALLOC_RELEASE(input1);

		ALLOC_DELETE(createIndexes, GpuIndexes);
	}
	
	SP_TEST_METHOD(CLASS_NAME, radixGPU_manyTimes)
	{
		GpuContext* context = GpuContext::init();
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint maxIterations = 3;
		//sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);
		sp_uint inputLength = 64;

		DOP18* input1 = getRandomKDOPs(inputLength);
		input1[2].min[0] = -0.01f;
		DOP18* input2 = ALLOC_COPY(input1, DOP18, inputLength);

		cl_mem inputGpu = gpu->createBuffer(input2, DOP18_SIZE * inputLength, CL_MEM_READ_ONLY, true);
		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_WRITE);

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_mem newIndexes = createIndexes->execute();

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0
			<< " -DORIENTATION_LENGTH=" << DOP18_ORIENTATIONS;

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu
			->init(gpu, buildOptions.str().c_str())
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, DOP18_STRIDER);

		AlgorithmSorting::quickSortNative(input1, inputLength, DOP18_SIZE, comparatorXAxisForQuickSortKDOP);

		cl_event lastEvent = createIndexes->lastEvent;
		sp_uint eventsLength = ONE_UINT;
		 
		for (sp_uint i = 0; i < maxIterations; i++)
		{
			cl_mem output = radixGpu->execute(eventsLength, &lastEvent);

			sp_uint* buffer = ALLOC_ARRAY(sp_uint, inputLength);
			lastEvent = gpu->commandManager->readBuffer(output, 4 * inputLength, buffer, ONE_UINT, &lastEvent);
			std::stringstream ss;
			ss << "BEGIN SORT" << END_OF_LINE;
			for (size_t i = 0; i < 64; i++)
				ss << buffer[i] << " , ";
			ss << "END SORT" << END_OF_LINE;
			OutputDebugStringA(ss.str().c_str());
			ALLOC_RELEASE(buffer);

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			lastEvent = gpu->commandManager->readBuffer(output, inputLength * SIZEOF_UINT, orderedIndexes, ONE_UINT, &radixGpu->lastEvent);

			for (sp_uint i = 0; i < inputLength; i++)
				for (sp_uint j = 0; i < DOP18_ORIENTATIONS; i++)
				{
					Assert::AreEqual(input1[i].min[j], input2[orderedIndexes[i]].min[j], L"Wrong value.", LINE_INFO());
					Assert::AreEqual(input1[i].max[j], input2[orderedIndexes[i]].max[j], L"Wrong value.", LINE_INFO());
				}

			gpu->commandManager->flush();
		}

		gpu->releaseBuffer(inputGpu);
		ALLOC_DELETE(radixGpu, GpuRadixSorting);
		ALLOC_RELEASE(input1);
		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

}

#undef CLASS_NAME

#endif // GPU_RADIX_SORTING_TEST

#endif // OPENCL_ENABLED
