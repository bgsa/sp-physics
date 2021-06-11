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

				kdops[i].scale(Vec3(3.0f, 3.0f, 3.0f));
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
		SP_TEST_METHOD_DEF(radixGPU_Count2);
		SP_TEST_METHOD_DEF(radixGPU_Count3);
		SP_TEST_METHOD_DEF(radixGPU_Count4);
		SP_TEST_METHOD_DEF(radixGPU_Count_Negative5);
		SP_TEST_METHOD_DEF(radixGPU_PrefixScan2);
		SP_TEST_METHOD_DEF(radixGPU_PrefixScan3);
		SP_TEST_METHOD_DEF(radixGPU_PrefixScan4);
		SP_TEST_METHOD_DEF(radixGPU_PrefixScan_Negative5);
		SP_TEST_METHOD_DEF(radixGPU_Reorder2);
		SP_TEST_METHOD_DEF(radixGPU_Reorder3);
		SP_TEST_METHOD_DEF(radixGPU_Reorder4);
		SP_TEST_METHOD_DEF(radixGPU_Reorder_Negative5);
		SP_TEST_METHOD_DEF(radixGPU_1);
		SP_TEST_METHOD_DEF(radixGPU_2);
		SP_TEST_METHOD_DEF(radixGPU_3);
		SP_TEST_METHOD_DEF(radixGPU_WithNegatives);
		SP_TEST_METHOD_DEF(radixGPU_WithKDOPs);
		SP_TEST_METHOD_DEF(radixGPU_WithKDOPs_2);
		SP_TEST_METHOD_DEF(radixGPU_manyTimes);
	};

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Count)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();
		
		sp_uint count = (sp_uint)std::pow(2.0, 17.0);
		sp_float* input = getRandom(count);

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, nullptr);
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

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_bool useExpoent = false;
		sp_size digitIndex = 3;
		sp_uint stride = 1u;
		sp_uint offset = 0u;
		const sp_uint inputSize = count * stride * sizeof(sp_uint);

		cl_mem inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

		cl_event evt;
		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		cl_mem indexesLengthGpu = gpu->createBuffer(&count, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem offsetGpu = gpu->createBuffer(&offset, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem useExpoentGpu = gpu->createBuffer(&useExpoent, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		cl_mem output = gpu->createBuffer(count * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		const sp_uint offsetTableSize = sizeof(sp_uint) * 10 * count;
		cl_mem offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		cl_mem offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		cl_mem offsetTableResult = offsetTable2;

		const sp_size threadsLength = gpu->getThreadLength(count) / 2;
		const sp_size groupLength = gpu->getGroupLength(threadsLength, count);
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * count)
			->setInputParameter(indexesLengthGpu, sizeof(sp_uint))
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize, &digitIndex, NULL, ZERO_UINT, &evt);

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, count);
		gpu->commandManager->readBuffer(offsetTable1, offsetTableSize, orderedIndexes, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

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

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Count2)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint expected[200] = {
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0
		};

		GpuIndexes* commandIndexes = ALLOC_NEW(GpuIndexes)();
		commandIndexes->init(gpu, nullptr);
		commandIndexes->setParametersCreateIndexes(inputLength);

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size groupLength = gpu->getGroupLength(threadsLength, inputLength);
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };
		const sp_uint elementsPerThread = inputLength / threadsLength;

		sp_size digitIndex = 3;
		const sp_uint inputSize = inputLength * sizeof(sp_uint);
		const sp_uint offsetTableSize = sizeof(sp_uint) * 10 * inputLength;

		cl_mem inputGpu = gpu->createBuffer(values, inputSize, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);

		cl_event evt;
		cl_mem indexesGpu = commandIndexes->execute(ZERO_UINT, NULL, &evt);
		gpu->releaseEvent(evt);

		cl_mem indexesLengthGpu = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem output = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		cl_mem offsetTableGPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesLengthGpu, sizeof(sp_uint))
			->setInputParameter(offsetTableGPU, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize, &digitIndex, NULL, ZERO_UINT, &evt);

		sp_float* temp = ALLOC_ARRAY(sp_float, inputLength);
		gpu->commandManager->readBuffer(inputGpu, inputSize, temp, ONE_UINT, &evt);

		sp_uint* offsetTable = (sp_uint*) ALLOC_SIZE(offsetTableSize);
		std::memset(offsetTable, 0, offsetTableSize);
		gpu->commandManager->readBuffer(offsetTableGPU, offsetTableSize, offsetTable, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("BEGIN OFFSET TABLE"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 10 / elementsPerThread; i++)
		{
			for (sp_uint j = 0; j < 10; j++)
			{
				sp_log_info1u(offsetTable[i * 10 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		for (sp_uint shift = 0; shift < threadsLength * 10; shift += 10)
		{
			sp_uint value = 0u;

			for (sp_uint i = 0; i < 10; i++)
				value += offsetTable[i + shift];

			if (value != inputLength / threadsLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}

		for (sp_uint i = 0; i < 200; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Count3)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint inputIndexes[20] = {
			12, 16, 2, 8, 11, 17, 18, 0, 1, 7, 5, 6, 9, 13, 15, 3, 4, 10, 19, 14
		};
		sp_uint expected[200] = {
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0
		};

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size groupLength = gpu->getGroupLength(threadsLength, inputLength);
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };
		const sp_uint elementsPerThread = inputLength / threadsLength;

		sp_size digitIndex = 4;
		const sp_uint inputSize = inputLength * sizeof(sp_uint);
		const sp_uint offsetTableSize = sizeof(sp_uint) * 10 * inputLength;

		cl_mem inputGpu = gpu->createBuffer(values, inputSize, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem indexesGpu = gpu->createBuffer(inputIndexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem indexesLengthGpu = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem output = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		cl_mem offsetTableGPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesLengthGpu, sizeof(sp_uint))
			->setInputParameter(offsetTableGPU, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize, &digitIndex, NULL, ZERO_UINT, &evt);

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		std::memset(offsetTable, 0, offsetTableSize);
		gpu->commandManager->readBuffer(offsetTableGPU, offsetTableSize, offsetTable, ONE_UINT, &evt);

		sp_log_info1s("BEGIN OFFSET TABLE"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 10 / elementsPerThread; i++)
		{
			for (sp_uint j = 0; j < 10; j++)
			{
				sp_log_info1u(offsetTable[i * 10 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		for (sp_uint shift = 0; shift < threadsLength * 10; shift += 10)
		{
			sp_uint value = 0u;

			for (sp_uint i = 0; i < 10; i++)
				value += offsetTable[i + shift];

			if (value != inputLength / threadsLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}

		for (sp_uint i = 0; i < 200; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Count4)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint inputIndexes[20] = {
			2, 17, 5, 14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19
		};
		sp_uint expected[200] = {
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0
		};

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size groupLength = gpu->getGroupLength(threadsLength, inputLength);
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };
		const sp_uint elementsPerThread = inputLength / threadsLength;

		sp_size digitIndex = 5;
		const sp_uint inputSize = inputLength * sizeof(sp_float);
		const sp_uint offsetTableSize = sizeof(sp_uint) * 10 * inputLength;

		cl_mem inputGpu = gpu->createBuffer(values, inputSize, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem indexesGpu = gpu->createBuffer(inputIndexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem indexesLengthGpu = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem output = gpu->createBuffer(inputLength * sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		cl_mem offsetTableGPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesLengthGpu, sizeof(sp_uint))
			->setInputParameter(offsetTableGPU, offsetTableSize)
			->buildFromProgram(program, "count")
			->execute(1, globalWorkSize, localWorkSize, &digitIndex, NULL, ZERO_UINT, &evt);

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		std::memset(offsetTable, 0, offsetTableSize);
		gpu->commandManager->readBuffer(offsetTableGPU, offsetTableSize, offsetTable, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("BEGIN OFFSET TABLE"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 10 / elementsPerThread; i++)
		{
			for (sp_uint j = 0; j < 10; j++)
			{
				sp_log_info1u(offsetTable[i * 10 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		for (sp_uint shift = 0; shift < threadsLength * 10; shift += 10)
		{
			sp_uint value = 0u;

			for (sp_uint i = 0; i < 10; i++)
				value += offsetTable[i + shift];

			if (value != inputLength / threadsLength)
				Assert::Fail(L"Wrong value.", LINE_INFO());
		}

		for (sp_uint i = 0; i < 200; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandCount, GpuCommand);
		gpu->releaseBuffer(output);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Count_Negative5)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint inputIndexes[20] = {
			14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19, 2, 17, 5
		};
		sp_uint expected[40] = {
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1
		};

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		const sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size groupLength = gpu->getGroupLength(threadsLength, inputLength);
		const sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		const sp_size localWorkSize[3] = { groupLength, 0, 0 };
		const sp_uint elementsPerThread = inputLength / threadsLength;

		const sp_uint inputSize = inputLength * sizeof(sp_float);
		const sp_uint offsetTableSize = sizeof(sp_uint) * 2 * inputLength;

		cl_mem inputGpu = gpu->createBuffer(values, inputSize, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem indexesGpu = gpu->createBuffer(inputIndexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem indexesLengthGpu = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR);
		cl_mem offsetTableGPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesLengthGpu, sizeof(sp_uint))
			->setInputParameter(offsetTableGPU, offsetTableSize)
			->buildFromProgram(program, "countNegatives")
			->execute(1, globalWorkSize, localWorkSize, 0, NULL, ZERO_UINT, &evt);

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		std::memset(offsetTable, 0, offsetTableSize);
		gpu->commandManager->readBuffer(offsetTableGPU, offsetTableSize, offsetTable, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("BEGIN OFFSET TABLE"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 2 / elementsPerThread; i++)
		{
			for (sp_uint j = 0; j < 2; j++)
			{
				sp_log_info1u(offsetTable[i * 2 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		for (sp_uint i = 0; i < 40; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandCount, GpuCommand);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_PrefixScan2)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint count = 20;
		sp_uint offsetTableValues[200] = {
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
			0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0
		};

		sp_uint threadsLength = gpu->getThreadLength(count);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, count);
		const sp_uint elementsPerThread = count / threadsLength;
		const sp_uint maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(threadsLength))) + ONE_UINT;

		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable1GPU = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem offsetTable2GPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		GpuCommand* commandPrefixScanSwapped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		sp_uint eventLength = ZERO_UINT;
		cl_event* previousEvents = nullptr;
		cl_event evt;
		sp_bool offsetChanged = false;
		sp_size offsetPrefixScanCpu = 10;
		for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
		{
			if (offsetChanged)
			{
				commandPrefixScanSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);
				gpu->releaseEvents(eventLength, previousEvents);
				previousEvents[0] = evt;
			}
			else
			{
				commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);

				if (eventLength == ZERO_UINT)
				{
					eventLength = ONE_UINT;
					previousEvents = ALLOC_NEW(cl_event)();
				}
				else
					gpu->releaseEvents(eventLength, previousEvents);

				previousEvents[0] = evt;
			}	

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		gpu->commandManager->readBuffer(offsetTable2GPU, offsetTableSize, offsetTable, ONE_UINT, previousEvents);

		sp_log_info1s("BEGIN OFFSET2"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 10; i++)
		{
			for (sp_uint j = 0; j < 10; j++)
			{
				sp_log_info1u(offsetTable[i * 10 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		sp_uint expected[200] = {
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 1, 1, 0, 0, 1, 0, 0,
			0, 0, 1, 1, 1, 0, 0, 1, 1, 0,
			0, 0, 1, 1, 1, 0, 1, 1, 1, 0,
			0, 0, 1, 1, 1, 0, 2, 1, 1, 0,
			0, 0, 1, 1, 2, 0, 2, 1, 1, 0,
			0, 0, 2, 1, 2, 0, 2, 1, 1, 0,
			0, 0, 2, 1, 2, 0, 3, 1, 1, 0,
			0, 0, 2, 1, 2, 0, 3, 1, 2, 0,
			0, 0, 3, 1, 2, 0, 3, 1, 2, 0,
			1, 0, 3, 1, 2, 0, 3, 1, 2, 0,
			1, 0, 3, 1, 2, 0, 4, 1, 2, 0,
			1, 0, 3, 1, 2, 0, 4, 1, 2, 1,
			1, 0, 3, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 3, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 4, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 5, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 5, 1, 2, 0, 5, 1, 3, 1
		};

		for (sp_uint i = 0; i < 200; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandPrefixScanSwapped, GpuCommand);
		sp_mem_delete(commandPrefixScan, GpuCommand);
		gpu->releaseBuffer(offsetTable1GPU);
		gpu->releaseBuffer(offsetTable2GPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_PrefixScan3)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint count = 20;
		sp_uint offsetTableValues[200] = {
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 1, 0, 0, 0, 0, 0, 0, 0
		};

		sp_uint threadsLength = gpu->getThreadLength(count);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, count);
		const sp_uint elementsPerThread = count / threadsLength;
		const sp_uint maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(threadsLength))) + ONE_UINT;

		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable1GPU = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem offsetTable2GPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		GpuCommand* commandPrefixScanSwapped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		sp_size eventLength = ZERO_UINT;
		cl_event* previousEvents = nullptr;
		cl_event evt;
		sp_bool offsetChanged = false;
		sp_size offsetPrefixScanCpu = 10;
		for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
		{
			if (offsetChanged)
			{
				commandPrefixScanSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);
				gpu->releaseEvents(eventLength, previousEvents);
				previousEvents[0] = evt;
			}
			else
			{
				commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);

				if (eventLength == ZERO_UINT)
				{
					eventLength = ONE_UINT;
					previousEvents = ALLOC_NEW(cl_event)();
				}
				else
					gpu->releaseEvents(eventLength, previousEvents);

				previousEvents[0] = evt;
			}

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		gpu->commandManager->readBuffer(offsetTable2GPU, offsetTableSize, offsetTable, ONE_UINT, previousEvents);

		sp_log_info1s("BEGIN OFFSET2"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 10; i++)
		{
			for (sp_uint j = 0; j < 10; j++)
			{
				sp_log_info1u(offsetTable[i * 10 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		sp_uint expected[200] = {
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
			1, 0, 0, 1, 0, 0, 0, 0, 1, 0,
			1, 0, 0, 1, 0, 0, 0, 0, 2, 0,
			1, 0, 0, 1, 0, 0, 0, 1, 2, 0,
			2, 0, 0, 1, 0, 0, 0, 1, 2, 0,
			2, 0, 0, 1, 0, 0, 0, 2, 2, 0,
			2, 0, 0, 2, 0, 0, 0, 2, 2, 0,
			2, 0, 0, 2, 0, 0, 0, 2, 3, 0,
			2, 0, 0, 2, 0, 0, 0, 3, 3, 0,
			3, 0, 0, 2, 0, 0, 0, 3, 3, 0,
			3, 0, 0, 2, 0, 1, 0, 3, 3, 0,
			3, 0, 0, 2, 1, 1, 0, 3, 3, 0,
			3, 0, 0, 2, 1, 1, 0, 4, 3, 0,
			3, 0, 0, 2, 1, 2, 0, 4, 3, 0,
			3, 0, 0, 2, 1, 2, 0, 4, 4, 0,
			3, 0, 0, 2, 1, 2, 0, 5, 4, 0,
			3, 0, 0, 3, 1, 2, 0, 5, 4, 0,
			3, 0, 0, 3, 1, 2, 0, 5, 5, 0,
			3, 0, 1, 3, 1, 2, 0, 5, 5, 0
		};

		for (sp_uint i = 0; i < 200; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandPrefixScanSwapped, GpuCommand);
		sp_mem_delete(commandPrefixScan, GpuCommand);
		gpu->releaseBuffer(offsetTable1GPU);
		gpu->releaseBuffer(offsetTable2GPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_PrefixScan4)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint count = 20;
		sp_uint offsetTableValues[200] = {
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 0, 0, 0, 0, 0, 0, 0, 0, 0
		};

		sp_uint threadsLength = gpu->getThreadLength(count);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, count);
		const sp_uint elementsPerThread = count / threadsLength;
		const sp_uint maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(threadsLength))) + ONE_UINT;

		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable1GPU = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem offsetTable2GPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		GpuCommand* commandPrefixScanSwapped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		sp_size eventLength = ZERO_UINT;
		cl_event* previousEvents = nullptr;
		cl_event evt;
		sp_bool offsetChanged = false;
		sp_size offsetPrefixScanCpu = 10;
		for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
		{
			if (offsetChanged)
			{
				commandPrefixScanSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);
				gpu->releaseEvents(eventLength, previousEvents);
				previousEvents[0] = evt;
			}
			else
			{
				commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);

				if (eventLength == ZERO_UINT)
				{
					eventLength = ONE_UINT;
					previousEvents = ALLOC_NEW(cl_event)();
				}
				else
					gpu->releaseEvents(eventLength, previousEvents);

				previousEvents[0] = evt;
			}

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		gpu->commandManager->readBuffer(offsetTable2GPU, offsetTableSize, offsetTable, ONE_UINT, previousEvents);

		sp_log_info1s("BEGIN OFFSET2"); sp_log_newline();
		for (sp_uint i = 0; i < offsetTableSize / sizeof(sp_uint) / 10; i++)
		{
			for (sp_uint j = 0; j < 10; j++)
			{
				sp_log_info1u(offsetTable[i * 10 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		sp_uint expected[200] = {
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 2, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			2, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			3, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			4, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			5, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			6, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			7, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			8, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			9, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			10, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			11, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			12, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			13, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			14, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			15, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			16, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			17, 3, 0, 0, 0, 0, 0, 0, 0, 0
		};

		for (sp_uint i = 0; i < 200; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandPrefixScanSwapped, GpuCommand);
		sp_mem_delete(commandPrefixScan, GpuCommand);
		gpu->releaseBuffer(offsetTable1GPU);
		gpu->releaseBuffer(offsetTable2GPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_PrefixScan_Negative5)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program; 
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint count = 20;
		sp_uint offsetTableValues[40] = {
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1,
			0, 1
		};

		sp_uint threadsLength = gpu->getThreadLength(count);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, count);
		const sp_uint elementsPerThread = count / threadsLength;
		const sp_uint maxIteration = (sp_uint)std::ceil(std::log(multiplyBy2(threadsLength))) + ONE_UINT;

		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable1GPU = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem offsetTable2GPU = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		GpuCommand* commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScanNegatives");

		GpuCommand* commandPrefixScanSwapped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2GPU, offsetTableSize)
			->setInputParameter(offsetTable1GPU, offsetTableSize)
			->buildFromProgram(program, "prefixScanNegatives");

		sp_size eventLength = ZERO_UINT;
		cl_event evt, *previousEvents = nullptr;
		sp_bool offsetChanged = false;
		sp_size offsetPrefixScanCpu = 2;
		for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
		{
			if (offsetChanged)
			{
				commandPrefixScanSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);
				gpu->releaseEvents(eventLength, previousEvents);
				previousEvents[0] = evt;
			}
			else
			{
				commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, eventLength, previousEvents, &evt);

				if (eventLength == ZERO_UINT)
				{
					eventLength = ONE_UINT;
					previousEvents = ALLOC_NEW(cl_event)();
				}
				else
					gpu->releaseEvents(eventLength, previousEvents);

				previousEvents[0] = evt;
			}

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		sp_uint* offsetTable = (sp_uint*)ALLOC_SIZE(offsetTableSize);
		gpu->commandManager->readBuffer(offsetTable2GPU, offsetTableSize, offsetTable, ONE_UINT, previousEvents);

		sp_log_info1s("BEGIN OFFSET2"); sp_log_newline();
		for (sp_uint i = 0; i < 20; i++)
		{
			for (sp_uint j = 0; j < 2; j++)
			{
				sp_log_info1u(offsetTable[i * 2 + j]);
				sp_log_info1s(", ");
			}
			sp_log_newline();
		}
		sp_log_info1s("END OFFSET TABLE"); sp_log_newline();

		sp_uint expected[40] = {
			0, 1,
			0, 2,
			0, 3,
			0, 4,
			0, 5,
			0, 6,
			0, 7,
			0, 8,
			0, 9,
			0, 10,
			0, 11,
			0, 12,
			0, 13,
			0, 14,
			0, 15,
			0, 16,
			0, 17,
			0, 18,
			0, 19,
			0, 20
		};

		for (sp_uint i = 0; i < 40; i++)
			Assert::AreEqual(expected[i], offsetTable[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandPrefixScanSwapped, GpuCommand);
		sp_mem_delete(commandPrefixScan, GpuCommand);
		gpu->releaseBuffer(offsetTable1GPU);
		gpu->releaseBuffer(offsetTable2GPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Reorder2)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint offsetTableValues[200] = {
			0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 1, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 1, 1, 0, 0, 0, 0, 0,
			0, 0, 1, 1, 1, 0, 0, 1, 0, 0,
			0, 0, 1, 1, 1, 0, 0, 1, 1, 0,
			0, 0, 1, 1, 1, 0, 1, 1, 1, 0,
			0, 0, 1, 1, 1, 0, 2, 1, 1, 0,
			0, 0, 1, 1, 2, 0, 2, 1, 1, 0,
			0, 0, 2, 1, 2, 0, 2, 1, 1, 0,
			0, 0, 2, 1, 2, 0, 3, 1, 1, 0,
			0, 0, 2, 1, 2, 0, 3, 1, 2, 0,
			0, 0, 3, 1, 2, 0, 3, 1, 2, 0,
			1, 0, 3, 1, 2, 0, 3, 1, 2, 0,
			1, 0, 3, 1, 2, 0, 4, 1, 2, 0,
			1, 0, 3, 1, 2, 0, 4, 1, 2, 1,
			1, 0, 3, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 3, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 4, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 5, 1, 2, 0, 5, 1, 2, 1,
			2, 0, 5, 1, 2, 0, 5, 1, 3, 1
		};

		sp_uint indexes[20] = {
			0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
		};

		sp_uint expected[20] = {
			12, 16, 2, 8, 11, 17, 18, 0, 1, 7, 5, 6, 9, 13, 15, 3, 4, 10, 19, 14 
		};
		
		sp_size digitIndex = 3;
		sp_uint threadsLength = gpu->getThreadLength(inputLength);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, inputLength);
		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem indexesInputGPU = gpu->createBuffer(indexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		cl_mem indexesOutputGpu = gpu->createBuffer(sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(values, sizeof(sp_float) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint))
			->setInputParameter(offsetTable, offsetTableSize)
			->setInputParameter(indexesInputGPU, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesOutputGpu, sizeof(sp_uint) * inputLength)
			->buildFromProgram(program, "reorder")
			->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, ZERO_UINT, NULL, &evt);

		sp_uint* result = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(indexesOutputGpu, sizeof(sp_uint) * inputLength, result, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("OUTPUT INDEXES"); sp_log_newline();
		for (sp_uint i = 0; i < inputLength; i++)
		{
			sp_log_info1u(result[i]);
			sp_log_info1s(" -> ");
			sp_log_info1u(values[result[i]]);
			sp_log_newline();
		}
		sp_log_info1s("END OUTPUT INDEXES"); sp_log_newline();

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandReorder, GpuCommand);
		gpu->releaseBuffer(offsetTable);
		gpu->releaseBuffer(indexesOutputGpu);
		gpu->releaseBuffer(indexesInputGPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Reorder3)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint offsetTableValues[200] = {
			0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
			0, 0, 0, 1, 0, 0, 0, 0, 1, 0,
			1, 0, 0, 1, 0, 0, 0, 0, 1, 0,
			1, 0, 0, 1, 0, 0, 0, 0, 2, 0,
			1, 0, 0, 1, 0, 0, 0, 1, 2, 0,
			2, 0, 0, 1, 0, 0, 0, 1, 2, 0,
			2, 0, 0, 1, 0, 0, 0, 2, 2, 0,
			2, 0, 0, 2, 0, 0, 0, 2, 2, 0,
			2, 0, 0, 2, 0, 0, 0, 2, 3, 0,
			2, 0, 0, 2, 0, 0, 0, 3, 3, 0,
			3, 0, 0, 2, 0, 0, 0, 3, 3, 0,
			3, 0, 0, 2, 0, 1, 0, 3, 3, 0,
			3, 0, 0, 2, 1, 1, 0, 3, 3, 0,
			3, 0, 0, 2, 1, 1, 0, 4, 3, 0,
			3, 0, 0, 2, 1, 2, 0, 4, 3, 0,
			3, 0, 0, 2, 1, 2, 0, 4, 4, 0,
			3, 0, 0, 2, 1, 2, 0, 5, 4, 0,
			3, 0, 0, 3, 1, 2, 0, 5, 4, 0,
			3, 0, 0, 3, 1, 2, 0, 5, 5, 0,
			3, 0, 1, 3, 1, 2, 0, 5, 5, 0
		};

		sp_uint indexes[20] = {
			12, 16, 2, 8, 11, 17, 18, 0, 1, 7, 5, 6, 9, 13, 15, 3, 4, 10, 19, 14
		};

		sp_uint expected[20] = {
			2, 17, 5, 14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19
		};

		sp_size digitIndex = 4;
		sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, inputLength);
		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem indexesInputGPU = gpu->createBuffer(indexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		cl_mem indexesOutputGpu = gpu->createBuffer(sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(values, sizeof(sp_float) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint))
			->setInputParameter(offsetTable, offsetTableSize)
			->setInputParameter(indexesInputGPU, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesOutputGpu, sizeof(sp_uint) * inputLength)
			->buildFromProgram(program, "reorder")
			->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, ZERO_UINT, NULL, &evt);

		sp_uint* result = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(indexesOutputGpu, sizeof(sp_uint) * inputLength, result, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("OUTPUT INDEXES"); sp_log_newline();
		for (sp_uint i = 0; i < inputLength; i++)
		{
			sp_log_info1u(result[i]);
			sp_log_info1s(" -> ");
			sp_log_info1u(values[result[i]]);
			sp_log_newline();
		}
		sp_log_info1s("END OUTPUT INDEXES"); sp_log_newline();

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandReorder, GpuCommand);
		gpu->releaseBuffer(offsetTable);
		gpu->releaseBuffer(indexesOutputGpu);
		gpu->releaseBuffer(indexesInputGPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Reorder4)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint offsetTableValues[200] = {
			0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 2, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			1, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			2, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			3, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			4, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			5, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			6, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			7, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			8, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			9, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			10, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			11, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			12, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			13, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			14, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			15, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			16, 3, 0, 0, 0, 0, 0, 0, 0, 0,
			17, 3, 0, 0, 0, 0, 0, 0, 0, 0
		};

		sp_uint indexes[20] = {
			2, 17, 5, 14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19
		};

		sp_uint expected[20] = {
			14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19, 2, 17, 5
		};

		sp_size digitIndex = 5;
		sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, inputLength);
		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy10(threadsLength);
		cl_mem offsetTable = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem indexesInputGPU = gpu->createBuffer(indexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		cl_mem indexesOutputGpu = gpu->createBuffer(sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(values, sizeof(sp_float) * inputLength)
			->setInputParameter(&inputLength, sizeof(sp_uint))
			->setInputParameter(offsetTable, offsetTableSize)
			->setInputParameter(indexesInputGPU, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesOutputGpu, sizeof(sp_uint) * inputLength)
			->buildFromProgram(program, "reorder")
			->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, ZERO_UINT, NULL, &evt);

		sp_uint* result = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(indexesOutputGpu, sizeof(sp_uint) * inputLength, result, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("OUTPUT INDEXES"); sp_log_newline();
		for (sp_uint i = 0; i < inputLength; i++)
		{
			sp_log_info1u(result[i]);
			sp_log_info1s(" -> ");
			sp_log_info1u(values[result[i]]);
			sp_log_newline();
		}
		sp_log_info1s("END OUTPUT INDEXES"); sp_log_newline();

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandReorder, GpuCommand);
		gpu->releaseBuffer(offsetTable);
		gpu->releaseBuffer(indexesOutputGpu);
		gpu->releaseBuffer(indexesInputGPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_Reorder_Negative5)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

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

		cl_program program;
		gpu->commandManager->buildProgram(source, sizeof(sp_char) * fileSize, nullptr, &program);

		ALLOC_RELEASE(source);

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};
		sp_uint offsetTableValues[40] = {
			0, 1,
			0, 2,
			0, 3,
			0, 4,
			0, 5,
			0, 6,
			0, 7,
			0, 8,
			0, 9,
			0, 10,
			0, 11,
			0, 12,
			0, 13,
			0, 14,
			0, 15,
			0, 16,
			0, 17,
			0, 18,
			0, 19,
			0, 20
		};

		sp_uint indexes[20] = {
			14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19, 2, 17, 5
		};

		sp_uint expected[20] = {
			14, 16, 0, 10, 9, 6, 15, 11, 18, 7, 13, 4, 12, 8, 1, 3, 19, 2, 17, 5
		};

		sp_size threadsLength = gpu->getThreadLength(inputLength);
		const sp_size defaultLocalWorkSize = gpu->getGroupLength(threadsLength, inputLength);
		sp_size globalWorkSize[3] = { threadsLength, 0, 0 };
		sp_size localWorkSize[3] = { defaultLocalWorkSize, 0, 0 };

		const sp_uint offsetTableSize = sizeof(sp_uint) * multiplyBy2(threadsLength);
		cl_mem offsetTable = gpu->createBuffer(offsetTableValues, offsetTableSize, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR, true);
		cl_mem indexesInputGPU = gpu->createBuffer(indexes, sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		cl_mem indexesOutputGpu = gpu->createBuffer(sizeof(sp_uint) * inputLength, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

		cl_event evt;
		GpuCommand* commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(&inputLength, sizeof(sp_uint))
			->setInputParameter(offsetTable, offsetTableSize)
			->setInputParameter(indexesInputGPU, sizeof(sp_uint) * inputLength)
			->setInputParameter(indexesOutputGpu, sizeof(sp_uint) * inputLength)
			->buildFromProgram(program, "reorderNegatives")
			->execute(ONE_UINT, globalWorkSize, localWorkSize, ZERO_UINT, ZERO_UINT, NULL, &evt);

		sp_uint* result = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(indexesOutputGpu, sizeof(sp_uint) * inputLength, result, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("OUTPUT INDEXES"); sp_log_newline();
		for (sp_uint i = 0; i < inputLength; i++)
		{
			sp_log_info1u(result[i]);
			sp_log_info1s(" -> ");
			sp_log_info1u(values[result[i]]);
			sp_log_newline();
		}
		sp_log_info1s("END OUTPUT INDEXES"); sp_log_newline();

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(expected[i], result[i], L"Wrong value.", LINE_INFO());

		sp_mem_delete(commandReorder, GpuCommand);
		gpu->releaseBuffer(offsetTable);
		gpu->releaseBuffer(indexesOutputGpu);
		gpu->releaseBuffer(indexesInputGPU);
		ALLOC_RELEASE(offsetTable);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_1)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);
		cl_mem inputLengthGPU = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes
			->init(gpu, nullptr)
			->setParametersCreateIndexes(inputLength);
		cl_mem initialIndexes = createIndexes->execute(ZERO_UINT, NULL, NULL);

		const sp_uint maxIterations = 20;
		for (sp_size i = 0; i < maxIterations; i++)
		{	
			sp_float* input1 = getRandom(inputLength);
			input1[2] = -0.01f;
			sp_float* input2 = ALLOC_COPY(input1, sp_float, inputLength);

			cl_mem inputGpu = gpu->createBuffer(input2, sizeof(sp_float) * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);

			AlgorithmSorting::native(input1, inputLength);

			GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();

			cl_event evt;
			cl_mem output = radixGpu
						->init(gpu, nullptr)
						->setParameters(inputGpu, inputLength, initialIndexes, inputLengthGPU)
						->execute(ZERO_UINT, NULL, &evt);

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			gpu->commandManager->readBuffer(output, inputLength * sizeof(sp_uint), orderedIndexes, ONE_UINT, &evt);
			gpu->releaseEvent(evt);

			for (sp_uint i = 0; i < 10; i++)
			{
				sp_log_debug1u(input1[i]);
				sp_log_debug1s(" == ");
				sp_log_debug1u(input2[orderedIndexes[i]]);
				sp_log_debug1s(" -> ");
				sp_log_debug1u(orderedIndexes[i]);
				sp_log_newline();
			}

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
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint iterations = 10u;
		std::chrono::milliseconds times[iterations];

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		for (sp_size i = 0; i < iterations; i++)
		{
			sp_size length = (sp_size)std::pow(2.0, 17.0);
			AABB* input1 = getRandomAABBs((sp_uint)length);
			AABB* input2 = ALLOC_COPY(input1, AABB, length);
			cl_mem inputGpu = gpu->createBuffer(input2, sizeof(AABB) * length, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);

			cl_mem newIndexesLength = gpu->createBuffer(&length, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
			createIndexes->setParametersCreateIndexes((sp_uint)length);
			cl_mem newIndexes = createIndexes->execute(ZERO_UINT, NULL, NULL);

			std::ostringstream buildOptions;
			buildOptions << " -DINPUT_LENGTH=" << length
				<< " -DINPUT_STRIDE=" << AABB_STRIDER
				<< " -DINPUT_OFFSET=0";

			GpuRadixSorting* radixSorting = ALLOC_NEW(GpuRadixSorting)();
			radixSorting->init(gpu, buildOptions.str().c_str())
				->setParameters(inputGpu, (sp_uint)length, newIndexes, newIndexesLength, AABB_STRIDER);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::quickSortNative(input1, length, sizeof(AABB), comparatorAABBirstAxisTest);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::milliseconds ms1 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_event evt;
			cl_mem output = radixSorting->execute(ZERO_UINT, NULL, &evt);

			currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::milliseconds ms2 = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime2 - currentTime);
			times[i] = ms2;

			sp_uint* result = ALLOC_ARRAY(sp_uint, length * sizeof(sp_uint));
			gpu->commandManager->readBuffer(output, length * sizeof(sp_uint), result, ONE_UINT, &evt);
			gpu->releaseEvent(evt);

			for (sp_uint i = 0; i < length; i++)
				Assert::AreEqual(input1[i].minPoint.x, input2[result[i]].minPoint.x, L"Wrong value.", LINE_INFO());

			gpu->releaseBuffer(inputGpu);
			ALLOC_RELEASE(input1);
			ALLOC_DELETE(radixSorting, GpuRadixSorting);
		}

		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_3)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		sp_uint inputLength = 20;
		sp_float values[20] = {
			33.0f, 84.0f, 102.0f, 87.0f,
			78.0f, 106.0f, 56.0f, 74.0f,
			82.0f, 46.0f, 38.0f, 72.0f,
			80.0f, 76.0f, 29.0f, 56.0f,
			30.0f, 102.0f, 72.0f, 88.0f
		};

		sp_float* values2 = ALLOC_COPY(values, sp_float, inputLength);
		cl_mem inputGpu = gpu->createBuffer(values2, sizeof(sp_float) * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);

		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_event evt;
		cl_mem newIndexes = createIndexes->execute(ZERO_UINT, NULL, &evt);

		sp_uint* initialIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(newIndexes, inputLength * sizeof(sp_uint), initialIndexes, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("BEGIN VALUES 1"); sp_log_newline();
		for (sp_uint i = 0; i < inputLength; i++)
		{
			sp_log_info1u(initialIndexes[i]);
			sp_log_info1s(" -> ");
			sp_log_info1f(values[initialIndexes[i]]);
			sp_log_newline();
		}
		sp_log_info1s("END"); sp_log_newline();

		AlgorithmSorting::native(values2, inputLength);

		sp_uint strider = 1u;
		sp_uint offset = 0u;
		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu->init(gpu, nullptr)
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, strider);

		cl_mem output = radixGpu->execute(ZERO_UINT, NULL, &evt);

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(output, inputLength * sizeof(sp_uint), orderedIndexes, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		sp_log_info1s("BEGIN VALUES 1"); sp_log_newline();
		for (sp_uint i = 0; i < inputLength; i++)
		{
			sp_log_info1u(orderedIndexes[i]);
			sp_log_info1s(" -> ");
			sp_log_info1f(values[orderedIndexes[i]]);
			sp_log_info1s(" == ");
			sp_log_info1f(values2[i]);
			sp_log_newline();
		}
		sp_log_info1s("END"); sp_log_newline();

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(values2[i], values[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(inputGpu);
		ALLOC_DELETE(radixGpu, GpuRadixSorting);
		ALLOC_RELEASE(values2);

		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_WithNegatives)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		sp_uint inputLength = 8u;
		sp_uint strider = 1u;
		sp_uint offset = 0u;

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);

		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_mem newIndexes = createIndexes->execute(ZERO_UINT, NULL, NULL);

		sp_float input1[8] = { 50.0f, 2.0f, -5.0f, 4.0f, 12.0f, -10.0f, -1.0f, 30.0f };
		sp_float input2[8] = { 50.0f, 2.0f, -5.0f, 4.0f, 12.0f, -10.0f, -1.0f, 30.0f };
		cl_mem inputGpu = gpu->createBuffer(input2, sizeof(sp_float) * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu->init(gpu, nullptr)
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, strider);

		AlgorithmSorting::native(input1, inputLength);

		cl_event evt;
		cl_mem output = radixGpu->execute(ZERO_UINT, NULL, &evt);

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(output, inputLength * sizeof(sp_uint), orderedIndexes, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

		for (sp_uint i = 0; i < inputLength; i++)
			Assert::AreEqual(input1[i], input2[orderedIndexes[i]], L"Wrong value.", LINE_INFO());

		gpu->releaseBuffer(inputGpu);
		ALLOC_DELETE(radixGpu, GpuRadixSorting);
		ALLOC_RELEASE(radixGpu);
		ALLOC_DELETE(createIndexes, GpuIndexes);
	}

	SP_TEST_METHOD(CLASS_NAME, radixGPU_WithKDOPs)
	{
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint maxIterations = 20;
		std::chrono::nanoseconds times[maxIterations];
		std::chrono::nanoseconds minTime(99999999999);

		sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);

		cl_mem inputLengthGPU = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr)
						->setParametersCreateIndexes(inputLength);
		cl_mem newIndexes = createIndexes->execute(ZERO_UINT, NULL, NULL);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0;

		for (sp_uint i = 0; i < maxIterations; i++)
		{
			DOP18* input1 = getRandomKDOPs(inputLength);
			input1[2].min[0] = -0.01f;
			DOP18* input2 = ALLOC_COPY(input1, DOP18, inputLength);
			cl_mem inputGpu = gpu->createBuffer(input2, DOP18_SIZE * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);

			std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			AlgorithmSorting::quickSortNative(input1, inputLength, DOP18_SIZE, comparatorXAxisForQuickSortKDOP);

			std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			std::chrono::nanoseconds ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);

			GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
			radixGpu
				->init(gpu, buildOptions.str().c_str())
				->setParameters(inputGpu, inputLength, newIndexes, inputLengthGPU, DOP18_STRIDER);

			currentTime = std::chrono::high_resolution_clock::now();

			cl_event evt;
			cl_mem output = radixGpu->execute(ZERO_UINT, NULL, &evt);

			currentTime2 = std::chrono::high_resolution_clock::now();
			times[i] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			minTime = std::min(times[i], minTime);

			sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
			gpu->commandManager->readBuffer(output, inputLength * sizeof(sp_uint), orderedIndexes, ONE_UINT, &evt);
			gpu->releaseEvent(evt);

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
		GpuContext* context = GpuContextInstance;
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
		cl_mem inputGpu = gpu->createBuffer(input1, DOP18_SIZE * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);

		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_mem newIndexes = createIndexes->execute(ZERO_UINT, NULL, NULL);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0;

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu
			->init(gpu, buildOptions.str().c_str())
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, DOP18_STRIDER);

		cl_event evt;
		cl_mem output = radixGpu->execute(ZERO_UINT, NULL, &evt);

		sp_uint* orderedIndexes = ALLOC_ARRAY(sp_uint, inputLength);
		gpu->commandManager->readBuffer(output, inputLength * sizeof(sp_uint), orderedIndexes, ONE_UINT, &evt);
		gpu->releaseEvent(evt);

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
		GpuContext* context = GpuContextInstance;
		GpuDevice* gpu = context->defaultDevice();

		const sp_uint maxIterations = 3;
		//sp_uint inputLength = (sp_uint)std::pow(2.0, 17.0);
		sp_uint inputLength = 64;

		DOP18* input1 = getRandomKDOPs(inputLength);
		input1[2].min[0] = -0.01f;
		DOP18* input2 = ALLOC_COPY(input1, DOP18, inputLength);

		cl_mem inputGpu = gpu->createBuffer(input2, DOP18_SIZE * inputLength, CL_MEM_READ_ONLY | CL_MEM_USE_HOST_PTR, true);
		cl_mem newIndexesLength = gpu->createBuffer(&inputLength, sizeof(sp_uint), CL_MEM_READ_WRITE | CL_MEM_USE_HOST_PTR);

		GpuIndexes* createIndexes = ALLOC_NEW(GpuIndexes)();
		createIndexes->init(gpu, nullptr);
		createIndexes->setParametersCreateIndexes(inputLength);
		cl_event evt;
		cl_mem newIndexes = createIndexes->execute(ZERO_UINT, NULL, &evt);

		std::ostringstream buildOptions;
		buildOptions << " -DINPUT_LENGTH=" << inputLength
			<< " -DINPUT_STRIDE=" << DOP18_STRIDER
			<< " -DINPUT_OFFSET=" << 0;

		GpuRadixSorting* radixGpu = ALLOC_NEW(GpuRadixSorting)();
		radixGpu
			->init(gpu, buildOptions.str().c_str())
			->setParameters(inputGpu, inputLength, newIndexes, newIndexesLength, DOP18_STRIDER);

		AlgorithmSorting::quickSortNative(input1, inputLength, DOP18_SIZE, comparatorXAxisForQuickSortKDOP);

		cl_event pEvent = evt, lastEvent;
		sp_uint eventsLength = ONE_UINT;
		 
		for (sp_uint i = 0; i < maxIterations; i++)
		{
			cl_mem output = radixGpu->execute(eventsLength, &pEvent, &lastEvent);
			gpu->releaseEvent(pEvent);

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
			lastEvent = gpu->commandManager->readBuffer(output, inputLength * sizeof(sp_uint), orderedIndexes, ONE_UINT, &lastEvent);
			std::swap(lastEvent, pEvent);

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
