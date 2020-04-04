#ifdef OPENCL_ENABLED

#include "GpuRadixSorting.h"

#define BUCKET_LENGTH 10u

namespace NAMESPACE_PHYSICS
{
	GpuRadixSorting* GpuRadixSorting::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;

		commandCreateIndexes = ALLOC_NEW(GpuIndexes)();
		commandCreateIndexes->init(gpu, buildOptions);

		IFileManager* fileManager = Factory::getFileManagerInstance();

		std::string sourceRadixSort = fileManager->readTextFile("RadixSorting.cl");
		sp_uint radixSortProgramIndex = gpu->commandManager->cacheProgram(sourceRadixSort.c_str(), SIZEOF_CHAR * sourceRadixSort.length(), buildOptions);

		program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];

		findMinMax = ALLOC_NEW(GpuFindMinMax)();
		findMinMax->init(gpu, buildOptions);

		delete fileManager;
		return this;
	}

	GpuRadixSorting* GpuRadixSorting::setParameters(sp_float* input, sp_uint inputLength, sp_uint strider, sp_uint offset)
	{
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;
		const sp_uint inputSize = inputLength * strider * SIZEOF_FLOAT;
		
		findMinMax->setParameters(input, inputLength, strider, offset);
		findMinMax->execute();

		commandCreateIndexes->setParametersCreateIndexes(inputLength);
		indexesGpu = commandCreateIndexes->execute();

		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		indexesLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_ONLY);
		digitIndexGpu = gpu->createBuffer(&digitIndex, SIZEOF_UINT, CL_MEM_READ_WRITE);
		useExpoentGpu = gpu->createBuffer(&useExpoent, SIZEOF_UINT, CL_MEM_READ_WRITE);
		outputIndexes = gpu->createBuffer(inputLength * SIZEOF_UINT, CL_MEM_READ_WRITE);
		
		gpu->commandManager->executeReadBuffer(findMinMax->output, SIZEOF_FLOAT * 2, minMaxValues, true);

		threadsLength = gpu->getThreadLength(inputLength);
		threadsLength = divideBy2(threadsLength);
		defaultLocalWorkSize = gpu->getGroupLength(threadsLength, inputLength);

		sp_uint maxLength = 2u;
		for (; maxLength < threadsLength; maxLength *= 2)
		{
			sp_uint divisor = nextDivisorOf(maxLength - 1, defaultLocalWorkSize);
			if (divisor > gpu->maxWorkGroupSize)
				break;
		}
		while (threadsLength >= maxLength)
			threadsLength = divideBy2(threadsLength);

		const sp_uint offsetTableSize = SIZEOF_UINT * 10u * threadsLength;
		offsetTable = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);

		commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(useExpoentGpu, SIZEOF_BOOL)
			->setInputParameter(findMinMax->output, SIZEOF_FLOAT * 2)
			->setInputParameter(offsetTable, offsetTableSize)
			->buildFromProgram(program, "count");

		commandPrefixScanUp = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable, offsetTableSize)
			->buildFromProgram(program, "prefixScanUp");

		commandPrefixScanDown = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable, offsetTableSize)
			->buildFromProgram(program, "prefixScanDown");

		commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(digitIndexGpu, SIZEOF_UINT)
			->setInputParameter(useExpoentGpu, SIZEOF_BOOL)
			->setInputParameter(offsetTable, offsetTableSize)
			->setInputParameter(findMinMax->output, SIZEOF_FLOAT * 2)
			->setInputParameter(outputIndexes, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorder");

		return this;
	}

	cl_mem GpuRadixSorting::execute()
	{
		const sp_float minValue = -std::min(0.0f, minMaxValues[0]);
		sp_uint digitIndex = 0u;
		sp_bool useExpoent = false;
		sp_bool indexesChanged = false;

		globalWorkSize[0] = threadsLength;
		localWorkSize[0] = defaultLocalWorkSize;
		
		while (true)  // for each digit in one element
		{
			if (indexesChanged)
				commandCount->updateInputParameter(1, outputIndexes);
			else
				commandCount->updateInputParameter(1, indexesGpu);
			commandCount->execute(1, globalWorkSize, localWorkSize);

			sp_uint threadLength = (sp_uint)std::ceil(globalWorkSize[0] / 2.0);
			sp_uint offset = 1u;
			while (globalWorkSize[0] != 1u)
			{
				offset = multiplyBy2(offset);
				globalWorkSize[0] = (sp_uint)std::ceil(globalWorkSize[0] / 2.0);

				if (globalWorkSize[0] < localWorkSize[0])
					localWorkSize[0] = globalWorkSize[0];

				commandPrefixScanUp->execute(1, globalWorkSize, localWorkSize, &offset);
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
			}

			globalWorkSize[0] = threadsLength;
			localWorkSize[0] = defaultLocalWorkSize;

			commandReorder
				->swapInputParameter(6, 7)
				->execute(1, globalWorkSize, localWorkSize);
			indexesChanged = !indexesChanged;

			if (++digitIndex > maxDigits)  // check the algorithm reach the result
			{
				if (useExpoent)
					break;

				useExpoent = true;
				digitIndex = 0;
				maxDigits = digitCount((sp_int)(minMaxValues[1] + minValue));  // MAX_DIGITS_EXPOENT - 1;
			}
		}

		if (indexesChanged)
			return indexesGpu;
		else
			return outputIndexes;
	}

	GpuRadixSorting::~GpuRadixSorting()
	{
		gpu->releaseBuffer(6, inputGpu, indexesGpu, indexesLengthGpu,
							digitIndexGpu, useExpoentGpu, offsetTable);

		ALLOC_DELETE(commandCreateIndexes, GpuIndexes);
		ALLOC_DELETE(findMinMax, GpuFindMinMax);
		ALLOC_DELETE(commandCount, GpuCommand);
		ALLOC_DELETE(commandPrefixScanUp, GpuCommand);
		ALLOC_DELETE(commandPrefixScanDown, GpuCommand);
		ALLOC_DELETE(commandReorder, GpuCommand);
	}
}

#undef BUCKET_LENGTH

#endif