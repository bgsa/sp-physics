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

		delete fileManager;
		return this;
	}

	GpuRadixSorting* GpuRadixSorting::setParameters(sp_float* input, sp_uint inputLength, sp_uint strider, sp_uint offset)
	{
		sp_bool useExpoent = false;
		sp_uint digitIndex = 0;
		const sp_uint inputSize = inputLength * strider * SIZEOF_FLOAT;
		
		commandCreateIndexes->setParametersCreateIndexes(inputLength);
		indexesGpu = commandCreateIndexes->execute();

		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		indexesLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_ONLY);
		outputIndexes = gpu->createBuffer(inputLength * SIZEOF_UINT, CL_MEM_READ_WRITE);

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


		offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		offsetTableResult = offsetTable2;

		commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

		commandCountSwapped = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(outputIndexes, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

		commandPrefixScanUp = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable, offsetTableSize)
			->buildFromProgram(program, "prefixScanUp");

		commandPrefixScanDown = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable, offsetTableSize)
			->buildFromProgram(program, "prefixScanDown");

		commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable2, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		commandPrefixScanSwaped = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTableResult, offsetTableSize)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexes, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorder");

		commandReorderSwapped = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTableResult, offsetTableSize)
			->setInputParameter(outputIndexes, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorder");

		return this;
	}

	//std::chrono::nanoseconds timeCount[8];
	//std::chrono::nanoseconds timeScan[8];
	//std::chrono::nanoseconds timeReorder[8];
	//sp_int in = 0;

	cl_mem GpuRadixSorting::execute()
	{
		sp_bool indexesChanged = false;
		sp_bool offsetChanged = false;
		cl_event previousEvent = NULL;
		
		globalWorkSize[0] = threadsLength;
		localWorkSize[0] = defaultLocalWorkSize;

		const sp_uint maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(threadsLength))) + 1u;
		
		for (sp_uint digitIndex = 0; digitIndex < 7; digitIndex++)  // for each digit in the number ...
		{
			//std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			if (previousEvent == NULL)
			{
				commandCount->execute(1, globalWorkSize, localWorkSize, &digitIndex, NULL, 0u);
				previousEvent = commandCount->lastEvent;
			}
			else
				if (indexesChanged)
				{
					commandCount->updateInputParameter(3, offsetTableResult);
					commandCountSwapped->execute(1, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, 1u);
					previousEvent = commandCountSwapped->lastEvent;
				}
				else
				{
					commandCount->updateInputParameter(3, offsetTableResult);
					commandCount->execute(1, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, 1u);
					previousEvent = commandCount->lastEvent;
				}

			//std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			//timeCount[in] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			//currentTime = std::chrono::high_resolution_clock::now();

			sp_bool prefixScanSwaped = false;
			sp_uint offsetPrefixScanCpu = BUCKET_LENGTH;
			for (sp_uint i = 0; i < maxIteration; i++)
			{
				if (prefixScanSwaped)
				{
					commandPrefixScanSwaped->execute(1, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, &previousEvent, 1u);
					previousEvent = commandPrefixScanSwaped->lastEvent;
				}
				else
				{
					commandPrefixScan->execute(1, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, &previousEvent, 1u);
					previousEvent = commandPrefixScan->lastEvent;
				}

				offsetChanged = !offsetChanged;
				prefixScanSwaped = !prefixScanSwaped;
				offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
			}
			offsetTableResult = offsetChanged ? offsetTable2 : offsetTable1;

			//currentTime2 = std::chrono::high_resolution_clock::now();
			//timeScan[in] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			//currentTime = std::chrono::high_resolution_clock::now();

			if (indexesChanged)
			{
				commandReorderSwapped->updateInputParameter(2, offsetTableResult);
				commandReorderSwapped->execute(1, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, 1u);
				previousEvent = commandReorderSwapped->lastEvent;
			}
			else
			{
				commandReorder->updateInputParameter(2, offsetTableResult);
				commandReorder->execute(1, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, 1u);
				previousEvent = commandReorder->lastEvent;
			}
			indexesChanged = !indexesChanged;

			//currentTime2 = std::chrono::high_resolution_clock::now();
			//timeReorder[in] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			//in++;
		}

		lastEvent = commandReorder->lastEvent;

		if (indexesChanged)
			output = indexesGpu;
		else
			output = outputIndexes;

		return output;
	}

	GpuRadixSorting::~GpuRadixSorting()
	{
		gpu->releaseBuffer(inputGpu);
		gpu->releaseBuffer(indexesGpu);
		gpu->releaseBuffer(outputIndexes);
		gpu->releaseBuffer(indexesLengthGpu);
		gpu->releaseBuffer(offsetTable);

		ALLOC_DELETE(commandCreateIndexes, GpuIndexes);
		ALLOC_DELETE(commandCount, GpuCommand);
		ALLOC_DELETE(commandCountSwapped, GpuCommand);
		ALLOC_DELETE(commandPrefixScanUp, GpuCommand);
		ALLOC_DELETE(commandPrefixScanDown, GpuCommand);
		ALLOC_DELETE(commandReorder, GpuCommand);
		ALLOC_DELETE(commandReorderSwapped, GpuCommand);
	}
}

#undef BUCKET_LENGTH

#endif // OPENCL_ENABLED