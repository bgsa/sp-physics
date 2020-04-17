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

		commandReverse = ALLOC_NEW(GpuReverse)();
		commandReverse->init(gpu, buildOptions);

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

		inputGpu = gpu->createBuffer(input, inputSize, CL_MEM_READ_ONLY);
		indexesLengthGpu = gpu->createBuffer(&inputLength, SIZEOF_UINT, CL_MEM_READ_ONLY);
		outputIndexesGpu = gpu->createBuffer(inputLength * SIZEOF_UINT, CL_MEM_READ_WRITE);

		commandCreateIndexes->setParametersCreateIndexes(inputLength);
		inputIndexesGpu = commandCreateIndexes->execute();

		threadsLength = gpu->getThreadLength(inputLength);
		defaultLocalWorkSize = gpu->getGroupLength(threadsLength, inputLength);

		maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(threadsLength))) + ONE_UINT;

		const sp_uint offsetTableSize = SIZEOF_UINT * TEN_UINT * threadsLength;
		offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);

		commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

		commandCountSwapped = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

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
			->setInputParameter(offsetTable1, offsetTableSize)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorder");

		commandReorderSwapped = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorder");


		const sp_uint offsetTableSizeNegatives = SIZEOF_UINT * TWO_UINT * threadsLength;
		offsetTable1Negatives = gpu->createBuffer(offsetTableSizeNegatives, CL_MEM_READ_WRITE);
		offsetTable2Negatives = gpu->createBuffer(offsetTableSizeNegatives, CL_MEM_READ_WRITE);

		commandCountNegative = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->buildFromProgram(program, "countNegatives");

		commandPrefixScanNegative = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2Negatives, offsetTableSizeNegatives)  //use buffer hosted GPU
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->buildFromProgram(program, "prefixScanNegatives");

		commandReorderNegative = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorderNegatives");


		cl_buffer_region region = { offsetTableSizeNegatives - TWO_UINT * SIZEOF_UINT, SIZEOF_UINT };
		negativeCounterLocation1 = gpu->createSubBuffer(offsetTable1Negatives, &region, CL_MEM_READ_ONLY);
		negativeCounterLocation2 = gpu->createSubBuffer(offsetTable2Negatives, &region, CL_MEM_READ_ONLY);
		commandReverse
			->updateInputLength(inputLength)
			->setParameters(inputIndexesGpu, outputIndexesGpu, negativeCounterLocation1);

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
		
		for (sp_uint digitIndex = ZERO_UINT; digitIndex < 7u; digitIndex++)  // for each digit in the number ...
		{
			//std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();

			if (previousEvent == NULL)
			{
				commandCount->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, NULL, ZERO_UINT);
				previousEvent = commandCount->lastEvent;
			}
			else
			{
				if (offsetChanged)
					commandCountSwapped->updateInputParameter(THREE_UINT, offsetTable2);

				if (indexesChanged)
				{	
					commandCountSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, ONE_UINT);
					previousEvent = commandCountSwapped->lastEvent;
				}
				else
				{
					commandCount->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, ONE_UINT);
					previousEvent = commandCount->lastEvent;
				}
			}

			//std::chrono::high_resolution_clock::time_point currentTime2 = std::chrono::high_resolution_clock::now();
			//timeCount[in] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			//currentTime = std::chrono::high_resolution_clock::now();

			sp_uint offsetPrefixScanCpu = BUCKET_LENGTH;
			for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
			{
				if (offsetChanged)
				{
					commandPrefixScanSwaped->execute(1, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, &previousEvent, ONE_UINT);
					previousEvent = commandPrefixScanSwaped->lastEvent;
				}
				else
				{
					commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, &previousEvent, ONE_UINT);
					previousEvent = commandPrefixScan->lastEvent;
				}

				offsetChanged = !offsetChanged;
				offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
			}

			//currentTime2 = std::chrono::high_resolution_clock::now();
			//timeScan[in] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			//currentTime = std::chrono::high_resolution_clock::now();

			if (offsetChanged)
				commandReorder->updateInputParameter(TWO_UINT, offsetTable2);

			if (indexesChanged)
			{	
				commandReorderSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, ONE_UINT);
				previousEvent = commandReorderSwapped->lastEvent;
			}
			else
			{
				commandReorder->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, &previousEvent, ONE_UINT);
				previousEvent = commandReorder->lastEvent;
			}
			indexesChanged = !indexesChanged;

			//currentTime2 = std::chrono::high_resolution_clock::now();
			//timeReorder[in] = std::chrono::duration_cast<std::chrono::nanoseconds>(currentTime2 - currentTime);
			//in++;
		}

		lastEvent = commandReorder->lastEvent;

		return executeNegatives(indexesChanged, offsetChanged);
		/*
		if (indexesChanged)
			output = outputIndexesGpu;
		else
			output = inputIndexesGpu;
			
		return output;
		*/
	}

	cl_mem GpuRadixSorting::executeNegatives(sp_bool indexChanged, sp_bool offsetChanged)
	{
		sp_uint offsetPrefixScanCpu = TWO_UINT;
		cl_event previousEvent = this->lastEvent;
		
		if (indexChanged)
			commandCountNegative->updateInputParameter(ONE_UINT, outputIndexesGpu);

		if (offsetChanged)
			commandCountNegative->updateInputParameter(THREE_UINT, offsetTable2Negatives);

		commandCountNegative
			->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
		previousEvent = commandCountNegative->lastEvent;

		if (offsetChanged)
			commandPrefixScanNegative->swapInputParameter(ZERO_UINT, ONE_UINT);

		offsetPrefixScanCpu = TWO_UINT;
		while(offsetPrefixScanCpu - 1 < threadsLength)
		{
			commandPrefixScanNegative
				->swapInputParameter(ZERO_UINT, ONE_UINT)
				->execute(1, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, &previousEvent, ONE_UINT);
			previousEvent = commandPrefixScanNegative->lastEvent;

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		if (offsetChanged) 
		{
			commandReorderNegative->updateInputParameter(TWO_UINT, offsetTable2Negatives);
			commandReverse->updateInputLength(negativeCounterLocation2);
		}

		if (indexChanged)
			commandReorderNegative->swapInputParameter(THREE_UINT, FOUR_UINT);
		else
			commandReverse->swapIndexes();
		
		commandReorderNegative->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
		this->lastEvent = commandReorderNegative->lastEvent;

		output = commandReverse->execute();
		this->lastEvent = commandReverse->lastEvent;
		
		return output;
	}

	GpuRadixSorting::~GpuRadixSorting()
	{
		gpu->releaseBuffer(negativeCounterLocation1);
		gpu->releaseBuffer(negativeCounterLocation2);

		gpu->releaseBuffer(inputGpu);
		gpu->releaseBuffer(inputIndexesGpu);
		gpu->releaseBuffer(outputIndexesGpu);
		gpu->releaseBuffer(indexesLengthGpu);
		gpu->releaseBuffer(offsetTable1);
		gpu->releaseBuffer(offsetTable2);

		ALLOC_DELETE(commandCreateIndexes, GpuIndexes);
		ALLOC_DELETE(commandCount, GpuCommand);
		ALLOC_DELETE(commandCountSwapped, GpuCommand);
		ALLOC_DELETE(commandPrefixScan, GpuCommand);
		ALLOC_DELETE(commandPrefixScanSwaped, GpuCommand);
		ALLOC_DELETE(commandReorder, GpuCommand);
		ALLOC_DELETE(commandReorderSwapped, GpuCommand);
		ALLOC_DELETE(commandReverse, GpuReverse);
	}
}

#undef BUCKET_LENGTH

#endif // OPENCL_ENABLED