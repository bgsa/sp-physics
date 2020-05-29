#ifdef OPENCL_ENABLED

#include "GpuRadixSorting.h"

#define BUCKET_LENGTH TEN_UINT

namespace NAMESPACE_PHYSICS
{
	GpuRadixSorting* GpuRadixSorting::init(GpuDevice* gpu, const sp_char* buildOptions)
	{
		if (this->gpu != NULL)
			return this;

		this->gpu = gpu;

		commandReverse = sp_mem_new(GpuReverse)();
		commandReverse->init(gpu, buildOptions);

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("RadixSorting.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		sp_mem_delete(filename, SpDirectory);
		const sp_size fileSize = file.length();
		sp_char* source = ALLOC_ARRAY(sp_char, fileSize);
		file.read(source, fileSize);
		file.close();

		sp_uint radixSortProgramIndex = gpu->commandManager->cacheProgram(source, SIZEOF_CHAR * fileSize, buildOptions);

		ALLOC_RELEASE(source);

		program = gpu->commandManager->cachedPrograms[radixSortProgramIndex];

		return this;
	}

	void GpuRadixSorting::updateIndexes(cl_mem newIndexes, cl_mem newIndexesLength)
	{
		commandCount
			->updateInputParameter(1, newIndexes)
			->updateInputParameter(2, newIndexesLength);

		commandCountSwapped
			->updateInputParameter(2, newIndexesLength);

		commandCountNegative
			->updateInputParameter(1, newIndexes)
			->updateInputParameter(2, newIndexesLength);

		commandReorder
			->updateInputParameter(1, newIndexesLength)
			->updateInputParameter(3, newIndexes);

		commandReorderSwapped
			->updateInputParameter(1, newIndexesLength);

		commandReorderNegative
			->updateInputParameter(1, newIndexesLength)
			->updateInputParameter(3, newIndexes);

		commandReverse->updateInputLength(newIndexesLength);
	}

	GpuRadixSorting* GpuRadixSorting::setParameters(cl_mem inputGpu, sp_uint inputLength, cl_mem indexesGpu, cl_mem indexesLengthGpu, sp_uint strider, sp_uint offset)
	{
		inputIndexesGpu = indexesGpu;
		const sp_uint inputSize = inputLength * strider * SIZEOF_FLOAT;

		outputIndexesGpu = gpu->createBuffer(inputLength * SIZEOF_UINT, CL_MEM_READ_WRITE);

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

	cl_mem GpuRadixSorting::execute(sp_uint previousEventsLength, cl_event* previousEvents)
	{
		sp_bool indexesChanged = false;
		sp_bool offsetChanged = false;
		
		globalWorkSize[0] = threadsLength;
		localWorkSize[0] = defaultLocalWorkSize;
		
		for (sp_uint digitIndex = ZERO_UINT; digitIndex < 7u; digitIndex++)  // for each digit in the number ...
		{
			if (offsetChanged)
				commandCountSwapped->updateInputParameter(THREE_UINT, offsetTable2);

			if (indexesChanged)
			{	
				commandCountSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, previousEventsLength);
				previousEvents[0] = commandCountSwapped->lastEvent;
				previousEventsLength = ONE_UINT;
			}
			else
			{
				commandCount->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, previousEventsLength);
				previousEvents[0] = commandCount->lastEvent;
				previousEventsLength = ONE_UINT;
			}

			sp_uint offsetPrefixScanCpu = BUCKET_LENGTH;
			for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
			{
				if (offsetChanged)
				{
					commandPrefixScanSwaped->execute(1, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, previousEvents, ONE_UINT);
					previousEvents[0] = commandPrefixScanSwaped->lastEvent;
				}
				else
				{
					commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, previousEvents, ONE_UINT);
					previousEvents[0] = commandPrefixScan->lastEvent;
				}

				offsetChanged = !offsetChanged;
				offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
			}

			if (offsetChanged)
				commandReorder->updateInputParameter(TWO_UINT, offsetTable2);

			if (indexesChanged)
			{	
				commandReorderSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, ONE_UINT);
				previousEvents[0] = commandReorderSwapped->lastEvent;
			}
			else
			{
				commandReorder->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, ONE_UINT);
				previousEvents[0] = commandReorder->lastEvent;
			}

			indexesChanged = !indexesChanged;
		}

		lastEvent = previousEvents[0];

		return executeNegatives(indexesChanged);
	}

	cl_mem GpuRadixSorting::executeNegatives(sp_bool indexChanged)
	{
		sp_bool offsetChanged = false;
		sp_uint offsetPrefixScanCpu = TWO_UINT;
		cl_event previousEvent = this->lastEvent;

		commandCountNegative
			->updateInputParameter(THREE_UINT, offsetTable1Negatives);

		commandPrefixScanNegative
			->updateInputParameter(0u, offsetTable2Negatives)
			->updateInputParameter(1u, offsetTable1Negatives);

		commandReorderNegative
			->updateInputParameter(2u, offsetTable1Negatives)
			->updateInputParameter(3u, inputIndexesGpu)
			->updateInputParameter(4u, outputIndexesGpu);

		commandReverse
			->updateInput(inputIndexesGpu, outputIndexesGpu);

		if (indexChanged)
		{
			commandCountNegative->updateInputParameter(ONE_UINT, outputIndexesGpu);
			commandReorderNegative->swapInputParameter(THREE_UINT, FOUR_UINT);
		}
		else
			commandReverse->swapIndexes();


		commandCountNegative->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
		previousEvent = commandCountNegative->lastEvent;

		offsetPrefixScanCpu = TWO_UINT; // prepare prefix scan
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
			commandReorderNegative->updateInputParameter(TWO_UINT, offsetTable2Negatives);

		commandReorderNegative->execute(1, globalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
		this->lastEvent = commandReorderNegative->lastEvent;


		sp_uint* buffer = ALLOC_ARRAY(sp_uint, 2);
		gpu->commandManager->executeReadBuffer(negativeCounterLocation1, 4 * 1, buffer);
		gpu->commandManager->executeReadBuffer(negativeCounterLocation2, 4 * 1, &buffer[1]);
		std::stringstream ss;
		ss << "NEGATIVE COUNTER: " << buffer[0] << " , " << buffer[1] << END_OF_LINE;
		OutputDebugStringA(ss.str().c_str());
		ALLOC_RELEASE(buffer);


		output = commandReverse->execute(ONE_UINT, &this->lastEvent);
		this->lastEvent = commandReverse->lastEvent;
		
		return output;
	}

	void GpuRadixSorting::dispose()
	{
		gpu->releaseBuffer(negativeCounterLocation1);
		gpu->releaseBuffer(negativeCounterLocation2);

		gpu->releaseBuffer(outputIndexesGpu);
		gpu->releaseBuffer(offsetTable1);
		gpu->releaseBuffer(offsetTable2);

		sp_mem_delete(commandCount, GpuCommand);
		sp_mem_delete(commandCountSwapped, GpuCommand);
		sp_mem_delete(commandPrefixScan, GpuCommand);
		sp_mem_delete(commandPrefixScanSwaped, GpuCommand);
		sp_mem_delete(commandReorder, GpuCommand);
		sp_mem_delete(commandReorderSwapped, GpuCommand);
		sp_mem_delete(commandReverse, GpuReverse);
	}
}

#undef BUCKET_LENGTH

#endif // OPENCL_ENABLED