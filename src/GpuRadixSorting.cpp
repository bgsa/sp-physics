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

		SpDirectory* filename = SpDirectory::currentDirectory()
			->add(SP_DIRECTORY_OPENCL_SOURCE)
			->add("RadixSorting.cl");

		SP_FILE file;
		file.open(filename->name()->data(), std::ios::in);
		sp_mem_delete(filename, SpDirectory);
		sp_size fileSize = file.length();
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
			->updateInputParameter(0, newIndexesLength)
			->updateInputParameter(2, newIndexes);
	}

	GpuRadixSorting* GpuRadixSorting::setParameters(cl_mem inputGpu, sp_uint inputLength, cl_mem indexesGpu, cl_mem indexesLengthGpu, sp_uint strider)
	{
		totalWorkSize[0] = inputLength;
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

		const sp_uint offsetTableSizeNegatives =  inputLength * TWO_UINT * SIZEOF_UINT;
		offsetTable1Negatives = gpu->createBuffer(offsetTableSizeNegatives, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);
		offsetTable2Negatives = gpu->createBuffer(offsetTableSizeNegatives, CL_MEM_READ_WRITE | CL_MEM_ALLOC_HOST_PTR);

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
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorderNegatives");
	
		return this;
	}

	cl_mem GpuRadixSorting::execute(sp_uint previousEventsLength, cl_event* previousEvents)
	{
		sp_bool indexesChanged = false;
		sp_bool offsetChanged = false;

		globalWorkSize[0] = threadsLength;
		localWorkSize[0] = defaultLocalWorkSize;

		for (sp_uint digitIndex = ZERO_UINT; digitIndex < 9u; digitIndex++)  // for each digit in the number ...
		{
			// COUNT *************
			if (offsetChanged)
				commandCountSwapped->updateInputParameter(THREE_UINT, offsetTable2);

			if (indexesChanged)
			{
				commandCountSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, previousEventsLength);
				previousEvents = &commandCountSwapped->lastEvent;
				previousEventsLength = ONE_UINT;
			}
			else
			{
				commandCount->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, previousEventsLength);
				previousEvents = &commandCount->lastEvent;
				previousEventsLength = ONE_UINT;
			}

			// PREFIX SCAN *********
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

			// REORDER ************
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
			->updateInputParameter(1u, offsetTable1Negatives)
			->updateInputParameter(2u, inputIndexesGpu)
			->updateInputParameter(3u, outputIndexesGpu);

		if (indexChanged)
		{
			commandCountNegative->updateInputParameter(1u, outputIndexesGpu);
			commandReorderNegative->swapInputParameter(2u, 3u);
		}

		commandCountNegative->execute(ONE_UINT, totalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
		previousEvent = commandCountNegative->lastEvent;

		offsetPrefixScanCpu = TWO_UINT; // prepare prefix scan
		while(offsetPrefixScanCpu - 1 < totalWorkSize[0])
		{
			commandPrefixScanNegative
				->swapInputParameter(ZERO_UINT, ONE_UINT)
				->execute(1, totalWorkSize, localWorkSize, &offsetPrefixScanCpu, &previousEvent, ONE_UINT);
			previousEvent = commandPrefixScanNegative->lastEvent;

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		if (offsetChanged)
			commandReorderNegative->updateInputParameter(1u, offsetTable2Negatives);
	
		commandReorderNegative->execute(1, totalWorkSize, localWorkSize, 0, &previousEvent, ONE_UINT);
		this->lastEvent = commandReorderNegative->lastEvent;

		return commandReorderNegative->getInputParameter(3u);
	}
	
	void GpuRadixSorting::dispose()
	{
		gpu->releaseBuffer(offsetTable1Negatives);
		gpu->releaseBuffer(offsetTable2Negatives);

		sp_mem_delete(commandReorderNegative, GpuCommand);
		sp_mem_delete(commandPrefixScanNegative, GpuCommand);
		sp_mem_delete(commandCountNegative, GpuCommand);

		gpu->releaseBuffer(outputIndexesGpu);
		gpu->releaseBuffer(offsetTable1);
		gpu->releaseBuffer(offsetTable2);

		sp_mem_delete(commandCount, GpuCommand);
		sp_mem_delete(commandCountSwapped, GpuCommand);
		sp_mem_delete(commandPrefixScan, GpuCommand);
		sp_mem_delete(commandPrefixScanSwaped, GpuCommand);
		sp_mem_delete(commandReorder, GpuCommand);
		sp_mem_delete(commandReorderSwapped, GpuCommand);
	}
}

#undef BUCKET_LENGTH

#endif // OPENCL_ENABLED