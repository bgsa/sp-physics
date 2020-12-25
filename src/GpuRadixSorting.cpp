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

	GpuRadixSorting* GpuRadixSorting::setParameters(cl_mem inputGpu, sp_uint inputLength, cl_mem indexesGpu, cl_mem indexesLengthGpu, sp_uint strider)
	{
		globalWorkSize[0] = inputLength;
		localWorkSize[0] = defaultLocalWorkSize;

		inputIndexesGpu = indexesGpu;
		const sp_uint inputSize = inputLength * strider * SIZEOF_FLOAT;

		
		outputIndexesGpu = gpu->createBuffer(inputLength * SIZEOF_UINT, CL_MEM_READ_WRITE);

		defaultLocalWorkSize = gpu->getGroupLength(inputLength, inputLength);

		maxIteration = (sp_uint)std::ceil(std::log(multiplyBy10(inputLength))) + ONE_UINT;

		const sp_uint offsetTableSize = SIZEOF_UINT * TEN_UINT * inputLength;
		offsetTable1 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);
		offsetTable2 = gpu->createBuffer(offsetTableSize, CL_MEM_READ_WRITE);

		commandCount = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

		commandPrefixScan = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable1, offsetTableSize)  //use buffer hosted GPU
			->setInputParameter(offsetTable2, offsetTableSize)
			->buildFromProgram(program, "prefixScan");

		commandReorder = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(indexesLengthGpu, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
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
		sp_float fs[2*18];
		gpu->commandManager->readBuffer(commandCount->getInputParameter(0u), 2*18*4, fs);
		sp_log_debug1s("KDOPS: "); sp_log_newline();
		for (sp_uint i = 0; i < 2 * 18; i++) {
			sp_log_debug1f((sp_float)fs[i]); sp_log_newline();
		}

		sp_uint ui[20];
		gpu->commandManager->readBuffer(inputIndexesGpu, 8, ui);
		sp_log_debug1s("INPUT SORT INDEX IN: "); sp_log_newline();
		sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
		sp_log_debug1f((sp_float)ui[1]); sp_log_newline();
		
		gpu->commandManager->readBuffer(outputIndexesGpu, 8, ui);
		sp_log_debug1s("INPUT SORT INDEX OUT: "); sp_log_newline();
		sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
		sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

		sp_bool indexesChanged = false;
		ui[0] = 0u; ui[1] = 1u;
		gpu->commandManager->updateBuffer(inputIndexesGpu, 8, ui);

		for (sp_uint digitIndex = ZERO_UINT; digitIndex < 9u; digitIndex++)  // for each digit in the number ...
		{
			sp_log_debug1sfnl("DIGIT INDEX: ", (sp_float) digitIndex);

			// COUNT *************
			if (indexesChanged)
				commandCount->updateInputParameter(1u, outputIndexesGpu);
			else
				commandCount->updateInputParameter(1u, inputIndexesGpu);

			gpu->commandManager->readBuffer(commandCount->getInputParameter(1u), 8, ui);
			sp_log_debug1s("INDEXES: "); sp_log_newline();
			sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

			commandCount->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEvents, previousEventsLength);
				
			previousEvents = &commandCount->lastEvent;
			previousEventsLength = ONE_UINT;
			
			// PREFIX SCAN *********
			commandPrefixScan
				->updateInputParameter(ZERO_UINT, offsetTable2)
				->updateInputParameter(ONE_UINT, offsetTable1);

			sp_uint offsetPrefixScanCpu = BUCKET_LENGTH;
			sp_bool offsetChanged = false;
			for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
			{
				commandPrefixScan
					->swapInputParameter(ZERO_UINT, ONE_UINT)
					->execute(1u, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, previousEvents, ONE_UINT);
				
				previousEvents[0] = commandPrefixScan->lastEvent;
		
				offsetChanged = !offsetChanged;
				offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
			}

			// REORDER ************
			if (indexesChanged)
			{
				commandReorder->updateInputParameter(3u, outputIndexesGpu);
				commandReorder->updateInputParameter(4u, inputIndexesGpu);
			}
			else
			{
				commandReorder->updateInputParameter(3u, inputIndexesGpu);
				commandReorder->updateInputParameter(4u, outputIndexesGpu);
			}


			gpu->commandManager->readBuffer(commandReorder->getInputParameter(3), 8, ui);
			sp_log_debug1s("REORDER INDEX POSITIVE IN: "); sp_log_newline();
			sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

			gpu->commandManager->readBuffer(commandReorder->getInputParameter(2), 20*4, ui);
			sp_log_debug1s("REORDER OFFSET POSITIVE IN: "); sp_log_newline();
			for (sp_uint i = 0; i < 20; i++) {
				sp_log_debug1f((sp_float)ui[i]); sp_log_newline();
			}


			if (offsetChanged)
				commandReorder->updateInputParameter(2u, offsetTable2);
			else
				commandReorder->updateInputParameter(2u, offsetTable1);
			
			commandReorder->execute(1u, globalWorkSize, localWorkSize, &digitIndex, previousEvents, ONE_UINT);
			previousEvents[0] = commandReorder->lastEvent;

			indexesChanged = !indexesChanged;
		}

		lastEvent = previousEvents[0];

		return executeNegatives(indexesChanged);
	}

	cl_mem GpuRadixSorting::executeNegatives(sp_bool indexChanged)
	{
		sp_uint ui[4];
		gpu->commandManager->readBuffer(inputIndexesGpu, 8, ui);
		sp_log_debug1s("INPUT SORT INDEX NEGATIVE IN: "); sp_log_newline();
		sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
		sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

		gpu->commandManager->readBuffer(outputIndexesGpu, 8, ui);
		sp_log_debug1s("INPUT SORT INDEX NEGATIVE OUT: "); sp_log_newline();
		sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
		sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

		if (indexChanged)
		{
			commandCountNegative->updateInputParameter(1u, outputIndexesGpu);

			commandReorderNegative
				->updateInputParameter(TWO_UINT, outputIndexesGpu)
				->updateInputParameter(THREE_UINT, inputIndexesGpu);
		}
		else
		{
			commandCountNegative->updateInputParameter(1u, inputIndexesGpu);

			commandReorderNegative
				->updateInputParameter(TWO_UINT, inputIndexesGpu)
				->updateInputParameter(THREE_UINT, outputIndexesGpu);
		}

		commandCountNegative->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, &lastEvent, ONE_UINT);
		lastEvent = commandCountNegative->lastEvent;

		// PREFIX COMMAND
		commandPrefixScanNegative
			->updateInputParameter(ZERO_UINT, offsetTable2Negatives)
			->updateInputParameter(ONE_UINT, offsetTable1Negatives);

		sp_bool offsetChanged = false;
		sp_uint offsetPrefixScanCpu = TWO_UINT;

		while(offsetPrefixScanCpu - 1 < globalWorkSize[0])
		{
			//const sp_uint offsetTableSizeNegatives = inputLength * TWO_UINT * SIZEOF_UINT;

			gpu->commandManager->readBuffer(offsetTable1Negatives, 16, ui);
			sp_log_debug1s("OFFSET TABLE NEGATIVE 1 IN: "); sp_log_newline();
			sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[1]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[2]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[3]); sp_log_newline();

			gpu->commandManager->readBuffer(offsetTable2Negatives, 16, ui);
			sp_log_debug1s("OFFSET TABLE NEGATIVE 2 IN: "); sp_log_newline();
			sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[1]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[2]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[3]); sp_log_newline();

			commandPrefixScanNegative
				->swapInputParameter(ZERO_UINT, ONE_UINT)
				->execute(1, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, &lastEvent, ONE_UINT);
			lastEvent = commandPrefixScanNegative->lastEvent;


			gpu->commandManager->readBuffer(offsetTable1Negatives, 16, ui);
			sp_log_debug1s("OFFSET TABLE NEGATIVE 1 OUT: "); sp_log_newline();
			sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[1]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[2]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[3]); sp_log_newline();

			gpu->commandManager->readBuffer(offsetTable2Negatives, 16, ui);
			sp_log_debug1s("OFFSET TABLE NEGATIVE 2 OUT: "); sp_log_newline();
			sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[1]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[2]); sp_log_newline();
			sp_log_debug1f((sp_float)ui[3]); sp_log_newline();


			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		if (offsetChanged)
			commandReorderNegative->updateInputParameter(ONE_UINT, offsetTable2Negatives);
		else	
			commandReorderNegative->updateInputParameter(ONE_UINT, offsetTable1Negatives);

		gpu->commandManager->readBuffer(commandReorderNegative->getInputParameter(2), 8, ui);
		sp_log_debug1s("INDEXES REORDER NEG: "); sp_log_newline();
		sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
		sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

		commandReorderNegative
			->execute(ONE_UINT, globalWorkSize, localWorkSize, 0, &lastEvent, ONE_UINT);
		
		lastEvent = commandReorderNegative->lastEvent;

		cl_mem out = commandReorderNegative->getInputParameter(THREE_UINT);

		gpu->commandManager->readBuffer(out, 8, ui);
		sp_log_debug1s("RESULT SORT INDEXES: "); sp_log_newline();
		sp_log_debug1f((sp_float)ui[0]); sp_log_newline();
		sp_log_debug1f((sp_float)ui[1]); sp_log_newline();

		return commandReorderNegative->getInputParameter(THREE_UINT);
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
		sp_mem_delete(commandPrefixScan, GpuCommand);
		sp_mem_delete(commandReorder, GpuCommand);
	}
}

#undef BUCKET_LENGTH

#endif // OPENCL_ENABLED