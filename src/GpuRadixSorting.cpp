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

		gpu->commandManager->buildProgram(source, SIZEOF_CHAR * fileSize, buildOptions, &program);

		ALLOC_RELEASE(source);
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

	sp_uint l;
	GpuRadixSorting* GpuRadixSorting::setParameters(cl_mem inputGpu, sp_uint inputLength, cl_mem indexesGpu, cl_mem inputLengthGPU, sp_uint strider)
	{
		l = inputLength;

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
			->setInputParameter(inputLengthGPU, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->buildFromProgram(program, "count");

		commandCountSwapped = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(inputLengthGPU, SIZEOF_UINT)
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
			->setInputParameter(inputLengthGPU, SIZEOF_UINT)
			->setInputParameter(offsetTable1, offsetTableSize)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorder");

		commandReorderSwapped = gpu->commandManager->createCommand()
			->setInputParameter(inputGpu, inputSize)
			->setInputParameter(inputLengthGPU, SIZEOF_UINT)
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
			->setInputParameter(inputLengthGPU, SIZEOF_UINT)
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->buildFromProgram(program, "countNegatives");

		commandPrefixScanNegative = gpu->commandManager->createCommand()
			->setInputParameter(offsetTable2Negatives, offsetTableSizeNegatives)  //use buffer hosted GPU
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->buildFromProgram(program, "prefixScanNegatives");

		commandReorderNegative = gpu->commandManager->createCommand()
			->setInputParameter(inputLengthGPU, SIZEOF_UINT)
			->setInputParameter(offsetTable1Negatives, offsetTableSizeNegatives)
			->setInputParameter(inputIndexesGpu, SIZEOF_UINT * inputLength)
			->setInputParameter(outputIndexesGpu, SIZEOF_UINT * inputLength)
			->buildFromProgram(program, "reorderNegatives");
	
		return this;
	}

	cl_mem GpuRadixSorting::execute(sp_uint previousEventsLength, cl_event* previousEvents, cl_event* evt)
	{
		sp_bool indexesChanged = false;
		sp_bool offsetChanged = false;
		cl_event lastEvent, pEvent;

		globalWorkSize[0] = threadsLength;
		localWorkSize[0] = defaultLocalWorkSize;

		for (sp_uint digitIndex = ZERO_UINT; digitIndex < 9u; digitIndex++)  // for each digit in the number ...
		{
			// COUNT *************
			if (offsetChanged)
				commandCountSwapped->updateInputParameter(THREE_UINT, offsetTable2);

			if (indexesChanged)
			{
				commandCountSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEventsLength, previousEvents, &lastEvent);

				pEvent = lastEvent;
				previousEventsLength = ONE_UINT;
			}
			else
			{
				commandCount->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, previousEventsLength, previousEvents, &lastEvent);

				pEvent = lastEvent;
				previousEventsLength = ONE_UINT;
			}

			// PREFIX SCAN *********
			sp_uint offsetPrefixScanCpu = BUCKET_LENGTH;
			for (sp_uint i = ZERO_UINT; i < maxIteration; i++)
			{
				if (offsetChanged)
				{
					commandPrefixScanSwaped->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, ONE_UINT, &pEvent, &lastEvent);
					gpu->releaseEvent(pEvent);
					pEvent = lastEvent;
				}
				else
				{
					commandPrefixScan->execute(ONE_UINT, globalWorkSize, localWorkSize, &offsetPrefixScanCpu, ONE_UINT, &pEvent, &lastEvent);
					gpu->releaseEvent(pEvent);
					pEvent = lastEvent;
				}

				offsetChanged = !offsetChanged;
				offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
			}

			// REORDER ************
			if (offsetChanged)
				commandReorder->updateInputParameter(TWO_UINT, offsetTable2);

			if (indexesChanged)
			{
				commandReorderSwapped->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, ONE_UINT, &pEvent, &lastEvent);
				gpu->releaseEvent(pEvent);
				pEvent = lastEvent;
			}
			else
			{
				commandReorder->execute(ONE_UINT, globalWorkSize, localWorkSize, &digitIndex, ONE_UINT, &pEvent, &lastEvent);
				gpu->releaseEvent(pEvent);
				pEvent = lastEvent;
			}

			indexesChanged = !indexesChanged;
		}

		return executeNegatives(indexesChanged, lastEvent, evt);
	}

	cl_mem GpuRadixSorting::executeNegatives(sp_bool indexChanged, cl_event previousEvent, cl_event* evt)
	{
		sp_bool offsetChanged = false;
		sp_uint offsetPrefixScanCpu = TWO_UINT;
		cl_event lastEvent;

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

		commandCountNegative->execute(ONE_UINT, totalWorkSize, localWorkSize, 0, ONE_UINT, &previousEvent, &lastEvent);
		gpu->releaseEvent(previousEvent);
		previousEvent = lastEvent;

		/* DEBUG PURPOSE
		sp_float* i = ALLOC_ARRAY(sp_float, l);
		gpu->commandManager->readBuffer(commandCountNegative->getInputParameter(0), sizeof(sp_float) * l, i);

		sp_uint* idx = ALLOC_ARRAY(sp_uint, l);
		gpu->commandManager->readBuffer(commandCountNegative->getInputParameter(1), sizeof(sp_uint) * l, idx);

		sp_uint* off = ALLOC_ARRAY(sp_uint, l * 2);
		gpu->commandManager->readBuffer(offsetTable1Negatives, sizeof(sp_uint) * 2 * l, off);

		for (sp_uint i = 0; i < l; i++)
		{
			for (sp_uint j = 0; j < 2u; j++)
			{
				sp_log_debug1u(off[i * 2 + j]);
				sp_log_debug1s(", ");
			}
			sp_log_newline();
		}
		*/

		const sp_uint offsetPrefixScanLimit = nextPowOf2(totalWorkSize[0]) + ONE_UINT;
		offsetPrefixScanCpu = TWO_UINT; // prepare prefix scan
		while(offsetPrefixScanCpu < offsetPrefixScanLimit)
		{
			commandPrefixScanNegative
				->swapInputParameter(ZERO_UINT, ONE_UINT)
				->execute(1, totalWorkSize, localWorkSize, &offsetPrefixScanCpu, ONE_UINT, &previousEvent, &lastEvent);

			gpu->releaseEvent(previousEvent);
			previousEvent = lastEvent;

			offsetChanged = !offsetChanged;
			offsetPrefixScanCpu = multiplyBy2(offsetPrefixScanCpu);
		}

		if (offsetChanged)
			commandReorderNegative->updateInputParameter(1u, offsetTable2Negatives);

		/* DEBUG PURPOSE
		sp_uint* offFinal = ALLOC_ARRAY(sp_uint, l * 2);
		gpu->commandManager->readBuffer(commandReorderNegative->getInputParameter(1u), sizeof(sp_uint) * 2 * l, offFinal);

		sp_log_debug1s("OFFSET FINAL"); sp_log_newline();
		for (sp_uint i = 0; i < l; i++)
		{
			for (sp_uint j = 0; j < 2u; j++)
			{
				sp_log_debug1u(offFinal[i * 2 + j]);
				sp_log_debug1s(", ");
			}
			sp_log_newline();
		}
		*/
	
		commandReorderNegative->execute(1, totalWorkSize, localWorkSize, 0, ONE_UINT, &previousEvent, evt);
		gpu->releaseEvent(previousEvent);

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

		if (program != nullptr)
		{
			HANDLE_OPENCL_ERROR(clReleaseProgram(program));
			program = nullptr;
		}
	}
}

#undef BUCKET_LENGTH

#endif // OPENCL_ENABLED